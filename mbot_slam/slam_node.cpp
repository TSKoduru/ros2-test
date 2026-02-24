#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "particle_filter.hpp"
#include "slam_utils.hpp"
#include "mapping.hpp"
#include "moving_laser_scan.hpp"
#include "obstacle_distance_grid.hpp"

#include <algorithm>
#include <tf2/utils.h>
#include <cmath>

class SLAMNode final : public rclcpp::Node
{
public:
    SLAMNode()
    : Node("mbot_slam_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      grid_(10.0, 10.0, 0.01, -5.0, -5.0)
    {
        // Particle filter
        particle_filter_ = std::make_unique<mbot_slam::ParticleFilter>();

        // Transform broadcaster for map -> odom
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/particle_cloud", 10);
        est_path_pub_ = create_publisher<nav_msgs::msg::Path>("/estimated_path", 10);
        odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        // Timer to periodically publish map
        map_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                       std::bind(&SLAMNode::publishMap, this));

        // Subscriptions
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&SLAMNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "SLAM node initialized. Waiting for first scan...");
    }
    
private:
    std::unique_ptr<mbot_slam::ParticleFilter> particle_filter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>                   tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>        tf_listener_;

    mbot_slam::OccupancyGrid       grid_;
    ObstacleDistanceGrid dist_grid_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr map_timer_;

    nav_msgs::msg::Path est_path_;
    nav_msgs::msg::Path odom_path_;
    geometry_msgs::msg::Pose prev_est_pose;

    bool has_initialized_{false};

    // Scan callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Lookup odometry transform
        nav_msgs::msg::Odometry odom_msg;
        try {
            // LaserScan.msg: Timestamp in the header is the acquisition time of the first ray in the scan
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", scan->header.stamp);

            odom_msg.header = tf_odom_base.header;
            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        // Auto-initialize on first scan from odometry origin
        if (!has_initialized_) {
            try {
                particle_filter_->initializeAtPose(odom_msg.pose.pose);
                particle_filter_->resetOdometry(odom_msg);

                // Initialize grid with initial scan
                MovingLaserScan initial_scan(*scan, odom_msg.pose.pose, odom_msg.pose.pose);
                updateGridWithScan(initial_scan);
                prev_est_pose = odom_msg.pose.pose;

                has_initialized_ = true;
                RCLCPP_INFO(get_logger(), "Particle filter initialized at odometry pose.");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Failed to lookup odom transform at init: %s", ex.what());
                return;
            }
        } else {
            // est_pose calculated based on the odometry at the time of the first ray in scan
            // which means est_pose is where the robot at when the first ray was taken.
            const auto est_pose = particle_filter_->update(odom_msg, *scan, dist_grid_);

            // Calculate the estimated pose at the end of the scan for interpolation
            const auto est_pose_end = calculateScanEndPose(est_pose, odom_msg, scan);

            // Interpolate the scan over the robot motion
            MovingLaserScan interpolated_scan(*scan, prev_est_pose, est_pose_end);
            prev_est_pose = est_pose;
            updateGridWithScan(interpolated_scan);

            // Broadcast map -> odom transform
            broadcastTf(est_pose, odom_msg.pose.pose, scan->header.stamp);

            // Publish pose, cloud, paths for visualization
            publishPose(est_pose, scan->header.stamp);
            publishCloud(scan->header.stamp);
            publishEstPath(est_pose, scan->header.stamp);
            publishOdomPath(odom_msg.pose.pose, scan->header.stamp);
        }
    }

    geometry_msgs::msg::Pose calculateScanEndPose(
        const geometry_msgs::msg::Pose &est_pose,
        const nav_msgs::msg::Odometry &odom_msg,
        const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        // If we want to interpolate the scan over the robot motion
        // we still need the pose at the end of the scan.
        // 1. Calculate scan end time
        const size_t num_rays = scan->ranges.size();
        double scan_duration_sec = 0.0;
        if (num_rays > 1 && scan->time_increment > 0.0) {
            scan_duration_sec = scan->time_increment * (num_rays - 1);
        }
        const rclcpp::Time scan_end_time = scan->header.stamp +
            rclcpp::Duration::from_seconds(std::max(0.0, scan_duration_sec));

        // 2. Lookup odometry at scan end time
        nav_msgs::msg::Odometry odom_msg_end;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base_end =
                tf_buffer_->lookupTransform("odom", "base_footprint", scan_end_time, rclcpp::Duration::from_seconds(0.1));

            odom_msg_end.header = tf_odom_base_end.header;
            odom_msg_end.pose.pose.position.x = tf_odom_base_end.transform.translation.x;
            odom_msg_end.pose.pose.position.y = tf_odom_base_end.transform.translation.y;
            odom_msg_end.pose.pose.position.z = tf_odom_base_end.transform.translation.z;
            odom_msg_end.pose.pose.orientation = tf_odom_base_end.transform.rotation;

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom_msg_end transform: %s", ex.what());
            return est_pose;  // Return start pose as fallback
        }

        // 3. Convert poses to tf2::Transform for math
        tf2::Transform T_map_base_start, T_odom_base_start;
        tf2::fromMsg(est_pose, T_map_base_start);
        tf2::fromMsg(odom_msg.pose.pose, T_odom_base_start); // This is odom at start time

        // 4. Calculate the correction: (map -> base) * (base -> odom) = (map -> odom)
        tf2::Transform T_map_odom_correction = T_map_base_start * T_odom_base_start.inverse();

        tf2::Transform T_odom_base_end;
        tf2::fromMsg(odom_msg_end.pose.pose, T_odom_base_end);

        // 5. Apply correction to get estimated map -> base at end of scan
        // This correction is also what we will broadcast as map -> odom
        tf2::Transform T_map_base_end = T_map_odom_correction * T_odom_base_end;

        // 6. Convert back to a Pose message
        geometry_msgs::msg::Pose est_pose_end;
        tf2::toMsg(T_map_base_end, est_pose_end);

        return est_pose_end;
    }

    void updateGridWithScan(const MovingLaserScan &scan)
    {
        for (const auto& ray : scan)
        {
            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);

            if (!ray_cells.empty()) {
                // Update cells as free along the ray, excluding the last cell
                for (size_t j = 0; j + 1 < ray_cells.size(); ++j) {
                    grid_.markCellFree(ray_cells[j].first, ray_cells[j].second);
                }

                // Mark the hit as occupied
                grid_.markCellOccupied(ray_cells.back().first, ray_cells.back().second);
            }
        }
        // Update distance grid ONCE after all rays are processed
        dist_grid_.computeDistFromMap(buildMapMessage());
    }
    
    nav_msgs::msg::OccupancyGrid buildMapMessage()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();

        map_msg.info.resolution = grid_.getResolution();
        map_msg.info.width = grid_.getWidth();
        map_msg.info.height = grid_.getHeight();
        map_msg.info.origin.position.x = grid_.getOriginX();
        map_msg.info.origin.position.y = grid_.getOriginY();
        map_msg.info.origin.position.z = 0;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data = grid_.getOccupancyGrid();
        return map_msg;
    }

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_msg = buildMapMessage();
        map_pub_->publish(map_msg);
    }

    // Helpers: publish pose/cloud
    void publishPose(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp) const
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "map";
        msg.pose.pose       = pose;

        pose_pub_->publish(msg);
    }

    void publishCloud(const rclcpp::Time &stamp) const
    {
        const auto &particles = particle_filter_->particleCloud().particles;

        // Downsample: show only every Nth particle
        static constexpr int DOWNSAMPLE = 2;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = stamp;
        cloud.height = 1;
        cloud.width = (particles.size() + DOWNSAMPLE - 1) / DOWNSAMPLE;

        // Set fields: X, Y, Z
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cloud.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (size_t i = 0; i < particles.size(); i += DOWNSAMPLE) {
            *iter_x = particles[i].pose.position.x;
            *iter_y = particles[i].pose.position.y;
            *iter_z = particles[i].pose.position.z + 0.02;  // slight offset for visibility
            ++iter_x; ++iter_y; ++iter_z;
        }

        cloud_pub_->publish(cloud);
    }

    void publishEstPath(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp)
    {
        est_path_.header.frame_id = "map";
        est_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = pose;
        est_path_.poses.push_back(pose_stamped);

        est_path_pub_->publish(est_path_);
    }

    void publishOdomPath(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp)
    {
        odom_path_.header.frame_id = "odom";
        odom_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = pose;
        odom_path_.poses.push_back(pose_stamped);

        odom_path_pub_->publish(odom_path_);
    }

    void broadcastTf(const geometry_msgs::msg::Pose  &map_base,
                     const geometry_msgs::msg::Pose  &odom_base,
                     const rclcpp::Time              &stamp) const
    {
        tf2::Transform tf_map_base;
        tf2::fromMsg(map_base, tf_map_base);

        tf2::Transform tf_odom_base;
        tf2::fromMsg(odom_base, tf_odom_base);

        const tf2::Transform tf_map_odom = tf_map_base * tf_odom_base.inverse();

        geometry_msgs::msg::TransformStamped out;
        out.header.stamp    = stamp;
        out.header.frame_id = "map";
        out.child_frame_id  = "odom";
        out.transform       = tf2::toMsg(tf_map_odom);

        tf_broadcaster_->sendTransform(out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();
    return 0;
}
