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
#include "obstacle_distance_grid.hpp"
#include "localization_utils.hpp"

#include <algorithm>

class LocalizationNode final : public rclcpp::Node
{
public:
    LocalizationNode()
    : Node("mbot_localization_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Declare parameter for controlling tf publishing
        // When using bag data, set to false to avoid tf conflicts
        this->declare_parameter("publish_tf", true);
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        if (publish_tf_) {
            RCLCPP_INFO(get_logger(), "TF publishing ENABLED. Set initial pose in RViz using '2D Pose Estimate' tool");
        } else {
            RCLCPP_INFO(get_logger(), "TF publishing DISABLED (bag playback mode)");
        }

        // Particle filter
        particle_filter_ = std::make_unique<mbot_localization::ParticleFilter>();

        // Transform broadcaster for map -> odom
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/particle_cloud", 10);
        est_path_pub_ = create_publisher<nav_msgs::msg::Path>("/estimated_path", 10);
        reference_path_pub_ = create_publisher<nav_msgs::msg::Path>("/reference_path", 10);
        odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);

        // Subscriptions
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&LocalizationNode::mapCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&LocalizationNode::scanCallback, this, std::placeholders::_1));

        init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

        // Here we use nav2_acml as ground truth pose for comparison    
        reference_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&LocalizationNode::amclCallback, this, std::placeholders::_1));
    }
    
private:
    std::unique_ptr<mbot_localization::ParticleFilter> particle_filter_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>                   tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>        tf_listener_;

    ObstacleDistanceGrid           dist_grid_;
    nav_msgs::msg::OccupancyGrid   map_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr             map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr              scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reference_sub_;

    nav_msgs::msg::Path est_path_;
    nav_msgs::msg::Path amcl_path_;
    nav_msgs::msg::Path odom_path_;

    bool have_map_{false};
    bool have_initial_pose_{false};
    bool publish_tf_{true};

    // Map callback
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        dist_grid_.computeDistFromMap(map_);
        have_map_ = true;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Map received (%ux%u)", map_.info.width, map_.info.height);
    }

    // Initial-pose callback
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        particle_filter_->initializeAtPose(msg->pose.pose);

        // Lookup odometry transform
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0));

            // Create odometry message from transform
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header = tf_odom_base.header;
            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
            particle_filter_->resetOdometry(odom_msg);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom transform at init time: %s", ex.what());
        }

        have_initial_pose_ = true;

        RCLCPP_INFO(
            get_logger(), "Particle filter initialised at (%.2f, %.2f)",
            msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    // Scan callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!have_map_) return;

        if (!have_initial_pose_) {
            if (publish_tf_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "Waiting for initial pose. Please set it in RViz using '2D Pose Estimate' tool");
            }
            return;
        }

        // Lookup odometry transform
        nav_msgs::msg::Odometry odom_msg;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_base =
                tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0));

            odom_msg.header = tf_odom_base.header;
            odom_msg.pose.pose.position.x = tf_odom_base.transform.translation.x;
            odom_msg.pose.pose.position.y = tf_odom_base.transform.translation.y;
            odom_msg.pose.pose.position.z = tf_odom_base.transform.translation.z;
            odom_msg.pose.pose.orientation = tf_odom_base.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        // Particle-filter update with odometry at scan timestamp
        const auto est_pose = particle_filter_->update(odom_msg, *scan, dist_grid_);

        publishPose(est_pose, scan->header.stamp);
        publishCloud(scan->header.stamp);
        publishEstPath(est_pose, scan->header.stamp);
        publishOdomPath(odom_msg.pose.pose, scan->header.stamp);

        if (publish_tf_) {
            broadcastTf(est_pose, odom_msg.pose.pose, scan->header.stamp);
        }
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        amcl_path_.header.frame_id = msg->header.frame_id;
        amcl_path_.header.stamp = msg->header.stamp;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        amcl_path_.poses.push_back(pose);

        reference_path_pub_->publish(amcl_path_);
    }

    // Helpers: publish pose/cloud
    void publishPose(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &stamp) const
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = map_.header.frame_id;
        msg.pose.pose       = pose;

        pose_pub_->publish(msg);
    }

    void publishCloud(const rclcpp::Time &stamp) const
    {
        const auto &particles = particle_filter_->particleCloud().particles;

        // Downsample: show only every Nth particle
        static constexpr int DOWNSAMPLE = 2; 

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = map_.header.frame_id;
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
        est_path_.header.frame_id = map_.header.frame_id;
        est_path_.header.stamp = stamp;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map_.header.frame_id;
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
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
