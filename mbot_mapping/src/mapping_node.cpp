#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mapping.hpp"
#include "moving_laser_scan.hpp"
#include <geometry_msgs/msg/pose.hpp>


class MappingNode : public rclcpp::Node
{
public:
    MappingNode()
    : Node("mbot_mapping_node"),
      grid_(10.0, 10.0, 0.05, -5.0, -5.0),  // 10x10m map, 5cm resolution, origin at center (-5,-5) to (5,5)
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        RCLCPP_INFO(this->get_logger(), "MappingNode will start in 1 second...");

        // Delay initialization by 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        double publish_rate = 1.0; // Hz
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), std::bind(&MappingNode::publishMap, this));
    }

private:
    OccupancyGrid grid_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool initialized_ = false;

    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Stamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);

        // Lookup odometry transform at scan timestamp
        geometry_msgs::msg::Pose current_pose;
        try {
            // LaserScan.msg: Timestamp in the header is the acquisition time of the first ray in the scan
            geometry_msgs::msg::TransformStamped tf_odom_lidar =
                tf_buffer_->lookupTransform("odom", "lidar_link", msg->header.stamp);
            current_pose.position.x = tf_odom_lidar.transform.translation.x;
            current_pose.position.y = tf_odom_lidar.transform.translation.y;
            current_pose.position.z = tf_odom_lidar.transform.translation.z;
            current_pose.orientation = tf_odom_lidar.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        if (!initialized_) {
            initialized_ = true;
            return;
        }
        
        // Calculate the odom pose at the end of the scan for interpolation
        const auto current_pose_end = calculateScanEndPose(current_pose, latest_scan_);
        MovingLaserScan scan(*msg, current_pose, current_pose_end);

        updateGrid(scan);
    }

    geometry_msgs::msg::Pose calculateScanEndPose(
        const geometry_msgs::msg::Pose &current_pose,
        const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        // Calculate scan end time
        const size_t num_rays = scan->ranges.size();
        double scan_duration_sec = 0.0;
        if (num_rays > 1 && scan->time_increment > 0.0) {
            scan_duration_sec = scan->time_increment * (num_rays - 1);
        }
        const rclcpp::Time scan_end_time = scan->header.stamp +
            rclcpp::Duration::from_seconds(std::max(0.0, scan_duration_sec));

        // Lookup odometry at scan end time with tolerance for extrapolation
        geometry_msgs::msg::Pose pose_end = current_pose;
        try {
            geometry_msgs::msg::TransformStamped tf_odom_lidar =
                tf_buffer_->lookupTransform("odom", "lidar_link", scan_end_time,
                                            rclcpp::Duration::from_seconds(0.1));  // 100ms tolerance

            pose_end.position.x = tf_odom_lidar.transform.translation.x;
            pose_end.position.y = tf_odom_lidar.transform.translation.y;
            pose_end.position.z = tf_odom_lidar.transform.translation.z;
            pose_end.orientation = tf_odom_lidar.transform.rotation;
        } catch (tf2::TransformException &ex) {
            // Use latest available transform if exact time not available
            try {
                geometry_msgs::msg::TransformStamped tf_odom_lidar =
                    tf_buffer_->lookupTransform("odom", "lidar_link", tf2::TimePointZero);

                pose_end.position.x = tf_odom_lidar.transform.translation.x;
                pose_end.position.y = tf_odom_lidar.transform.translation.y;
                pose_end.position.z = tf_odom_lidar.transform.translation.z;
                pose_end.orientation = tf_odom_lidar.transform.rotation;
            } catch (tf2::TransformException &ex2) {
                RCLCPP_DEBUG(this->get_logger(), "Using start pose as fallback: %s", ex2.what());
                // Return start pose as fallback
            }
        }

        return pose_end;
    }

    void updateGrid(const MovingLaserScan& scan)
    {
        for (const auto& ray : scan)
        {
            RCLCPP_INFO(this->get_logger(), "Origin: (%f, %f)", ray.origin.x, ray.origin.y);
            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);
                
            if (!ray_cells.empty()) {

                // TODO #3: update the grid based on ray_cells here
                // We have a vector of cells along the ray, how to mark them?
                // Hints: use grid_.markCellFree(x_idx, y_idx) and grid_.markCellOccupied(x_idx, y_idx)

                // Doing the last cell first to make processing faster

                // TODO 3.2 - Update the last cell only if there was a valid hit
                std::pair<int, int> p = ray_cells.back();
                ray_cells.pop_back();
                // if (ray.range < scan.getMaxRange()) {
                    grid_.markCellOccupied(p.first, p.second);
                // }

                // TODO 3.1 - loop through all ray_cells. Update cells as free along the ray, excluding the last cell
                while(ray_cells.size() > 1) {
                   std::pair<int, int> p = ray_cells.back();
                   ray_cells.pop_back();
                   grid_.markCellFree(p.first, p.second);
                }

            }
        }
    }
    
    float yawFromQuat(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return static_cast<float>(yaw);
    }

    void publishMap()
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
        map_pub_->publish(map_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}