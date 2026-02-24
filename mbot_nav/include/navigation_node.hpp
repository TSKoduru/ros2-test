#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "astar.hpp"
#include "obstacle_distance_grid.hpp"

#include <memory>
#include <optional>
#include <cmath>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode();

private:
    // Subscribers callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Planning
    void planPath();
    void publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path);
    void publishPath(const mbot_interfaces::msg::Pose2DArray& path);

    // Pose retrieval
    bool getRobotPoseFromTF(geometry_msgs::msg::Pose2D& pose);
    void updateInitialPose();

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    // Publishers
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<geometry_msgs::msg::Pose2D> initial_pose_;
    std::optional<geometry_msgs::msg::Pose2D> goal_pose_;
    double goal_reached_threshold_ = 0.15;

    // Planners
    mbot_nav::AStarPlanner astar_planner_;
    ObstacleDistanceGrid dist_grid_;

    // TF support
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Configuration
    std::string pose_source_;  // "manual" or "tf"
    rclcpp::TimerBase::SharedPtr tf_update_timer_;
};