#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "frontier_explorer.hpp"
#include "astar.hpp"
#include "obstacle_distance_grid.hpp"

#include <memory>
#include <optional>
#include <cmath>

class ExplorationNode : public rclcpp::Node
{
public:
    ExplorationNode();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallback();
    void planPath();
    void publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path);
    void publishPath(const mbot_interfaces::msg::Pose2DArray& path);

    // Pose retrieval
    bool getRobotPose(geometry_msgs::msg::Pose2D &pose);

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    // Timer for periodic frontier exploration
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<geometry_msgs::msg::Pose2D> current_goal_;
    double goal_reached_threshold_ = 0.15;

    // Planners
    mbot_nav::AStarPlanner astar_planner_;
    mbot_nav::FrontierExplorer frontier_explorer_;
    ObstacleDistanceGrid dist_grid_;

    // TF support
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Exploration state
    std::optional<geometry_msgs::msg::Pose2D> origin_pose_;
    bool is_returning_ = false;
    int no_frontier_count_ = 0;  // Counter for consecutive "no frontier" results
    int no_frontier_threshold_ = 10;  // Give it N iterations before returning to origin
};
