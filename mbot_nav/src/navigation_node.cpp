#include "navigation_node.hpp"
#include "astar.hpp"
#include "obstacle_distance_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

NavigationNode::NavigationNode()
    : Node("navigation_node")
{
    // Declare and get pose_source parameter
    this->declare_parameter<std::string>("pose_source", "manual");
    pose_source_ = this->get_parameter("pose_source").as_string();

    RCLCPP_INFO(this->get_logger(), "Pose source: %s", pose_source_.c_str());

    // Publishers
    waypoints_pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
        "/waypoints", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);

    // Setup TF listener if using TF mode
    if (pose_source_ == "tf") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a timer to periodically update initial pose from TF
        tf_update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavigationNode::updateInitialPose, this));

        RCLCPP_INFO(this->get_logger(), "Using TF-based pose lookup");
    } else {
        // Subscribe to /initialpose topic in manual mode
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::QoS(10),
            std::bind(&NavigationNode::initialPoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Using manual pose from RViz");
    }

    // Subscriptions
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        rclcpp::QoS(10),
        std::bind(&NavigationNode::goalPoseCallback, this, std::placeholders::_1));
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
    dist_grid_.computeDistFromMap(*msg);
    // RCLCPP_INFO(this->get_logger(), "Updated distance grid from map");
}

void NavigationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Extract Pose2D from PoseWithCovarianceStamped
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    // Convert quaternion to theta
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose2d.theta = yaw;

    initial_pose_ = pose2d;
    RCLCPP_INFO(this->get_logger(), "Received initial pose: (%.2f, %.2f, %.2f)",
                pose2d.x, pose2d.y, pose2d.theta);

    // If we already have a goal, replan
    if (goal_pose_) {
        planPath();
    }
}

void NavigationNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Extract Pose2D from PoseStamped
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = msg->pose.position.x;
    pose2d.y = msg->pose.position.y;

    // Convert quaternion to theta
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose2d.theta = yaw;

    goal_pose_ = pose2d;
    RCLCPP_INFO(this->get_logger(), "Received goal pose: (%.2f, %.2f, %.2f)",
                pose2d.x, pose2d.y, pose2d.theta);

    // Plan path immediately if we have initial pose
    if (initial_pose_) {
        planPath();
    }
}

void NavigationNode::planPath()
{
    // we call planPath() in astar.cpp here to get the path
    // path will get published as waypoints, and your motion controller should follow the path

    if (!latest_map_) {
        RCLCPP_WARN(this->get_logger(), "No map available yet");
        return;
    }

    if (!initial_pose_ || !goal_pose_) {
        RCLCPP_WARN(this->get_logger(), "Missing initial_pose or goal_pose");
        return;
    }

    mbot_interfaces::msg::Pose2DArray path;
    bool success = astar_planner_.planPath(dist_grid_, initial_pose_.value(), goal_pose_.value(), path);

    if (!success || path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to plan path from (%.2f, %.2f) to (%.2f, %.2f)",
                    initial_pose_->x, initial_pose_->y, goal_pose_->x, goal_pose_->y);
        return;
    }

    publishWaypoints(path);
    publishPath(path);
}

void NavigationNode::publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path)
{
    waypoints_pub_->publish(path);
}

void NavigationNode::publishPath(const mbot_interfaces::msg::Pose2DArray& path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& pose2d : path.poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose2d.x;
        pose_stamped.pose.position.y = pose2d.y;
        pose_stamped.pose.position.z = 0.0;

        // Convert theta to quaternion
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = std::sin(pose2d.theta / 2.0);
        pose_stamped.pose.orientation.w = std::cos(pose2d.theta / 2.0);

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);
}

bool NavigationNode::getRobotPoseFromTF(geometry_msgs::msg::Pose2D& pose)
{
    try {
        // Get the transform from map to base_footprint
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

        // Extract position
        pose.x = transform.transform.translation.x;
        pose.y = transform.transform.translation.y;

        // Convert quaternion to theta
        tf2::Quaternion q(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pose.theta = yaw;

        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Robot pose TF look up error: %s", ex.what());
        return false;
    }
}

void NavigationNode::updateInitialPose()
{
    geometry_msgs::msg::Pose2D pose;
    if (getRobotPoseFromTF(pose)) {
        initial_pose_ = pose;

        // Check if goal has been reached
        if (goal_pose_) {
            double dx = pose.x - goal_pose_->x;
            double dy = pose.y - goal_pose_->y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                goal_pose_.reset();  // Clear goal so we stop replanning
            } else {
                // Still navigating to goal - replan with current robot pose
                planPath();
            }
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
