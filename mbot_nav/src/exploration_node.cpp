#include "exploration_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

ExplorationNode::ExplorationNode()
: Node("exploration_node")
{
    // Publishers
    waypoints_pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
        "/waypoints", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);

    // Subscriptions
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

    // Timer to run frontier exploration
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),
        std::bind(&ExplorationNode::timerCallback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Get initial origin pose from TF
    geometry_msgs::msg::Pose2D initial_pose;
    if (getRobotPose(initial_pose)) {
        origin_pose_ = initial_pose;
        RCLCPP_INFO(this->get_logger(), "ExplorationNode initialized with origin pose: (%.2f, %.2f, %.2f)",
                    origin_pose_->x, origin_pose_->y, origin_pose_->theta);
    } else {
        RCLCPP_WARN(this->get_logger(), "ExplorationNode initialized but could not get initial pose from TF. Will retry on first timer callback.");
    }
}

void ExplorationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
    dist_grid_.computeDistFromMap(*msg);
}

void ExplorationNode::timerCallback()
{
    if (!latest_map_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for map...");
        return;
    }

    geometry_msgs::msg::Pose2D robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;
    }

    // Retry getting origin pose if we don't have it yet
    if (!origin_pose_) {
        geometry_msgs::msg::Pose2D initial_pose;
        if (getRobotPose(initial_pose)) {
            origin_pose_ = initial_pose;
            RCLCPP_INFO(this->get_logger(), "Origin pose now available: (%.2f, %.2f, %.2f)",
                        origin_pose_->x, origin_pose_->y, origin_pose_->theta);
        }
        return;  // Wait for origin pose before starting exploration
    }

    // Check if current goal was reached
    if (current_goal_) {
        double dx = robot_pose.x - current_goal_->x;
        double dy = robot_pose.y - current_goal_->y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= goal_reached_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            current_goal_.reset(); // set current_goal_ empty
        } else {
            // Still navigating to current goal - replan with current robot pose
            planPath();
            return;
        }
    }

    // If returning and reached origin, exploration is complete
    if (is_returning_) {
        double dx = robot_pose.x - origin_pose_->x;
        double dy = robot_pose.y - origin_pose_->y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= goal_reached_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Returned to origin. Exploration complete.");
        } else {
            // Still returning - replan with current robot pose
            planPath();
        }
        return;  // Stay idle after returning to origin
    }

    //Find a valid frontier, we call selectGoal() here
    // Try to find a frontier goal
    auto goal = frontier_explorer_.selectGoal(robot_pose, dist_grid_);
    if (!goal) {
        // No frontier found - increment counter and give it another chance
        no_frontier_count_++;
        RCLCPP_WARN(this->get_logger(), "No frontier found (%d/%d). Waiting for map update...",
                    no_frontier_count_, no_frontier_threshold_);

        // Only return to origin after multiple consecutive failures
        if (no_frontier_count_ >= no_frontier_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Exploration done. Returning to origin.");
            current_goal_ = origin_pose_.value();
            is_returning_ = true;
            no_frontier_count_ = 0;  // Reset counter
            planPath();
        }
        return;
    }

    // Found a frontier - reset the counter and proceed
    no_frontier_count_ = 0;

    // Found a frontier - plan path to it
    current_goal_ = goal.value();
    planPath();
}

void ExplorationNode::planPath()
{
    if (!latest_map_) {
        RCLCPP_WARN(this->get_logger(), "No map available yet");
        return;
    }

    geometry_msgs::msg::Pose2D robot_pose;
    if (!getRobotPose(robot_pose)) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose");
        return;
    }

    if (!current_goal_) {
        RCLCPP_WARN(this->get_logger(), "No goal set");
        return;
    }

    mbot_interfaces::msg::Pose2DArray path;
    bool success = astar_planner_.planPath(dist_grid_, robot_pose, current_goal_.value(), path);

    if (!success || path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to plan path from (%.2f, %.2f) to (%.2f, %.2f)",
                    robot_pose.x, robot_pose.y, current_goal_->x, current_goal_->y);
        return;
    }

    // print the planned path
    RCLCPP_INFO(this->get_logger(), "Planned path with %zu waypoints:", path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto& pose = path.poses[i];
        RCLCPP_INFO(this->get_logger(), "  Waypoint %zu: (%.2f, %.2f, %.2f)", i, pose.x, pose.y, pose.theta);
    }
    // start and goal
    RCLCPP_INFO(this->get_logger(), "Start: (%.2f, %.2f), Goal: (%.2f, %.2f)", robot_pose.x, robot_pose.y, current_goal_->x, current_goal_->y);  
    publishWaypoints(path);
    publishPath(path);
}

void ExplorationNode::publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path)
{
    waypoints_pub_->publish(path);
}

void ExplorationNode::publishPath(const mbot_interfaces::msg::Pose2DArray& path)
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

bool ExplorationNode::getRobotPose(geometry_msgs::msg::Pose2D &pose)
{
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);

        pose.x = tf.transform.translation.x;
        pose.y = tf.transform.translation.y;

        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pose.theta = yaw;

        return true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
