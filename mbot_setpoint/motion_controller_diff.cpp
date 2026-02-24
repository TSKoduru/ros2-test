#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include <rclcpp/qos.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DiffMotionController : public rclcpp::Node {
public:
    DiffMotionController() : Node("diff_motion_controller") {
        // Declare parameter for localization mode
        this->declare_parameter<bool>("use_localization", false);
        use_localization_ = this->get_parameter("use_localization").as_bool();

        if (use_localization_) {
            // use TF for localization
            RCLCPP_INFO(this->get_logger(), "Using LOCALIZATION mode (TF map->base_footprint)");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        } else {
            // use odometry
            RCLCPP_INFO(this->get_logger(), "Using ODOMETRY mode (/odom topic)");
            rclcpp::QoS odom_qos = rclcpp::SensorDataQoS();
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", odom_qos, std::bind(&DiffMotionController::odom_callback, this, _1));
        }

        // Subscribe to /waypoints topic
        goal_subscriber_ = this->create_subscription<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10, std::bind(&DiffMotionController::goal_callback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&DiffMotionController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "DiffMotionController initialized");
    }

private:
    bool use_localization_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<mbot_interfaces::msg::Pose2DArray>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;
    bool goal_received_ = false;
    bool use_unicycle_control_ = false; // Set to true to use unicycle control instead of RTR

    std::vector<geometry_msgs::msg::Pose2D> goal_queue_;

    double lin_error_sum_ = 0.0;
    double lin_error_last_ = 0.0;
    double ang_error_sum_ = 0.0;
    double ang_error_last_ = 0.0;
    double alpha_error_sum_ = 0.0;
    double alpha_error_last_ = 0.0;
    double beta_error_sum_ = 0.0;
    double beta_error_last_ = 0.0;

    // Shared thresholds for goal reaching
    const double dist_thresh_ = 0.1;
    const double angle_thresh_ = 0.2;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void goal_callback(const mbot_interfaces::msg::Pose2DArray::SharedPtr msg) {
        if (msg->poses.empty()) return;

        goal_queue_.clear();
        for (const auto &pose : msg->poses) {
            goal_queue_.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu goals", msg->poses.size());

        // Skip waypoints we're already at
        while (!goal_queue_.empty()) {
            const auto& first = goal_queue_.front();
            double dx = first.x - current_x_;
            double dy = first.y - current_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < dist_thresh_) {
                goal_queue_.erase(goal_queue_.begin());
            } else {
                break;
            }
        }

        // Load the first unreached goal
        if (!goal_queue_.empty()) {
            geometry_msgs::msg::Pose2D first = goal_queue_.front(); 
            goal_queue_.erase(goal_queue_.begin());
            goal_x_ = first.x;
            goal_y_ = first.y;
            goal_theta_ = first.theta;
            goal_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting goal: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
            RCLCPP_INFO(this->get_logger(), "Starting position: (%.2f, %.2f, %.2f)", current_x_, current_y_, current_theta_);
        } else {
            goal_received_ = false;
        }
    }
        
    void timer_callback() {
        if (!goal_received_) return;

        // Update pose based on mode
        if (use_localization_) {
            // Get robot pose from TF (map -> base_footprint)
            try {
                geometry_msgs::msg::TransformStamped transform =
                    tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0));

                current_x_ = transform.transform.translation.x;
                current_y_ = transform.transform.translation.y;

                // print current pose for debugging
                RCLCPP_INFO(this->get_logger(), "Current pose from TF: (%.2f, %.2f, %.2f)", current_x_, current_y_, current_theta_);

                // Extract yaw from quaternion
                double siny_cosp = 2 * (transform.transform.rotation.w * transform.transform.rotation.z +
                                        transform.transform.rotation.x * transform.transform.rotation.y);
                double cosy_cosp = 1 - 2 * (transform.transform.rotation.y * transform.transform.rotation.y +
                                            transform.transform.rotation.z * transform.transform.rotation.z);
                current_theta_ = std::atan2(siny_cosp, cosy_cosp);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Localization required! Cannot get map->base_footprint transform: %s", ex.what());
                return;
            }
        }
        // else: pose is already updated by odom_callback

        // Calculate errors
        /*
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_angle - current_theta_);

        // PID gains
        // TODO #4: Tune these gains for better performance
        
        double Kp_lin = 20.0, Ki_lin = 0.03, Kd_lin = 5;
        double Kp_ang = 30.0, Ki_ang = 0.07, Kd_ang = 0.5;
        double Kp_ang_lin = 5, Ki_ang_lin = 0.05, Kd_ang_lin = 0; // Angular corrections DURING linear phase
        

        double dt = 0.05;  // Match 50ms timer period
        double integral_limit = 100.0;  // Windup protection
        */

        geometry_msgs::msg::Twist cmd = compute_rtr_control();

        // Safety limits
        cmd.linear.x = std::clamp(cmd.linear.x, -0.2, 0.2);
        cmd.angular.z = std::clamp(cmd.angular.z, -0.25*M_PI, 0.25*M_PI);
        cmd_vel_publisher_->publish(cmd);
    }

    geometry_msgs::msg::Twist compute_unicycle_control(){
        
        // Calculate errors
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double target_angle_error = normalize_angle(target_angle - current_theta_);
        double final_angle_error = normalize_angle(goal_theta_ - current_theta_);

        // PID gains
        double Kp_lin = 20.0, Ki_lin = 0.03, Kd_lin = 5;
        double Kp_alpha = 30.0, Ki_alpha = 0.07, Kd_alpha = 0.5;
        double Kp_beta = -4, Ki_beta = 0, Kd_beta = -1.2; 

        double dt = 0.05;  // Match 50ms timer period
        double integral_limit = 100.0;  // Windup protection

        geometry_msgs::msg::Twist cmd;

        if (distance > dist_thresh_) {
            // Handle P terms
            cmd.angular.z += Kp_alpha * target_angle_error + Kp_beta * final_angle_error;
            cmd.linear.x += Kp_lin * distance;

            //integrate errors while preventing windup
            alpha_error_sum_ = std::clamp(alpha_error_sum_ + target_angle_error * dt, 
                                        -1 * integral_limit, integral_limit);
            beta_error_sum_ = std::clamp(beta_error_sum_ + final_angle_error * dt, 
                                        -1 * integral_limit, integral_limit);
            lin_error_sum_ = std::clamp(lin_error_sum_ + distance * dt, 
                                        -1 * integral_limit, integral_limit);
            
            // Handle I terms
            cmd.angular.z += Ki_alpha * alpha_error_sum_ + Ki_beta * beta_error_sum_;
            cmd.linear.x += Ki_lin * lin_error_sum_;

            //calculate approximate derivatives and handle D terms

            double approximate_derivative = (target_angle_error - alpha_error_last_) / dt;
            cmd.angular.z += Kd_alpha * approximate_derivative;
            approximate_derivative = (final_angle_error - beta_error_last_) / dt;
            cmd.angular.z += Kd_beta * approximate_derivative;

            approximate_derivative = (distance - lin_error_last_) / dt;
            cmd.linear.x += Kd_lin * approximate_derivative;

        } else {
            // Control for beta (final orientation error)
            cmd.angular.z += Kp_beta * final_angle_error;
            beta_error_sum_ = std::clamp(beta_error_sum_ + final_angle_error * dt, 
                                        -1 * integral_limit, integral_limit); // Prevent windup on the I term with clamping
            cmd.angular.z += Ki_beta * beta_error_sum_;
            double approximate_derivative = (final_angle_error - beta_error_last_) / dt;
            cmd.angular.z += Kd_beta * approximate_derivative;

            if (std::fabs(final_angle_error) < angle_thresh_) {
                // Goal complete
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                lin_error_sum_ = 0.0;
                lin_error_last_ = 0.0;
                alpha_error_sum_ = 0.0;
                beta_error_sum_ = 0.0;
                alpha_error_last_ = 0.0;
                beta_error_last_ = 0.0;

                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                load_next_goal();
            }
        }
        return cmd;
    }

    geometry_msgs::msg::Twist compute_rtr_control() {
        
        geometry_msgs::msg::Twist cmd;
        // Calculate errors
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_error = normalize_angle(target_angle - current_theta_);

        // PID gains
        // TODO #4: Tune these gains for better performance
        double Kp_lin = 20.0, Ki_lin = 0.03, Kd_lin = 5;
        double Kp_ang = 30.0, Ki_ang = 0.07, Kd_ang = 0.5;
        double Kp_ang_lin = 3, Ki_ang_lin = 0.05, Kd_ang_lin = 0; // Angular corrections DURING linear phase

        double dt = 0.05;  // Match 50ms timer period
        double integral_limit = 100.0;  // Windup protection

        // STATE 1: Turn toward target
        if (distance > dist_thresh_ && std::fabs(angle_error) > angle_thresh_) {

            cmd.angular.z = 0;

            // Handle P term
            cmd.angular.z += Kp_ang * angle_error;

            // Handle I term
            ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt, 
                                        -1 * integral_limit, integral_limit); // Prevent windup on the I term with clamping
            cmd.angular.z += Ki_ang * ang_error_sum_;

            // Handle D term
            double approximate_derivative = (angle_error - ang_error_last_) / dt;
            cmd.angular.z += Kd_ang * approximate_derivative;

            // Update terms
            ang_error_last_ = angle_error;   
        }
        // STATE 2: Drive forward with heading correction
        else if (distance > dist_thresh_) {

            double angle_scale = std::clamp(1.0 - std::fabs(angle_error) / M_PI, 0.5, 1.0);

            // Same setup as above, just replacing angular terms with linear
            // Note that 'distance' is our error; It's the distance from the target
            // We also still do P control on angle error to do error correction

            geometry_msgs::msg::Twist msg;
            cmd.linear.x = 0;
            cmd.angular.z = 0;

            // P
            cmd.linear.x += Kp_lin * distance;
            cmd.angular.z += Kp_ang_lin * angle_error;

            // I
            lin_error_sum_ = std::clamp(lin_error_sum_ + distance * dt, 
                                        -1 * integral_limit, integral_limit); // Prevent windup on the I term with clamping
            cmd.linear.x += Ki_lin * lin_error_sum_;
            ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt, 
                -1 * integral_limit, integral_limit); // Prevent windup on the I term with clamping
            cmd.angular.z += Ki_ang_lin * ang_error_sum_;

            // D
            double approximate_derivative = (distance - lin_error_last_) / dt;
            cmd.linear.x += Kd_lin * approximate_derivative;
            approximate_derivative = (angle_error - ang_error_last_) / dt;
            cmd.angular.z += Kd_ang_lin * approximate_derivative;

            // Scale linear velocity based on heading error
            cmd.linear.x *= angle_scale;

            // Bookkeeping
            lin_error_last_ = distance;
            ang_error_last_ = angle_error;

        }
        // STATE 3: Rotate to final orientation
        else {

            angle_error = normalize_angle(goal_theta_ - current_theta_);

            if (std::fabs(angle_error) > angle_thresh_) {

                // Copy and paste from section 1, literally no change

                geometry_msgs::msg::Twist msg;
                cmd.angular.z = 0;
    
                // Handle P term
                cmd.angular.z += Kp_ang * angle_error;
    
                // Handle I term
                ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt, 
                                            -1 * integral_limit, integral_limit); // Prevent windup on the I term with clamping
                cmd.angular.z += Ki_ang * ang_error_sum_;
    
                // Handle D term
                double approximate_derivative = (angle_error - ang_error_last_) / dt;
                cmd.angular.z += Kd_ang * approximate_derivative;
    
                // Update terms
                ang_error_last_ = angle_error;


            } else {
                // Goal complete
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                lin_error_sum_ = 0.0;
                lin_error_last_ = 0.0;
                ang_error_sum_ = 0.0;
                ang_error_last_ = 0.0;

                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                load_next_goal();
            }
        }

        return cmd;
    }    
        
    void load_next_goal() {
        if (!goal_queue_.empty()) {
            geometry_msgs::msg::Pose2D next = goal_queue_.front();
            goal_queue_.erase(goal_queue_.begin());
            goal_x_ = next.x;
            goal_y_ = next.y;
            goal_theta_ = next.theta;
            RCLCPP_INFO(this->get_logger(), "Next goal: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_theta_);
            RCLCPP_INFO(this->get_logger(), "Starting position: (%.2f, %.2f, %.2f)", current_x_, current_y_, current_theta_);
        } else {
            goal_received_ = false;
        }
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffMotionController>());
    rclcpp::shutdown();
    return 0;
}
