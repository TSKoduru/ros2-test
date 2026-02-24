#include <cmath>
#include <rclcpp/qos.hpp>
#include "rclcpp/rclcpp.hpp"
#include "mbot_interfaces/msg/encoders.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using std::placeholders::_1;

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             48.0  // Default encoder resolution.

// MBot Classic Parameters
#define DIFF_WHEEL_DIAMETER          0.0837
#define DIFF_WHEEL_RADIUS            0.04183
#define DIFF_BASE_RADIUS             0.07786
#define MOT_R                        1   // Right motor slot
#define MOT_L                        0   // Left motor slot

// Encoder polarity
// TODO #1: Adjust these values based on your specific robot configuration
#define ENCODER_POLARITY_L       1
#define ENCODER_POLARITY_R       1

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("odometry_node") {
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        encoder_subscriber_ = this->create_subscription<mbot_interfaces::msg::Encoders>(
            "/encoders", qos_profile, std::bind(&OdometryNode::encoder_callback, this, _1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        RCLCPP_INFO(this->get_logger(), "OdometryNode initialized.");
    }

private:
    rclcpp::Subscription<mbot_interfaces::msg::Encoders>::SharedPtr encoder_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;

    // Persistent odometry state
    float odom_x_ = 0.0f;
    float odom_y_ = 0.0f;
    float odom_theta_ = 0.0f;

    // Timer for tracking callback interval
    rclcpp::Time last_callback_time_ = rclcpp::Time(0);

    void encoder_callback(const mbot_interfaces::msg::Encoders msg) {
        // Local variables for this callback
        float wheel_vel_l, wheel_vel_r;
        float vx, vy, wz;

        // Calculate actual time interval between callbacks
        rclcpp::Time current_time = this->get_clock()->now();
        float dt_seconds = 0.016;  // Default fallback

        if (last_callback_time_.nanoseconds() > 0) {
            dt_seconds = (current_time - last_callback_time_).seconds();
        }
        last_callback_time_ = current_time;

        // Calculate wheel velocities using encoder delta_time (timing from firmware)
        mbot_calculate_wheel_speed(
            msg.delta_time,
            msg.delta_ticks[MOT_L],
            msg.delta_ticks[MOT_R],
            &wheel_vel_l,
            &wheel_vel_r
        );

        mbot_calculate_diff_body_vel(
            wheel_vel_l,
            wheel_vel_r,
            &vx,
            &vy,
            &wz
        );

        mbot_calculate_odometry(
            vx,
            vy,
            wz,
            dt_seconds,
            &odom_x_,
            &odom_y_,
            &odom_theta_
        );

        nav_msgs::msg::Odometry odom_msg;
        publish_odometry_data(msg, &odom_msg, vx, vy, wz);

        publish_tf_data(msg, &odom_msg);
    }

    void mbot_calculate_wheel_speed(float dt, int encoder_tick_L, int encoder_tick_R, float* wheel_vel_L, float* wheel_vel_R){
        float conversion = 1E6f * (1.0f / GEAR_RATIO) * (1.0f / ENCODER_RES) * 2.0f * M_PI;

        if (dt <= 0) { /* Avoid division by zero or invalid dt */ 
            *wheel_vel_L = 0.0f;
            *wheel_vel_R = 0.0f;
            return; 
        }

        *wheel_vel_L = ENCODER_POLARITY_L * (conversion / dt) * encoder_tick_L;
        *wheel_vel_R = ENCODER_POLARITY_R * (conversion / dt) * encoder_tick_R;
    }

    static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz) {
        // Calculate forward velocity and angular velocity
        *vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
        *vy = 0.0;
        *wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
    }

    void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
        *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
        *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
        *theta += wz * dt;

        // Normalize theta to [-pi, pi]
        while (*theta > M_PI) *theta -= 2.0 * M_PI;
        while (*theta <= -M_PI) *theta += 2.0 * M_PI;
    }

    void publish_odometry_data(const mbot_interfaces::msg::Encoders msg, nav_msgs::msg::Odometry *odom_msg, float vx, float vy, float wz){
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_footprint";
        odom_msg->header.stamp.sec = msg.stamp.sec;
        odom_msg->header.stamp.nanosec = msg.stamp.nanosec;

        odom_msg->pose.pose.position.x = odom_x_;
        odom_msg->pose.pose.position.y = odom_y_;
        odom_msg->pose.pose.position.z = 0.0;

        odom_msg->pose.pose.orientation = calculate_quaternion(odom_theta_);

        odom_msg->twist.twist.linear.x = vx;
        odom_msg->twist.twist.linear.y = vy;
        odom_msg->twist.twist.angular.z = wz;

        odom_publisher_->publish(*odom_msg);
    }

    void publish_tf_data(const mbot_interfaces::msg::Encoders msg, nav_msgs::msg::Odometry *odom_msg){
        //tf message population
        tf2_msgs::msg::TFMessage tf_msg;
        geometry_msgs::msg::TransformStamped transform;
        tf_msg.transforms = {transform};

        tf_msg.transforms[0].header.stamp.sec = msg.stamp.sec;
        tf_msg.transforms[0].header.stamp.nanosec = msg.stamp.nanosec;
        tf_msg.transforms[0].header.frame_id = odom_msg->header.frame_id; // "odom"
        tf_msg.transforms[0].child_frame_id = odom_msg->child_frame_id;   // "base_footprint"
        tf_msg.transforms[0].transform.translation.x = odom_msg->pose.pose.position.x;
        tf_msg.transforms[0].transform.translation.y = odom_msg->pose.pose.position.y;
        tf_msg.transforms[0].transform.translation.z = odom_msg->pose.pose.position.z;
        tf_msg.transforms[0].transform.rotation = odom_msg->pose.pose.orientation;

        tf_publisher_->publish(tf_msg);
    }

    geometry_msgs::msg::Quaternion calculate_quaternion(float theta){
        geometry_msgs::msg::Quaternion out;
        float cy = cos(theta * 0.5);
        float sy = sin(theta * 0.5);
        out.w = cy;
        out.x = 0.0;
        out.y = 0.0;
        out.z = sy;
        return out;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}