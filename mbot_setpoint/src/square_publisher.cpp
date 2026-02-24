#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class SquarePublisher : public rclcpp::Node {
public:
    SquarePublisher() : Node("square_publisher") {
        pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>("/waypoints", 10);

        // Use a timer to delay publishing, giving subscribers time to connect
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                mbot_interfaces::msg::Pose2DArray goal_array;
                goal_array.poses = {
                    pose(1.0, 0.0, 0.0),
                    pose(1.0, 1.0, M_PI_2),
                    pose(0.0, 1.0, M_PI),
                    pose(0.0, 0.0, 0.0)
                };

                pub_->publish(goal_array);
                RCLCPP_INFO(this->get_logger(), "Published square trajectory with %zu poses.", goal_array.poses.size());

                // Cancel timer after publishing once
                timer_->cancel();
            });
    }

private:
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static geometry_msgs::msg::Pose2D pose(double x, double y, double theta) {
        geometry_msgs::msg::Pose2D p;
        p.x = x;
        p.y = y;
        p.theta = theta;
        return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}
