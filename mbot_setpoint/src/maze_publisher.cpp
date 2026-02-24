#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class MazePublisher : public rclcpp::Node {
public:
    MazePublisher() : Node("maze_publisher") {
        pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>("/waypoints", 10);

        // Use a timer to delay publishing, giving subscribers time to connect
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                mbot_interfaces::msg::Pose2DArray goal_array;
                goal_array.poses = {
                    pose(0.00, 0.00, 0.79),
                    pose(0.31, -0.01, -0.79),
                    pose(0.32, -0.02, -0.79),
                    pose(0.33, -0.03, -0.79),
                    pose(0.34, -0.04, -0.79),
                    pose(0.35, -0.05, -0.79),
                    pose(0.36, -0.06, -0.79),
                    pose(0.37, -0.07, -0.79),
                    pose(0.38, -0.08, -0.79),
                    pose(0.39, -0.09, -0.79),
                    pose(0.40, -0.10, -0.79),
                    pose(0.41, -0.11, -0.79),
                    pose(0.42, -0.12, -0.79),
                    pose(0.43, -0.13, -0.79),
                    pose(0.44, -0.14, -0.79),
                    pose(0.45, -0.15, -0.79),
                    pose(0.46, -0.16, -0.79),
                    pose(0.47, -0.17, -0.79),
                    pose(0.48, -0.18, -0.79),
                    pose(0.49, -0.19, -0.79),
                    pose(0.50, -0.20, -0.79),
                    pose(0.69, -0.46, -0.00),
                    pose(0.70, -0.46, -0.79),
                    pose(0.71, -0.47, -0.79),
                    pose(0.72, -0.48, -0.00),
                    pose(0.73, -0.48, -0.79),
                    pose(0.74, -0.49, -0.79),
                    pose(0.80, -0.51, 0.00)
                };
                // goal_array.poses = {
                //     pose(0.61, 0.0, 0.0),
                //     pose(0.61, 0.61, 0),
                //     pose(1.22, 0.61, -M_PI / 2),
                //     pose(1.22, -0.61, 0.0),
                //     pose(1.83, -0.61, 0),
                //     pose(1.83, 0.61, 0),
                //     pose(2.44, 0.61, -M_PI / 2),
                //     pose(2.44, 0, 0),
                //     pose(3.05, 0, 0)
                // };

                pub_->publish(goal_array);
                RCLCPP_INFO(this->get_logger(), "Published a maze trajectory with %zu poses.", goal_array.poses.size());

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
    rclcpp::spin(std::make_shared<MazePublisher>());
    rclcpp::shutdown();
    return 0;
}
