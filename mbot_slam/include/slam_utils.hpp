#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace mbot_slam {

// Wrap angle to (-π, π]
inline double wrapToPi(double angle)
{
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    while (angle >   M_PI) angle -= 2.0 * M_PI;
    return angle;
}

// Signed shortest difference a − b, wrapped to (-π, π]
inline double angleDiff(double a, double b)
{
    return wrapToPi(a - b);
}

// Extract planar yaw from quaternion
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);
    return yaw;
}

// Set orientation from yaw angle
inline void setOrientationFromYaw(geometry_msgs::msg::Pose& p, double yaw)
{
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = std::sin(yaw * 0.5);
    p.orientation.w = std::cos(yaw * 0.5);
}

} // namespace mbot_slam
