#include "moving_laser_scan.hpp"

#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace {

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           float t)
{
    geometry_msgs::msg::Point out;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.z = a.z + t * (b.z - a.z);
    return out;
}

// Wrap angle to (-pi, pi]
inline double wrapToPi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// Extract planar yaw from quaternion
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);
    return yaw;
}

} // namespace

MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const Pose& start_pose,
                                 const Pose& end_pose)
{
    interpolateRay(scan, start_pose, end_pose);
}

void MovingLaserScan::interpolateRay(const sensor_msgs::msg::LaserScan& scan,
                                     const Pose& start_pose,
                                     const Pose& end_pose)
{
    const size_t num_rays = scan.ranges.size();
    rays_.clear();
    rays_.reserve(num_rays);

    // Extract yaw from quaternions (world-frame base yaw at scan start/end)
    const double yaw_start = yawFromQuaternion(start_pose.orientation);
    const double yaw_end = yawFromQuaternion(end_pose.orientation);

    // Shortest angular delta
    double delta_yaw = wrapToPi(yaw_end - yaw_start);

    // Avoid divide-by-zero when num_rays==0 or num_rays==1
    const float denom = (num_rays > 1) ? static_cast<float>(num_rays - 1) : 1.0f;

    // Iterate over all beams
    for (size_t i = 0; i < num_rays; ++i)
    {
        const float range = scan.ranges[i];

        // Drop garbage / under-range / out-of-range
        if (std::isnan(range) || range < scan.range_min || range >= scan.range_max) {
            continue;
        }

        // Beam angle in the laser frame
        const float scan_angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;

        const float t = static_cast<float>(i) / denom;
        const double yaw = yaw_start + t * delta_yaw;  // shortest-path yaw interpolation

        InterpolatedRay ray;

        // TODO #1: Compute ray.origin, ray.theta, and ray.range (after interpolation). Use linearInterpolatePoint() function.
        ray.origin = linearInterpolatePoint(start_pose.position, end_pose.position, t);
        ray.theta = wrapToPi(scan_angle + yaw);
        ray.range = range;
        rays_.push_back(ray);
    }
}
