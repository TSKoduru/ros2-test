#ifndef MOVING_LASER_SCAN_HPP
#define MOVING_LASER_SCAN_HPP

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct InterpolatedRay
{
    geometry_msgs::msg::Point origin;
    float theta;  // Angle relative to the origin
    float range;
};

class MovingLaserScan
{
public:
    using Pose = geometry_msgs::msg::Pose;

    MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                    const Pose& start_pose,
                    const Pose& end_pose);

    std::vector<InterpolatedRay>::const_iterator begin() const { return rays_.begin(); }
    std::vector<InterpolatedRay>::const_iterator end() const { return rays_.end(); }

private:
    std::vector<InterpolatedRay> rays_;

    void interpolateRay(const sensor_msgs::msg::LaserScan& scan,
                        const Pose& start_pose,
                        const Pose& end_pose);
};

#endif // MOVING_LASER_SCAN_HPP
