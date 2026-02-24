#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "mapping.hpp"
#include "obstacle_distance_grid.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mbot_slam
{

class SensorModel
{
public:
    static constexpr double SIGMA_HIT = 0.005;
    static constexpr int    RAY_STRIDE = 2;
    static constexpr double Z_HIT = 0.95;      // Weight for accurate measurements
    static constexpr double Z_RAND = 0.05;     // Weight for noise/outliers
    explicit SensorModel();

    double likelihood(const geometry_msgs::msg::Pose&   pose,
                      const sensor_msgs::msg::LaserScan& scan,
                      const ObstacleDistanceGrid&        dist_grid) const;

private:
    bool getDistanceAt(double world_x, double world_y, const ObstacleDistanceGrid& grid, float& distance) const;

    double sigma_hit_;
    int    ray_stride_;
};

} // namespace mbot_slam
