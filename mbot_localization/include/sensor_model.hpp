#pragma once
/********************************************
 * mbot_localization :: SensorModel
 *
 * Likelihood-field laser sensor model
 * using an ObstacleDistanceGrid.
 *
 *  p(z_i | x) ≈ z_hit * exp( -d_i² / (2σ_hit²) ) + z_rand * U
 ********************************************/

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "obstacle_distance_grid.hpp"

#include <algorithm>
#include <cmath>

namespace mbot_localization
{

class SensorModel
{
public:
    static constexpr double SIGMA_HIT = 0.1;
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

} // namespace mbot_localization
