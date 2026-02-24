#include "sensor_model.hpp"
#include "obstacle_distance_grid.hpp"
#include "slam_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <limits>

namespace mbot_slam
{

SensorModel::SensorModel()
    : sigma_hit_{SIGMA_HIT},
      ray_stride_{RAY_STRIDE}
{}

bool SensorModel::getDistanceAt(double world_x, double world_y, const ObstacleDistanceGrid& grid, float& distance) const
{
    const auto& grid_origin = grid.getOrigin();
    const float grid_res = grid.getResolution();
    const int grid_w = grid.getWidth();
    const int grid_h = grid.getHeight();

    const double gx = (world_x - grid_origin.position.x) / grid_res;
    const double gy = (world_y - grid_origin.position.y) / grid_res;
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if (x0 < 0 || y0 < 0 || x1 >= grid_w || y1 >= grid_h) return false;

   // TODO #1: Read and understand the following code. 
   // Hint: Bilinear interpolation for distance?
    const float fx = static_cast<float>(gx - x0);
    const float fy = static_cast<float>(gy - y0);
    const float d00 = grid.getDistance(x0, y0);
    const float d10 = grid.getDistance(x1, y0);
    const float d01 = grid.getDistance(x0, y1);
    const float d11 = grid.getDistance(x1, y1);

    const float d0 = d00 * (1 - fx) + d10 * fx;
    const float d1 = d01 * (1 - fx) + d11 * fx;
    distance = d0 * (1 - fy) + d1 * fy;
    return true;
}

double SensorModel::likelihood(const geometry_msgs::msg::Pose&    pose,
                               const sensor_msgs::msg::LaserScan& scan,
                               const ObstacleDistanceGrid&         grid) const
{
    const double robot_x = pose.position.x;
    const double robot_y = pose.position.y;
    const double robot_yaw = yawFromQuaternion(pose.orientation);
    const double inv_two_sigma2 = 1.0 / (2.0 * sigma_hit_ * sigma_hit_);

    // TODO #2: Compute uniform probability distribution over range
        // Hint: p_uniform = 1.0 / (scan.range_max - scan.range_min)
    const double p_uniform = 1.0 / (scan.range_max - scan.range_min);



    double log_sum = 0.0;
    int used = 0;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if ((static_cast<int>(i) % std::max(1, ray_stride_)) != 0) continue;

        const double range = static_cast<double>(scan.ranges[i]);
        if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) continue;

        const double scan_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        // Lidar is mounted backward (180 degree offset from base_footprint)
        double world_angle = scan_angle + robot_yaw + M_PI;
        world_angle = wrapToPi(world_angle);  // Normalize to (-π, π]

        const double endpoint_x = robot_x + range * std::cos(world_angle);
        const double endpoint_y = robot_y + range * std::sin(world_angle);

        float distance_to_obstacle;
        if (!getDistanceAt(endpoint_x, endpoint_y, grid, distance_to_obstacle)) continue;

        // TODO #3: Compute sensor likelihood for this ray
        // Hint: 
        //       Compute Gaussian p_hit = exp(-(d²) / (2σ²)) where σ = sigma_hit_
        //       Then mixture model p = Z_HIT * p_hit + Z_RAND * p_uniform
        const double p_hit = std::exp(-(distance_to_obstacle) * (distance_to_obstacle) * inv_two_sigma2);
        const double p = Z_HIT * p_hit + Z_RAND * p_uniform;

        log_sum += std::log(p);
        ++used;
    }

    if (used == 0) return 1e-9;

    return std::max(std::exp(log_sum / used), 1e-9);
}

} // namespace mbot_localization
