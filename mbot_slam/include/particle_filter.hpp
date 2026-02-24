#pragma once
/******************************************************
 * mbot_slam :: ParticleFilter
 *
 * Classic Monte-Carlo Localization (MCL) wrapper that
 * glues together:
 *   • ActionModel   – odometry-based prediction
 *   • SensorModel   – likelihood-field laser score
 *
 * The filter works entirely with standard ROS 2 message
 * types (Pose, LaserScan, OccupancyGrid, Odometry).
 ******************************************************/

#include "action_model.hpp"
#include "sensor_model.hpp"
#include "mapping.hpp"
#include "obstacle_distance_grid.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>

#include <vector>
#include <random>

namespace mbot_slam
{

// ParticleFilter class
class ParticleFilter
{
public:
    // Configuration constants
    static constexpr int NUM_PARTICLES = 800;
    static constexpr double RESAMPLE_THRESHOLD = 0.5;  // Resample when Neff < threshold * N_particles

    ParticleFilter();

    // Initialization
    // Place all particles exactly at 'pose'
    void initializeAtPose(const geometry_msgs::msg::Pose& pose);

    // Reset odometry baseline after global re-initialization
    void resetOdometry(const nav_msgs::msg::Odometry& odom)
    { action_model_.setOdomReference(odom); }

    // Main update cycle
    // Full MCL step: prediction + measurement + resample
    // Returns best-estimate pose in the map frame
    geometry_msgs::msg::Pose update(const nav_msgs::msg::Odometry&      odom,
                                    const sensor_msgs::msg::LaserScan&  scan,
                                    const ObstacleDistanceGrid&        dist_grid);

    // Getters
    const geometry_msgs::msg::Pose&       poseEstimate() const { return pose_estimate_; }
    const nav2_msgs::msg::ParticleCloud&  particleCloud() const { return particle_cloud_; }
    SensorModel&                          sensorModel()        { return sensor_model_; }

private:
    // Helpers for the three sub-steps
    nav2_msgs::msg::ParticleCloud systematicResample() const;
    nav2_msgs::msg::ParticleCloud propagate(const nav2_msgs::msg::ParticleCloud& prior);
    nav2_msgs::msg::ParticleCloud weightParticles(const nav2_msgs::msg::ParticleCloud& proposal,
                                                  const sensor_msgs::msg::LaserScan& scan,
                                                  const ObstacleDistanceGrid&  grid) const;

    geometry_msgs::msg::Pose computeBestEstimate(const nav2_msgs::msg::ParticleCloud& cloud) const;
    double computeEffectiveSampleSize(const nav2_msgs::msg::ParticleCloud& cloud) const;

    // Data members
    int                           num_particles_;   // number of particles
    nav2_msgs::msg::ParticleCloud particle_cloud_;  // particle cloud with header (frame_id, timestamp) and particles array
    geometry_msgs::msg::Pose      pose_estimate_;   // single best guess

    ActionModel               action_model_;    // motion model
    SensorModel               sensor_model_;    // laser likelihood

    mutable std::mt19937      random_gen;       // pseudo-random generator for resampling
};

} // namespace mbot_slam
