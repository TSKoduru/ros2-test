#include "particle_filter.hpp"
#include "localization_utils.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>

#include <chrono>
#include <iostream>

using namespace mbot_localization;


// Constructor
ParticleFilter::ParticleFilter()
: num_particles_(NUM_PARTICLES),
  action_model_(),
  sensor_model_()
{
    particle_cloud_.particles.resize(num_particles_);

    std::random_device seed_source;
    random_gen = std::mt19937(seed_source());
}

void ParticleFilter::initializeAtPose(const geometry_msgs::msg::Pose& pose)
{
    const double w = 1.0 / static_cast<double>(num_particles_);
    for (auto& p : particle_cloud_.particles) {
        p.pose   = pose;
        p.weight = w;
    }
    pose_estimate_ = pose;
}

// Main update cycle
geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const ObstacleDistanceGrid&        dist_grid)
{
    auto start = std::chrono::steady_clock::now();
    bool moved = action_model_.processOdometry(odom);
    nav2_msgs::msg::ParticleCloud resampled_particles = systematicResample();
    nav2_msgs::msg::ParticleCloud prior = moved ? propagate(resampled_particles) : resampled_particles;
    particle_cloud_ = weightParticles(prior, scan, dist_grid);
    pose_estimate_ = computeBestEstimate(particle_cloud_);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "[ParticleFilter] Update time: "
              << elapsed.count() << " ms, using " << num_particles_ << " particles." << std::endl;
              
    return pose_estimate_;
}

// Systematic resampling: keep high-weight particles, discard low-weight ones
nav2_msgs::msg::ParticleCloud ParticleFilter::systematicResample() const
{

    // Build cumulative distribution of weights
    // Each index represents sum of probabilities up to and including the current point
    std::vector<double> cdf(num_particles_);
    cdf[0] = particle_cloud_.particles[0].weight;
    for (int i = 1; i < num_particles_; ++i) {
        cdf[i] = cdf[i - 1] + particle_cloud_.particles[i].weight;
    }


    // Step 2: Handle edge case (if all particles have zero weight?)
    double total_weight = cdf.back();
    if (total_weight <= 1e-9) {
        return particle_cloud_;
    }


    // Step 3: Normalize to probability [0, 1]
    for (double w : cdf) {
        w /= total_weight;
    }

    // Step 4: Draw N equally-spaced samples and copy particles. 
    //You may find  std::uniform_real_distribution<double> unif() helpful.
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);
    //(fill in code below)

    std::default_random_engine rng;
    std::uniform_real_distribution<double> unif(0.0, 1.0 / num_particles_);
    double r = unif(rng);
    int i = 0;

    for (int m = 1; m < num_particles_; m++) {
        double U = r + (m-1) * (1.0 / num_particles_);
        while (U > cdf[i] && i < num_particles_ - 1) {
            i++;
        }
        resampled.particles.push_back(particle_cloud_.particles[i]);
    }

    return resampled;
}

// Propagate each particle through ActionModel
nav2_msgs::msg::ParticleCloud ParticleFilter::propagate(const nav2_msgs::msg::ParticleCloud& resampled_particles)
{
    nav2_msgs::msg::ParticleCloud prior;
    prior.particles.reserve(num_particles_);

    for (const auto& particle : resampled_particles.particles) {
        nav2_msgs::msg::Particle q = particle;
        q.pose = action_model_.propagateParticle(particle.pose);
        prior.particles.push_back(std::move(q));
    }
    return prior;
}

// Weight each particle with SensorModel
nav2_msgs::msg::ParticleCloud ParticleFilter::weightParticles(const nav2_msgs::msg::ParticleCloud& prior,
                                                                const sensor_msgs::msg::LaserScan& scan,
                                                                const ObstacleDistanceGrid& dist_grid) const
{
    nav2_msgs::msg::ParticleCloud posterior = prior;
    double sum_w = 0.0;

    for (auto& p : posterior.particles) {
        p.weight = sensor_model_.likelihood(p.pose, scan, dist_grid);
        sum_w   += p.weight;
    }

    // Avoid division by zero
    if (sum_w <= 0.0) {
        const double w = 1.0 / static_cast<double>(num_particles_);
        for (auto& p : posterior.particles) p.weight = w;
        return posterior;
    }

    // Normalize
    for (auto& p : posterior.particles) p.weight /= sum_w;
    return posterior;
}

// Compute pose estimate (weighted mean on SE2)
geometry_msgs::msg::Pose ParticleFilter::computeBestEstimate(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;

    for(const nav2_msgs::msg::Particle p : cloud.particles) {

        // Update position estimate using particle weight
        x += p.pose.position.x * p.weight;
        y += p.pose.position.y * p.weight;

        // Update cos and sin sums(Need to manually make quaternion; Otherwise we get linker errors)
        tf2::Quaternion q(
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w
        );
    
        // Convert to RPY
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        double theta = yaw;
        cos_sum += std::cos(theta) * p.weight;
        sin_sum += std::sin(theta) * p.weight;
    }


    geometry_msgs::msg::Pose est;
    est.position.x = x;
    est.position.y = y;
    est.position.z = 0.0;
    setOrientationFromYaw(est, std::atan2(sin_sum, cos_sum));
    return est;
};