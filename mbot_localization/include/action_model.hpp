#pragma once
/***************************************
 * mbot_localization :: ActionModel
 *
 * Odometry-based sample-motion model
 * (rot1, trans, rot2) implementation.
 *
 * Depends **only** on standard ROS 2
 * messages and <random>/<cmath>.
 ***************************************/

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <random>
#include <cmath>

namespace mbot_localization
{

class ActionModel
{
public:
    ActionModel();

    // Set the odometry baseline reference.
    void setOdomReference(const nav_msgs::msg::Odometry& odom);

    // Process a new odometry measurement.
    bool processOdometry(const nav_msgs::msg::Odometry& odom);

    // Propagate a particle by applying noisy motion from the motion model.
    geometry_msgs::msg::Pose propagateParticle(const geometry_msgs::msg::Pose& pose);

private:
    nav_msgs::msg::Odometry prev_odom_;

    // Motion-model parameters (alpha values from Probabilistic Robotics)
    double k1_{0.1};            // rotational noise coefficient
    double k2_{0.05};           // translational noise coefficient
    double k3_{0.05};           // rotational noise coefficient when translating
    double k4_{0};              // translational noise coefficient when rotating
    double min_trans_{0.0025};  // [m] ignore motion below this
    double min_rot_ {0.02};     // [rad] ignore rotation below this

    // (rot1, trans, rot2) motion decomposition
    double rot1_{0.0}, trans_{0.0}, rot2_{0.0};
    /* Corresponding 1-σ values */
    double rot1_std_{0.0}, trans_std_{0.0}, rot2_std_{0.0};

    /* Random-number generator */
    std::mt19937 random_gen;
};

}
