#include "action_model.hpp"
#include "slam_utils.hpp"

#include <random>
#include <utility>
#include <cmath>

using namespace mbot_slam;

ActionModel::ActionModel()
{
    std::random_device rd;
    random_gen = std::mt19937(rd());
}

void ActionModel::setOdomReference(const nav_msgs::msg::Odometry& odom)
{
    prev_odom_   = odom;
}

bool ActionModel::processOdometry(const nav_msgs::msg::Odometry& odom)
{
    // Delta translation in the odom frame
    const double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;

    // Extract planar headings
    const double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
    const double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);


    // TODO #1: Compute the rotation-translation-rotation motion components
    //          Replace following 0s with the correct expressions
    double delta_trans = std::sqrt(dx * dx + dy * dy);
    double delta_theta = wrapToPi(theta_curr - theta_prev);
    
    // Safety check for pure rotation to avoid STD blowing up
    if (delta_trans < 0.001) {
        rot1_ = 0.0;
        trans_ = delta_trans;
        rot2_ = delta_theta;
    } else {
        rot1_ = angleDiff(std::atan2(dy, dx), theta_prev);
        trans_ = delta_trans;
        rot2_ = angleDiff(delta_theta, rot1_);
    }

    const bool moved =
        (delta_trans >= min_trans_) ||
        (std::abs(delta_theta) >= min_rot_);

    // TODO (From Teja): Add additional parameters to represent translational inaccuracy during rotation and vice versa?
    // See lesson 9 for more info. May not be critical
    
    if (moved) {
        // TODO #2: calcuate standard deviations (alpha-model)
        // Hint: use k1_ and k2_ member variables
        rot1_std_  = sqrt(k1_ * std::abs(rot1_) + k3_ * std::abs(trans_));
        trans_std_ = sqrt(k2_ * std::abs(trans_) + k4_ * std::abs(rot2_));
        rot2_std_  = sqrt(k1_ * std::abs(rot2_) + k3_ * std::abs(trans_));
    }

    prev_odom_ = odom;

    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    /// TODO #3: Sample noisy motion components
    // Hint: use std::normal_distribution
    // Hint: random_gen is available as a member variable
    // e.g., std::normal_distribution<double> dist(mean, std_dev);
    //       double sample = dist(random_gen);
    //       where mean should be the odometry
    // Replace 0s with the correct expressions
    double sampled_rot1  = std::normal_distribution<double>(rot1_, rot1_std_)(random_gen);
    double sampled_trans = std::normal_distribution<double>(trans_, trans_std_)(random_gen);
    double sampled_rot2  = std::normal_distribution<double>(rot2_, rot2_std_)(random_gen);

    // TODO #4: Update particle position and orientation based on sampled motion
    // Replace 0s with the correct expressions
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = pose.position.x + sampled_trans * std::cos(yawFromQuaternion(pose.orientation) + sampled_rot1);
    new_pose.position.y = pose.position.y + sampled_trans * std::sin(yawFromQuaternion(pose.orientation) + sampled_rot1);
    setOrientationFromYaw(new_pose, wrapToPi(yawFromQuaternion(pose.orientation) + sampled_rot1 + sampled_rot2));

    return new_pose;

}
