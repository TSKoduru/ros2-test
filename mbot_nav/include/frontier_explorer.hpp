#pragma once

#include <vector>
#include <optional>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "obstacle_distance_grid.hpp"

namespace mbot_nav
{

class FrontierExplorer
{
public:
    FrontierExplorer();

    std::optional<geometry_msgs::msg::Pose2D> selectGoal(
        const geometry_msgs::msg::Pose2D &robot_pose,
        const ObstacleDistanceGrid &dist_grid);

    // Minimum distance (meters) to consider a frontier
    double min_frontier_distance = 0.2;

    // Clearance (meters) required around goal positions from obstacles
    double goal_clearance = 0.15;

    // Distance (meters) for clustering nearby frontiers
    double frontier_cluster_distance = 0.15;

private:
    // Detects frontier cells and returns their world coordinates
    std::vector<geometry_msgs::msg::Point> detectFrontiers(
        int robot_gx, int robot_gy,
        const ObstacleDistanceGrid &dist_grid);

    // Clusters frontier points by distance threshold, returns centroids
    std::vector<geometry_msgs::msg::Point> clusterFrontiers(
        const std::vector<geometry_msgs::msg::Point> &frontiers);

    bool isFrontierCell(int x, int y, const ObstacleDistanceGrid &dist_grid);

    geometry_msgs::msg::Point gridToWorld(
        int x, int y,
        const ObstacleDistanceGrid &dist_grid);
};

} // namespace mbot_nav
