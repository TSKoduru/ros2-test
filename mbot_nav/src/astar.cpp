#include "astar.hpp"
#include <cmath>
#include <limits>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <iomanip>

using namespace mbot_nav;

AStarPlanner::AStarPlanner() {}

float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

bool AStarPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                            const geometry_msgs::msg::Pose2D& start,
                            const geometry_msgs::msg::Pose2D& goal,
                            mbot_interfaces::msg::Pose2DArray& path) {
    path.poses.clear();
    mbot_interfaces::msg::Pose2DArray raw_path;
    int width = dist_grid.getWidth();

    int start_x = static_cast<int>((start.x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
    int start_y = static_cast<int>((start.y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());
    int goal_x = static_cast<int>((goal.x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
    int goal_y = static_cast<int>((goal.y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open;
    std::unordered_map<int, float> cost_so_far;
    std::unordered_map<int, int> came_from;

    int start_idx = toIndex(start_x, start_y, width);
    int goal_idx = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[start_idx] = 0.0f;
    came_from[start_idx] = start_idx;

    const int dx[8] = {1, -1, 0, 0,  1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1,  1, -1, 1, -1};

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            auto path_indices = reconstructPath(came_from, goal_idx, width);
            for (size_t i = 0; i < path_indices.size(); ++i) {
                const auto& [x, y] = path_indices[i];
                geometry_msgs::msg::Pose2D pose;
                pose.x = x * dist_grid.getResolution() + dist_grid.getOrigin().position.x;
                pose.y = y * dist_grid.getResolution() + dist_grid.getOrigin().position.y;

                // TODO #3: add pose.theta here - how would you decide the pose's heading?
                // Hint: loop through path_indices and compute what's nx, ny, and thus compute pose.theta
                // Hint: what is the heading at goal pose?
                if (i < path_indices.size() - 1) {
                    const auto& [nx, ny] = path_indices[i + 1];
                    pose.theta = std::atan2(ny - y, nx - x);
                } else {
                    pose.theta = 0.0f;  // default value for the goal
                }

                raw_path.poses.push_back(pose);
            }
            path = smoothPath(dist_grid, raw_path);   // prune / smooth
            return true;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!dist_grid.isCellInGrid(nx, ny)) continue;
            int neighbor_idx = toIndex(nx, ny, width);
            float obs_dist = dist_grid.getDistance(nx, ny);
            int occupancy = dist_grid.getOccupancy(nx, ny);

            // TODO #1: Compute the cost penalty and moving cost (heuristics)
            float penalty = 0.0f;

            // Penalize known obstacles heavily
            if (occupancy > 50 || obs_dist < 0.1f) {
                penalty = 1000.0f;
            }
            // Penalize unknown regions mildly
            else if (occupancy == -1) {
                penalty = 10.0f;
            }
            // Otherwise, cost is based on inverse of obstacle distance
            else {
                penalty = 5.0f / (obs_dist + 0.1f);  // add small value to avoid division by zero
            }
            // compute move_cost and new_cost. 
            // You will also need to Add new AStarNode to open queue, which contains (nx,ny) and their g-cost and h-cost
            // You may find heuristic() helpful. 
            // Incorporate the penalty you computed above in g_cost
            float move_cost = (i < 4) ? 1.0f : std::sqrt(2.0f);  // straight vs diagonal
            float new_cost = cost_so_far[curr_idx] + move_cost + penalty;
            if (cost_so_far.find(neighbor_idx) == cost_so_far.end() || new_cost < cost_so_far[neighbor_idx]) {
                cost_so_far[neighbor_idx] = new_cost;
                float priority = new_cost + heuristic(nx, ny, goal_x, goal_y);
                open.push({nx, ny, new_cost, priority});
                came_from[neighbor_idx] = curr_idx;
            }
        }
    }

    return false;  // Path not found
}

std::vector<std::pair<int, int>> AStarPlanner::reconstructPath(
    const std::unordered_map<int, int>& came_from, int goal_idx, int width) {

    std::vector<std::pair<int, int>> path;
    // TODO #2: construct the raw path here
    // Hint: You may find std::reverse() helpful
    int cur_idx = goal_idx;
    while (came_from.at(cur_idx) != cur_idx) {
        path.push_back({cur_idx % width, cur_idx / width});
        cur_idx = came_from.at(cur_idx);
    }
    path.push_back({cur_idx % width, cur_idx / width});  // add start
    std::reverse(path.begin(), path.end());

    return path;
}

bool AStarPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid, float x0, float y0, float x1, float y1)
{
    const float step = dist_grid.getResolution() * 0.5f;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float dist = std::hypot(dx, dy);
    const float angle = std::atan2(dy, dx);

    for (float d = 0.0f; d <= dist; d += step) {
        float x = x0 + std::cos(angle) * d;
        float y = y0 + std::sin(angle) * d;
        int ix = static_cast<int>((x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
        int iy = static_cast<int>((y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());
        if (!dist_grid.isCellInGrid(ix, iy) || dist_grid.getDistance(ix, iy) < 0.2f) {
            RCLCPP_INFO(rclcpp::get_logger("AStarPlanner"), "%b, %b", !dist_grid.isCellInGrid(ix, iy), dist_grid.getDistance(ix, iy) < 0.2f);
            return false;
        }
    }
    return true;
}

mbot_interfaces::msg::Pose2DArray AStarPlanner::smoothPath(
    const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path)
{

    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) return pruned;

    pruned.poses.push_back(raw_path.poses.front());

    // TODO #4: Smooth the path here
    // Hint: say if you have 100 waypoints and they are in tiny step, 
    // maybe we can downsize them by removing midpoints
    // utilize the function isLineFree()
    size_t last_idx = 0;
    for (size_t i = 1; i < raw_path.poses.size(); ++i) {
        const auto& last_pose = pruned.poses.back();
        const auto& current_pose = raw_path.poses[i];

        if (!isLineFree(dist_grid, last_pose.x, last_pose.y, current_pose.x, current_pose.y)) {
            pruned.poses.push_back(raw_path.poses[i - 1]);
        }
    }

    // log path size difference
    RCLCPP_INFO(rclcpp::get_logger("AStarPlanner"), "Raw path size: %zu, Pruned path size: %zu", raw_path.poses.size(), pruned.poses.size());
    pruned.poses.push_back(raw_path.poses.back());
    // remove first waypoint
        // if (!pruned.poses.empty()) {
        //     pruned.poses.erase(pruned.poses.begin());
        // }
    return pruned;
}