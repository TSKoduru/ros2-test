#include "obstacle_distance_grid.hpp"
#include <limits>
#include <cmath>

ObstacleDistanceGrid::ObstacleDistanceGrid()
: width_(0), height_(0), resolution_(0.05f)
{}

void ObstacleDistanceGrid::computeDistFromMap(const nav_msgs::msg::OccupancyGrid &map)
{
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_ = map.info.origin;

    occupancy_ = map.data;
    distances_.assign(width_ * height_, std::numeric_limits<float>::max());

    computeDistances();
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

float ObstacleDistanceGrid::getDistance(int x, int y) const
{
    if (!isCellInGrid(x, y)) return std::numeric_limits<float>::max();
    return distances_[y * width_ + x];
}

int8_t ObstacleDistanceGrid::getOccupancy(int x, int y) const
{
    if (!isCellInGrid(x, y)) return -1;  // unknown if out of bounds
    return occupancy_[y * width_ + x];
}

void ObstacleDistanceGrid::computeDistances()
{
    std::queue<GridCell> queue;

    // TODO: Compute distances to nearest obstacle for each cell, then fill distances_ vector
    // occupancy_ is 1D array representing 2D grid
    // distances_ is also 1D array representing 2D grid
    // later we will use dist_grid_ in sensor model to compare expected vs actual measurements



    // Step 1: Initialize queue with all occupied cells (value >= 50)
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            if (occupancy_[idx] >= 50) {
                distances_[idx] = 0.0f;
                queue.emplace(x, y);
            }
        }
    }

    // Step 2: Flood-fill using 8-connected neighbors and Euclidean distance
    const int dx[8] = {1, -1, 0, 0,  1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1,  1, -1, 1, -1};
    
    while (!queue.empty()) {
        GridCell cell = queue.front();
        queue.pop();
        int curr_idx = cell.y * width_ + cell.x;
        float curr_dist = distances_[curr_idx];

        for (int i = 0; i < 8; ++i) {
            int nx = cell.x + dx[i];
            int ny = cell.y + dy[i];
            if (!isCellInGrid(nx, ny)) continue;

            int neighbor_idx = ny * width_ + nx;
            float new_dist = curr_dist + ((i < 4) ? 1.0f : std::sqrt(2.0f)) * resolution_;
            if (new_dist < distances_[neighbor_idx]) {
                distances_[neighbor_idx] = new_dist;
                queue.emplace(nx, ny);
            }
        }
    }
    
}

