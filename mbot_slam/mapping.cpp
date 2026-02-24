#include "mapping.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace mbot_slam {

    OccupancyGrid::OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y)
    : resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y)
    {
        width_ = static_cast<int>(width_m / resolution);
        height_ = static_cast<int>(height_m / resolution);
        data_.resize(width_ * height_, std::numeric_limits<float>::quiet_NaN());
    }

    void OccupancyGrid::markCellOccupied(int x, int y)
    {
        int idx = toIndex(x, y);
        if (idx >= 0 && idx < static_cast<int>(data_.size()))
        {
            if (std::isnan(data_[idx])) data_[idx] = 0.0f;
            data_[idx] = std::min(data_[idx] + kHitLogOdds, kMaxLogOdds);
        }
    }

    void OccupancyGrid::markCellFree(int x, int y)
    {
        int idx = toIndex(x, y);
        if (idx >= 0 && idx < static_cast<int>(data_.size()))
        {
            if (std::isnan(data_[idx])) data_[idx] = 0.0f;
            data_[idx] = std::max(data_[idx] + kMissLogOdds, kMinLogOdds);
        }
    }

    std::vector<int8_t> OccupancyGrid::getOccupancyGrid() const
    {
        std::vector<int8_t> result;
        result.reserve(data_.size());

        for (float log_odds : data_)
        {
            if (std::isnan(log_odds)) {
                result.push_back(-1);  // unknown
            } else {
                float prob = 1.0f - 1.0f / (1.0f + std::exp(log_odds));
                int rounded = static_cast<int>(std::round(prob * 100.0f));
                result.push_back(static_cast<int8_t>(std::clamp(rounded, 0, 100)));
            }
        }

        return result;
    }

    int OccupancyGrid::worldToGridX(float x) const {
        return static_cast<int>((x - origin_x_) / resolution_);
    }

    int OccupancyGrid::worldToGridY(float y) const {
        return static_cast<int>((y - origin_y_) / resolution_);
    }

    int OccupancyGrid::toIndex(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= width_ || y >= height_)
            return -1;
        return y * width_ + x;
    }

    std::vector<std::pair<int, int>> bresenhamRayTrace(
        float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid)
    {
        std::vector<std::pair<int, int>> cells;

        // start and end point in world
        float world_start_x = origin_x, world_start_y = origin_y;
        // print origin points
        float world_end_x = world_start_x + range * cos(theta), world_end_y = world_start_y + range * sin(theta);

        // Convert to grid coordinates use grid.worldToGridX and grid. worldToGridY
        int grid_start_x = grid.worldToGridX(world_start_x), grid_start_y = grid.worldToGridY(world_start_y);
        int grid_end_x = grid.worldToGridX(world_end_x), grid_end_y = grid.worldToGridY(world_end_y);
        

        // Bresenham's algorithm

        int dx = abs(grid_end_x - grid_start_x), dy = abs(grid_end_y - grid_start_y); // Total distance between start and end ('Delta x/y')
        int sx = grid_start_x < grid_end_x ? 1 : -1; // Move left or right? (sx = 'Step X')
        int sy = grid_start_y < grid_end_y ? 1: -1; // Move top or bottom? (sy = 'Step Y')

        int err = dx - dy;
        int curr_x = grid_start_x, curr_y = grid_start_y;


        while (true) {
            cells.push_back(std::make_pair(curr_x, curr_y)); 
            if (curr_x == grid_end_x && curr_y == grid_end_y) break;
            
            int double_err = 2 * err;
            if (double_err >= -dy) {
                err -= dy;
                curr_x += sx;
            }
            if (double_err <= dx) {
                err += dx;
                curr_y += sy;
            }
        }

        return cells;
    }
}