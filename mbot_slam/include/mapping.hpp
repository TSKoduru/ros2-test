#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <cstdint>

namespace mbot_slam
{

class OccupancyGrid {
public:
    OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y);

    void markCellOccupied(int x, int y);
    void markCellFree(int x, int y);

    std::vector<int8_t> getOccupancyGrid() const;

    int worldToGridX(float x) const;
    int worldToGridY(float y) const;
    int toIndex(int x, int y) const;

    float getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getOriginX() const { return origin_x_; }
    float getOriginY() const { return origin_y_; }

private:
    float resolution_;
    float origin_x_, origin_y_;
    int width_, height_;
    std::vector<float> data_;

    const float kHitLogOdds = 0.5f;  
    const float kMissLogOdds = -0.4f;
    // const float kMinLogOdds = -2.0f;
    // const float kMaxLogOdds = 3.5f;
    // Use much larger values to make the map "sticky"
    const float kMinLogOdds = -20.0f;
    const float kMaxLogOdds = 30.0f;
};

// Bresenham-style ray tracing from sensor origin to hit
std::vector<std::pair<int, int>> bresenhamRayTrace(
    float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid);

} // namespace mbot_slam
