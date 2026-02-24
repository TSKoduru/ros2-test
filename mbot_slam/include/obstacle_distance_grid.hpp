#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <queue>
#include <geometry_msgs/msg/pose.hpp>

struct GridCell {
    int x;
    int y;
    GridCell(int x_, int y_) : x(x_), y(y_) {}
};

class ObstacleDistanceGrid
{
public:
    ObstacleDistanceGrid();
    void computeDistFromMap(const nav_msgs::msg::OccupancyGrid &map);
    float getDistance(int x, int y) const;
    int8_t getOccupancy(int x, int y) const;
    bool isCellInGrid(int x, int y) const;

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }
    geometry_msgs::msg::Pose getOrigin() const { return origin_; }

private:
    void computeDistances();

    int width_;
    int height_;
    float resolution_; // meters/cell
    std::vector<float> distances_;
    std::vector<int8_t> occupancy_;
    geometry_msgs::msg::Pose origin_;
};