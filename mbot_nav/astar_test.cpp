#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mbot_nav/msg/pose2_d_array.hpp>

#include "astar.hpp"
#include "obstacle_distance_grid.hpp"

#include <cstdlib>
#include <cmath>

static nav_msgs::msg::OccupancyGrid makeBlankMap(int width, int height, float resolution)
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = width;
    map.info.height = height;
    map.info.resolution = resolution;
    map.info.origin.position.x = 0.0f;
    map.info.origin.position.y = 0.0f;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(width * height, 0);
    return map;
}

static inline int idx(int x, int y, int W) { return y * W + x; }

static void addOuterWalls(nav_msgs::msg::OccupancyGrid& map)
{
    const int W = map.info.width, H = map.info.height;
    for (int x = 0; x < W; ++x) {
        map.data[idx(x, 0, W)] = 100;
        map.data[idx(x, H-1, W)] = 100;
    }
    for (int y = 0; y < H; ++y) {
        map.data[idx(0, y, W)] = 100;
        map.data[idx(W-1, y, W)] = 100;
    }
}

static void addWall(nav_msgs::msg::OccupancyGrid& map, int cx, int cy0, int cy1, int thickness)
{
    const int W = map.info.width;
    cy0 = std::max(0, cy0);
    cy1 = std::min((int)map.info.height - 1, cy1);
    for (int y = cy0; y <= cy1; ++y)
        for (int dx = 0; dx < thickness; ++dx)
            map.data[idx(cx + dx, y, W)] = 100;
}

static geometry_msgs::msg::Pose2D worldToPose(float x, float y, float theta)
{
    geometry_msgs::msg::Pose2D p;
    p.x = x;
    p.y = y;
    p.theta = theta;
    return p;
}

// Check waypoints for collisions and proximity to obstacles
static bool checkPathCollisions(const mbot_interfaces::msg::Pose2DArray& path, const ObstacleDistanceGrid& grid)
{
    const float COLLISION_THRESHOLD = 0.0f;  // goes through obstacle
    const float WARNING_THRESHOLD = 0.08f;   // 8cm from obstacle

    bool has_collision = false;
    bool has_warning = false;

    for (size_t i = 0; i < path.poses.size(); ++i) {
        float x = path.poses[i].x;
        float y = path.poses[i].y;

        // Convert world coords to grid coords
        float resolution = grid.getResolution();
        int cx = static_cast<int>(x / resolution);
        int cy = static_cast<int>(y / resolution);

        // Check if in grid
        if (!grid.isCellInGrid(cx, cy)) {
            RCLCPP_ERROR(rclcpp::get_logger("astar_test"),
                "Waypoint %zu (%.2f, %.2f) is outside grid", i, x, y);
            has_collision = true;
            continue;
        }

        // Get distance to nearest obstacle
        float dist = grid.getDistance(cx, cy);

        if (dist <= COLLISION_THRESHOLD) {
            RCLCPP_ERROR(rclcpp::get_logger("astar_test"),
                "Waypoint %zu (%.2f, %.2f) goes through obstacle, will collision", i, x, y);
            has_collision = true;
        } else if (dist <= WARNING_THRESHOLD) {
            RCLCPP_WARN(rclcpp::get_logger("astar_test"),
                "Waypoint %zu (%.2f, %.2f) too close to obstacle (%.3fm), might collision", i, x, y, dist);
            has_warning = true;
        }
    }

    return (!has_collision && !has_warning);
}


// Verify final heading of path
static bool verifyFinalHeading(const mbot_interfaces::msg::Pose2DArray& path,
                               float expected_theta,
                               float tolerance = 0.1f)
{
    if (path.poses.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("astar_test"),
            "Path is empty, cannot verify heading");
        return false;
    }

    const float final_theta = path.poses.back().theta;
    const float theta_diff = std::abs(final_theta - expected_theta);

    RCLCPP_INFO(rclcpp::get_logger("astar_test"),
        "Final heading: %.4f rad, expected: %.4f rad, diff: %.4f rad", final_theta, expected_theta, theta_diff);

    if (theta_diff <= tolerance) {
        RCLCPP_INFO(rclcpp::get_logger("astar_test"),
            "HEADING VERIFIED: Matches goal orientation");
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("astar_test"),
            "HEADING MISMATCH: Expected ~%.4f rad, got %.4f rad (diff: %.4f rad)", expected_theta, final_theta, theta_diff);
        return false;
    }
}

// Test: Simple path with wall obstacle
// Map: 3m x 3m, resolution 0.05m -> 60x60 cells
// Outer walls on borders
// Vertical Wall at x=1.5m, y=[1.0m, 2.0m], thickness=10cm (2 cells)
void runPlannerTest(mbot_nav::AStarPlanner& planner)
{
    const float resolution = 0.05f;
    const float world_width = 3.0f;
    const float world_height = 3.0f;

    int grid_width = static_cast<int>(world_width / resolution);
    int grid_height = static_cast<int>(world_height / resolution);

    auto map = makeBlankMap(grid_width, grid_height, resolution);
    addOuterWalls(map);

    // Wall at x=1.5m, y=[1.0m, 2.0m], thickness=10cm
    // Convert to cells: x=1.5/0.05=30, y=[1.0/0.05=20, 2.0/0.05=40], thickness=0.1/0.05=2
    int wall_x_cell = static_cast<int>(1.5f / resolution);
    int wall_y_start = static_cast<int>(1.0f / resolution);
    int wall_y_end = static_cast<int>(2.0f / resolution);
    int wall_thickness = static_cast<int>(0.1f / resolution);

    addWall(map, wall_x_cell, wall_y_start, wall_y_end, wall_thickness);

    ObstacleDistanceGrid grid;
    grid.computeDistFromMap(map);

    // Start: (1.0m, 1.5m) facing right, Goal: (2.5m, 1.5m) facing left
    auto start = worldToPose(1.0f, 1.5f, 0.0f);
    auto goal = worldToPose(2.5f, 1.5f, M_PI);

    mbot_interfaces::msg::Pose2DArray path;
    bool ok = planner.planPath(grid, start, goal, path);

    if (ok && path.poses.size() > 0) {
        bool collision_free = checkPathCollisions(path, grid);
        if (collision_free) {
            RCLCPP_INFO(rclcpp::get_logger("astar_test"), "[PASS] Planner Test: found path with %zu waypoints", path.poses.size());
            verifyFinalHeading(path, M_PI);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("astar_test"), "[FAIL] Planner Test: path has collisions or warnings");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("astar_test"), "[FAIL] Planner Test: no path found");
    }
}


class AStarTestNode : public rclcpp::Node
{
public:
    AStarTestNode()
    : Node("astar_test_node")
    {
        mbot_nav::AStarPlanner planner;

        runPlannerTest(planner);

        rclcpp::shutdown();
        std::exit(EXIT_SUCCESS);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarTestNode>();
    rclcpp::spin(node);
    return 0;
}
