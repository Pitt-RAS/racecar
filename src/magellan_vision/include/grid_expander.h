#pragma once

#include <tuple>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class GridExpander {
public:
    GridExpander(const int nodes_to_expand);
    void build(const geometry_msgs::TransformStamped& transform,
               nav_msgs::OccupancyGrid& grid,
               const sensor_msgs::LaserScan& scan) const;
    void expand(nav_msgs::OccupancyGrid& grid, const std::pair<int, int>& point) const;
private:
    void point_to_map(const geometry_msgs::Point& lidar_point,
                      const nav_msgs::OccupancyGrid& grid,
                      std::pair<int, int>& point) const;
    bool point_to_idx(const nav_msgs::OccupancyGrid& grid, int x, int y, int& idx) const;
    const int expand_nodes_;
};
