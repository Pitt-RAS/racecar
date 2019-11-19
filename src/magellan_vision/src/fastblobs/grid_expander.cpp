#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include "grid_expander.h"

GridExpander::GridExpander(const int nodes_to_expand) : expand_nodes_(nodes_to_expand) {
}

void GridExpander::build(const geometry_msgs::TransformStamped& transform,
                         nav_msgs::OccupancyGrid& grid,
                         const sensor_msgs::LaserScan& scan) const {
    std::fill(grid.data.begin(),
              grid.data.end(),
              1);

    geometry_msgs::PointStamped tf_point;
    for (int i = 0; i < scan.ranges.size(); i++) {
        double angle = scan.angle_min + scan.angle_increment * i;
        tf_point.point.x = std::cos(angle) * scan.ranges[i];
        tf_point.point.y = std::sin(angle) * scan.ranges[i];
        tf_point.point.z = 0;

        tf2::doTransform(tf_point, tf_point, transform);

        std::pair<int, int> point;
        point_to_map(tf_point.point, grid, point);
        expand(grid, point);
    }
}

void GridExpander::expand(nav_msgs::OccupancyGrid& grid, const std::pair<int, int>& point) const {
    int idx;
    for (int x = -expand_nodes_; x <= expand_nodes_; x++)
        for (int y = -expand_nodes_; y <= expand_nodes_; y++)
            if (x * x + y * y <= expand_nodes_ * expand_nodes_ &&
                point_to_idx(grid, point.first + x, point.second + y, idx))
                grid.data[idx] = 100;
}

void GridExpander::point_to_map(const geometry_msgs::Point& lidar_point, const nav_msgs::OccupancyGrid& grid,
                                std::pair<int, int>& point) const {
    point.first = (lidar_point.x - grid.info.origin.position.x) / grid.info.resolution;
    point.second = (lidar_point.y - grid.info.origin.position.y) / grid.info.resolution;
}

bool GridExpander::point_to_idx(const nav_msgs::OccupancyGrid& grid, int x, int y, int& idx) const {
    if (x < 0 || x > grid.info.width || y < 0 || y > grid.info.height)
        return false;
    idx = y * grid.info.width + x;
    return true;
}
