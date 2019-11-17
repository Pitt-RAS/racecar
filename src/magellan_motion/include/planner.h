#ifndef MAGELLAN_PLANNER_H
#define MAGELLAN_PLANNER_H

// system and ros headers
#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <cmath>

// data structure headers
#include <queue>
#include <unordered_map>

// message header
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using nav_msgs::Path;

namespace MagellanPlanner {
// struct to define nodes for planners
struct Successor {
    bool closed;
    double gCost;     // g* value
    double hCost;     // H value
    int xPose;
    int yPose;
    int key;
    std::shared_ptr<Successor> parent;
};


class PathPlanner {
public:
    PathPlanner(ros::NodeHandle& nh, double resolution);
    int getKey(int x, int y);
    Path getPlan(std::shared_ptr<Successor> goalNode);
    Path plan(Point goal);
private:
    bool isFree(int x, int y);
    double getHeuristic(int x, int y);
    bool isGoal(int x, int y);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    std::unordered_map<int, std::shared_ptr<Successor> > nodes;

    std::function<bool(const std::shared_ptr<const Successor>&,
                       const std::shared_ptr<const Successor>&)>
    comp_ = [](const std::shared_ptr<const Successor>& a,
               const std::shared_ptr<const Successor>& b) {
                return (a->gCost + a->hCost) > (b->gCost + b->hCost);
            };

    std::priority_queue<std::shared_ptr<Successor>,
                        std::vector<std::shared_ptr<Successor> >,
                        decltype(comp_)>
    open_;

    nav_msgs::OccupancyGrid _map;

    ros::Subscriber map_sub;

    int startX;
    int startY;
    int goalX;
    int goalY;
    double _resolution;
    bool _has_map;
};
}
#endif // MAGELLAN_PLANNER_H
