#include "planner.h"

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

static const double SQRT2_MINUS_ONE = 0.4142135624;
static const double SQRT2 = 1.4142135624;
static const double INF_VALUE = 999999;

// 8-connected grid
static const int NUMOFDIRS = 8;
static const int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
static const int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
static const double costs[NUMOFDIRS] = {SQRT2,  1,  SQRT2, 1,  1, SQRT2, 1, SQRT2};

using namespace MagellanPlanner;

PathPlanner::PathPlanner(ros::NodeHandle& nh, double resolution)
        : startX(0),
          startY(0),
          goalX(0),
          goalY(0),
          _resolution(resolution),
          open_(comp_),
          map_sub(nh.subscribe("/fake_map", 10, &PathPlanner::mapCallback, this))
{
    mapSize = 10 / resolution;
    _has_map = false;
}

int PathPlanner::getKey(double x, double y) {
    // gets a unique number for any 2 integers
    // first convert to cell coordinates
    int xMap = (int) x / _resolution;
    int yMap = (int) y / _resolution;
    return ((xMap + yMap) * (xMap + yMap + 1) / 2 + yMap);
}

Path PathPlanner::getPlan(std::shared_ptr<Successor> goalNode) {
    Path p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    double totalCost = 0;

    std::vector<PoseStamped> planVector;

    if (goalNode == nullptr) {
        ROS_ERROR("PathPlanner: Goal Node is null");
        return p;
    }

    std::shared_ptr<const Successor> parent = goalNode;

    while (parent != nullptr) {
        PoseStamped pt;
        pt.pose.position.x = parent->xPose;
        pt.pose.position.y = parent->yPose;
        totalCost = totalCost + parent->gCost;

        planVector.push_back(pt);
        parent = parent->parent;
    }

    ROS_INFO_STREAM("PathPlanner: total plan cost: " << totalCost);

    std::reverse(planVector.begin(), planVector.end());

    p.poses = planVector;
    return p;
}

Path PathPlanner::plan(Point goal) {
    Path p;
    if (!_has_map) {
        ROS_ERROR("PathPlanner: Does not have map yet");
        return p;
    }

    ROS_INFO("PathPlanner: Start");
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime =
        std::chrono::high_resolution_clock::now();

    goalX = goal.x;
    goalY = goal.y;

    if (!isFree(goalX, goalY)) {
        ROS_ERROR("PathPlanner: Goal is not free!!!!");
        return p;
    }

    nodes.clear();

    std::shared_ptr<Successor> start = std::make_shared<Successor>();
    start->hCost = getHeuristic(0, 0);
    start->gCost = 0;
    start->xPose = 0;
    start->yPose = 0;
    start->key = getKey(0,0);
    start->closed = false;

    open_.push(start);
    nodes.insert({start->key, start});

    int numExpand = 0;

    int numSucc = 0;
    int numErased = 0;
    int numInsert = 0;

    while (ros::ok() && !open_.empty()) {
        numExpand++;

        std::shared_ptr<Successor> next = open_.top();
        open_.pop();

        if (next->closed)
            continue;

        if (isGoal(next->xPose, next->yPose)) {
            const auto t_end = std::chrono::high_resolution_clock::now();
            ROS_INFO_STREAM("PathPlanner: Goal found!");
            ROS_INFO_STREAM("PathPlanner: num nodes expanded: " << numExpand);
            ROS_INFO_STREAM("PathPlanner: Planning time: "
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                t_end - startTime).count()
                            << "ms");

            Path result = getPlan(next);
            return result;
        }

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            numSucc++;
            // generate all valid successors
            double newx = next->xPose + dX[dir] * .1;
            double newy = next->yPose + dY[dir] * .1;

            double xCellsRaw = newx / _resolution;
            double yCellsRaw = newy / _resolution;

            int xCells = (int) xCellsRaw;
            int yCells = (int) yCellsRaw;

            int key = getKey(xCells, yCells);
            double cc = costs[dir] + next->gCost;

            bool alreadyOpen = (nodes.count(key) != 0);
            bool goodAdd = false;

            if (alreadyOpen) {
                if (!nodes[key]->closed) {
                    double oldCost = nodes[key]->gCost;
                    goodAdd = (cc < oldCost);
                }
            } else {
                goodAdd = true;
            }

            if (goodAdd) {
                if (newx >= 0 && newx <= 10 && newy >= 0 && newy <= 10) {
                    // if inside map
                    if (isFree(newx,newy)) {
                        // if free
                        std::shared_ptr<Successor> newNode = std::make_shared<Successor>();
                        newNode->gCost = cc;
                        newNode->hCost = getHeuristic(newx, newy);
                        newNode->xPose = newx;
                        newNode->yPose = newy;
                        newNode->closed = false;
                        newNode->key = key;
                        newNode->parent = next;

                        // remove the duplicate
                        if (alreadyOpen) {
                            nodes[key]->closed = true;
                            nodes.erase(key);
                            numErased++;
                        }

                        nodes.insert({key, newNode});
                        open_.push(newNode);
                        numInsert++;
                    }
                }
            }
        }
    }

    ROS_ERROR_STREAM("PathPlanner: NO PATH FOUND!! NODES EXPANDED: " << numExpand);
    ROS_ERROR_STREAM(
        "PathPlanner: NUM SUCC: " << numSucc << " NUM ERASED: " << numErased << " NUM INSERT: " << numInsert);

    return p;
}

bool PathPlanner::isFree(double x, double y) {
    int mapWidth = _map.info.width;
    double mapResolution = _map.info.resolution;

    // origin isnt needed because the origin should be center
    // of the robot and x, y should be from robot center
    // the future we should be more general in our frames

    if (_resolution != mapResolution) {
        ROS_WARN_THROTTLE(1, "Path Planner: resoltuion from msg does not match what was expected");
    }

    // convert x, y to points in map
    double xRaw = x / .01;
    double yRaw = y / .01;

    int xMap = (int) xRaw;
    int yMap = (int) yRaw;

    if (xMap < 0 || yMap < 0) {
        ROS_ERROR_STREAM("Path Planner: X: " << xMap << " and Y: " << yMap << " cell in isFree is negative");
        return false;
    }

    if (xMap > mapWidth || yMap > mapWidth) {
        ROS_ERROR_STREAM("PathPlanner: X or Y cells are outside map. X = " << xMap << " Y = " << yMap);
        return false;
    }

    // convert to row major order
    double indexRaw = yMap * mapWidth + xMap;
    int index = (int) indexRaw;

    return (_map.data[index] != 100);
}

double PathPlanner::getHeuristic(double x, double y) {
    // eight connected grid
    double h = (SQRT2_MINUS_ONE * MIN(abs(goalX - x), abs(goalY - y)) +
                MAX(abs(goalX - x), abs(goalY - y)));
    return h;
}

bool PathPlanner::isGoal(double x, double y) {
    double dist = std::sqrt(std::pow((x - goalX), 2)) + std::sqrt(std::pow((y - goalY), 2));

    return (dist <= _resolution);
}

void PathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    _map = *msg;
    _has_map = true;
}
