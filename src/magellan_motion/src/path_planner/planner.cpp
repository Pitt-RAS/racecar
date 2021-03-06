#include "planner.h"

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

static const double SQRT2_MINUS_ONE = 0.4142135624;
static const double SQRT2 = 1.4142135624;

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
          map_sub(nh.subscribe("/grid", 10, &PathPlanner::mapCallback, this))
{
    if (!nh.getParam("float_precision", floatPrecision)) {
        throw std::runtime_error("Path Planner: missing parameter floatPrecision");
    }

    if (!nh.getParam("float_precision_divide", floatPrecisionDivide)) {
        throw std::runtime_error("Path Planner: missing parameter floatPrecisionDivide");
    }

    if (!nh.getParam("transform_frame", transformFrame)) {
        throw std::runtime_error("Path Planner: missing parameter transformFrame");
    }

    _has_map = false;
}

int PathPlanner::getKey(int x, int y) {
    // gets a unique number for any 2 integers
    // adding 500 since we want these to be positives
    // TODO: in future, add map width from incoming messages as class member
    int xFinal = x + 500;
    int yFinal = y + 500;

    return ((xFinal + yFinal) * (xFinal + yFinal + 1) / 2 + yFinal);
}

Path PathPlanner::getPlan(std::shared_ptr<Successor> goalNode, tf2_ros::Buffer& tfBuffer) {
    Path p;
    p.header.frame_id = transformFrame;
    p.header.stamp = ros::Time::now();
    double totalCost = 0;

    geometry_msgs::PoseStamped startPoint;
    geometry_msgs::PoseStamped startPointTransformed;

    // lookup transform between rosparam frame & baselink
    try{
        auto transform = tfBuffer.lookupTransform(transformFrame,
                                                  "base_link",
                                                  ros::Time::now(),
                                                  ros::Duration(1.0));

        tf2::doTransform(startPoint, startPointTransformed, transform);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("PathPlanner getPlan: error in lookupTransform");
        ROS_ERROR("%s",ex.what());
        return p;
    }

    std::vector<PoseStamped> planVector;

    if (goalNode == nullptr) {
        ROS_ERROR("PathPlanner: Goal Node is null");
        return p;
    }

    std::shared_ptr<const Successor> parent = goalNode;

    while (parent != nullptr) {
        PoseStamped pt;
        pt.pose.position.x = static_cast<float>(parent->xPose) * floatPrecisionDivide +
                             startPointTransformed.pose.position.x;
        pt.pose.position.y = static_cast<float>(parent->yPose) * floatPrecisionDivide +
                             startPointTransformed.pose.position.y;
        pt.header.frame_id = transformFrame;
        totalCost = totalCost + parent->gCost;

        planVector.push_back(pt);
        parent = parent->parent;
    }

    ROS_INFO_STREAM("PathPlanner: total plan cost: " << totalCost);

    std::reverse(planVector.begin(), planVector.end());

    p.poses = planVector;
    return p;
}

Path PathPlanner::plan(Point goal, tf2_ros::Buffer& tfBuffer) {
    Path p;
    if (!_has_map) {
        ROS_ERROR("PathPlanner: Does not have map yet");
        return p;
    }

    ROS_INFO("PathPlanner: Start");
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime =
        std::chrono::high_resolution_clock::now();

    // casts the float value (precision to hundredth's place) to an int to be used by the planner
    // this is so that we can handle floating values better (ex 1.97 --> 197)

    goalX = static_cast<int>(goal.x * floatPrecision);
    goalY = static_cast<int>(goal.y * floatPrecision);

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

            Path result = getPlan(next, tfBuffer);
            return result;
        }

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            numSucc++;
            // generate all valid successors
            int newx = next->xPose + dX[dir];
            int newy = next->yPose + dY[dir];

            if (isFree(newx,newy)) {
                // if free and inside map
                int key = getKey(newx, newy);
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

    ROS_ERROR_STREAM("PathPlanner: NO PATH FOUND!! NODES EXPANDED: " << numExpand);
    ROS_ERROR_STREAM("PathPlanner: NUM SUCC: " << numSucc << " ERASED: " << numErased << " INSERT: " << numInsert);
    ROS_ERROR_STREAM("PathPlanner: GOALX: " << goalX << " GOALY: " << goalY);

    return p;
}

bool PathPlanner::isFree(int x, int y) {
    int mapWidth = static_cast<int>(_map.info.width);

    int xMap = x + mapWidth / 2;
    int yMap = y + mapWidth / 2;

    if (xMap > mapWidth || yMap > mapWidth) {
        ROS_DEBUG_STREAM("PathPlanner: X or Y cells are outside map. X = " << x << " Y = " << y);
        return false;
    }

    if (xMap < 0 || yMap < 0) {
        ROS_DEBUG_STREAM("Path Planner: X: " << x << " and Y: " << y << " cell in isFree is negative");
        return false;
    }

    // convert to row major order
    int index = yMap * mapWidth + xMap;

    if (index < 0) {
        ROS_ERROR_STREAM("Path Planner: Index: " << index << " in isFree is negative");
        return false;
    }

    return (_map.data[index] != 100);
}

double PathPlanner::getHeuristic(int x, int y) {
    // eight connected grid
    double xDiff = static_cast<double>(goalX - x);
    double yDiff = static_cast<double>(goalY - y);

    double h = (SQRT2_MINUS_ONE * MIN(abs(xDiff), abs(yDiff)) + MAX(abs(xDiff), abs(yDiff)));
    return h;
}

bool PathPlanner::isGoal(int x, int y) {
    return (x == goalX && y == goalY);
}

void PathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    _map = *msg;
    _has_map = true;
}
