////////////////////////////////////////////////////////////////////////////
//
// Path Planner
//
// This is the top level class for the Path Planner node.
//
// Requests come into the planner via an action over an action server.
//
// Plans are returned to the requestor
//
////////////////////////////////////////////////////////////////////////////

// ROS headers
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// message headers
#include <magellan_motion/PlannerRequestAction.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

#include "planner.h"

typedef actionlib::SimpleActionServer<magellan_motion::PlannerRequestAction> Server;

// Main entry point for the motion planner
int main(int argc, char** argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "trajectory_planner");

    ROS_INFO("MotionPlanner begin");

    // Create a node handle for the node
    ros::NodeHandle nh;

    // This node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh ("~");

    // LOAD PARAMETERS
    double update_frequency = 50;

    // Update frequency retrieve
    //ROS_ASSERT(private_nh.getParam("~planner_update_frequency", update_frequency));

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();

        ros::Duration(.005).sleep();
    }

    Server server(nh, "planner_request", false);
    server.start();

    ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("plan", 1000);

    // construct planner
    double resolution = .01; // 10cm
    MagellanPlanner::PathPlanner planner(nh, resolution);

    // Cache the time
    ros::Time last_time = ros::Time::now();

    // update frequency of the node
    ros::Rate rate(update_frequency);

    magellan_motion::PlannerRequestGoalConstPtr goal_;

    // Run until ROS says we need to shutdown
    while (ros::ok()) {

        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we have a new timestamp. This can be a problem with simulated
        // time that does not update with high precision.
        if (current_time > last_time) {
            last_time = current_time;

            // goal was canceled
            if (server.isPreemptRequested()) {
                server.setPreempted();
                ROS_INFO("Preempt requested. Planning goal was canceled");
            }

            // get a new goal from action server
            if (server.isNewGoalAvailable()) {
                if (server.isActive()) {
                    ROS_WARN("New goal received with current goal still running");
                }

                // planning goal
                goal_ = server.acceptNewGoal();
                ROS_INFO("New goal accepted by planner");

                Path plan = planner.plan(goal_->goal);
                plan_pub.publish(plan);

                magellan_motion::PlannerRequestResult result_;

                result_.success = true;
                server.setSucceeded(result_);
            }
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }

    // All is good.
    return 0;
}
