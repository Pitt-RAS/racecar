#include <ros/ros.h>
#include "path_follower.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 100;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double lookahead_distance;
    if ( !private_nh.getParam("lookahead_distance", lookahead_distance) ) {
        ROS_ERROR("lookahead_distance param unset");
        ros::shutdown();
        return 0;
    }

    double lookahead_multiplier;
    if ( !private_nh.getParam("lookahead_multiplier", lookahead_multiplier) ) {
        ROS_ERROR("Lookahead multiplier param unset");
        ros::shutdown();
        return 0;
    }

    double discretization;
    if ( !private_nh.getParam("discretization", discretization) ) {
        ROS_ERROR("discretization param unset");
        ros::shutdown();
        return 0;
    }

    double max_velocity;
    if ( !private_nh.getParam("max_vel", max_velocity) ) {
        ROS_ERROR("Maximum velocity param unset");
        ros::shutdown();
        return 0;
    }

    double max_acceleration;
    if ( !private_nh.getParam("max_acc", max_acceleration) ) {
        ROS_ERROR("Maximum acceleration param unset");
        ros::shutdown();
        return 0;
    }

    double turn_velocity_multiplier;
    if ( !private_nh.getParam("turn_velocity_multiplier", turn_velocity_multiplier) ) {
        ROS_ERROR("Turn velocity multiplier  param unset");
        ros::shutdown();
        return 0;
    }

    PathFollower path_follower(nh,
                               discretization,
                               lookahead_distance,
                               lookahead_multiplier,
                               max_velocity,
                               max_acceleration,
                               turn_velocity_multiplier);

    while (ros::ok()) {
        try {
            path_follower.Update();
        }
        catch (tf2::TransformException e) {
            ROS_ERROR("path_follower: Failed to lookup transform");
        }
        ros::spinOnce();
        rate.sleep();
    }
}
