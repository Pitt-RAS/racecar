#include <ros/ros.h>
#include "waypoint_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_driver");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 100;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double max_velocity;
    if ( !private_nh.getParam("max_vel", max_velocity) ) {
        ROS_ERROR("Maximum velocity param unset");
        ros::shutdown();
        return 0;
    }

    double turn_velocity_multiplier;
    if ( !private_nh.getParam("turn_velocity_multiplier", turn_velocity_multiplier) ) {
        ROS_ERROR("Turn velocity multiplier  param unset");
        ros::shutdown();
        return 0;
    }

    double kP;
    if ( !private_nh.getParam("kP", kP) ) {
        ROS_ERROR("kP param unset");
        ros::shutdown();
        return 0;
    }

    WaypointDriver waypoint_driver(nh,
                                   max_velocity,
                                   turn_velocity_multiplier,
                                   kP);

    while (ros::ok()) {
        try {
            waypoint_driver.Update();
        }
        catch (tf2::TransformException e) {
            ROS_ERROR("waypoint_driver: Failed to lookup transform");
        }
        ros::spinOnce();
        rate.sleep();
    }
}
