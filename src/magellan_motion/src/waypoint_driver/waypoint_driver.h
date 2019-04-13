#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

class WaypointDriver {
public:
    WaypointDriver(ros::NodeHandle& nh,
                   double max_vel,
                   double turn_velocity_multiplier,
                   double kP);
    void Update();
private:
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber path_subscriber_;
    ros::Publisher velocity_publisher_;
    ros::Publisher turning_radius_publisher_;

    nav_msgs::Path::ConstPtr current_path_;
    int path_start_index_;

    double max_vel_;
    double turn_velocity_multiplier_;

    double kP_;

    void UpdatePath(nav_msgs::Path::ConstPtr path);
};
