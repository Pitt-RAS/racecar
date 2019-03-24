#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "path_markers.h"
#include "velocity_limiter.h"

class PathFollower {
public:
    PathFollower(ros::NodeHandle& nh,
                 double discretization,
                 double lookahead_distance,
                 double lookahead_multiplier,
                 double max_vel,
                 double max_acc,
                 double turn_velocity_multiplier);
    void Update();
private:
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber path_subscriber_;
    ros::Publisher velocity_publisher_;
    ros::Publisher turning_radius_publisher_;

    PathMarkers markers_;
    VelocityLimiter velocity_limiter_;

    nav_msgs::Path::ConstPtr current_path_;
    int path_start_index_;

    double discretization_;
    double lookahead_distance_;
    double lookahead_multiplier_;
    double max_vel_;
    double max_acc_;
    double turn_velocity_multiplier_;

    void UpdatePath(nav_msgs::Path::ConstPtr path);
};

#endif
