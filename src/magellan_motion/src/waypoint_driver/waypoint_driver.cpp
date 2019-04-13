#include "waypoint_driver.h"
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <optional>

static inline double L2Norm(const geometry_msgs::PoseStamped& pose) {
    return std::sqrt(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y);
}

WaypointDriver::WaypointDriver(ros::NodeHandle& nh,
                               double max_vel,
                               double turn_velocity_multiplier,
                               double kP) :
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        path_subscriber_(nh.subscribe("/path", 10, &WaypointDriver::UpdatePath, this)),
        velocity_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_velocity", 10)),
        turning_radius_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_turning_radius", 10)),
        current_path_(),
        path_start_index_(0),
        max_vel_(max_vel),
        turn_velocity_multiplier_(turn_velocity_multiplier),
        kP_(kP) {
}

void WaypointDriver::UpdatePath(nav_msgs::Path::ConstPtr path) {
    if ( current_path_ )
        return;
    current_path_ = path;
    path_start_index_ = 0;
}

void WaypointDriver::Update() {
    if ( !current_path_ )
        return;

    if ( current_path_->poses.size() == 0 ) {
        ROS_WARN("Commanded path is empty");
        return;
    }



    if ( path_start_index_ >= current_path_->poses.size() ) {
        ROS_WARN("waypoint_driver: Out of waypoints, setting zero velocity");
        std_msgs::Float64 velocity_command;
        velocity_command.data = 0;
        velocity_publisher_.publish(velocity_command);
        return;
    }


    auto transform = tf_buffer_.lookupTransform("base_link",
                                                current_path_->header.frame_id,
                                                ros::Time::now(),
                                                ros::Duration(1.0));
    geometry_msgs::PoseStamped waypoint = current_path_->poses[path_start_index_];
    tf2::doTransform(waypoint, waypoint, transform);
    if ( waypoint.pose.position.x <= 0 && L2Norm(waypoint) <= 2 ) {
        path_start_index_++;
        return;
    }

    double turning_radius = waypoint.pose.position.y * kP_;

    static std_msgs::Float64 velocity_command;
    velocity_command.data = max_vel_;
    velocity_publisher_.publish(velocity_command);

    static std_msgs::Float64 turning_command;
    turning_command.data = turning_radius;
    turning_radius_publisher_.publish(turning_command);
}
