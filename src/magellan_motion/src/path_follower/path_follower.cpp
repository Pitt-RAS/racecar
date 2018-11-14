#include "path_follower.h"
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <optional>

static inline double L2Norm(const geometry_msgs::PoseStamped& pose) {
    return std::sqrt(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y);
}

PathFollower::PathFollower(ros::NodeHandle& nh,
                           double discretization,
                           double lookahead_distance,
                           double max_vel,
                           double max_acc) :
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
        velocity_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_velocity", 10)),
        turning_radius_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_turning_radius", 10)),
        markers_(nh, 10),
        velocity_limiter_(max_vel, max_acc),
        current_path_(),
        path_start_index_(0),
        discretization_(discretization),
        max_acc_(max_acc),
        lookahead_distance_(lookahead_distance) {
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    current_path_ = path;
    path_start_index_ = 0;
}

void PathFollower::Update() {
    if ( !current_path_ )
        return;

    if ( current_path_->poses.size() == 0 ) {
        ROS_WARN("Commanded path is empty");
        return;
    }


    auto transform = tf_buffer_.lookupTransform("base_link",
                                                current_path_->header.frame_id,
                                                ros::Time::now(),
                                                ros::Duration(1.0));


    auto it = current_path_->poses.begin();
    geometry_msgs::PoseStamped closest_point;
    tf2::doTransform(*it, closest_point, transform);
    geometry_msgs::PoseStamped temp;
    for ( it += 0; it != current_path_->poses.end(); ++it) {
        tf2::doTransform(*it, temp, transform);
        if ( L2Norm(temp) > L2Norm(closest_point) )
            break;
        closest_point = temp;
    }
    double cross_track_error = L2Norm(closest_point);
    path_start_index_ = std::distance(current_path_->poses.begin(), it) - 1;

    int points_to_end = std::distance(it, current_path_->poses.end());
    double estimated_remaining_distance = points_to_end * discretization_;

    int lookahead_points = (lookahead_distance_ + cross_track_error) / discretization_;
    it += std::min(points_to_end - 1, lookahead_points);

    geometry_msgs::PoseStamped lookahead_pose;
    tf2::doTransform(*it, lookahead_pose, transform);

    double turning_radius = 0;
    if ( lookahead_pose.pose.position.y != 0 )
        turning_radius = -(lookahead_distance_ * lookahead_distance_) / (2.0 * lookahead_pose.pose.position.y);

    static std_msgs::Float64 velocity_command;
    double speed = 0;
    if ( estimated_remaining_distance > 0 )
        speed = max_vel_;
    velocity_command.data = speed;
    velocity_publisher_.publish(velocity_command);

    static std_msgs::Float64 turning_command;
    turning_command.data = turning_radius;
    turning_radius_publisher_.publish(turning_command);

    markers_.UpdateClosestPoint(closest_point);
    markers_.UpdateLookahead(lookahead_pose);
    markers_.Update();
}
