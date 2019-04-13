#include "path_follower.h"
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <optional>

static inline double L2Norm(const geometry_msgs::PoseStamped& pose) {
    return std::sqrt(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y);
}

static inline double L1Norm(const geometry_msgs::PoseStamped& pose) {
    return pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y;
}

PathFollower::PathFollower(ros::NodeHandle& nh,
                           ros::NodeHandle& private_nh,
                           double discretization,
                           double lookahead_distance,
                           double lookahead_multiplier,
                           double max_vel,
                           double max_acc,
                           double turn_velocity_multiplier) :
        nh_(nh),
        private_nh_(private_nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        user_input_subscriber_(nh.subscribe("/platform/user_input", 10, &PathFollower::UpdateUserInput, this)),
        path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
        velocity_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_velocity", 10)),
        turning_radius_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_turning_radius", 10)),
        markers_(nh, 10),
        velocity_limiter_(max_vel, max_acc),
        current_path_(),
        path_start_index_(0),
        discretization_(discretization),
        lookahead_distance_(lookahead_distance),
        lookahead_multiplier_(lookahead_multiplier),
        max_vel_(max_vel),
        max_acc_(max_acc),
        turn_velocity_multiplier_(turn_velocity_multiplier) {
}

void PathFollower::UpdateUserInput(std_msgs::Int32::ConstPtr input) {
    if ( input->data == 1811 )
        path_start_index_ = 0;
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    current_path_ = path;
    //path_start_index_ = 0;
}

void PathFollower::Update() {
    if ( !current_path_ )
        return;

    if ( current_path_->poses.size() == 0 ) {
        ROS_WARN("Commanded path is empty");
        velocity_limiter_.Reset();
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
    for ( it += path_start_index_; it != current_path_->poses.end(); ++it) {
        tf2::doTransform(*it, temp, transform);
        if ( L2Norm(temp) > L2Norm(closest_point) )
            break;
        closest_point = temp;
    }
    double cross_track_error = L2Norm(closest_point);
    path_start_index_ = std::distance(current_path_->poses.begin(), it) - 1;

    int points_to_end = std::distance(it, current_path_->poses.end());
    double estimated_remaining_distance = points_to_end * discretization_;

    int lookahead_points = (lookahead_distance_ + cross_track_error * lookahead_multiplier_) / discretization_;
    it += std::min(points_to_end - 1, lookahead_points);

    geometry_msgs::PoseStamped lookahead_pose;
    tf2::doTransform(*it, lookahead_pose, transform);

    double turning_radius = 0;
    if ( lookahead_pose.pose.position.y != 0 )
        turning_radius = -(pow(L2Norm(lookahead_pose), 2.0)) / (2.0 * lookahead_pose.pose.position.y);

    static std_msgs::Float64 velocity_command;
    velocity_command.data = 0;
    //velocity_limiter_.Update(estimated_remaining_distance);

    if ( estimated_remaining_distance > 0 )
        velocity_command.data = max_vel_;

    if ( std::abs(turning_radius) < 1.5 )
        velocity_command.data = std::min(velocity_command.data, 0.8);

    velocity_publisher_.publish(velocity_command);

    static std_msgs::Float64 turning_command;
    turning_command.data = turning_radius;

    turning_radius_publisher_.publish(turning_command);

    markers_.UpdateClosestPoint(closest_point);
    markers_.UpdateLookahead(lookahead_pose);
    markers_.Update();
}
