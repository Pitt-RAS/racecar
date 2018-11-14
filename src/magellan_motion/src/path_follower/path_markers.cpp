#include "path_markers.h"

PathMarkers::PathMarkers(ros::NodeHandle& nh, int marker_hz) :
        nh_(nh),
        update_period_(1.0 / marker_hz),
        next_update_(ros::Time::now()),
        lookahead_marker_(),
        closest_point_marker_(),
        lookahead_marker_publisher_(nh.advertise<visualization_msgs::Marker>("/markers/lookahead_marker", 10)),
        closest_point_marker_publisher_(nh.advertise<visualization_msgs::Marker>("/markers/closest_point_marker",
                                                                                 10)) {
    lookahead_marker_.header.frame_id = "base_link";
    lookahead_marker_.type = visualization_msgs::Marker::SPHERE;
    lookahead_marker_.action = visualization_msgs::Marker::MODIFY;

    lookahead_marker_.scale.x = 0.1;
    lookahead_marker_.scale.y = 0.1;
    lookahead_marker_.scale.z = 0.1;

    lookahead_marker_.color.r = 0.0f;
    lookahead_marker_.color.g = 1.0f;
    lookahead_marker_.color.b = 0.0f;
    lookahead_marker_.color.a = 1.0;

    closest_point_marker_.header.frame_id = "base_link";
    closest_point_marker_.type = visualization_msgs::Marker::SPHERE;
    closest_point_marker_.action = visualization_msgs::Marker::MODIFY;

    closest_point_marker_.scale.x = 0.1;
    closest_point_marker_.scale.y = 0.1;
    closest_point_marker_.scale.z = 0.1;

    closest_point_marker_.color.r = 0.0f;
    closest_point_marker_.color.g = 1.0f;
    closest_point_marker_.color.b = 0.0f;
    closest_point_marker_.color.a = 1.0;
}

void PathMarkers::UpdateLookahead(geometry_msgs::PoseStamped& pose) {
    lookahead_marker_.pose = pose.pose;
    lookahead_marker_.pose.orientation.x = 0;
    lookahead_marker_.pose.orientation.y = 0;
    lookahead_marker_.pose.orientation.z = 0;
    lookahead_marker_.pose.orientation.w = 1;
}

void PathMarkers::UpdateClosestPoint(geometry_msgs::PoseStamped& pose) {
    closest_point_marker_.pose = pose.pose;
    closest_point_marker_.pose.orientation.x = 0;
    closest_point_marker_.pose.orientation.y = 0;
    closest_point_marker_.pose.orientation.z = 0;
    closest_point_marker_.pose.orientation.w = 1;
}

void PathMarkers::Update() {
    if ( next_update_ < ros::Time::now() ) {
        lookahead_marker_.header.stamp = ros::Time::now();
        closest_point_marker_.header.stamp = ros::Time::now();

        lookahead_marker_publisher_.publish(lookahead_marker_);
        closest_point_marker_publisher_.publish(closest_point_marker_);

        next_update_ = ros::Time::now() + update_period_;
    }
}
