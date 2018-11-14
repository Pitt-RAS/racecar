#ifndef PATH_MARKERS_H
#define PATH_MARKERS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class PathMarkers {
public:
    PathMarkers(ros::NodeHandle& nh, int marker_hz);
    void UpdateLookahead(geometry_msgs::PoseStamped& pose);
    void UpdateClosestPoint(geometry_msgs::PoseStamped& pose);
    void Update();
private:
    ros::NodeHandle& nh_;
    ros::Duration update_period_;
    ros::Time next_update_;

    visualization_msgs::Marker lookahead_marker_;
    visualization_msgs::Marker closest_point_marker_;

    ros::Publisher lookahead_marker_publisher_;
    ros::Publisher closest_point_marker_publisher_;
};

#endif
