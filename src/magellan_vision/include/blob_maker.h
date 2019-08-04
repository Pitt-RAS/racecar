#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <optional>

class BlobMaker {
public:
    BlobMaker(ros::NodeHandle& nh, double discretization, double max_distance, double kernel_size);
    void UpdateObstacles(sensor_msgs::LaserScan& laser);
    void UpdateLaserScan(sensor_msgs::LaserScan::ConstPtr laser);
    void Update();
private:
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double discretization_;
    double max_distance_;
    cv::Mat grid;
    ros::Subscriber laser_sub;
    ros::Publisher grid_pub;
    cv::Mat kernel_;
    nav_msgs::OccupancyGrid grid_msg;
    std::optional<sensor_msgs::LaserScan> last_scan_;
};
