#include "blob_maker.h"

BlobMaker::BlobMaker(ros::NodeHandle& nh, double discretization, double max_distance, double kernel_size) :
    nh_(nh),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    discretization_(discretization),
    max_distance_(max_distance),
    grid(max_distance * 2 * 100, max_distance * 100 * 2, CV_8U),
    laser_sub(nh.subscribe("/scan", 1, &BlobMaker::UpdateLaserScan, this)),
    grid_pub(nh.advertise<nav_msgs::OccupancyGrid>("/grid", 5)),
    kernel_(cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size))){
}

void BlobMaker::UpdateLaserScan(sensor_msgs::LaserScan::ConstPtr laser) {
    last_scan_ = *laser;
}

void BlobMaker::UpdateObstacles(sensor_msgs::LaserScan& laser) {
    grid.setTo(0);
    double angle_per_step = (laser.angle_max - laser.angle_min) / laser.ranges.size();

    auto transform = tf_buffer_.lookupTransform("base_link",
                                                laser.header.frame_id,
                                                ros::Time::now(),
                                                ros::Duration(1.0));

    geometry_msgs::PoseStamped lidar_point;
    lidar_point.header = laser.header;
    lidar_point.pose.orientation.w = 1;

    for ( int idx = 0; idx < laser.ranges.size(); idx++ ) {
        double angle = laser.angle_min + angle_per_step * idx;

        double range = laser.ranges[idx];

        if ( !isfinite(range) ) {
            continue;
        }

        double x = cos(angle) * range;
        double y = sin(angle) * range;
        lidar_point.pose.position.x = x;
        lidar_point.pose.position.y = y;
        tf2::doTransform(lidar_point, lidar_point, transform);
        lidar_point.pose.position.x *= 100.0;
        lidar_point.pose.position.y *= 100.0;
        lidar_point.pose.position.x += max_distance_ * 100.0;
        lidar_point.pose.position.y += max_distance_ * 100.0;
        //ROS_INFO("x %2.0f y %2.0f", lidar_point.pose.position.x, lidar_point.pose.position.y);
        
        if ( lidar_point.pose.position.x < 0 || lidar_point.pose.position.x > max_distance_ * 100 * 2 )
            continue;
        if ( lidar_point.pose.position.y < 0 || lidar_point.pose.position.y > max_distance_ * 100 * 2 )
            continue;

        grid.at<uchar>(lidar_point.pose.position.y, lidar_point.pose.position.x) = 100;
    }

    cv::dilate(grid, grid, kernel_);
}

void BlobMaker::Update() {
    if ( !last_scan_ )
        return;

    UpdateObstacles(*last_scan_);

    grid_msg.header = last_scan_->header;
    grid_msg.header.frame_id = "base_link";
    grid_msg.info.resolution = discretization_;
    grid_msg.info.height = grid.rows;
    grid_msg.info.width = grid.cols;
        
    grid_msg.info.origin.position.y = -max_distance_;
    grid_msg.info.origin.position.x = -max_distance_;
    
    grid_msg.info.origin.orientation.w = 1;

    grid_msg.data.assign(grid.begin<uchar>(), grid.end<uchar>());
    
    grid_pub.publish(grid_msg);
}
