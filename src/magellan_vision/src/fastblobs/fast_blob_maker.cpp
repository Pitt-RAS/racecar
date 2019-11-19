#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "grid_expander.h"

class FastBlobNode {
public:
    FastBlobNode(ros::NodeHandle& nh)
            : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_),
              laser_sub_(nh.subscribe("/scan", 1, &FastBlobNode::update_laser_scan, this)),
              grid_pub_(nh.advertise<nav_msgs::OccupancyGrid>("/grid", 1)), grid_(), last_scan_(), expander_(30) {
        grid_.header.frame_id = "base_link";
        grid_.info.resolution = 0.01;
        grid_.info.height = 1000;
        grid_.info.width = 1000;

        grid_.info.origin.position.y = -5;
        grid_.info.origin.position.x = -5;

        grid_.info.origin.orientation.w = 1;

        grid_.data.resize(grid_.info.height * grid_.info.width);
    }

    void update_laser_scan(const sensor_msgs::LaserScan::ConstPtr& laser) {
        last_scan_ = laser;
    }

    void update() {
        if (!last_scan_ || last_scan_->header.frame_id.empty()) {
            ROS_WARN("fast_blob_maker: No scan");
            return;
        }
        grid_.header.stamp = last_scan_->header.stamp;
        auto transform = tf_buffer_.lookupTransform(grid_.header.frame_id, last_scan_->header.frame_id,
                                                    last_scan_->header.stamp, ros::Duration(2.0));

        expander_.build(transform, grid_, *last_scan_);
        grid_pub_.publish(grid_);
    }

private:
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber laser_sub_;
    ros::Publisher grid_pub_;

    sensor_msgs::LaserScan::ConstPtr last_scan_;
    nav_msgs::OccupancyGrid grid_;

    GridExpander expander_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_blobs");

    ros::NodeHandle nh;

    FastBlobNode node(nh);

    ros::Rate rate(10);

    while (ros::ok()) {
        try {
            node.update();
        } catch (tf2::TransformException e) {
            ROS_ERROR("fast_blob_maker: Failed to lookup transform (%s)", e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }
}
