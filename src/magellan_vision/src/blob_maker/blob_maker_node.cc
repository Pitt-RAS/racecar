#include <ros/ros.h>
#include "blob_maker.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "blob_maker");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 1;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double discretization = 0.01;
//    if ( !private_nh.getParam("discretization", discretization) ) {
//        ROS_ERROR("discretization param unset");
//        ros::shutdown();
//        return 0;
//    }
    
    double max_distance = 5;
//    if ( !private_nh.getParam("max_distance", max_distance) ) {
//        ROS_ERROR("max_distance param unset");
//        ros::shutdown();
//        return 0;
//    }
    
    double kernel_size = 30;
//    if ( !private_nh.getParam("kernel_size", kernel_size) ) {
//        ROS_ERROR("kernel_size param unset");
//        ros::shutdown();
//        return 0;
//    }
    BlobMaker blob_maker(nh, discretization, max_distance, kernel_size);

    while (ros::ok()) {
        try {
            blob_maker.Update();
        }
        catch (tf2::TransformException e) {
            ROS_ERROR("BlobMaker: Failed to lookup transform");
        }
        ros::spinOnce();
        rate.sleep();
    }
}
