#ifndef ENCODER_PUBLISHER_H
#define ENCODER_PUBLISHER_H

#include <ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <magellan_core/EncoderCount.h>
#include "Rate.h"

class EncoderPublisher {
public:
    EncoderPublisher(ros::NodeHandle& nh);
    void Update();
private:
    ros::NodeHandle& nh_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;
    ros::Publisher velocity_publisher_;
    magellan_core::EncoderCount encoder_total_msg_;
    ros::Publisher encoder_total_publisher_;
    Rate update_rate_;
    Rate update_rate_debug_;
    double compute_distance(double steps);
    double compute_velocity(double delta);
};

#endif
