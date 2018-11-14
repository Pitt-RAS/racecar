#ifndef ENCODER_PUBLISHER_H
#define ENCODER_PUBLISHER_H

#include <ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <magellan_core/EncoderCount.h>
#include "Rate.h"

class EncoderPublisher {
public:
    EncoderPublisher(ros::NodeHandle& nh);
    void Update(bool forward);
private:
    ros::NodeHandle& nh_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;
    ros::Publisher velocity_publisher_;
    magellan_core::EncoderCount encoder_count_msg_;
    ros::Publisher encoder_count_publisher_;
    Rate update_rate_;
    Rate update_rate_debug_;
    long int last_left_count_;
    long int last_right_count_;
};

#endif
