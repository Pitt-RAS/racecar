#ifndef VELOCITY_LIMITER_H
#define VELOCITY_LIMITER_H

#include <ros/ros.h>

class VelocityLimiter {
public:
    VelocityLimiter(double max_vel, double max_acc);

    double Update(double remaining_path_length);
    void Reset();
private:
    double max_vel_;
    double max_acc_;
    double velocity_;

    ros::Time last_update_;
};

#endif
