#include "velocity_limiter.h"

VelocityLimiter::VelocityLimiter(double max_vel, double max_acc) :
        max_vel_(max_vel),
        max_acc_(max_acc),
        velocity_(0),
        last_update_(ros::Time::now()) {
}

double VelocityLimiter::Update(double remaining_path_length) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_update_).toSec();
    last_update_ = now;

    velocity_ += max_acc_ * dt;

    double max_allowed_velocity = std::sqrt(2.0 * max_acc_ * remaining_path_length);
    ROS_WARN("remaining path length %2.4f\n", remaining_path_length);
    velocity_ = std::min(velocity_, max_allowed_velocity);
    velocity_ = std::max(velocity_, max_vel_);
}

void VelocityLimiter::Reset() {
    velocity_ = 0;
}
