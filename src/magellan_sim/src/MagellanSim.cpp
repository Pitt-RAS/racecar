#include "MagellanSim.h"

MagellanSim::MagellanSim(ros::NodeHandle& nh) :
        node_handle_(nh),
        commanded_velocity_(0.0),
        commanded_radius_(0.0),
        velocity_(0.0),
        steering_angle_(0.0),
        refresh_rate_(10.0) {
    throttle_subscriber_ =
        node_handle_.subscribe("/platform/cmd_velocity", 1, &MagellanSim::UpdateThrottle, this);
    steering_subscriber_ =
        node_handle_.subscribe("platform/cmd_turning_radius", 1, &MagellanSim::UpdateSteering, this);
    velocity_publisher_ =
        node_handle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/platform/velocity", 1, true);
    turning_radius_publisher_ =
        node_handle_.advertise<std_msgs::Float64>("/platform/turning_radius", 1, true);
    time_ = ros::Time::now();
    imu_publisher =
        node_handle_.advertise<sensor_msgs::Imu>("/platform/imu", 1, true);
    imu_seq = 0;
}

void MagellanSim::run() {
    stop_flag = false;
    while (node_handle_.ok()) {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        if (abs(velocity_ - commanded_velocity_) > 0.005) {
            // Update velocity
            double sim_accel = 5.96; // acceleration, 0-40 mph in 3 seconds
            ros::Duration delta_time = current_time - time_;
            double delta_vel = commanded_velocity_ - velocity_;
            if (delta_vel > 0.1) {
                // positive acceleration
                velocity_ += sim_accel * delta_time.toSec();
            } else if (delta_vel < -0.1) {
                velocity_ -= sim_accel * delta_time.toSec();
            } else {
                velocity_ = commanded_velocity_;
            }
        }
        Update();
        ROS_DEBUG("Velocity: %0.2f\t Turning Radius: %0.2f", velocity_, commanded_radius_);
        refresh_rate_.sleep();
        time_ = current_time;
    }
}

/*
 * Member function to have the sim update and publish simulated values
 */
void MagellanSim::Update() {
    if (stop_flag) {
        Stop();
        UpdateYaw();
    }else{
        UpdateVelocity();
        UpdateYaw();
    }
}

void MagellanSim::UpdateVelocity() {
    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = time_;
    twist_msg.twist.twist.linear.x = velocity_;
    velocity_publisher_.publish(twist_msg);
}

void MagellanSim::UpdateYaw() {
    std_msgs::Float64 course_msg;
    course_msg.data = commanded_radius_;
    turning_radius_publisher_.publish(course_msg);

    //mine
    sensor_msgs::Imu imu_msg;
    geometry_msgs::Vector3 imu_angular_velocity;
    double imu_angular_velocity_covariance[9] = {1e-9,     0,      0,
                                                 0,        1e-9,   0,
                                                 0,        0,      1e-9};

    imu_msg.angular_velocity_covariance[0] = 1e-9;
    imu_msg.angular_velocity_covariance[4] = 1e-9;
    imu_msg.angular_velocity_covariance[8] = 1e-9;
    if (commanded_radius_ > 0 && !stop_flag) {
        imu_angular_velocity.z = -0.75;
    }else if (commanded_radius_ < 0 && !stop_flag) {
        imu_angular_velocity.z = 0.75;

    }else if (commanded_radius_ == 0 || stop_flag) {
        imu_angular_velocity.z = 0;
    }
    imu_msg.angular_velocity = imu_angular_velocity;

    std_msgs::Header imu_header;
    imu_header.stamp = time_.now();
    imu_header.frame_id = "base_link";
    imu_header.seq = imu_seq;
    imu_seq++;
    imu_msg.header = imu_header;

    imu_publisher.publish(imu_msg);

}

/*
 * Updates commanded velocity from message and then updates sim's velocity
 * At this point, this member function just matches the velocity as doing the math
 * Seem to have not a significant enough difference to justify complexity
 */
void MagellanSim::UpdateThrottle(const std_msgs::Float64& cmd_velocity) {
    commanded_velocity_ = cmd_velocity.data;
    if(commanded_velocity_ == 0.0)
        stop_flag = true;
    else
        stop_flag = false;
}

/*
 * Updates commanded velocity from message and upodates sim's turning radius
 * At this point the sim just immediately matches the commanded turning cmd_radius
 */
void MagellanSim::UpdateSteering(const std_msgs::Float64& cmd_radius) {
    commanded_radius_ = cmd_radius.data;
}

void MagellanSim::Stop() {
    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = time_;
    twist_msg.twist.twist.linear.x = 0;
    velocity_publisher_.publish(twist_msg);

    std_msgs::Float64 course_msg;
    course_msg.data = 0;
    turning_radius_publisher_.publish(course_msg);

    sensor_msgs::Imu imu_msg;
    geometry_msgs::Vector3 imu_angular_velocity;
}
