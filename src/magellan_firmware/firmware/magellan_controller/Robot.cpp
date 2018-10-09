#include "Robot.h"

Robot::Robot(ros::NodeHandle& nh) :
        current_state_(MODE_DISABLED),
        nh_(nh),
        transmitter_(nh),
        heartbeat_(),
        throttle_pwm_(ESC_PWM),
        steering_pwm_(SERVO_PWM),
        throttle_subscriber_("/platform/throttle", &Robot::UpdateThrottle, this),
        throttle_percent_(0.0),
        steering_subscriber_("/platform/steering", &Robot::UpdateSteering, this),
        steering_angle_(0.0),
        imu_(nh) {
    nh.subscribe(throttle_subscriber_);
    nh.subscribe(steering_subscriber_);

    steering_pwm_.ConfigOffset(STEERING_OFFSET);
    throttle_pwm_.ConfigLowLimit(THROTTLE_MIN);
    steering_pwm_.ConfigLowLimit(STEERING_MIN);

    DisabledInit();
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    // Get the throttle percent from the Transmitter Interface and pass it to PWM
    throttle_pwm_.Set(transmitter_.throttle_percent());

    // Get the steering angle from the Transmitter Interface and pass it to PWM
    steering_pwm_.Set(transmitter_.steering_angle() / 90.0);
}

void Robot::AutonomousInit() {
    steering_pwm_.Set(0.0);
    throttle_pwm_.Set(0.0);

    throttle_percent_ = 0.0;
    steering_angle_ = 0.0;
}

void Robot::AutonomousPeriodic() {
    throttle_pwm_.Set(throttle_percent_);
    steering_pwm_.Set(steering_angle_ / 90.0);
}

void Robot::DisabledInit() {
    steering_pwm_.Set(0.0);
    throttle_pwm_.Set(0.0);
}

void Robot::DisabledPeriodic() {
}

void Robot::AlwaysPeriodic() {
    imu_.Update();
}

void Robot::UpdateThrottle(const std_msgs::Float64& cmd_throttle_percent_) {
    throttle_percent_ = cmd_throttle_percent_.data;
}

void Robot::UpdateSteering(const std_msgs::Float64& cmd_steering_angle_) {
    steering_angle_ = cmd_steering_angle_.data;
}

void Robot::Update() {
    // Subsystem updates
    transmitter_.Update();
    heartbeat_.Update();

    if ( transmitter_.WantsEnable() && transmitter_.WantsAutonomous() && current_state_ != MODE_AUTONOMOUS ) {
        AutonomousInit();
        current_state_ = MODE_AUTONOMOUS;
        heartbeat_.SetState(current_state_);
    }
    else if ( transmitter_.WantsEnable() && !transmitter_.WantsAutonomous() && current_state_ != MODE_TELEOP ) {
        TeleopInit();
        current_state_ = MODE_TELEOP;
        heartbeat_.SetState(current_state_);
    }
    else if ( !transmitter_.WantsEnable() && current_state_ != MODE_DISABLED ) {
        DisabledInit();
        current_state_ = MODE_DISABLED;
        heartbeat_.SetState(current_state_);
    }

    if ( current_state_ == MODE_DISABLED )
        DisabledPeriodic();
    else if ( current_state_ == MODE_AUTONOMOUS )
        AutonomousPeriodic();
    else if ( current_state_ == MODE_TELEOP )
        TeleopPeriodic();

    AlwaysPeriodic();
}
