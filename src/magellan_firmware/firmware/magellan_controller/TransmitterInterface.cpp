#include "config.h"
#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface(ros::NodeHandle& nh) :
        nh_(nh),
        r9_(TX_SERIALPORT),
        watchdog_(TX_TIMEOUT),
        throttle_percent_(0),
        steering_angle_(0),
        enabled_(false),
        autonomous_(false),
        user_(0),
        state_(DISABLED),
        state_timer_(0) {
    r9_.begin();
    pinMode(RUN_BUTTON, INPUT_PULLUP);
}

bool GetButton() {
    static bool lastState = false;
    static bool verifyingPress = false;
    static unsigned long timer = 0;

    if ( verifyingPress ) {
        if ( millis() > timer ) {
            // Still pressed
            if ( !digitalRead(RUN_BUTTON) ) {
                lastState = true;
                verifyingPress = false;
                return true;
            }
            else {
                lastState = false;
                verifyingPress = false;
                return false;
            }
        }
    }
    else {
        if ( !digitalRead(RUN_BUTTON) && !lastState ) {
            verifyingPress = true;
            timer = millis() + 200;
        }
        else if ( digitalRead(RUN_BUTTON) )
            lastState = false;
    }
    return false;
}

void TransmitterInterface::Update() {
    switch (state_) {
    case ENABLED:
        autonomous_ = true;
        enabled_ = true;
        user_ = 1;
        if ( GetButton() ) {
            state_ = DISABLED;
        }
        break;
    case POSE_RESET:
        user_ = 1811;
        if ( millis() > state_timer_ )
            state_ = ENABLED;
        break;
    case DISABLED:
        if ( GetButton() ) {
            state_ = POSE_RESET;
            state_timer_ = millis() + 1000;
            enabled_ = true;
            autonomous_ = false;
        }
        else
            state_ = DISABLED;
        enabled_ = false;
        autonomous_ = false;
        break;
    }

    //if ( r9_.read(&channels_[0], &fail_safe_, &lost_frame_) ) {
    //    enabled_ = channels_[4] > 1500;
    //    autonomous_ = channels_[5] > 1500;
    //    user_ = channels_[6];

    //    // Fail safe is only set if the radio disconnects
    //    // This watchdog handles if the receiver disconnects
    //    watchdog_.Feed();

    //    throttle_percent_ = channels_[2] - 172;
    //    // Scale 0 to positive 1
    //    throttle_percent_ /= 1640;

    //    // Center around 0
    //    throttle_percent_ -= 0.5;

    //    steering_angle_ = channels_[0] - 1000;
    //    // Scale -90 to +90
    //    steering_angle_ /= 828;
    //    steering_angle_ *= -90;
    //}
}

bool TransmitterInterface::WantsEnable() {
    return state_ == ENABLED;
    return enabled_;
}

bool TransmitterInterface::WantsAutonomous() {
    return true;
    return autonomous_;
}

bool TransmitterInterface::IsConnected() {
    return true;
}

double TransmitterInterface::throttle_percent() {
    return throttle_percent_;
}

double TransmitterInterface::steering_angle() {
    return steering_angle_;
}

uint16_t TransmitterInterface::user_setting() {
    return user_;
}
