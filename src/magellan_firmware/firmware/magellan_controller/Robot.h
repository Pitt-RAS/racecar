#ifndef ROBOT_H
#define ROBOT_H

#include "RobotState.h"
#include "TransmitterInterface.h"
#include "Rate.h"
#include "config.h"
#include <ros.h>
#include "HeartbeatLED.h"

class Robot {
public:
    Robot(ros::NodeHandle& nh);

    void TeleopInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
    void DisabledInit();
    void DisabledPeriodic();
    void AlwaysPeriodic();

    void Update();
private:
    RobotState current_state_;
    ros::NodeHandle& nh_;
    TransmitterInterface transmitter_;
    HeartbeatLED heartbeat_;
};

#endif
