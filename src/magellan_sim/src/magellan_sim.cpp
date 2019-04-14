#include <ros/ros.h>
#include "MagellanSim.h"

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "magellan_sim");
    ros::NodeHandle node;
    MagellanSim sim(node);
    ROS_INFO("Starting Magellan Sim");
    sim.run();
}
