#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

std_msgs::Float64 cmd_vel;
std_msgs::Float64 cmd_turn;
std_msgs::String debug_string;

float current_x = 0;
float current_y = 0;
float current_theta = 0;

void printDebug(std::string ting, ros::Publisher debug_publisher);

float nextPoint[4][2] = {{0,0},{2,0},{2,2},{0,2}}; // this is garbage change later 

int main(int argc, char** argv){
	// Setup publishers and rate 
	ros::init(argc, argv, "force_cmd_publisher");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ros::Publisher velocity_publisher = nh.advertise<std_msgs::Float64>("/platform/cmd_velocity", 10);
	ros::Publisher turning_radius_publisher = nh.advertise<std_msgs::Float64>("/platform/cmd_turning_radius", 10);
	ros::Publisher debug_publisher = nh.advertise<std_msgs::String>("/my_debug", 10);
	ros::Subscriber odom_ekf_subscriber = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odom_ekf_callback);
	tf2_ros::TransformBroadcaster br;

	double rate_hz = 100;
	private_nh.getParam("rate", rate_hz);
	ros::Rate rate(rate_hz);

	// Setup transform listener
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	// End setup

	//waypoint is called fake_point

	auto transform = tf_buffer.lookupTransform("base_link", "fake_point", ros::Time(0), ros::Duration(10.0));
	br.sendTransform(transform);
	cmd_vel.data = 1.0;
	cmd_turn.data = 0.8;
	while(ros::ok()){
		velocity_publisher.publish(cmd_vel);
		turning_radius_publisher.publish(cmd_turn);
		printDebug("Fuck", debug_publisher);

		ros::spinOnce();
		rate.sleep();
	}
}

/*
void updateNextPoint(){
	if(){

	}
}
*/

void printDebug(std::string ting, ros::Publisher debug_publisher){
	debug_string.data = ting;
	debug_publisher.publish(debug_string);
}

void odom_ekf_callback(nav_msgs::Odometry msg){
	current_x = msg.pose.x;
	current_y = msg.pose.y;
	current_theta = something;
}