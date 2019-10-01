#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <math.h>
#include <tf/transform_datatypes.h>

std_msgs::Float64 cmd_vel;
std_msgs::Float64 cmd_turn;
std_msgs::String debug_string;

float current_x = 0;
float current_y = 0;
double current_theta = 0;
nav_msgs::Odometry current_odometry;

float goal_x = 0;
float goal_y = 0;
float goal_theta = 0;

int pointTick = 0;

float euclideanDistance = 0;
float distanceTolerance = 0.2;

void printDebug(std::string ting, ros::Publisher debug_publisher);
void updateNextPoint(float tolerance);
void updateTheta(float tolerance, float constant);
void odom_ekf_callback(nav_msgs::Odometry::ConstPtr msg);
void updateEuclideanDistance();
void updateRobot(ros::Publisher velocity_publisher, ros::Publisher turning_radius_publisher, float tolerance);
std::string floatToString (float number);
void setVelocity(float velocity);



float points[4][2] = {{0,0},{1,0},{1,1},{0,1}}; // this is garbage change later

int main(int argc, char** argv) {
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

    //waypoint is called fake_waypoint


    ros::Time now = ros::Time::now();
    if (tf_buffer.canTransform("base_link", "fake_waypoint", now, ros::Duration(3.0))) {
        auto transform = tf_buffer.lookupTransform("base_link", "fake_waypoint", now);
        br.sendTransform(transform);
    }

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        if (tf_buffer.canTransform("base_link", "fake_waypoint", now, ros::Duration(3.0))) {
            auto transform = tf_buffer.lookupTransform("base_link", "fake_waypoint", now);
            //transform.header.frame_id = "odom";
            printDebug(transform.header.frame_id, debug_publisher);
            br.sendTransform(transform);
        }

        //updateRobot(velocity_publisher, turning_radius_publisher, 0.2);

        //printDebug(floatToString(cmd_vel.data) + " <-cmd_vel theta-> " + floatToString(current_theta), debug_publisher);
        cmd_turn.data = 1;
        cmd_vel.data = 1;
        velocity_publisher.publish(cmd_vel);
        turning_radius_publisher.publish(cmd_turn);
        ros::spinOnce();
        rate.sleep();
    }
}

void updateRobot(ros::Publisher velocity_publisher, ros::Publisher turning_radius_publisher, float tolerance) {
    updateEuclideanDistance();
    //do some kinematics for this
    if (euclideanDistance > 5) {
        setVelocity(1);
    }
    if (euclideanDistance > 2 && euclideanDistance < 5) {
        setVelocity(0.5);
    }
    if (euclideanDistance <= tolerance) {
        setVelocity(0);
    }

    updateTheta(0.05, 6); //TODO: determine constant so that smallest angle * constant = lowest cmd_turn value and visa versa
    updateNextPoint(tolerance);
    velocity_publisher.publish(cmd_vel);
    turning_radius_publisher.publish(cmd_turn);
}

void updateNextPoint(float tolerance) {
    if (abs(current_x - points[pointTick][0]) < tolerance && abs(current_y - points[pointTick][1]) < tolerance) {
        pointTick++;
        goal_x = points[pointTick][0];
        goal_y = points[pointTick][1];
    }
    goal_x = 3;
    goal_y = 2;
}

void updateTheta(float tolerance, float constant) {

    if (abs(goal_theta - current_theta) > tolerance) {
        float theta_difference = current_theta - goal_theta;
        if (theta_difference > 0) {
            cmd_turn.data = 1;
        }
        if (theta_difference < 0) {
            cmd_turn.data = -1;
        }
    }else{
        cmd_turn.data = 0;
    }
}

void setVelocity(float velocity) {
    cmd_vel.data = velocity;
}

void printDebug(std::string ting, ros::Publisher debug_publisher) {
    debug_string.data = ting;
    debug_publisher.publish(debug_string);
}

void odom_ekf_callback(nav_msgs::Odometry::ConstPtr msg) {
    //current_odometry = msg;

    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf::Quaternion quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    current_theta = yaw;
}

std::string floatToString (float number) {
    std::ostringstream buff;
    buff << number;
    return buff.str();
}

void updateEuclideanDistance() {
    euclideanDistance = sqrt(pow(current_x - goal_x, 2) + pow(current_y - goal_y, 2));
}
