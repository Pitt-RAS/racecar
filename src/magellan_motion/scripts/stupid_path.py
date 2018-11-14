#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('stupid_path')

goal_distance = 10
step_distance = 0.01

steps = int(goal_distance / step_distance)

path = Path()
path.header.frame_id = "odom"

for step in range(steps):
    pose = PoseStamped()
    pose.pose.position.x = step_distance * step
    pose.pose.position.y = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)

for step in range(steps):
    pose = PoseStamped()
    pose.pose.position.x = goal_distance + step_distance * step
    pose.pose.position.y = -0.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)

publisher = rospy.Publisher("/path", Path, queue_size=10)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    path.header.stamp = rospy.Time.now()
    publisher.publish(path)
    rate.sleep()
