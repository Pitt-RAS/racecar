#!/usr/bin/env python

# Colors are formatted in bgr8 

import cv2
import numpy as np
import rospy
# import sys
# import roslib
from cv_bridge import CvBridge, CvBridgeError
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Obstacle_Detection:
	def __init__(self):
		self.obstacle_list = []
		self.laser_sub = rospy.Subscriber("/camera/scan",LaserScan,self.callback)
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.distance_pub = rospy.Publisher("/cameron/distances_of_obstacles",Float32,queue_size=10)
        self.bridge_obj = CvBridge()
        

    def getDistances(self, data):
        # Make method that gets the distance of a given (x,y) point from the LaserScan
        for i, obstacle in enumerate(self.obstacle_list):
            obstacle.dist = data.ranges[obstacle.x_center]
            self.distance_pub.publish(obstacle.dist)

    def get_obstacles(self,data):
    	self.obstacle_list.clear()
    	#Set params for blob detection
    	params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 200
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = True
        params.maxInertiaRatio = 0.5
        params.filterByColor = True
        params.blobColor = 255
        detector = cv2.SimpleBlobDetector_create(params)

        # Set upper and lower boundaries for red
        lower_red = np.array([0, 0, 153])
        upper_red = np.array([102, 102, 225])
        lower_blue = np.array([100,0,0])
        upper_blue = np.array([255,102,102])
        upper_yellow = np.array([102,255,255])
        lower_yellow = np.array([0,153,153])
        # find the colors within the specified boundaries and apply
        # the mask
        red_mask = cv2.inRange(cv_image, lower_red, upper_red)
        yellow_mask = cv2.inRange(cv_image,lower_yellow,upper_yellow)
        blue_mask = cv2.inRange(cv_image,lower_blue,upper_blue)
        # Detect keypoints
        red_keypoints = detector.detect(red_mask)
        blue_keypoints = detector.detect(blue_mask)
        yellow_keypoints = detector.detect(yellow_mask)
        # Make one list of call keypoints
        keypoints = [red_keypoints, blue_keypoints, yellow_keypoints]
        for i in range(0,len(red_keypoints)):
        	cone = cone = Obstacle("cone", "red", red_keypoints[i].pt[0], red_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)
        for i in range(0,len(blue_keypoints)):
        	cone = cone = Obstacle("cone", "blue", blue_keypoints[i].pt[0], blue_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)
        for i in range(0,len(yellow_keypoints)):
        	cone = cone = Obstacle("cone", "yellow", yellow_keypoints[i].pt[0], yellow_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)


# This class will become more important when we have many different obstacles that represent different things
class Obstacle:

    def __init__(self, type_of_obstacle, color_of_obstacle, x_center, y_center):
        self.type_of_obstacle = type_of_obstacle
        self.color_of_obstacle = color_of_obstacle
        self.x_center = x_center
        self.y_center = y_center
        self.dist = -1  # default to -1 so we can check later if the distance has been verified


class CenterOfCones(object):

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_obj.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            # TODO Make a bailout here


def main():
    center_of_cones_obj = CenterOfCones(center_of_cones_obj)
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
