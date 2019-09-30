#!/usr/bin/env python
import cv2
import numpy as np
import rospy
# import sys
# import roslib
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class List_Of_Obstacles:

    def __init__(self):
        self.obstacle_list = []
        self.laser_sub = rospy.Subscriber("/camera/scan", LaserScan, self.callback)
        self.distance_pub = rospy.Publisher("/cameron/distances_of_obstacles", Float32, queue_size=10)

    def add(self, object):
        self.obstacle_list.append(object)

    def clearList(self):
        self.obstacle_list = []

    # (TODO)
    def callback(self, data):
        pass

class Obstacle_Detection:
    def __init__(self):
    	self.obstacle_list = []
    	self.laser_sub = rospy.Subscriber("/camera/scan",LaserScan,self.laser_callback)
    	self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.distance_pub = rospy.Publisher("/cameron/distances_of_obstacles",Float32,queue_size=10)
        self.midpoint_pub = rospy.Publisher("/cameron/midpoint", Point, queue_size = 10)
        self.bridge_obj = CvBridge()

    def laser_callback(self,data):
        if len(self.obstacle_list) != 0:
            self.get_distances(data)
        else:
            print("No obstacles in obstalce_list")

    def camera_callback(self,data):
    	try:
            cv_image = self.bridge_obj.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            # TODO Make a bailout here
        self.get_obstacles(cv_image)

    def get_distances(self, data):
        # Make method that gets the distance of a given (x,y) point from the LaserScan
        for i, obstacle in enumerate(self.obstacle_list):
            obstacle.dist = data.ranges[int(obstacle.x_center)]
            print(str(obstacle.dist))
            self.distance_pub.publish(obstacle.dist)
        if len(self.obstacle_list) == 2:
        	point = Point()
        	point.x = obstacle_list[0].dist
        	point.y = 0
        	point.z = 0
        	self.midpoint_pub.publish(point)


# This class will become more important when we have many different obstacles that represent different things
class Obstacle:

    def __init__(self, type_of_obstacle, color_of_obstacle, x_center, y_center):
        self.type_of_obstacle = type_of_obstacle
        self.color_of_obstacle = color_of_obstacle
        self.x_center = x_center
        self.y_center = y_center
        self.dist = -1  # default to -1 so we can check later if the distance has been verified


class CenterOfCones(object):

    def __init__(self):
        self.bridge_obj = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)

    # Camera uses bgr8 color encoding
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_obj.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        params = cv2.SimpleBlobDetector_Params()
    def get_obstacles(self,image):
    	self.obstacle_list = []
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

        obs_list = List_Of_Obstacles()
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
        red_keypoints = detector.detect(red_mask)
        blue_keypoints = detector.detect(blue_mask)
        yellow_keypoints = detector.detect(yellow_mask)
        obs_list.clearList()
        for i in range(0, len(keypoints)):
            cone = Obstacle("cone", "red", keypoints[i].pt[0], keypoints[i].pt[1])
            obs_list.add(cone)
        red_mask = cv2.inRange(image, lower_red, upper_red)
        yellow_mask = cv2.inRange(image,lower_yellow,upper_yellow)
        blue_mask = cv2.inRange(image,lower_blue,upper_blue)
        # Detect keypoints
        red_keypoints = detector.detect(red_mask)
        blue_keypoints = detector.detect(blue_mask)
        yellow_keypoints = detector.detect(yellow_mask)
        # Make one list of call keypoints
        keypoints = [red_keypoints, blue_keypoints, yellow_keypoints]
        for i in range(0,len(red_keypoints)):
        	cone = Obstacle("cone", "red", red_keypoints[i].pt[0], red_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)
        for i in range(0,len(blue_keypoints)):
        	cone = Obstacle("cone", "blue", blue_keypoints[i].pt[0], blue_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)
        for i in range(0,len(yellow_keypoints)):
        	cone = Obstacle("cone", "yellow", yellow_keypoints[i].pt[0], yellow_keypoints[i].pt[1])
        	self.obstacle_list.append(cone)

    def clean_up(self):
        cv2.destroyAllWindows()

# This class will become more important when we have many different obstacles that represent different things
class Obstacle:

    def __init__(self, type_of_obstacle, color_of_obstacle, x_center, y_center):
        self.type_of_obstacle = type_of_obstacle
        self.color_of_obstacle = color_of_obstacle
        self.x_center = x_center
        self.y_center = y_center
        self.dist = -1.0  # default to -1 so we can check later if the distance has been verified

def main():
    center_of_cones_obj = Obstacle_Detection()
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
