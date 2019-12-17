#!/usr/bin/env python

import sys
import math
import threading


import cv2
import numpy as np


# ROS imports
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image  # ROS Image Message
from geometry_msgs.msg import Point
from magellan_core.msg import PointArray
from cv_bridge import CvBridge, CvBridgeError  # Converts b/w OpenCV Image and ROS Image Message

# point_arr = PointArray()


class PubSubNode(object):
    def __init__(self):
        # TODO FIX THIS
        try:
            # Global Constant
            self._VERBOSE = rospy.get_param("verbose")
        except KeyError as e:
            rospy.logerr("LineDetector: error in looking up param. {}".format(e))
            raise

        self._lock = threading.RLock()

        '''Initialize ros publisher, ros subscriber'''
        self._lines_object = Lines()
        # topic where we publish
        self._image_pub = rospy.Publisher("/perception/color/image_processed", Image, queue_size=5)
        self._point_arr_pub = rospy.Publisher('/perception/detected_points', PointArray, queue_size=5)
        self._bridge = CvBridge()

        # subscribed Topic
        self._subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._callback, queue_size=1)
        self._image = None

    def run_detects(self):
        with self._lock:
            # header = std_msgs.msg.Header()
            # header.stamp = rospy.Time.now()
            # point_arr.clear()
            # point_arr.header = header
            # point_arr.points = []
            # Line detection
            if self._image is not None:
                image_np = self._lines_object.detect(self._image)

                # conversion back to Image
                try:
                    ros_msg = self._bridge.cv2_to_imgmsg(image_np)
                except CvBridgeError:
                    rospy.logerr('Could not convert image')
                    return

                # Publish Processed Image amnd Points
                self._image_pub.publish(ros_msg)

    '''Callback function of subscribed topic.
    Here images get converted and features detected and published'''

    def _callback(self, ros_data):
        with self._lock:
            try:
                self._image = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
            except CvBridgeError:
                rospy.logerr('Could not convert image')
                return


''' Class Lines: Detects points in an Image that aligns into a line.
    Outputs the image with detected points overlayed on it.'''


class Lines(object):

    def __init__(self):
        self._ddepth = cv2.CV_16S
        try:
            self._KERNEL_SIZE = rospy.get_param("kernel_size")
            self._RHO = rospy.get_param("rho")
            self._THETA = rospy.get_param("theta")
            self._THRESHOLD = rospy.get_param("threshold")
            self._MINLINELENGTH = rospy.get_param("min_line_length")
            self._MAXLINEGAP = rospy.get_param("max_line_gap")
        except KeyError as e:
            rospy.logerr("LineDetector: error in looking up param. {}".format(e))
            raise

    def skeletize(self, img, size, skel):
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while not rospy.is_shutdown():
            eroded = cv2.erode(img, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            zeros = size - cv2.countNonZero(img)
            if zeros == size:
                break

        return skel

    def detect(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        ret, thresh4 = cv2.threshold(blurred, 200, 255, cv2.THRESH_TOZERO)
        size = np.size(thresh4)
        skel = np.zeros(thresh4.shape, np.uint8)

        skeleton = self.skeletize(thresh4, size, skel)

        dst = cv2.Laplacian(skeleton, self._ddepth, ksize=self._KERNEL_SIZE)
        abs_dst = cv2.convertScaleAbs(dst)

        linesP = cv2.HoughLinesP(abs_dst, rho=self._RHO, theta=self._THETA, threshold=self._THRESHOLD,
                                 lines=np.array([]), minLineLength=self._MINLINELENGTH, maxLineGap=self._MAXLINEGAP)

        if linesP is not None:
            for line in linesP:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1)
                    p1 = Point()
                    p2 = Point()
                    p1.x = x1
                    p1.y = y1
                    p1.z = 0
                    point_arr.extend(p1)
                    p2.x = x2
                    p2.y = y2
                    p2.z = 0
                    point_arr.extend(p2)
                    # <-- Calculating the slope.
                    if math.fabs(slope) < .5:
                        # <-- Only consider extreme slope
                        continue
                    if x1 < cv_image.shape[1]/2 and x2 < cv_image.shape[1]/2:
                        # <-- If the slope is negative, left group
                        cv2.circle(cv_image, (x1, y1), (5), (0, 0, 255), 3)
                        cv2.circle(cv_image, (x2, y2), (5), (0, 0, 255), 3)
                    else:  # <-- Otherwise, right group.
                        cv2.circle(cv_image, (x1, y1), (5), (0, 255, 0), 3)
                        cv2.circle(cv_image, (x2, y2), (5), (0, 255, 0), 3)
        return cv_image


def main(args):
    rospy.init_node('LinesNode')
    node_ = PubSubNode()
    r = rospy.get_param("rate")
    rate = rospy.Rate(r)
    try:
        while not rospy.is_shutdown():
            node_.run_detects()
            rate.sleep()

    except KeyboardInterrupt as e:
        rospy.logfatal("shutting down ROS Lines detector Module. Error: {}".format(e))


if __name__ == '__main__':
    main(sys.argv)
