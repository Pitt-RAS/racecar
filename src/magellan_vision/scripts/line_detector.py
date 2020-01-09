#!/usr/bin/env python

import sys
import math
import threading
import cv2
import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image  # ROS Image Message
from geometry_msgs.msg import Point
from magellan_core.msg import PointArray
from cv_bridge import CvBridge, CvBridgeError  # Converts b/w OpenCV Image and ROS Image Message


class PubSubNode(object):
    def __init__(self):
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
        self._spline_point_arr_pub = rospy.Publisher("/perception/spline_points", PointArray, queue_size=5)
        self._bridge = CvBridge()

        # subscribed Topic
        self._subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._callback, queue_size=1)
        self._image = None

    def run_detects(self):
        with self._lock:
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_link"
            point_arr.header = header
            spline_point_arr.header = header

            # Line detection
            if self._image is not None:
                image_np = self._lines_object.detect(self._image)
                # conversion back to Image
                try:
                    ros_msg = self._bridge.cv2_to_imgmsg(image_np)
                except (CvBridgeError, TypeError) as e:
                    rospy.logerr('Could not convert image. Error: {}'.format(e))
                    return
                # Publish Processed Image amnd Points
                self._image_pub.publish(ros_msg)
                self._point_arr_pub.publish(point_arr)
                self._spline_point_arr_pub.publish(spline_point_arr)


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

    def interpolate_spline(self, points, image):
        x_list = []
        y_list = []
        spline_points = []
        spline_point_arr.points = []

        for point in points:
            x_list.append(point[0])
            y_list.append(point[1])

        if len(x_list) == 0:
            return

        X = np.asarray(x_list)
        Y = np.asarray(y_list)
        denom = X.dot(X) - X.mean() * X.sum()
        m = (X.dot(Y) - Y.mean() * X.sum()) / denom
        b = (Y.mean() * X.dot(X) - X.mean() * X.dot(Y)) / denom
        predicted_y = m * X + b
        predicted_line = np.array([X, predicted_y]).T
        cv2.drawContours(image, [predicted_line.astype(int)], 0, (255, 0, 0), 4)

        alpha = np.linspace(0, len(predicted_line)-1, 10)
        for value in alpha:
            spline_points.append(predicted_line[int(value)])
        for point in spline_points:
            p1 = Point()
            p1.x = point[0]
            p1.y = point[1]
            p1.z = 0
            spline_point_arr.points.append(p1)
            cv2.circle(image, (int(point[0]), int(point[1])), (10), (255, 0, 0), 8)

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
            point_arr.points = []
            right_points = []
            left_points = []
            for line in linesP:
                for x1, y1, x2, y2 in line:
                    if(0 < x1 < 640 and 0 < x2 < 640 and 0 < y1 < 480 and 0 < y2 < 480):
                        slope = (y2 - y1) / (x2 - x1)
                        p1 = Point()
                        p2 = Point()
                        p1.x = x1
                        p1.y = y1
                        p1.z = 0
                        point_arr.points.append(p1)
                        p2.x = x2
                        p2.y = y2
                        p2.z = 0
                        point_arr.points.append(p2)
                        if math.fabs(slope) < .5:
                            continue
                        if x1 < cv_image.shape[1]/2 and x2 < cv_image.shape[1]/2:
                            left_points.append([x1,y1])
                            left_points.append([x2,y2])
                            cv2.circle(cv_image, (x1, y1), (5), (0, 0, 255), 3)
                            cv2.circle(cv_image, (x2, y2), (5), (0, 0, 255), 3)
                        else:
                            right_points.append([x1,y1])
                            right_points.append([x2,y2])
                            cv2.circle(cv_image, (x1, y1), (5), (0, 255, 0), 3)
                            cv2.circle(cv_image, (x2, y2), (5), (0, 255, 0), 3)
            
            self.interpolate_spline(right_points, cv_image)
            self.interpolate_spline(left_points, cv_image)
            return cv_image


def main(args):
    rospy.init_node('LinesNode')
    node_ = PubSubNode()
    r = rospy.get_param("rate")
    rate = rospy.Rate(r)
    global point_arr
    point_arr = PointArray()
    global spline_point_arr
    spline_point_arr = PointArray()
    try:
        while not rospy.is_shutdown():
            node_.run_detects()
            rate.sleep()

    except KeyboardInterrupt as e:
        rospy.logfatal("shutting down ROS Lines detector Module. Error: {}".format(e))


if __name__ == '__main__':
    main(sys.argv)
