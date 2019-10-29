#!/usr/bin/env python

'''
Subscribes to the /camera/color/image_raw topic(RGB Images from realsense camera) and publishes to the
/output/color/image_processed topic(Processed RGB raw Images)

#TODO: Publish to other topics such as Detected points, Depth Image etc..

Run Instructions:
    #1 Source the workspace setup folder
    #2 run the ./init.sh file from the magellan_vision directory # Will be removed in future release
'''

# Python libs
import sys
import time
import math
from statistics import mean

# Numpy and OpenCV
import cv2
import numpy as np

# ROS imports
import rospy
from sensor_msgs.msg import Image  # ROS Image Message
from cv_bridge import CvBridge, CvBridgeError  # Converts b/w OpenCV Image and ROS Image Message
from magellan_core.msg import Float64Arr

# Global Constant
VERBOSE = True


# Class PubSubNode: Subscribes to the /camera/color/image_raw topic(RGB Images from realsense camera) and publishes
#                   to the /output/color/image_processed topic(Processed RGB raw Images)
# TODO: Publish to multiple topics: Raw Image; Processed Image?; Detected Points/Lines; Depth image; etc..
class PubSubNode:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self._lines_object = Lines()
        # topic where we publish
        self._image_pub = rospy.Publisher("/perception/color/image_processed", Image, queue_size=5)
        self._point_arr_pub = rospy.Publisher('/perception/detectedPoints', Float64Arr, queue_size=10)
        self._bridge = CvBridge()

        # subscribed Topic
        self._subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._callback, queue_size=1)
        if VERBOSE:
            print "subscribed to /camera/color/image_raw"

    def _callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE:
            print 'received image of type: "%s"' % type(ros_data)
        time0 = time.time()
        # conversion to cv2
        try:
            image_np = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError:
            return

        # Line detection
        time1 = time.time()
        # TODO: image_np is inherently changed in the Lines() class. Might be useful for it to have its own layer
        image_np = self._lines_object.detect(image_np)
        time2 = time.time()
        if VERBOSE:
            print 'Detection processed at %s Hz.' % (1/(time2-time1))

        # conversion back to Image
        try:
            ros_msg = self._bridge.cv2_to_imgmsg(image_np)
        except CvBridgeError:
            return

        # Publish Processed Image
        self._image_pub.publish(ros_msg)
        self._point_arr_pub.publish(self._points_arr)
        time3 = time.time()
        if VERBOSE:
            print 'Subscribe to publish frequency: %s Hz' % (1/(time3-time0))

# Class Lines: Detects points in an Image that aligns into a line. Outputs the image with detected points overlayed
#              on it.
class Lines(Object):

    def __init__(self):
        self._ddepth = cv2.CV_16S
        self._kernel_size = 3
        self._points_arr = []


    def skeletize(self, img, size, skel):
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        done = False
        while(not done):
            eroded = cv2.erode(img, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            zeros = size - cv2.countNonZero(img)
            if zeros == size:
                done = True
        return skel

    def detect(self, cv_image):

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        ret, thresh4 = cv2.threshold(blurred, 200, 255, cv2.THRESH_TOZERO)
        size = np.size(thresh4)
        skel = np.zeros(thresh4.shape, np.uint8)

        skeleton = self.skeletize(thresh4, size, skel)

        dst = cv2.Laplacian(skeleton, self._ddepth, ksize=self._kernel_size)
        abs_dst = cv2.convertScaleAbs(dst)

        linesP = cv2.HoughLinesP(abs_dst, rho=6, theta=np.pi / 60, threshold=50,
                                 lines=np.array([]), minLineLength=30, maxLineGap=10)

        if linesP is not None:
            for line in linesP:
                for  x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1)
                    self._points_arr.append((x1,y1))
                    self._points_arr.append((x2,y2))
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
    # ROS Node Initialization
    # disable_signals flag allows catching signals(Excecptions) such as the KeyboardInterrupt, otherwise try/except
    # Exceptions may never be handled

    rospy.init_node('LinesNode', anonymous=True, disable_signals=True)
    PubSubNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down ROS Lines detector Module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
