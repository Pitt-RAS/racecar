#!/usr/bin/env python

#TODO: Code Description

'''
Publishes raw camera image to RawImage Topic. Can be viewed using rviz
#TODO: Publish to other topics such as Detected points, Depth Image etc.. 
#TODO: 
Run Instructions: 
    #1 Run: roscore
    #2 Source the workspace setup folder
    #3 run the ros wrapper
    #3 Run: rosrun magellan_vision line_detector.py
'''

# Python libs
import sys, time

# Processing Libs
import cv2
import numpy as np
import math
from statistics import mean

# ROS imports
import rospy
from sensor_msgs.msg import Image #ROS Image Message
from cv_bridge import CvBridge, CvBridgeError #Converts b/w OpenCV Image and ROS Image Message

#Global Constants
ddepth = cv2.CV_16S
kernel_size = 3
VERBOSE = True

#Global Variables
skipped_frames = 0
num_of_frames = 0 #FPS Calculation

#TODO: Class Description
#TODO: Publish to multiple topics: Raw Image; Processed Image?; Detected Points/Lines; Depth image; etc.. 
class PubSubNode:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/color/image_processed", Image, queue_size = 5)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        if VERBOSE:
            print "subscribed to /camera/color/image_raw"
                # Topic and Message type

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE:
            print 'received image of type: "%s"'%type(ros_data)
        time0 = time.time()
        ## conversion to cv2 ##
        image_np = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")

        ## Line detection##
        lines_object = Lines()
        
        time1 = time.time()
        lines_object.detect(image_np)
        time2 = time.time()
        if VERBOSE:
            print 'Detection processed %s Hz.'%(1/(time2-time1))

        
        ## conversion back to Image
        ros_msg = self.bridge.cv2_to_imgmsg(image_np)

        #Publish new image
        self.image_pub.publish(ros_msg)
        time3 = time.time()
        if VERBOSE:
            print 'Subscribe to publish frequency: %s Hz'%(1/(time3-time0))
               
#TODO: Class Description
class Obstacle:

    def __init__(self, type_of_obstacle, color_of_obstacle, x_center, y_center):
        self.type_of_obstacle = type_of_obstacle
        self.color_of_obstacle = color_of_obstacle
        self.x_center = x_center
        self.y_center = y_center
        self.dist = -1  # default to -1 so we can check later if the distance has been verified

#TODO: Class Description
class Lines:

    def __init__(self):
        self.centers = 0

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

    def draw_lines(img, lines, color=[0, 255, 0], thickness=3):
        if lines is None:
            return
        img = np.copy(img)
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                3,
            ),
            dtype=np.uint8,
        )
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img

    def best_fit(x_points, y_points):

        m = (((mean(x_points)*mean(y_points)) - mean(x_points*y_points)) /
             ((mean(x_points)*mean(x_points))-mean(x_points*x_points)))
        b = mean(y_points) - m*mean(x_points)

        return m, b

    def detect(self, cv_image):

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        ret, thresh4 = cv2.threshold(blurred, 200, 255, cv2.THRESH_TOZERO)
        size = np.size(thresh4)
        skel = np.zeros(thresh4.shape, np.uint8)

        skeleton = self.skeletize(thresh4, size, skel)

        dst = cv2.Laplacian(skeleton, ddepth, ksize=kernel_size)
        abs_dst = cv2.convertScaleAbs(dst)

        linesP = cv2.HoughLinesP(abs_dst, rho=6, theta=np.pi / 60, threshold=50,
                                 lines=np.array([]), minLineLength=30, maxLineGap=10)

        if linesP is not None:
            for line in linesP:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1)
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


    def clean_up(self):
        cv2.destroyAllWindows()


def main(args):
    #ROS Node Initialization
    #disable_signals flag allows catching signals(Excecptions) such as the KeyboardInterrupt, otherwise try/except Exceptions may never be handled
    rospy.init_node('LinesNode',anonymous = True, disable_signals = True)
    node = PubSubNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down ROS Lines detector Module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
