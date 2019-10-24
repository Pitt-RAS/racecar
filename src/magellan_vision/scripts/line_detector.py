#!/usr/bin/env python

#TODO: Code Description

'''
Publishes raw camera image to RawImage Topic. Can be viewed using rviz
#TODO: Publish to other topics such as Detected points, Depth Image etc.. 
#TODO: 
Run Instructions: 
    #1 Run: roscore
    #2 Source the workspace setup folder
    #3 Run: rosrun magellan_vision line_detector.py
'''

import cv2
import numpy as np
import math
from statistics import mean
import pyrealsense2 as rs #Camera Import

#ROS imports
import rospy
from sensor_msgs.msg import Image #ROS Image Message
from cv_bridge import CvBridge, CvBridgeError #Converts b/w OpenCV Image and ROS Image Message

#Global Constants
ddepth = cv2.CV_16S
kernel_size = 3

#Global Variables
skipped_frames = 0
num_of_frames = 0 #FPS Calculation

#TODO: Class Description
#TODO: Publish to multiple topics: Raw Image; Processed Image?; Detected Points/Lines; Depth image; etc.. 
class ImagePublisherROS:
    def __init__(self,topic,msgType):
        # Paramaters
        if msgType == Image:
            self.bridge = CvBridge()
        
        # Topic and Message type
        self.topic = topic
        self.msgType = msgType

        # Publisher
        self.pub = rospy.Publisher(topic, msgType, queue_size=10) # Modify to accomodate a general topic Eg: Raw Image Topic...etc

    def publish(self, data):
        if not rospy.is_shutdown():
            if isinstance(data, np.ndarray): #If data is a Numpy Array(CV Image)
                data = self.bridge.cv2_to_imgmsg(data) #Convert CV Image to ROS Message Image

            if isinstance(data, self.msgType): #Only publish if the data received is of the same type as message type
                rospy.loginfo('Publishing ' + self.topic)
                self.pub.publish(data)
            else:
                rospy.logerr('Data passed of Type: %s is not an instance of Class: %s'%(type(data),type(self.msgType())))
               
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
        
        cv2.namedWindow('Detections', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Detections', cv_image)
        cv2.waitKey(25)

    def clean_up(self):
        cv2.destroyAllWindows()


def main():
    #ROS Node Initialization
    #disable_signals flag allows catching signals(Excecptions) such as the KeyboardInterrupt, otherwise try/except Exceptions may never be handled
    rospy.init_node('ImagePublisherNode',anonymous = True, disable_signals = True)
    rawImagePub = ImagePublisherROS('RawImage',Image) # Args: (Topic, MessageType)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    go = True
    while go:
        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()  # also I think this is possible: ir = frames[0]
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Publish Raw image from the Image Publisher Node to the RawImage Topic
            rawImagePub.publish(depth_frame)

            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('Raw Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Raw Realsense', images)
            cv2.waitKey(25)
            key = cv2.waitKey(25)
            lines_obj = Lines()
            lines_obj.detect(color_image) #Warning: color_image is modified in Lines class to show detected circles as overlay

            if key == 27:
                go = False
                lines_obj.clean_up()
                cv2.destroyAllWindows()
                break

        except (KeyboardInterrupt, SystemExit):
            lines_obj.clean_up()
            cv2.destroyAllWindows()
            raise
            
        except rospy.ROSInterruptException: #TODO: Verify Functionality
            pass


if __name__ == '__main__':
    main()
