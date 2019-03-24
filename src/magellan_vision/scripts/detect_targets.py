#!/usr/bin/env python
import cv2 as cv
import numpy as np
from scipy import ndimage
import math
import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TargetDetector():
    def __init__(self):
        # Temporary for testing - hues from the images in rules document
        self.hueArch = 39
        self.huePole = 0
        self.hueRamp = 180

        # These should probably be ROSparams
        self.hueMinArch = self.hueArch - 20
        self.hueMaxArch = self.hueArch + 20
        self.hueMinRamp = self.hueRamp - 50
        self.hueMaxRamp = self.hueRamp + 50
        self.hueMinPole = self.huePole - 20
        self.hueMaxPole = self.huePole + 20
        self.cannyThresh = 100
        self.houghArchThresh = 70
        self.verticalArchThresh = 10
        self.houghPoleThresh = 40
        self.verticalPoleThresh = 5

        # I think these should be the same for all targets
        # Sat should be wide for various outdoors conditions
        self.satMin = 50
        self.satMax = 255
        # Low val = less color - raise min to reduce risk of picking up non-target objects
        self.valMin = 150
        self.valMax = 255

        self.lookForArch = True
        self.lookForRamp = True
        self.lookForPole = True
        self.debugOutput = True

        self.bridge = CvBridge()

        self.polePublisher = rospy.Publisher('target/pole', Float32MultiArray, queue_size=10)
        self.rampPublisher = rospy.Publisher('target/ramp', Float32, queue_size=10)
        self.archPublisher = rospy.Publisher('target/arch', Float32MultiArray, queue_size=10)
        rospy.Subscriber('camera/color/image_raw', Image, self.newFrameCallback)
        rospy.init_node('target_detector')

    def newFrameCallback(self, newFrame):
        self.frame = self.bridge.imgmsg_to_cv2(newFrame, 'bgr8')
        if self.lookForArch:
            (archMidPt, archWidth) = self.detectArch()
            if archMidPt is not None:
                archMsg = Float32MultiArray()
                archMsg.data = [archMidPt, archWidth]
                self.archPublisher.publish(archMsg)
            if self.debugOutput:
                if archMidPt is not None:
                    print('There is an arch centered at x={} with width={}'.format(archMidPt, archWidth))
                else:
                    print('No arch detected')
        if self.lookForRamp:
            rampCenter = self.detectRamp()
            if rampCenter is not None:
                rampMsg = Float32()
                # Only publish the horizontal location of the ramp
                rampMsg.data = rampCenter[0]
                self.rampPublisher.publish(rampMsg)
            if self.debugOutput:
                if rampCenter is not None:
                    print('There is a ramp centered at x={}'.format(rampCenter))
                else:
                    print('No ramp detected')
        if self.lookForPole:
            poleCenters = self.detectPole()
            if poleCenters is not None:
                poleMsg = Float32MultiArray()
                poleMsg.data = poleCenters
                self.polePublisher.publish(poleMsg)
            if self.debugOutput:
                if poleCenters is not None:
                    for poleCenter in poleCenters:
                        print('There is a pole centered at x={}'.format(poleCenter))
                else:
                    print('No poles detected')

# Get the current frame from ROS
    def detectArch(self):
        # frame = cv.imread('Hoop.png', cv.IMREAD_COLOR)
        frame = self.frame
        frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        archThreshold = cv.inRange(frameHSV, (self.hueMinArch, self.satMin, self.valMin),
                                   (self.hueMaxArch, self.satMax, self.valMax))

# Arch detection should look for two vertical lines and find the midpoint
        lines = cv.HoughLinesP(archThreshold, 1, np.pi/180, self.houghArchThresh)

        poles = []
        archMidPt = None
        for line in lines:
            line = line[0]
            if abs(line[0] - line[2]) < self.verticalArchThresh:
                x = (line[0] + line[2]) / 2
                sameLine = False
                for pole in poles:
                    if abs(x - pole) < 10:
                        sameLine = True
                if not sameLine:
                    poles.append(x)
                    if self.debugOutput:
                        cv.line(frame, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3)
                        # cv.imshow("lines", frame)
                        cv.waitKey(0)

        if len(poles) == 2:
            archMidPt = (poles[0] + poles[1]) / 2
            archWidth = abs(poles[0] - poles[1])
            return (archMidPt, archWidth)
        else:
            return (None, None)

    def detectRamp(self):
        # frame = cv.imread('Ramp.png', cv.IMREAD_COLOR)
        frame = self.frame
        frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        rampThreshold = cv.inRange(frameHSV, (self.hueMinRamp, self.satMin, self.valMin),
                                   (self.hueMaxRamp, self.satMax, self.valMax))
        center = ndimage.measurements.center_of_mass(rampThreshold)
        centerImage = cv.circle(frame, (int(center[1]), int(center[0])), 10, (0, 0, 255), 3)  # noqa: F841
        if self.debugOutput:
            # cv.imshow("threshold", centerImage)
            cv.waitKey(0)
        return center

    def detectPole(self):
        # frame = cv.imread('Stanchion2.png', cv.IMREAD_COLOR)
        frame = self.frame
        frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        poleThreshold = cv.inRange(frameHSV, (self.hueMinPole, self.satMin, self.valMin),
                                   (self.hueMaxPole, self.satMax, self.valMax))
        lines = cv.HoughLinesP(poleThreshold, 1, np.pi/180, self.houghPoleThresh)

        poles = []
        poleLengths = []
        finalLines = []
        for line in lines:
            line = line[0]
            if abs(line[0] - line[2]) < self.verticalPoleThresh:
                x = (line[0] + line[2]) / 2
                sameLine = False
                myLength = math.sqrt((line[0] - line[2])**2 + (line[1] - line[3])**2)
                for i in range(len(poles)):
                    if abs(x - poles[i]) < 20:
                        sameLine = True
                        if myLength > poleLengths[i]:
                            poles[i] = x
                            poleLengths[i] = myLength
                            finalLines[i] = line

                if not sameLine:
                    poles.append(x)
                    poleLengths.append(myLength)
                    finalLines.append(line)

        if self.debugOutput:
            for line in finalLines:
                cv.line(frame, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3)
                # cv.imshow("lines", frame)
                # cv.waitKey(0)

        return poles


if __name__ == '__main__':
    detector = TargetDetector()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
