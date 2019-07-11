#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

imgPublisher = rospy.Publisher('camera/color/image_raw', Image, queue_size=10)
rospy.init_node('fake_camera')
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    fakeImgMsg = Image()
    imgPublisher.publish(fakeImgMsg)
    rate.sleep()
