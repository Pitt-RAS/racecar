#!/usr/bin/env python

import rospy
import numpy as np
import sys
import random

from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':
    rospy.init_node('fake_obstacle')

    map_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=10)
    rate_ = rospy.Rate(5)

    resolution = 1.0
    x_meters =1
    y_meters=1

    print(x_meters)

    if rospy.has_param('~x_meters'):
    	x_meters = rospy.get_param('/x_meters')
    else:
    	print("x_meters does not exist")

    if rospy.has_param('y_meters'):
    	y_meters = rospy.get_param('y_meters')
    else:
    	print("y_meters does not exist")

    if rospy.has_param('~resolution'):
    	resolution = rospy.get_param('~resolution')
    else:
    	print("resolution does not exist")

    numX=int(x_meters/resolution)
    numY=int(y_meters/resolution)

    x_bounds_lower = 600
    x_bounds_upper = 700

    y_bounds_free = 400
    y_bounds_free_upper = 500

    map_im = np.zeros(numX*numY, dtype=int)

    if '-s' in sys.argv:
        print("straighaway")

    if '-m' in sys.argv:
        right = random.getrandbits(1)
        if right == 1:  # right line is missing
            print("right")
        else:  # left line is missing
            print("left")

        print("missing line")

    if '-c' in sys.argv:
        print("curve")

    # fake obstacles centered at (x/2,y/2)
    for y in range(0, numY):
        for x in range(0, numX):
            index = numY * y + x
            if x_bounds_lower <= x <= x_bounds_upper:
                if y <= y_bounds_free or y >= y_bounds_free_upper:
                    # inside obstacle
                    map_im[index] = 100

    map_ = map_im.tolist()

    msg_ = OccupancyGrid()
    msg_.header.frame_id = 'base_link'
    msg_.data = map_
    msg_.info.height = numY
    msg_.info.width = numX
    msg_.info.resolution = resolution
    msg_.info.origin.position.x = -5
    msg_.info.origin.position.y = -5

    while not rospy.is_shutdown():
        msg_.header.stamp = rospy.Time.now()
        map_pub.publish(msg_)
        rate_.sleep()
