#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':
    rospy.init_node('fake_obstalces')

    map_pub = rospy.Publisher('/fake_map', OccupancyGrid, queue_size=10)
    rate_ = rospy.Rate(50)

    x_meters = 10
    y_meters = 10

    resolution = .01  # 10 cm

    numX = int(x_meters/resolution)
    numY = int(y_meters/resolution)

    x_bounds_lower = 5/resolution
    x_bounds_upper = 6/resolution

    y_bounds_free = 4/resolution
    y_bounds_free_upper = 5/resolution

    map_im = np.zeros(numX*numY, dtype=int)

    for y in range(0, numY):
        for x in range(0, numX):
            index = numY * y + x
            if x_bounds_lower <= x <= x_bounds_upper:
                if y <= y_bounds_free or y >= y_bounds_free_upper:
                    # inside obstacle
                    map_im[index] = 100

    map_ = map_im.tolist()

    msg_ = OccupancyGrid()
    msg_.header.frame_id = 'map'
    msg_.data = map_
    msg_.info.height = numY
    msg_.info.width = numX
    msg_.info.resolution = resolution

    while not rospy.is_shutdown():
        msg_.header.stamp = rospy.Time.now()
        map_pub.publish(msg_)
        rate_.sleep()
