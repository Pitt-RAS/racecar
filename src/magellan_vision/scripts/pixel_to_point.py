#!/usr/bin/env python

import rospy
import message_filters
import std_msgs
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from magellan_core.msg import PointArray
from cv_bridge import CvBridge, CvBridgeError


class PubSubNode(object):
    def __init__(self):
        self._point_arr_sub = message_filters.Subscriber("/perception/spline_points", PointArray)
        self._camera_info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self._camera_depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [self._point_arr_sub, self._camera_depth_sub, self._camera_info_sub], 10, 0.1)
        self._ts.registerCallback(self.callback)
        self._point_arr_pub = rospy.Publisher("/perception/world_coord_points", PointArray, queue_size=5)
        self._bridge = CvBridge()
        self._depth_image = None
        self._depth_array = None

    def callback(self, spline_pixels, camera_depth, camera_info):
        cx = camera_info.P[2]
        cy = camera_info.P[6]
        fx = camera_info.P[0]
        fy = camera_info.P[5]
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"
        point_arr.header = header
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(camera_depth)

        except (CvBridgeError, TypeError) as e:
            rospy.logerr("Could not convert image. Error {}".format(e))
            return

        if self._depth_image is not None:
            point_arr.points = []
            self._depth_array = np.array(self._depth_image, dtype=np.float32)
            for pixel in spline_pixels:
                # TODO: Fix this rudimentary invalid pixel value checking
                if(int(pixel.x) > 640 or int(pixel.y) > 480 or int(pixel.x) < 0 or int(pixel.y) < 0):
                    continue
                # TODO: Get the mean (or median) of points in a BxB window around the point
                depth = self._depth_array[int(pixel.y)][int(pixel.x)]
                Px = (pixel.x-cx)*(depth)/fx
                Py = (pixel.y-cy)*(depth)/fy
                p1 = Point()
                p1.x = Px
                p1.y = Py
                p1.z = 0
                point_arr.points.append(p1)

            self._point_arr_pub.publish(point_arr)


def main():
    rospy.init_node("pixel_to_point_node")
    global point_arr
    point_arr = PointArray()
    PubSubNode()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
