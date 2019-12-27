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
        self._point_arr_sub = message_filters.Subscriber("/perception/detected_points", PointArray)
        self._camera_info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self._camera_depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self._ts = message_filters.ApproximateTimeSynchronizer(\
        	[self._point_arr_sub, self._camera_depth_sub, self._camera_info_sub], 10, 0.1)
        self._ts.registerCallback(self.callback)
        self._point_arr_pub = rospy.Publisher("/perception/world_coord_points", PointArray, queue_size=5)
        self._bridge = CvBridge()
        self._depth_image = None
        self._depth_array = None

    def callback(self, point_arr, camera_depth, camera_info):
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

        except CvBridgeError as e:
            rospy.logerr("Could not convert image. Error {}".format(e))

        if self._depth_image is not None:
            self._depth_array = np.array(self._depth_image, dtype=np.float32)
            for point in point_arr.points:
                if(int(point.x) > 640 or int(point.y) > 480 or int(point.x) < 0 or int(point.y) < 0):
                    continue
                depth = self._depth_array[int(point.y)][int(point.x)]
                Px = (point.x-cx)*(depth)/fx
                Py = (point.y-cy)*(depth)/fy
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

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
