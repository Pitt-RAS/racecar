#!/usr/bin/env python
import math
import rospy
import numpy as np

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped, Point


class ContinuousPathNode(object):

    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        #self._raw_path = rospy.get_param('~path')
        self._path = []
        self._step_size = rospy.get_param('~step_size')
        self._frame_id = rospy.get_param('~path_frame')
        self._odom_frame_id = rospy.get_param('~odom_frame')
        self._publisher = rospy.Publisher('/path', Path, queue_size=5)
        self.raw_path_sub = rospy.Subscriber('/noah/raw_path', Point, self.raw_path_callback)

    # so raw_path should be a topic where a geometry_msgs Point gets published
    def raw_path_callback(self, data):
        now = rospy.Time.now()
        try:
            transform = self._tf_buffer.lookup_transform(self._odom_frame_id, self._frame_id, now, rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logwarn("fake_path: Couldn't lookup transform {} -> {}".format(self._frame_id, self._odom_frame_id))
            return
        #self._path = []
        header = Header(frame_id=self._frame_id, stamp=now)
        waypoint = PointStamped()
        waypoint.header = header
        waypoint.point.x = data.x
        waypoint.point.y = data.y

        waypoint = do_transform_point(waypoint, transform)
        rospy.loginfo("fake_path: appending coordinates | _path length = " + str(len(self._path)))
        self._path.append([waypoint.point.x, waypoint.point.y])
        rospy.loginfo("fake_path: Path retransform")
        self.publish_path()

    def publish_path(self):
        if len(self._path) < 2:
            rospy.logwarn("fake_path: Path is empty" + str(len(self._path)))
            return
        path = list(self._path)
        path_msg = Path()
        path_msg.header.frame_id = self._odom_frame_id

        last = np.array(path.pop(0))

        while len(path) != 0:
            curr = np.array(path.pop(0))

            num_steps = math.ceil(np.linalg.norm(last-curr)/self._step_size)

            plan = zip(np.linspace(last[0], curr[0], num_steps), np.linspace(last[1], curr[1], num_steps))

            for point in plan:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = point[0]
                pose_msg.pose.position.y = point[1]
                path_msg.poses.append(pose_msg)

            last = curr
        self._publisher.publish(path_msg)


if __name__ == '__main__':
    rospy.init_node('continuous_path')

    node = ContinuousPathNode()
    rate = rospy.Rate(1)
    retransform_period = rospy.Duration(5)
    last_retransform = rospy.Time(0)
    debug_raw_path_pub = rospy.Publisher('/noah/raw_path', Point, queue_size=5)

    debug_point = Point()
    debug_tick = 0
    rospy.sleep(2)
    
    #initialize first point on path to origin
    debug_point.x = 0
    debug_point.y = 0
    debug_point.z = 0
    debug_raw_path_pub.publish(debug_point)
    while not rospy.is_shutdown():
        rospy.loginfo("enter spin")
        rospy.spin()
        rospy.loginfo("exit spin")