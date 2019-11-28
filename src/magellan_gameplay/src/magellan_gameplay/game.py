#! /usr/bin/env python
import rospy
import actionlib
import threading

from geometry_msgs.msg import PoseStamped
from magellan_motion.msg import PlannerRequestAction, PlannerRequestGoal


class GameplayNode(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient("planner_request", PlannerRequestAction)
        self._client.wait_for_server()

        self._sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._point_cb)

        self._lock = threading.RLock()

        if rospy.is_shutdown():
            raise rospy.ROSInitException()

    def _point_cb(self, msg):
        with self._lock:
            try:
                state = self._client.get_state()
                if state != 3 and state != 9:
                    self._client.cancel_all_goals()
            except Exception:
                rospy.logwarn('GameplayNode: ERROR IN CANCEL PLAN REQUEST')

        goal_ = PlannerRequestGoal()
        goal_.header.frame_id = msg.header.frame_id
        goal_.header.stamp = rospy.Time.now()
        goal_.goal.x = msg.pose.position.x
        goal_.goal.y = msg.pose.position.y

        self._client.send_goal(goal_)


if __name__ == '__main__':
    rospy.init_node('magellan_gameplay')
    node_ = GameplayNode()
    rospy.spin()
