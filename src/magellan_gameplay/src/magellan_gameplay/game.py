#! /usr/bin/env python
import rospy
import actionlib

from magellan_motion.msg import PlannerRequestAction, PlannerRequestGoal


class GameplayNode(object):
    def __init__(self):
        self._planner_client = actionlib.SimpleActionClient("planner_request", PlannerRequestAction)

        self._planner_client.wait_for_server()

        if rospy.is_shutdown():
            raise rospy.ROSInitException()

        self._goals = [(1, 2), (3, 4)]

        self._rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            goal_ = PlannerRequestGoal()
            goal_.goal.x = 7
            goal_.goal.y = 3

            self._planner_client.send_goal_and_wait(goal_)

            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('magellan_gameplay')
    node_ = GameplayNode()
    node_.run()
