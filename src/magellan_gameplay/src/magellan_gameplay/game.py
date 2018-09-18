#! /usr/bin/env python
import sys
import rospy

from gameplay_exception import GameplayException
from magellan_core.msg import WaypointStamped

def _go_to_goal(goal):
    waypoint = WaypointStamped()
    waypoint.waypoint.goal.position.x = goal[0]
    waypoint.waypoint.goal.position.y = goal[1]
    pub.publish(waypoint)

def _cone_to_target(cones, goal):
    return None

def _goal_done(goal):
    return False

def _robot_is_safe():
    return True

def _goal_to_completion(goal):
    _go_to_goal(goal)
    while not rospy.is_shutdown() and not _goal_done(goal):
        if not _robot_is_safe():
            rospy.logerr('Robot deemed to be in unsafe state! Taking down gameplay')
            raise GameplayException('Robot deemed unsafe, taking down gameplay')
        rospy.sleep(.5)

if __name__ == '__main__':
    rospy.init_node('magellan_gameplay')

    global pub
    pub = rospy.Publisher('waypoint',
                          WaypointStamped,
                          queue_size=1)

    goals = [(1, 2), (3, 4)]
    cones = [(2, 2)]

    for goal in goals:
        if rospy.is_shutdown():
            break

        cone = _cone_to_target(cones, goal)
        while not rospy.is_shutdown() and cone is not None:
            _goal_to_completion(cone)
            cone = _cone_to_target(cones, goal)

        _goal_to_completion(goal)

