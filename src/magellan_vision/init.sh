#!/bin/sh
# TODO: Script file description
###source setup file befor running this

echo "Starting roscore..."
BASE=`rospack find magellan_vision`/scripts
roscore &
sleep 5

echo "Starting realsense2.0 publishers..."
roslaunch realsense2_camera rs_camera.launch &
sleep 10

echo "Starting Line detector..."
rosrun magellan_vision line_detector.py


# Using & after a command runs it in background
# use jobs command to retrieve background processes
# to kill processes just close the terminal or use kill <process id>
# process id is fond by ps
