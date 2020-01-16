#!/usr/bin/python

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from magellan_core.msg import PointArray
# TODO: Add the message type for detected points

# initialize node
rospy.init_node('laser_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
car_pose = tf.TransformListener()

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.01
width = 500
height = 500

# Initialize car pose relative to world
x_car = 0.0
y_car = 0.0

# square size of the car footprint [m]
footprint = 0.1

# Map update rate (defaulted to 5 Hz)
rate = 5.0

# Range data
car_range = 0.0

# Sequence for laser scan messages
scan_sequence = 0

def points_to_scan(msg):
    lsr_msg = LaserScan()
    # Is base_link the correct frame?
    lsr_msg.header.stamp = rospy.Time.now()
    lsr_msg.header.frame_id = "base_link"
    # TODO: double check that angle_min and angle_max are correct AND convert to rads
    angle_min = -0.688802599907 #radians
    angle_max = 0.682107746601 #radians
    angle_increment = 0.00214539980516
    num_of_indexes = 641
    range_min = 0.449999988079 #meters
    range_max = 10.0 #meters

    # Fill array with positive infinity values
    pos_inf = float("inf")
    ranges = numpy.full((num_of_indexes,),pos_inf)

    for point in msg.points:
        angle = numpy.arctan(float(point.y)/(float(point.x)+0.00000000001)
        dist = sqrt(point.y*point.y + point.x*point.x)
        if(angle<angle_min or angle>angle_max):
            continue
        index = angle//angle_increment
        ranges[index] = dist

    lsr_msg.angle_min = angle_min
    lsr_msg.angle_max = angle_max
    lsr_msg.angle_increment = angle_increment
    lsr_msg.scan_time = 0.0329999998212 # TODO: confirm this
    lsr_msg.time_increment = 0.0
    lsr_msg.ranges = ranges
    lsr_msg.range_min = range_min
    lsr_msg.range_max = range_max
    lsr_msg.intensities = []

    return lsr_msg

def set_free_cells(grid, position, size):
    # set free the cells occupied by the car
    # grid:                ndarray [width,height]
    # position:            [x y] pose of the car
    # size:             r     radius of the footprint
    global resolution

    off_x = position[1] // resolution + width  // 2
    off_y = position[0] // resolution + height // 2

    # set the roi to 1: known free positions
    for i in range(-size//2, size//2):
        for j in range(-size//2, size//2):
            grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid, position, orientation, position_sonar, quaternion_sonar, car_range):
    # set the occupied cells when detecting an obstacle
    # grid:                ndarray [width,height]
    # position:            [x y] pose of the car
    # orientation:      quaternion, orientation of the car
    global resolution

    off_x = position[1] // resolution + width  // 2
    off_y = position[0] // resolution + height // 2

    euler = tf.transformations.euler_from_quaternion(orientation)

    if not car_range == 0.0:

        rotMatrix = numpy.array([[numpy.cos(euler[2]),   numpy.sin(euler[2])],
                                 [-numpy.sin(euler[2]),  numpy.cos(euler[2])]])
        obstacle = numpy.dot(rotMatrix,numpy.array([0, (car_range + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])

        rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])

        # set probability of occupancy to 100 and neighbor cells to 50
        grid[int(obstacle[0]), int(obstacle[1])] = int(100)
        if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(1):
            grid[int(obstacle[0]+1), int(obstacle[1])]   = int(50)
        if  grid[int(obstacle[0]),      int(obstacle[1]+1)] < int(1):
            grid[int(obstacle[0]),   int(obstacle[1]+1)] = int(50)
        if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(1):
            grid[int(obstacle[0]-1), int(obstacle[1])]   = int(50)
        if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(1):
            grid[int(obstacle[0]),   int(obstacle[1]-1)] = int(50)

        t = 0.5
        i = 1
        free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
        while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
            grid[int(free_cell[0]), int(free_cell[1])] = int(0)
            free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
            i = i+1;

def callback_range(msg):
    # callback range
    global car_range
    car_range = points_to_scan(msg)


# Subscribers
# TODO: Change this subscribtion to the correct topic and message type
detected_points_sub = rospy.Subscriber("/perception/world_coord_points", PointArray, callback_range)

# Publishers
occ_pub = rospy.Publisher("/car/map", OccupancyGrid, queue_size = 10)




if __name__ == '__main__':
    # set grid parameters
    if rospy.has_param("occupancy_rate"):
        rate = rospy.get_param("occupancy_rate")

    if rospy.has_param("grid_resolution"):
        resolution = rospy.get_param("grid_resolution")

    if rospy.has_param("grid_width"):
        width = rospy.get_param("grid_width")

    if rospy.has_param("grid_height"):
        height = rospy.get_param("grid_height")

    # fill map_msg with the parameters from launchfile
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.data = range(width*height)

    # initialize grid with -1 (unknown)
    grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
             dtype=numpy.int)
    grid.fill(int(-1))

    # set map origin [meters]
    map_msg.info.origin.position.x = - width // 2 * resolution
    map_msg.info.origin.position.y = - height // 2 * resolution

    loop_rate = rospy.Rate(rate)

    while not rospy.is_shutdown():

        try:
            t = car_pose.getLatestCommonTime("/car_base_link", "/world")
            position, quaternion = car_pose.lookupTransform("/world", "/car_base_link", t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            t = car_pose.getLatestCommonTime("/car_base_link", "/camera_link")
            position_sonar, quaternion_sonar = car_pose.lookupTransform("/car_base_link", "/camera_link", t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # write 0 (null obstacle probability) to the free areas in grid
        set_free_cells(grid, position, int(footprint//resolution))

        # write p>0 (non-null obstacle probability) to the occupied areas in grid
        set_obstacle(grid, position, quaternion, position_sonar, quaternion_sonar, car_range)

        # stamp current ros time to the message
        map_msg.header.stamp = rospy.Time.now()

        # build ros map message and publish
        for i in range(width*height):
            map_msg.data[i] = grid.flat[i]
        occ_pub.publish(map_msg)

        loop_rate.sleep()

