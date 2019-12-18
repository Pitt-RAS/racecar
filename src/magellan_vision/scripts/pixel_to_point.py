import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from magellan_core.msg import PointArray

class PubSubNode(object):
	def __init__(self):
		self._point_arr_sub = message_filters.Subscriber("/perception/detected_points", PointArray)
		self._camera_info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
		self._camera_depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
		self._ts = message_filters.TimeSynchronizer([point_arr_sub, camera_depth,sub, camera_info_sub], 10)
		self._ts.RegisterCallback(callback)
		self._point_arr_pub = rospy.Publisher("/perception/world_coord_points", PointArray, queue_size=5)


	def callback((point_arr, camera_depth, camera_info)):
		cx = camera_info.P[2]
		cy = camera_info.P[6]
		fx = camera_info.P[0]
		fy = camera_info.P[5]

		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "camera_link"
		point_arr.header = header
		

		for point in range(point_arr.points):
			depth = camera_depth.get_depth(point.x, point.y)
			Px = (point-cx)*(depth)/fx
			Py = (point-cy)*(depth)/fy
			p1 = Point()
			p1.x = Px
			p1.y = Py
			p1.z = 0
			point_arr.points.append(p1)

		self._point_arr_sub.publish(point_arr)
			

def main():
	rospy.init_node("pixel_to_point_node")
	node_ = PubSubNode()
	r = rospy.get_param("rate")
	rate = rospy.Rate(r)
	global point_arr
	point_arr = PointArray()

	while not rospy.is_shutdown():
		rospy.spin()



if __name__ == '__main__':
	main()


