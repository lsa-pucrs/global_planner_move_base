#!/usr/bin/env python

import tf
import math
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData

pos = Pose()
map = OccupancyGrid()
mapMeta = MapMetaData()

def uint8(n):
	return n & 0xFF

def poseCallback(msg):
	global pos
	pos = msg.pose.pose
	
def mapCallback(msg):
	global map
	map = msg

def mapMetaCallback(msg):
	global mapMeta
	mapMeta = msg

def costmap_listener():
	# Create subscribers
	sub1 = rospy.Subscriber('/odom', Odometry, poseCallback)
	sub2 = rospy.Subscriber('/map_metadata', MapMetaData, mapMetaCallback)
	sub3 = rospy.Subscriber('/move_base_node/global_costmap/costmap', OccupancyGrid, mapCallback)
	
	# Initialize node
	rospy.init_node('pos_listener', anonymous=True)
	
	# Define rate at 10 Hz
	rate = rospy.Rate(10)
	
	# Define map meta data
	res = mapMeta.resolution
	map_origin_x = mapMeta.origin.position.x
	map_origin_y = mapMeta.origin.position.y
	
	while not rospy.is_shutdown():
		# Determine the robot's position on the costmap occupancy grid
		grid_x = int((pos.position.x - map_origin_x) / res)
		grid_y = int((pos.position.y - map_origin_y) / res)
		
		# Print whatever's in there (map.data is row-major order)
		print map.data[grid_y*4000+grid_x]

if __name__ == "__main__":
	try:
		costmap_listener()
	except rospy.ROSInterruptException:
		pass
