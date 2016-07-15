#!/usr/bin/env python

import tf
import math
import rospy
from Queue import PriorityQueue
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# --- Globals ---- 
# Position
init = PoseStamped()		# Initial position
goal = PoseStamped()		# Goal position
pos = PoseStamped()			# Current position

# Mapping
costmap = OccupancyGrid()	# Costmap, the inflated occupancy grid
mapInfo = MapMetaData()		# Useful information about the map (e.g. resolution, width, height)
occupancyThresh = 50		# Value to decide safe zones for the robot in the occupancy grid

# Planning
gScore = []					# The gScore set

# Utilities (e.g. flags, etc)
haveInitial = 0
haveGoal = 0

# ---- Subscriber Callbacks ----
# Initial pose msg comes from /initialpose topic, which is of PoseStamped() type
def initCallback(msg):
	global init
	global haveInitial
	init.pose = msg.pose.pose
	quaternion = (init.pose.orientation.x, 
				  init.pose.orientation.y,
				  init.pose.orientation.z, 
				  init.pose.orientation.w)
	rospy.loginfo("Initial pose received: (%s, %s, %s)",
				  init.pose.position.x,
				  init.pose.position.y,
				  tf.transformations.euler_from_quaternion(quaternion)[2]*180.0/math.pi)
	haveInitial += 1

# Odometry msgs come from /odom topic, which are of the Odometry() type
def odomCallback(msg):
	global pos
	pos.pose = msg.pose.pose
	
# Goal msg comes from /move_base_simple/goal topic, which is of the PoseStamped() type
def goalCallback(msg):
	global goal
	global haveGoal
	goal = msg
	quaternion = (goal.pose.orientation.x, 
				  goal.pose.orientation.y,
				  goal.pose.orientation.z, 
				  goal.pose.orientation.w)
	rospy.loginfo("Goal pose received: (%s, %s, %s)",
				  goal.pose.position.x,
				  goal.pose.position.y,
				  tf.transformations.euler_from_quaternion(quaternion)[2]*180.0/math.pi)
	haveGoal += 1

# Costmap comes from /move_base_node/global_costmap/costmap topic, which is of the OccupancyGrid() type
def costmapCallback(msg):
	global costmap
	costmap = msg
	
# Map meta data comes from /map_metadata topic, which is of the MapMetaData() type
def mapInfoCallback(msg):
	global mapInfo
	mapInfo = msg

def planner():
	# Create publisher
	pathPub = rospy.Publisher('/path', Path, queue_size=10)
	
	# Create subscribers
	odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
	initSub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initCallback)
	goalSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback)
	cMapSub = rospy.Subscriber('move_base_node/global_costmap/costmap', OccupancyGrid, costmapCallback)
	infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
	
	# Initialize node
	rospy.init_node('global_planner', anonymous=True)
	
	# Set rate
	r = rospy.Rate(10) # 10 Hz
	
	# Wait for goal pose
	rospy.loginfo("Waiting for initial and goal poses")
	while haveGoal == 0:
		pass
	
	# Search
	path = search()
	path.header.frame_id = "map"
	
	# Publish the path continuously
	while not rospy.is_shutdown():
		pathPub.publish(path)	

def search():
		
def poseToGrid(pose):
	# Converts from pose in meters to pose in grid units (helper function)
	grid_x = int((pose.pose.position.x - mapInfo.origin.position.x) / mapInfo.resolution)
	grid_y = int((pose.pose.position.y - mapInfo.origin.position.y) / mapInfo.resolution)
	auxPose = PoseStamped()
	auxPose.pose.position.x = grid_x
	auxPose.pose.position.y = grid_y
	return auxPose
	
def gridToPose(pose):
	# Converts from grid units to pose in meters (helper function)
	x = (pose.pose.position.x*mapInfo.resolution) + mapInfo.origin.position.x
	y = (pose.pose.position.y*mapInfo.resolution) + mapInfo.origin.position.y
	auxPose = PoseStamped()
	auxPose.pose.position.x = x
	auxPose.pose.position.y = y
	return auxPose

def setWaypoint(wx, wy):
	aux = PoseStamped()
	aux.pose.position.x = wx
	aux.pose.position.y = wy
	return aux

if __name__ == "__main__":
	try:
		planner()
	except rospy.ROSInterruptException:
		pass
