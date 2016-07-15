#!/usr/bin/env python

import tf
import math
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry

pos = Pose()
x = 0.0
y = 0.0
yaw = None
Kv = 1.5
Kt = 2.0
goto_goal = 1
path = Path()

def odomCallback(msg):
	global pos
	global x
	global y
	global yaw
	pos = msg.pose.pose
	# Transform the current pose quaternion into euler coordinates (roll, pitch, yaw)
	quaternion = (
		pos.orientation.x, 
		pos.orientation.y,
		pos.orientation.z, 
		pos.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	x = pos.position.x
	y = pos.position.y
	
def pathCallback(msg):
	global path
	path = msg
	path.header.frame_id = "odom"

def move():
	# Create publisher
	twistPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	# Create subscribers
	odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
	pathSub = rospy.Subscriber('path', Path, pathCallback)
	
	# Initialize node
	rospy.init_node('move_planner', anonymous=True)
	
	# Set rate
	r = rospy.Rate(10) # 10 Hz
	
	# Wait for path messages
	rospy.loginfo("Waiting for path")
	rospy.wait_for_message('/path', Path)
	rospy.loginfo("Path received!")
	
	# Make the robot follow the path
	followPath(twistPub)

def followPath(publisher):
	# Iterate the path from second until second-to-last position
	for i in range(1,len(path.poses)-1):
		goto(publisher, path.poses[i], 0.5)
		rospy.loginfo("Arrived at pose %s", i)
		
	# Go to last position (goal) with greater precision
	goto(publisher, path.poses[len(path.poses)-1], 0.01)
	rospy.loginfo("Reached goal at (%s, %s)", path.poses[len(path.poses)-1].pose.position.x, path.poses[len(path.poses)-1].pose.position.y)
	

def goto(pub, currGoal, tolerated_distance):
	# Velocity command
	vel = Twist()
	
	# Get current goal coordinates (ignores goal orientation for now)
	gx = currGoal.pose.position.x
	gy = currGoal.pose.position.y
	
	while True:
		# Define current position (according to odom) and goal
		goal_dist = math.sqrt(((gx-x)**2) + ((gy-y)**2))

		# Do the thing
		vel.linear.x = Kv*goal_dist
		vel.angular.z = Kt*(math.atan2((gy - y), (gx - x)) - yaw)
		
		# Publish velocity commands
		pub.publish(vel)
	
		if goal_dist <= tolerated_distance:
			# Arrived at current goal with some tolerance
			vel.linear.x = 0.0;
			break
		
def setWaypoint(wx, wy):
	aux = Pose()
	aux.position.x = wx
	aux.position.y = wy
	return aux
	
if __name__ == "__main__":
	try:
		move()
	except rospy.ROSInterruptException:
		pass
