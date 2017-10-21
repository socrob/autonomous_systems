#!/usr/bin/env python
import roslib; roslib.load_manifest('uwb_driver')
import rospy

from std_msgs.msg import String, Float32, Empty,Header
import std_srvs.srv
from uwb_driver.msg import uwb
from uwb_driver.msg import uwb_array
from geometry_msgs.msg import *

import Uwb
import random
import tf
import math
import robotpos
import numpy as np

STDEV_XY = 0.2
STDEV_TH = np.deg2rad(30)


COV0 = [STDEV_XY**2, 0, 0, 0, 0, 0,
        0, STDEV_XY**2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, STDEV_TH**2]
        
lr=0
robot_pos=[0,0,0]
control=False
CONTROL_RATE=5

def talker():
	global robot_pos
	global control
	global CONTROL_RATE
	global world_frame
	
	rate = rospy.Rate(CONTROL_RATE)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = lr.lookupTransform(world_frame,"uwb_base_link", rospy.Time())
			robot_pos[0]=trans[0]
			robot_pos[1]=trans[1]
			robot_pos[2]=rot
			control=True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			control=False
			pass
		rate.sleep()

def reset_amcl(args):
	global pose_msg
	global pub
	global robot_pos
	global control
	
	if(control):
		pose_msg.header.stamp = rospy.Time()
		pose_msg.pose.pose.position.x = robot_pos[0]/100
		pose_msg.pose.pose.position.y = robot_pos[1]/100
		pose_msg.pose.pose.orientation.z = robot_pos[2][2]
		pose_msg.pose.pose.orientation.w = robot_pos[2][3]
		pub.publish(pose_msg)
		#(True, "Resetting initialpose")
		return []
	else:
		#(False, "No uwb frame available")
		return []

if __name__ == '__main__':
	try:
		
		rospy.init_node('uwb_amcl_service')
		pub= rospy.Publisher("initialpose", PoseWithCovarianceStamped)
		s = rospy.Service('uwb_amcl_reset',  std_srvs.srv.Empty, reset_amcl)
		pose_msg = PoseWithCovarianceStamped(header=Header(frame_id="/map"),pose=PoseWithCovariance(pose=Pose(position=Point(0, 0, 0),orientation=Quaternion(0, 0, 0, 1)),covariance=COV0))
		lr=tf.TransformListener()
		world_frame=rospy.get_param("world_frame")
		talker()
		rospy.spin()
		
		
		
	except rospy.ROSInterruptException:
		pass
