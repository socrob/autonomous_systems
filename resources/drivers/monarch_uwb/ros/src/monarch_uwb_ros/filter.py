#!/usr/bin/env python
import roslib; roslib.load_manifest('uwb_driver')
import rospy

from uwb_driver.msg import uwb
from uwb_driver.msg import uwb_array
import random
import math
import mean
import variance
import numpy


window=10

init_dists={"0000002a":[0]*window,"0000002b":[0]*window,"0000002c":[0]*window}

def callback(array):
	
	global pub
	global init_dists
	global tag_id
	
	
	if(array.tag_id==tag_id):	
		for i in range(len(["0000002a","0000002b","0000002c"])):
			val=100*array.data[i].radius
			init_dists[array.data[i].anchor_id].append(val)
			init_dists[array.data[i].anchor_id].pop(0)
			array.data[i].variance=numpy.var(init_dists[array.data[i].anchor_id])
			array.data[i].radius=numpy.mean(init_dists[array.data[i].anchor_id])
		
		
	pub.publish(array)




if __name__ == '__main__':
	try:
				
		rospy.init_node('uwb_filtering')
		if(rospy.has_param("~id")):
				tag_id=rospy.get_param("~id")
		else:
			exit()
		
		rospy.Subscriber("tag_"+str(tag_id)+"/uwb_raw", uwb_array, callback)
		pub = rospy.Publisher("tag_"+str(tag_id)+"/uwb_filtered", uwb_array)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
