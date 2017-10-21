#!/usr/bin/env python
import roslib; roslib.load_manifest('monarch_uwb')
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

from monarch_uwb.msg import uwb
from monarch_uwb.msg import uwb_array


import math
import numpy
from array import array
from scipy import optimize
import rospkg
import tf
import thread
import time

# PARAMETERS
#ACCEPTED_THRESHOLD_ROT=10
#ACCEPTED_THRESHOLD_TRANS=math.pi/8

DIST_THRESHOLD= 1800 # maxima distancia a cada anchor(em cm)
TAG_DIST_THRESHOULD =60 # distancia entre tag de forma a evitar tags muito juntas
TAG_COUNT_TO_MINIMIZE=10

#GLOBAL VARS
fingerprinting_data={"baselink":[],"distances":[]}
tags_config=[]
tags_global={}
current_dist=[0,0,0]
inicial_estimation={}
tag_positions=[]
pos_ant=[0,0,0]
results={}
configured=False
current_data={"distances":{},"tag_pos":{},"anchor_ids":[]}
current_data2={"distances":{},"tag_pos":{},"anchor_ids":[]}
SHOW=False
MUTEX=False

#Configure tags position regarding base_link
def config_tags(tags_config):
	global tags_global
	
	for tag in tags_config:
		tags_global[tag['id']]={"pos":Vector3()}
		tags_global[tag['id']]["pos"].z=tag['z']
		tags_global[tag['id']]["pos"].y=tag['y']
		tags_global[tag['id']]["pos"].x=tag['x']


#define and configure data structures
def init_config(array):
	global current_data
	global current_data2
	global inicial_estimation
	global results
	global configured
	config_tags(rospy.get_param("tags_config"))
	
	for i in range(len(array.data)):
		results[array.data[i].anchor_id]=[0,0,0]
		inicial_estimation[array.data[i].anchor_id]=[0,0,150]
		current_data["anchor_ids"].append(array.data[i].anchor_id)
		current_data["distances"][array.data[i].anchor_id]=[]
		current_data["tag_pos"]=[]
		current_data2["anchor_ids"].append(array.data[i].anchor_id)
		current_data2["distances"][array.data[i].anchor_id]=[]
		current_data2["tag_pos"]=[]
	configured=True
# ---------------------------------------------------------------------------------------------------------
# START MINIMIZATION SET OF FUNCTIONS
# ---------------------------------------------------------------------------------------------------------
def total_triangCostFunction(anc_pos,m_tag_t,m_dists_t):
	acc_dummy=0
	for tag,dist in zip(m_tag_t,m_dists_t):
		acc_dummy=acc_dummy+singleCostFunction(anc_pos,tag,dist)**2
	return acc_dummy

def singleCostFunction(anc_pos,tag,r):
	return math.sqrt((anc_pos[0]-tag[0])**2+(anc_pos[1]-tag[1])**2+(anc_pos[2]-tag[2])**2)-r
	
def get_3Dposition_lmq(m_tag_t,m_dists_t,inicial_estimation):
	res=optimize.fmin_slsqp(total_triangCostFunction, inicial_estimation,args=(m_tag_t,m_dists_t),iprint =0, iter=100, acc=1e-06)
	return res
	
def V3SQR(v1,v2):
	return numpy.sqrt((v1[0]-v2[0])**2+(v1[1]-v2[1])**2+(v1[2]-v2[2])**2)
	
def remove_outliers(tst):
	res=[]
	for i in range(len(tst)):
		std=numpy.mean(tst,0)
		mean=numpy.mean(tst,0)
		each=[abs(tst[i][0]-mean[0]),abs(tst[i][1]-mean[1])]
		if(each[0]<std[0] and each[1]<std[1]):
			res.append(tst[i])
	return res

# ---------------------------------------------------------------------------------------------------------
# END MINIMIZATION SET OF FUNCTIONS
# ---------------------------------------------------------------------------------------------------------
	
def callback(array):
	
	global distances
	global current_data
	global lr
	global SHOW
	global teste_inicial
	global tags_global
	global MUTEX
	global configured
	global TAG_DIST_THRESHOULD
	global pos_ant
	
	if(configured==False):
		init_config(array)

	if(MUTEX==True):
		return
	
	try:
		(trans,rot) = lr.lookupTransform(world_frame,"base_link", rospy.Time())
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		rospy.loginfo("No groundtruth frame found")
		return
		
	EulerAngle=tf.transformations.euler_from_quaternion(rot)
	
	robot=[100*trans[0],100*trans[1],100*trans[2],EulerAngle]
	#robot=[100*trans[0],100*trans[1],100*trans[2],rot[2]/rot[3]]
	
	
	
		
	
	fingerprinting_data["baselink"].append(robot)
	
	#Transformations
	if(array.tag_id==tag_id):
		tag_pos=[0,0,0]
		tag_pos[0]=robot[0]+tags_global[tag_id]["pos"].y*math.sin(robot[2]-math.pi)
		tag_pos[1]=robot[1]+tags_global[tag_id]["pos"].y*math.cos(robot[2]-math.pi)
		tag_pos[2]=tags_global[tag_id]["pos"].z
	
	
		if(V3SQR(tag_pos,pos_ant)< TAG_DIST_THRESHOULD):
			return
		pos_ant=tag_pos
	
	
		MUTEX=True
		current_data["tag_pos"].append(tag_pos)
		
		for a in range(len(current_data["anchor_ids"])):
			dist=100*array.data[a].radius
			current_data["distances"][array.data[a].anchor_id].append(dist)
		MUTEX=False
	
		
		
		



def ReInit():
	global current_data
	global current_data2
	
	current_data2["tag_pos"]=[current_data[0]]
	for a in range(len(current_data2["anchor_ids"])):
		current_data2["distances"][current_data2["anchor_ids"][a]].append(current_data["distances"][current_data["anchor_ids"][a]][0])
		
def discretizer():
	global current_data
	global current_data2
	global TAG_DIST_THRESHOULD
	global MUTEX
	while(True):
		
		if(MUTEX ==False and configured==True):
			MUTEX=True
			
			for a in range(len(current_data["tag_pos"])):
				candidate_p=current_data["tag_pos"][a]
				candidate_a={}
				for z in range(len(current_data["anchor_ids"])):
					candidate_a[current_data["anchor_ids"][z]]=current_data["distances"][current_data["anchor_ids"][z]][a]
				suitable=True
				for e in range(len(current_data2["tag_pos"])):
					cur=current_data2["tag_pos"][e]
					if(V3SQR(candidate_p,cur)< TAG_DIST_THRESHOULD):
						suitable=False
				if(suitable==True):
					current_data2["tag_pos"].append(candidate_p)
					for z in range(len(current_data["anchor_ids"])):
						current_data2["distances"][current_data["anchor_ids"][z]].append(candidate_a[current_data["anchor_ids"][z]])
			
			MUTEX=False
			time.sleep(1)

def estimate_anchors_pos():
	global inicial_estimation
	global results
	global tag_positions
	global tags_config
	global current_data2
	global lr
	global SHOW
	global TAG_COUNT_TO_MINIMIZE
	
	LOOP=True
	
	ready2publish=0
	
	while(LOOP):
		for i in range(len(current_data2["anchor_ids"])): #for each anchor
			current_anchor_id=current_data2["anchor_ids"][i]
			
			if(len(current_data2["tag_pos"]) > TAG_COUNT_TO_MINIMIZE):
				estimate=get_3Dposition_lmq(current_data2["tag_pos"],current_data2["distances"][current_anchor_id],inicial_estimation[current_anchor_id])
				results[current_anchor_id]=estimate.tolist()
				inicial_estimation[current_anchor_id]=results[current_anchor_id]
				ready2publish=ready2publish+1
		
		if(ready2publish >=3):
			ready2publish=0
		time.sleep(1)
			
					
def myhook():
	global current_data2
	
	rospack = rospkg.RosPack()
	temp=rospack.get_path('monarch_uwb')
	rospy.loginfo("dumping anchor's position into anchors_pos.yaml");
	f = open(rospack.get_path('uwb_driver')+'/config_files/anchors_pos.yaml','w')
	f.write(str(results))
	f.close()
			


if __name__ == '__main__':
	try:
		rospy.init_node('uwb_mapping')
		if(rospy.has_param("~id")):
				tag_id=rospy.get_param("~id")
		else:
			rospy.signal_shutdown("What tag? missing id parameter")
		
		if(rospy.has_param("world_frame")):
			world_frame=rospy.get_param("world_frame")
		else:	
			rospy.signal_shutdown("Map frame? missing world_frame parameter")
			
		thread.start_new_thread(estimate_anchors_pos, () )
		thread.start_new_thread(discretizer, () )
		rospy.Subscriber("tag_"+str(tag_id)+"/uwb_filtered", uwb_array, callback)
		lr=tf.TransformListener()
		rospy.on_shutdown(myhook)
			
		rospy.spin()
				
			
	except rospy.ROSInterruptException:
		pass
