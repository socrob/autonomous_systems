'''
Created on 20 de Fev de 2014

@author: Fialho
'''
import serial
import numpy
import thread
from monarch_uwb.msg import uwb
import math
from random import random
import mean
import time
#tag1="0000002a 1.78 0000002b 2.78 0000002c 3.78 0000001 000000\n\r";
readings="";

def dist(p1,p2):
	res=math.sqrt( (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
	res=res/100;
	return round(res,2);

def generate_sim_reading(tag,anchors,noise):
	
	res="";
	dummy_anchors=anchors.keys()
	
	for a_id in dummy_anchors:
		norm=dist(tag["pos"],anchors[a_id])
		
		res+= str(a_id)
		res+= " ";
		res+= str("%.2f" % noise(norm))
		res+=" ";
	res+=tag["tag_id"]
	res+=" 000000\n\r"
	
	return res
	

	
class Driver(object):
	def __init__(self, port=None):
		
		self.buffer=''
		self.anchors=[uwb(),uwb(),uwb()]
		self.tag_id=''
		self.current_ts=0
		
		
			
		if(port is not None):
			try:
				self.ser = serial.Serial(port, 230400, timeout=0)
			except serial.SerialException:
				print("error")
			
			self.offline=False
		else:
			self.offline=True
	
				
	def set_sim_data(self,options):
		global readings
		readings=generate_sim_reading(options["tag"],options["anchors"],options["noise"])
			
	def calib_dist(self,dist_list):
		x=numpy.array(dist_list)
		ms=numpy.array([1.0529,1.0518,1.0495])
		bs=numpy.array([0.1871,0.3231,0.1465])
		return numpy.dot(numpy.identity(3)*ms,x)+bs
	
	
	def read_values(self):
		global readings
		read_val=''
		if(self.offline==False):
			if(self.ser.inWaiting()>0):
				read_val = self.ser.read(self.ser.inWaiting())
			self.buffer+=read_val
			command=self.findcommand()
			if(command !=False):
				values=command.split(" ")
				for i in range(0,3):
					self.anchors[i].anchor_id=str(values[i*2])
					self.anchors[i].radius=float(values[(i*2)+1])
				self.tag_id=str(values[6])
				self.current_ts=int(values[7],16)
		else:
			if(len(readings)>0):
				values=readings.split(" ")
				for i in range(0,3):
					self.anchors[i].anchor_id=str(values[i*2])
					self.anchors[i].radius=float(values[(i*2)+1])
				self.tag_id=str(values[6])
				self.current_ts=int(values[7],16)
			
			
	def get_current_timestamp(self):
		return self.current_ts			
	
	def start_aquire(self):
		try:
			thread.start_new_thread( self.read_values, () )
		except:
			print "Error: unable to start thread"
	
	def get_tag_id(self):
			return self.tag_id
			
	def get_anchors(self,num):
		if(len(self.anchors)>=num):
			return self.anchors
		else:
			return False
			
	def findcommand(self):
		count=self.buffer.count('\n\r')
		if count>1:
			start_command_index=self.buffer[:self.buffer.rfind('\n\r')].rfind('\n\r')
			end_command_index=self.buffer.rfind('\n\r')
			command=self.buffer[start_command_index+2:end_command_index]
			self.buffer=self.buffer[end_command_index:]
			return command
		return False

	def stop_aquire(self):
		self.ser.write('b\n')

	def plotvalue(self):
		for i in range(0,2):
			print("anchor["+str(self.ids[i])+"]: "+str(self.values[i]))
