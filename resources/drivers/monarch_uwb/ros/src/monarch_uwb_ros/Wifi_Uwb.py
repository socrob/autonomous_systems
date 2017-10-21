'''
Created on 20 de Fev de 2014

@author: Fialho
'''

import numpy
import thread
from monarch_uwb.msg import uwb
import math
from random import random
import mean
import time
import socket





readings="";




	

	
class Driver(object):
	def __init__(self, ip=None):
		
		self.buffer=''
		self.anchors=[uwb(),uwb(),uwb()]
		self.tag_id=''
		self.current_ts=0
		try:
			self.clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.clientsocket.connect((ip, 8890))
		except:
			raise Exception('No connection')
	
	def close(self):
		self.clientsocket.close()
		
	def read_values(self):
		
		
		
		try:
			read_val = self.clientsocket.recv(200)
			self.buffer+=read_val
			command=self.findcommand()
			print "command"
			print command
			if(command !=False):
				values=command.split(" ")
				for i in range(0,3):
					self.anchors[i].anchor_id=str(values[i*2])
					self.anchors[i].radius=float(values[(i*2)+1])
				self.tag_id=str(values[6])
				self.current_ts=int(values[7],16)
		except:
			raise Exception('No connection')
				
	def get_tag_id(self):
			return self.tag_id
	
	def get_current_timestamp(self):
		return self.current_ts	
		
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
