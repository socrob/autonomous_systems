import numpy as np


class stack_var(object):
	def __init__(self, window):
		self.init_dists=[0]*window
		self.acc=window
	def update(self,dist):
		self.init_dists.append(dist)
		self.init_dists.pop(0)
		return np.var(self.init_dists)
		
			
