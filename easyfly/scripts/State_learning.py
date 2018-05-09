#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
#from std_msgs.msg import float32
from geometry_msgs.msg import Twist
import numpy as np
from easyfly.msg import pos_ctrl_sp
from easyfly.msg import att_est
from easyfly.msg import pos_est
from easyfly.srv import Learning

class Learning_Cfs():
	def __init__(self,group_index,n_S0,n_S1,n_S2):
		self.S1_posEststamp = []
		self.S1_pos_est = []
		self.S1_vel_est = []
		self.isBeginS1 = False
		self.Findvel_diff = False
		self.Findatt_diff = False
		self.isFirstReceive = True
		self.T1 = 0
		self.T2 = 0
		self.vel_end = np.array( [0.0,0.0,0.0] )
		self.att_end = np.array( [0.0,0.0,0.0] )
		self.vel_temp = np.array( [0.0,0.0,0.0] )
		self.att_temp = np.array( [0.0,0.0,0.0] )
		self.isNextPeriod = False
		self.count_train = 0
		self.group_index = group_index
		self.isFirstatt = True
		self.Startpoint = rospy.get_rostime()
		self.n_S0 = n_S0 #number of trails for state 1 startpoint 
		self.n_S1 = n_S1 #number of trails for state 1 end_vel training
		self.n_S2 = n_S2 #number of trails for state 2 roll angle training
		msg_name = "/vehicle%d/pos_est" %self.group_index
		self.sub_posest = rospy.Subscriber(msg_name, pos_est, self.pos_estCallback)
		msg_name = "/vehicle%d/att_est" %self.group_index
		self.sub_attest = rospy.Subscriber(msg_name, att_est, self.attCallback)
		msg_name = "/vehicle%d/pos_ctrl_sp" %self.group_index
		self.pos_ctrl = rospy.Subscriber(msg_name, pos_ctrl_sp, self.pos_ctrlCallback)
		msg_name = "/vehicle%d/vel_difference" %self.group_index
		self.pub_d_vel = rospy.Publisher(msg_name, Vector3, queue_size=2)
		msg_name = "/vehicle%d/att_difference" %self.group_index
		self.pub_d_att = rospy.Publisher(msg_name, Vector3, queue_size=2)
		rospy.spin()
	def pos_ctrlCallback(self, data):
		if(data.State_index == 0):
			pass
		elif(data.State_index == 1):
			if (self.isFirstReceive):
   				self.count_train = data.count_training
		   	   	self.isBeginS1 = True
		   	   	self.T1 = data.T
		   	   	self.vel_end[0] = data.vel_end.x
		   	   	self.vel_end[1] = data.vel_end.y
		   	   	self.vel_end[2] = data.vel_end.z
				self.isFirstReceive = False
				self.Startpoint = data.header.stamp
			elif (data.count_training == self.count_train + 1):  #next training
		   			self.count_train = data.count_training
					self.Startpoint = data.header.stamp
					self.Findvel_diff = False

		elif (data.State_index == 2): #State2
			self.vel_temp = self.vel_end - np.array([self.S1_vel_est[self.count_train].x,self.S1_vel_est[self.count_train].y,self.S1_vel_est[self.count_train].z])
			vel_diff = Vector3
			vel_diff.linear.x = vel_temp[0]
			vel_diff.linear.y = vel_temp[1]
			vel_diff.linear.z = vel_temp[2]
			self.pub_d_vel.publish(vel_diff)

			self.att_end[0] = data.roll_sp
			self.att_end[1] = data.pitch_sp
			self.att_end[2] = data.yaw_sp
				
		elif (data.State_index == 3): #State3
			self.att_temp = self.vel_end - np.array([self.S2_att_est[self.count_train].x,self.S2_att_est[self.count_train].y,self.S2_att_est[self.count_train].z])
			att_diff = Vector3
			att_diff.linear.x = att_temp[0]
			att_diff.linear.y = att_temp[1]
			att_diff.linear.z = att_temp[2]
			self.pub_d_att.publish(att_diff)

			self.vel_end[0] = data.vel_end.x
			self.vel_end[1] = data.vel_end.y
		   	self.vel_end[2] = data.vel_end.z

			isFirstReceive = True

	def pos_estCallback(self, data):
		if (self.Findvel_diff == False):
			temp_stamp =  data.header.stamp
			seconds = (temp_stamp - self.Startpoint).to_sec() #floating point
			if (self.T1*0.99 < seconds < self.T1*1.01):
				self.S1_pos_est.append(data.pos_est)
				self.S1_vel_est.append(data.vel_est)
				self.S1_posEststamp.append(temp_stamp)
				Findvel_diff = True
			
	def attCallback(self, data):
		if(self.isFirstatt): 
			self.starttime = data.header.stamp
			self.isFirstatt = False
		else:
			'''now = rospy.get_rostime()
			duration = (now - self.starttime).to_sec()
			sentence = "Att_got from learning.py:  %f, time passed: %f  " %(data.att_est.x,duration)
			print sentence'''
			if (self.Findatt_diff == False):
				temp_stamp =  data.header.stamp
				seconds = (temp_stamp - self.Startpoint).to_sec() #floating point
				if (self.T2*0.99 < seconds < self.T2*1.01):
					self.S2_att_est.append(data.att_est)
					self.S2_attEststamp.append(temp_stamp)
					print "-------------- Att_got from learning.py --------------" 
					att_diff = Twist()
					att_diff.linear.x = att_temp[0]
					att_diff.linear.y = att_temp[1]
					att_diff.linear.z = att_temp[2]
					self.pub_d_att.publish(att_diff)

def start_learning(req):
	learning = Learning_Cfs(req.group_index,req.numberOfTrailsS0,req.numberOfTrailsS1,req.numberOfTrailsS2)

def learning_Server():
	rospy.init_node('cf_learning')
	s = rospy.Service('/learning_service', Learning, start_learning)
	print "Ready to learn."
	rospy.spin()
if __name__ == '__main__':
	learning_Server()