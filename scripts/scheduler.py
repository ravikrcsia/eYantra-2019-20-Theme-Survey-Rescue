#!/usr/bin/env python
#from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from std_msgs.msg import Float64
import csv
import json
import math
import time

class sr_scheduler():

	def __init__(self):
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr',SRDroneStats,self.food_med_count)
		rospy.Subscriber('/det_time',SRInfo,self.det_time)
		rospy.Subscriber('/det_lit',SRInfo,self.det_lit)

		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)

		self.decided_msg = SRInfo()
		
		self.base=''
		self.base_cell=[]

		self.sorted_cell=[]
		self.info_data = ['']*8

		self.dete_time_data = [0]*8
		self.dete_lit_data = ['']*8

		self.status=''
		self.st_cell=''

		self.res_flag=1
		self.flag=1
		self.p=1
		self.start_time=0
		self.timer_re=time.time()

		self.food=3
		self.medicine=3

		self.closed_cell=[]
		self.temp_sorted=['']*8

		self.new_flag=0

		self.selected_cell_list=[]

	def food_med_count(self,msg):
		self.food=msg.foodOnboard
		self.medicine=msg.medOnboard

		#print 'food :', self.food
		#print 'medicine :', self.medicine

	def sort_cell(self):
		tsvdata=[]
		cell=[]
		dis=[]

		with open("/home/ravikrcsia/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json", mode='r') as read_file:
			self.data = json.load(read_file) #To store .json data

		l=sorted(tuple(self.data), key=str)

		with open('/home/ravikrcsia/catkin_ws/src/survey_and_rescue/scripts/LED_OrgConfig.tsv', 'r') as tsv_in:
			tsvreader=csv.reader(tsv_in, delimiter='\t')

			for record in tsvreader:
				tsvdata.append(record)

		for i in range(len(tsvdata)):
			if ((tsvdata[i])[1]=='BASE'):
				self.base=(tsvdata[i])[0]
				self.base_cell=self.data[self.base]

		self.closed_cell=self.base_cell
				
		print 'BASE :',self.base
		print 'BASE LOCATION :',self.base_cell
		#print self.det_time(self)

		for i in range(len(tsvdata)-1):
			cell.append((tsvdata[i])[0])
			x=(self.data[(tsvdata[i])[0]])[0]
			y=(self.data[(tsvdata[i])[0]])[1]
			dis.append(round(self.cal_dis(x,y),2))

		pose_dis=dict(zip(dis,cell))
		m=sorted(tuple(pose_dis), key=float)

		for i in range(len(tsvdata)-1):
			self.sorted_cell.append(pose_dis[m[i]])

		print(self.sorted_cell)

		self.info=dict(zip(self.sorted_cell, self.info_data))
		self.dete_lit=dict(zip(self.sorted_cell, self.dete_lit_data))
		self.dete_time=dict(zip(self.sorted_cell, self.dete_time_data))

		self.selected_cell_list=self.sorted_cell

		#self.dyn_dis_cal('C5')

	def cal_dis(self,x2,y2):
		dist = math.sqrt((x2 - self.closed_cell[0])**2 + (y2 - self.closed_cell[1])**2)
		return dist

	def dyn_dis_cal(self, clo):
		cell=[]
		dis=[]

		self.closed_cell=self.data[clo]

		self.temp_sorted[0]=clo

		for i in self.sorted_cell:
			if(i!=clo):
				cell.append(i)
				x=(self.data[i])[0]
				y=(self.data[i])[1]
				dis.append(self.cal_dis(x,y))

		pose_dis=dict(zip(dis,cell))
		m=sorted(tuple(pose_dis), key=float)

		for i in range(1, len(self.sorted_cell)):
			self.temp_sorted[i]=pose_dis[m[i-1]]
			print(self.temp_sorted)

		print(self.temp_sorted)

	def det_time(self,msg):

		if(msg.location in self.sorted_cell):
			self.dete_time[msg.location]=float(msg.info)

			#count=self.sorted_cell.index(msg.location)
			#self.dete_time[count]=float(msg.info)

	def det_lit(self,msg):

		info=[]
		dete_time=[]
		dete_lit=[]

		if(msg.location in self.sorted_cell):
			self.dete_lit[msg.location]=msg.info

			if(self.dete_lit[msg.location]=='0'):
				self.info[msg.location]=''

			#print(self.sorted_cell)
			#print(self.info)
			#print(self.dete_lit)

			for i in self.sorted_cell:
				info.append(self.info[i])
				dete_lit.append(self.dete_lit[i])
				dete_time.append(self.dete_time[i])

			pose_dict = list(zip(self.sorted_cell, info, dete_lit, dete_time))
			print('===')
			for i in pose_dict:
				print(i)
			print('===')
			print(self.selected_cell_list)

			print 'self.res_flag  :', self.res_flag
			print 'self.flag :', self.flag
			#print 'food', self.food
			#print 'medicine', self.medicine

	def detection_callback(self, msg):

		if(msg.location in self.sorted_cell):
			self.info[msg.location]=msg.info
		
	def cont_detection_beacon(self, cells):

		for i in cells:
			if((self.info[i]=='RESCUE' and self.res_flag==1 and self.dete_lit[i]=='1' and self.dete_time[i]>=time.time()+7) or (self.info[i]=='RESCUE' and self.flag==0 and self.res_flag==1 and self.dete_lit[i]=='1' and self.dete_time[i]>=time.time()+7)):
				self.decided_msg.location = i
				self.decided_msg.info = self.info[i]
				self.decision_pub.publish(self.decided_msg)
				self.res_flag=0

				self.dyn_dis_cal(i)
				self.selected_cell_list=self.temp_sorted
				self.new_flag=1

			elif(self.food==0 and self.medicine==0 and self.res_flag==1 and self.flag==1):
				self.decided_msg.location = self.base
				self.decided_msg.info = 'BASE'
				self.decision_pub.publish(self.decided_msg)
				self.res_flag=0
				self.flag=0

				self.selected_cell_list=self.sorted_cell


			elif(self.flag==1 and self.food!=0 and self.info[i]=='FOOD' and self.res_flag==1 and self.dete_lit[i]=='1' and self.dete_time[i]>=time.time()+5):
				self.decided_msg.location = i
				self.decided_msg.info = 'FOOD'
				self.decision_pub.publish(self.decided_msg)
				self.flag=0

				self.dyn_dis_cal(i)
				self.selected_cell_list=self.temp_sorted
				self.new_flag=1


			elif(self.flag==1 and self.medicine!=0 and self.info[i]=='MEDICINE' and self.res_flag==1 and self.dete_lit[i]=='1' and self.dete_time[i]>=time.time()+5):
				self.decided_msg.location = i
				self.decided_msg.info = 'MEDICINE'
				self.decision_pub.publish(self.decided_msg)
				self.flag=0

				self.dyn_dis_cal(i)
				self.selected_cell_list=self.temp_sorted
				self.new_flag=1

		if(self.new_flag==1):	
			if(self.dete_lit[self.decided_msg.location]=='0' and self.decided_msg.info=='RESCUE'):
				self.flag=1
				self.res_flag=1
			elif(self.dete_lit[self.decided_msg.location]=='0' and (self.decided_msg.info=='RESCUE' or self.decided_msg.info=='MEDICINE')):
				self.flag=1

#REMEMBER TO REPLESIH THE FOOD AND MEDICINE

		#self.decided_msg.location = msg.location
		#self.decided_msg.info = msg.info
		#self.decision_pub.publish(self.decided_msg)

	def serviced_callback(self,msg):
		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc

		if(self.decided_msg.info=='RESCUE' and (msg.info=='SUCCESS' or msg.info=='FAILURE') and msg.location==self.decided_msg.location):
			self.res_flag=1
			self.flag=1
		elif((self.decided_msg.info=='FOOD' or self.decided_msg.info=='MEDICINE') and (msg.info=='SUCCESS' or msg.info=='FAILURE') and msg.location==self.decided_msg.location):
			self.flag=1
		elif(self.decided_msg.info=='BASE' and msg.info=='SUCCESS'and msg.location==self.base):
			self.res_flag=1
			self.flag=1
		elif(msg.info=='END'):
			self.decided_msg.location = self.base
			self.decided_msg.info = 'BASE'
			self.decision_pub.publish(self.decided_msg)

	def shutdown_hook(self):
		# This function will run when the ros shutdown request is recieved.
		# For instance, when you press Ctrl+C when this is running
		pass

		#For this we can first hover above the base and then disarm (OR) we can pass a co-ordinate of base with decrease in z-axis and then disarm



def main(args):
	
	sched = sr_scheduler()
	sched.sort_cell()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		sched.cont_detection_beacon(sched.selected_cell_list)
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)