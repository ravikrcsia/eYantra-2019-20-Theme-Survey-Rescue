#!/usr/bin/env python
#* Team Id : SR#7147
#* Author List : Ravikumar Chaurasia
#* Filename: beacon_detector.py
#* Theme: Survay And Rescue
#* Functioning : This script acting as a node to detect the beacon color and publish the message on topic /detection_info with other datas.
#				 It takes the ROI file and does the detection only to that specific 36 regions or we can only check for 8 position provided by the eYantra.
#				 It also check for the status of the service which will be either successful or fail, after which it will change its detection.

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
import imutils
import copy
import time

class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.det_time_msg=SRInfo()
		self.det_lit_msg=SRInfo()
		self.bridge = CvBridge()

		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10)
		self.det_time = rospy.Publisher('/det_time', SRInfo, queue_size=10)
		self.det_lit = rospy.Publisher('/det_lit', SRInfo, queue_size=10)

 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)

 		self.z=[]
 		self.l=[]
 		#self.x_y=[]
		self.contours_arr=[]
		self.img = None
		self.frame1 = None
		self.lit_st=[0]*36
		#self.lit_led=['']*36
		self.lit_det=['']*36

	def load_rois(self, file_path = '/home/ravikrcsia/catkin_ws/src/survey_and_rescue/scripts/rect_info.pkl'):
		try:
			# s.rois = np.load("rois.npy")
			with open(file_path, 'rb') as input:
   				self.rect_list = pickle.load(input)
   			self.z=tuple(self.rect_list)
			self.l=sorted(self.z, key=str)

			for i in range(36):
				self.contours_arr.append(self.rect_list[self.l[i]])

		except IOError, ValueError:
			print("File doesn't exist or is corrupted")

 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)

 	def serviced_callback(self, msg):
 		pass
 		
	def detect_color_contour_centers(self):

		for i in range(36):
			frame=self.img.copy()

			self.frame1=frame[((self.contours_arr[i])[0])[1]:((self.contours_arr[i])[1])[1], ((self.contours_arr[i])[0])[0]:((self.contours_arr[i])[1])[0]] 

			hsv = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2HSV)

			#cv2.rectangle(self.img, (self.contours_arr[i])[0], (self.contours_arr[i])[1], (0,255,0), 2)
			red_lower=np.array([136,87,140])
			red_upper=np.array([180,255,255])

			#defining the Range of Blue color
			blue_lower=np.array([110,110,110])
			blue_upper=np.array([130,255,255])
			
			#defining the Range of yellow color
			green_lower=np.array([20,100,150])
			green_upper=np.array([90,255,255])

			self.red=cv2.inRange(hsv, red_lower, red_upper)
			self.blue=cv2.inRange(hsv,blue_lower,blue_upper)
			self.green=cv2.inRange(hsv,green_lower,green_upper)

			
			if(self.red.any()>self.blue.any() and self.red.any()>self.green.any() and (self.lit_det[i]!='RESCUE' and self.lit_st[i]==1)):
					#if(self.detect_info_msg.info!=self.l[i]):
				self.detect_info_msg.location = self.l[i]
				self.detect_info_msg.info = 'RESCUE'
				print ('RED at :', self.l[i])
				self.detect_pub.publish(self.detect_info_msg)

				self.lit_det[i]='RESCUE'

				self.det_time_msg.location = self.l[i]
				self.det_time_msg.info=str(time.time()+10)
				self.det_time.publish(self.det_time_msg)

			elif(self.blue.any()>self.red.any() and self.blue.any()>self.green.any() and (self.lit_det[i]!='MEDICINE' and self.lit_st[i]==1)):
				self.detect_info_msg.location = self.l[i]
				self.detect_info_msg.info = 'MEDICINE'
				print ('BLUE at :', self.l[i])

				self.detect_pub.publish(self.detect_info_msg)
				self.lit_det[i]='MEDICINE'

				self.det_time_msg.location = self.l[i]
				self.det_time_msg.info=str(time.time()+30)
				self.det_time.publish(self.det_time_msg)

			elif(self.green.any()>self.red.any() and self.green.any()>self.blue.any() and (self.lit_det[i]!='FOOD' and self.lit_st[i]==1)):
				self.detect_info_msg.location = self.l[i]
				self.detect_info_msg.info = 'FOOD'
				print ('GREEN at :', self.l[i])
	
				self.detect_pub.publish(self.detect_info_msg)
				self.lit_det[i]='FOOD'

				self.det_time_msg.location = self.l[i]
				self.det_time_msg.info=str(time.time()+30)
				self.det_time.publish(self.det_time_msg)

		#cv2.imshow('Real_one', self.img)
		#cv2.imshow('Real_two', self.frame1)
	
	def check_whether_lit(self):

		for i in range(36):
			frame=self.img.copy()

			self.frame1=frame[((self.contours_arr[i])[0])[1]:((self.contours_arr[i])[1])[1], ((self.contours_arr[i])[0])[0]:((self.contours_arr[i])[1])[0]] 

			hsv = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2HSV)

			#cv2.rectangle(self.img, (self.contours_arr[i])[0], (self.contours_arr[i])[1], (0,255,0), 2)
			red_lower=np.array([136,87,140])
			red_upper=np.array([180,255,255])

			#defining the Range of Blue color
			blue_lower=np.array([110,110,110])
			blue_upper=np.array([130,255,255])
			
			#defining the Range of yellow color
			green_lower=np.array([20,100,150])
			green_upper=np.array([90,255,255])

			self.red=cv2.inRange(hsv, red_lower, red_upper)
			self.blue=cv2.inRange(hsv,blue_lower,blue_upper)
			self.green=cv2.inRange(hsv,green_lower,green_upper)

			if(self.red.any()!=0 or self.blue.any()!=0 or self.green.any()!=0):
				self.lit_st[i]=1#ON case

				self.det_lit_msg.location = self.l[i]
				self.det_lit_msg.info='1'
				self.det_lit.publish(self.det_lit_msg)
			else:
				self.lit_st[i]=0#OFF case

				self.lit_det[i]=''

				self.det_time_msg.location = self.l[i]
				self.det_time_msg.info='0'
				self.det_time.publish(self.det_time_msg)

				self.det_lit_msg.location = self.l[i]
				self.det_lit_msg.info='0'
				self.det_lit.publish(self.det_lit_msg)

		
		#self.detect_info_msg.location = ''
		#self.detect_info_msg.info = ''

def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(30)
		# rate = rospy.Rate(5)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.check_whether_lit()
			s.detect_color_contour_centers()
			#if(s.detect_info_msg.location != '' and s.detect_info_msg.info!=''):
			
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
