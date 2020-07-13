#!/usr/bin/env python
#* Team Id : SR#7147
#* Author List : Ravikumar Chaurasia
#* Filename: roi_detector.py
#* Theme: Survay And Rescue
#* Functioning : This script detects and store the ROI with their cell/block name. We used the pickle module to store ROI arrays, with name "rect_info.pkl"
# 				 We did this using OpenCV, detected the square(quadilateral like square by checking the ratio of w and h) by setting the threshold of area.

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import contours
import pickle
import imutils
import copy
import numpy as np
import itertools

class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
		self.cnts = None
		self.block_name = ["A6","A5","A4","A3","A2","A1","B6","B5","B4","B3","B2","B1","C6","C5","C4","C3","C2","C1","D6","D5","D4","D3","D2","D1","E6","E5","E4","E3","E2","E1","F6","F5","F4","F3","F2","F1"]
		self.detec_node = []
		self.contours_arr = []
		self.contours_x_y = []
		self.i=0
	
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


	'''You will need to implement an image processing algorithm to detect the Regions of Interest (RoIs)
	The standard process flow is:
	i)		Standard pre-processing on the input image (de-noising, smoothing etc.)
	ii)		Contour Detection
	iii)	Finding contours that are square, polygons with 4 vertices
	iv)		Implementing a threshold on their area so that only contours that are the size of the cells remain'''
	def detect_rois(self):
		img_copy=self.img
		blurred = cv2.GaussianBlur(img_copy, (5, 5), 0)
		edged = cv2.Canny(blurred, 120, 255, 1)

		self.cnts = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)

		threshold_max_area = 7000
		threshold_min_area = 4000

		(self.cnts, boundingBoxes) = contours.sort_contours(self.cnts, method="left-to-right")

		for c in self.cnts:
		    peri = cv2.arcLength(c, True)
		    approx = cv2.approxPolyDP(c, 0.035 * peri, True)
		    (x, y, w, h) = cv2.boundingRect(approx)
		    aspect_ratio = w / float(h)
		    area = cv2.contourArea(c)
		    print(area) 
		    if area < threshold_max_area and area > threshold_min_area :
		        cv2.drawContours(self.img,[c], 0, (0,0,0), 2)
		        cv2.rectangle(self.img, (x, y), (x+w,y+h), (0,255,0), 2)
		        self.contours_x_y=[(x,y),(x+w, y+h)]
		        self.detec_node.append(c)
		        self.draw_cell_names(img_copy, c, self.i)
        		self.contours_arr.append(self.contours_x_y)
        		self.i+=1


		# Add your Code here
		# You may add additional function parameters
		print("Detected Contours :", self.detec_node)
		cv2.imshow("Detected ROIs", img_copy) #A copy is chosen because self.img will be continuously changing due to the callback function
		cv2.waitKey(100)


	'''	Please understand the order in which OpenCV detects the contours.
		Before saving the files you may sort them, so that their order corresponds will the cell order
		This will help you greatly in the next part. '''
	def sort_rois(self):
		(self.cnts, boundingBoxes) = contours.sort_contours(self.cnts, method="left-to-right")
		
		# Add your Code here
	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via raw_input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	'''	You may save the list using anymethod you desire
	 	The most starightforward way of doing so is staright away pickling the objects
		You could also save a dictionary as a json file, or, if you are using numpy you could use the np.save functionality
		Refer to the internet to find out more '''
	def save_rois(self):
		pose_dict = dict(zip(self.block_name, self.contours_arr))
		pickle.dump(pose_dict, open("/home/ravikrcsia/catkin_ws/src/survey_and_rescue/scripts/rect_info.pkl", "wb"))
		print("Recording Successful")
		#Add your code here

	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self, img, c, i):
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
 
	#draw the countour number on the image
		cv2.putText(img, "{}".format(self.block_name[self.i]), (cX - 20, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
 
	#return the image with the contour number drawn on it
		return img

def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while True:
			if r.img is not None:
				r.detect_rois()
				if(len(r.detec_node)!=36):
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change ##Enter tweaks, this is not necessary##?")
					if(new_thresh_flag):
						continue#Change settings as per your desire
					else:
						exit(0)
				else:
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					#else:
						#Change more settings
		#r.draw_cell_names(r.img) # Again, this is optional
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
