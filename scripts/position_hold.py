#!/usr/bin/env python
#* Team Id : SR#7147
#* Author List : Ravikumar Chaurasia
#* Filename: position_hold.py
#* Theme: Survay And Rescue
#* Functioning : This script acting as a node to control the PlutoX drone, it takes the commend from schedule.py(Scheduling NODE) like where to go in a given sequence.
#				 Here we used dynamic PID constant(Kp, Kd, Ki) for BASE, EDGEs, REMAINING location. Even we added a landing funtion for the drone at the base station.
#				 We take the "cell_coords.json", to get the cell location and LED_OrgConfig.tsv to get to know the base.

'''
This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from survey_and_rescue.msg import *
import rospy
import time, sys
import json
import csv


class Edrone():

	def __init__(self):
		
		rospy.init_node('drone_control')
		rospy.Subscriber('/decision_info',SRInfo,self.setpoint)
		# initializing ros node with name drone_control

		with open("cell_coords.json", mode='r') as read_file:
			self.data = json.load(read_file) #To store .json data

		tsvdata=[]	

		with open('/home/ravikrcsia/catkin_ws/src/survey_and_rescue/scripts/LED_OrgConfig.tsv', 'r') as tsv_in:
			tsvreader=csv.reader(tsv_in, delimiter='\t')

			for record in tsvreader:
				tsvdata.append(record)

		for i in range(len(tsvdata)):
			if ((tsvdata[i])[1]=='BASE'):
				self.base=(tsvdata[i])[0]
				self.base_cell=self.data[self.base]

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint_cell=self.base
		self.setpoint = self.data[self.setpoint_cell] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type <strong>edrone_msgs</strong> and initializing values
		self.cmd = edrone_msgs()
		self.error_msg = Float64()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and
		#computing correscponding PID parameters, change the parameters

		#self.Kp = [55,55,270]#[17.25,17.25,190.08] #[9,13,90]
		#self.Ki = [0,0,0]#[0,0,0]#[0,0.008,0]
		#self.Kd = [25.5,23.5,5]#[10,10,0]#[21.8,21.5,15]

		#Insted of using static PID constant, we used dynamic PID constant.

		self.prev_error = [0.0,0.0,0.0]
		self.current_time = time.time()
		self.previous_time = time.time()
		self.max_values = [1535,1535,2000]
		self.min_values = [1000,1000,1000]
		self.error = [0.0,0.0,0.0]
		self.cumulative_error = 0.0
		self.de = [0.0,0.0,0.0]
		self.add_error = [0.0,0.0,0.0]

		self.timer_on=0
		self.counter=0
		self.wait_count=0
		#-----------------------Add other required variables for pid here ----------------------------------------------
		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		
		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds

		self.sample_time = 0.010
		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.error_pub_alt = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.error_pub_pit = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.error_pub_rol = rospy.Publisher('/roll_error', Float64, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		#-----------------------------------------------------------------------------------------------------------
		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, /pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		#------------------------------------------------------------------------------------------------------------
		self.arm() # ARMING THE DRONE
	# Disarming condition of the drone
	def setpoint(self,msg):
		self.setpoint_cell=msg.location
		self.setpoint = self.data[self.setpoint_cell]

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)

	def land(self):
		for throttle in range(1500, 1400, -10):
		   self.cmd.rcThrottle = throttle
		   self.cmd.rcRoll = 1500
		   self.cmd.rcYaw = 1500
		   self.cmd.rcPitch = 1500
		   self.cmd.rcAUX4 = 1100
		   self.command_pub.publish(self.cmd)
		   rospy.sleep(0.00001)
		self.disarm()
	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		#---------------------------------------------------------------------------------------------------------------
	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pit):
		self.Kp[1] = pit.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pit.Ki * 0.008
		self.Kd[1] = pit.Kd * 0.3

	def roll_set_pid(self,rol):
		self.Kp[0] = rol.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = rol.Ki * 0.008
		self.Kd[0] = rol.Kd * 0.3
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):

		if(self.l=='E4'):#For BASE !!!
			self.Kp = [19.55,18.55,180] #[9,13,90]
			self.Ki = [-0.026,-0.043,0.017]#[0,0.008,0]
			self.Kd = [29,30,28.8]#[21.8,21.5,15]
		else:#For Else PLACE !!
			self.Kp = [20.55,22.5,180] #[9,13,90]
			self.Ki = [-0.024,-0.033,0.017]#[0,0.008,0]
			self.Kd = [28.7,29,28.8]#[21.8,21.5,15]

			#Here note that we can even use another PID constant value for edge cell, so that we can control the drone with ease on the edge.

		self.current_time = time.time()

		if self.sample_time < self.current_time - self.previous_time:

		    self.error[0] = self.drone_position[0] - self.setpoint[0]
		    self.error[1] = self.setpoint[1] - self.drone_position[1]
		    self.error[2] = self.setpoint[2] - self.drone_position[2]

		    self.error_msg.data = self.error[0]
		    self.error_pub_rol.publish(self.error_msg)

		    self.error_msg.data = self.error[1]
		    self.error_pub_pit.publish(self.error_msg)

		    self.error_msg.data = self.error[2]
		    self.error_pub_alt.publish(self.error_msg)
	
		    
		    #to compute output.
		    dt = self.current_time - self.previous_time

		    self.de[0] = self.error[0] - self.prev_error[0]
		    self.de[1] = self.error[1] - self.prev_error[1]
		    self.de[2] = self.error[2] - self.prev_error[2]

		    self.add_error[0] = self.add_error[0] + self.error[0] * dt
		    self.add_error[1] = self.add_error[1] + self.error[1] * dt
		    self.add_error[2] = self.add_error[2] + self.error[2] * dt


		    self.out_roll = self.Kp[0] * self.error[0] + self.Ki[0] * self.add_error[0] + self.Kd[0] * (self.de[0] / dt)
		    self.out_pitch = self.Kp[1] * self.error[1] + self.Ki[1] * self.add_error[1] + self.Kd[1] * (self.de[1] / dt)
		    self.out_altitude = self.Kp[2] * self.error[2] + self.Ki[2] * self.add_error[2] + self.Kd[2] * (self.de[2] / dt)

		    self.cmd.rcRoll = 1500 - self.out_roll
		    self.cmd.rcPitch = 1500 - self.out_pitch
		    self.cmd.rcThrottle = 1500 - self.out_altitude

		    if self.cmd.rcRoll > self.max_values[0]:
		    	self.cmd.rcRoll = self.max_values[0]
		    	

		    if self.cmd.rcPitch > self.max_values[1]:
		    	self.cmd.rcPitch = self.max_values[1]
		    	

		    if self.cmd.rcThrottle > self.max_values[2]:
		    	self.cmd.rcThrottle = self.max_values[2]
		   	 

		    self.prev_error[0] = self.error[0]
		    self.prev_error[1] = self.error[1]
		    self.prev_error[2] = self.error[2]

		    self.previous_time = self.current_time
		

		    #-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calculate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#   6. Limit the output value and the final command value between the maximum(<strong>2000</strong>) and minimum(<strong>1000</strong>)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)
		
	def status(self):

		sys.stdout.write('\rSetpoint :'+'{}'.format(self.setpoint_cell))
		sys.stdout.flush()


if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(70)
	print '\n'#specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		e_drone.status()
		r.sleep()
