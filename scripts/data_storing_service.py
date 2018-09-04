#!/usr/bin/env python

import roslib #Ros libraries
import rospy #Python library
import rosbag #To store data
import numpy #To change variables types
import time #To set ROSBAG timing
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg
import geometry_msgs.msg
from user_srv.srv import *# My personal service
from std_msgs.msg import Int32, String, Bool
from time import gmtime, strftime # Library to get the date and time for data saving with ROSBAG
import math
import rospkg #To get the path of the Robag directory in the package


class data_storer():

	def __init__(self):

################################CLASS INITIALISATION#############################
		#Subscriber that receives data from the ORBBEC SDK
		self.skel_sub = rospy.Subscriber(
			"/body_tracker/skeleton",
			body_tracker_msgs.msg.Skeleton,
			self.positionCallback) #Name // Type of message and Callback

		#Service node initialisation (Used for writing data in bag and txt files)
		self.service = rospy.Service(
			"recorder",
			UserService,
			self.control_loop)

		# Skeletal body tracker addresses to access joint data

		self.Attributes = [
			"joint_position_head",
			"joint_position_neck",
			"joint_position_shoulder", #Always 0 by default
			"joint_position_spine_top",
			"joint_position_spine_mid",
			"joint_position_spine_bottom",
			"joint_position_left_shoulder",
			"joint_position_left_elbow",
			"joint_position_left_hand",
			"joint_position_right_shoulder",
			"joint_position_right_elbow",
			"joint_position_right_hand"]

	# Global class variables

		self.rospack = rospkg.RosPack() #Initialise ROS function for directory searching
		self.BagPath = self.rospack.get_path('master_dissertation') +"/experiment_bags/" #Preferred location to save data in the Project
		#self.counter = 0 #
		self.BodyID = -1 #Initialise user bodyID
		self.WritingTime=rospy.Time.now() #Pointer to write in the right plaec in the ROSBAG
		self.StartingWritingTime = rospy.Time.now()
		self.WritingFlag = False #To know if the files are being written
		self.Setup = False
		self.OpenBag = False # To know when to start writing

###################################CALLBACK METHOD###################################
    #Function to be run when some data arrives from the camera SDK

	def positionCallback(self, data):

		##########Saving data into a BAG file and TXT files############

		#If the bag was open and is ready to write...
		if self.OpenBag:
			if data.body_id == self.BodyID: #AND the information that arrives comes from the body detected by main code
		#Start ready and saving data

				self.WritingFlag = True #Let the other methods know that information is being written
				Skel = body_tracker_msgs.msg.Skeleton() #Object created from message type to keep incoming data
				Skel = data #Save new skeletal data in Skel
				#####BAG SAVING######
				self.bag.write("/body_tracker/skeleton",Skel, rospy.Time.now() - self.WritingTime + self.StartingWritingTime) #Write all data on the bag joining the queue of information saved
				#####TXT SAVING######
				for BodyPart in self.Attributes: #For each bodypart (JOINT) save its three coordinates one after the other
					self.text.write("%f,%f,%f\t" % ((getattr(getattr(data,BodyPart),"x")),getattr(getattr(data,BodyPart),"y"),getattr(getattr(data,BodyPart),"z")))
				self.text.write("%i" % len(self.Attributes)) #Save the number of joint that have been saved
				self.text.write("\n")
				rospy.loginfo("recording")
				self.WritingFlag = False #Let the other methods know that writing finished

	def control_loop(self,req):

		self.BodyID = req.BodyID # Update the body that we want to record

		#Bag opening and writing
		time.sleep(1)
		#Create a NEW bag if needed OTHERWISE append information to an already existing bag
		if req.BagState == "New":
			self.bag = rosbag.Bag(req.BagPath +".bag","w") # Write

		else:
			self.bag = rosbag.Bag(req.BagPath +".bag","a") # Append
		#Read --> 'r' Write --> 'w' Append --> 'a (http://docs.ros.org/api/rosbag/html/python/)'

		#Txt opening and writing
		self.text = open(req.WholePath +".txt","w") #New txt files are created all the time. No appending.
		self.text.writelines("%s\t" % position for position in self.Attributes) #Write a descriptive line at the beginning of the text file
		self.text.write("\n")
		self.text.write("XYZ order\n") #Inform about how the coordinates order
		self.WritingTime=rospy.Time.now() #Variable to set the time bits for recording
		self.OpenBag = True # The bag has been opened to start writing in the CALLBACK

		time.sleep(1) #Spend one second recording

		#Wait for the callback to finish writing before closing the bag
		while self.WritingFlag == True:
			StillWriting = 1

		self.OpenBag = False # The bag has been closed stop writing in the CALLBACK
		#Bag and txt closing
		self.bag.close()
		self.text.close()
		self.StartingWritingTime = rospy.Time(self.bag.get_end_time()) #Record time for the next service call

		return UserServiceResponse(True) #Return service completion of the task

################################MAIN SCRIPT#############################

if __name__ == '__main__': #Main function that calls other functions

	rospy.init_node('body_data_saver', anonymous=True) #Node initialisation

	n = data_storer() #Instantiation of the class

	rospy.spin() #To keep the node running until it is stopped by the user
