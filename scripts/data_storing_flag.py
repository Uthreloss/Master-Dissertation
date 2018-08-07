#!/usr/bin/env python
import roslib #Ros libraries
import rospy #Python library
import tf #transform library
import rosbag #To store data
import numpy #To change variables types
import time #To set ROSBAG timing
import message_filters #To receive the data from Pepper and the camera at the same.
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg
import geometry_msgs.msg
from user_msgs.msg import UserData# My personal messages
from std_msgs.msg import Int32, String, Bool
from time import gmtime, strftime # Library to get the date and time for data saving with ROSBAG
import math
import rospkg #To get the path of the Robag directory in the package


class tf_broadcaster():

	def __init__(self):
		#########CHECK#########
		#Subscriber 1 (Skeleton)
		self.skel_sub = rospy.Subscriber(
			"/body_tracker/skeleton",
			body_tracker_msgs.msg.Skeleton,
			self.positionCallback) #Name // Type of message and Callback

		#Subscriber 2 (Pepper Flag)
		self.flag_sub = rospy.Subscriber(
			"/pepper/data",
			UserData,
			self.recordCallback) #Name // Type of message and Callback

		self.Attributes = [
			#"joint_position_head",
			#"joint_position_neck",
			#"joint_position_shoulder", #Always 0 by default
			"joint_position_spine_top",
			"joint_position_spine_mid",
			#"joint_position_spine_bottom",
			#"joint_position_left_shoulder",
			#"joint_position_left_elbow",
			#"joint_position_left_hand",
			"joint_position_right_shoulder",
			"joint_position_right_elbow",
			"joint_position_right_hand"]

		self.rospack = rospkg.RosPack()
		self.BagPath = self.rospack.get_path('master_dissertation') +"/experiment_bags/" #Where the bag are saved
		self.counter = 0
		self.User = UserData()
		self.User.Flag = False #To start recording
		##Time variable need to be duration objects/type
		self.WritingTime=rospy.Time.now() #Pointer to write in the right plaec in the ROSBAG
		self.OpenBag = False # To know when to start writing
		self.StartingWritingTime = rospy.Time.now()
		self.WritingFlag = False

#		self.BagName = strftime("%Y-%m-%d %H:%M:%S", gmtime()) #Opening bag with the name associated ith the date
#		self.bag = rosbag.Bag(str(self.BagName) +".bag","w") # Make sure that you run the code in the directory in which you want to store the bag

	def positionCallback(self, data): #Function that is called when the Subscriber/listener receives data
		br = tf.TransformBroadcaster() #tf object creation


		# for BodyPart in Attributes: #For each bodypart send/create a tf transform
		# 	self.tfTransform(br,BodyPart,BodyPOS)
		##########Saving data into a BAG file############
		if self.User.Flag and self.OpenBag:# and data.body_id == numpy.int64(self.User.Flag/10): for 2 bodies detection avoidance
			self.WritingFlag = True
			Skel = body_tracker_msgs.msg.Skeleton() #Object created from message type
			Skel = data #Save BodyPOS data in Skel
			self.bag.write("/body_tracker/skeleton",Skel, rospy.Time.now() - self.WritingTime + self.StartingWritingTime) #Write all data on the bag THE NAME OF THE TOPIC SHOULD BE THE SAME AS THE REAL CAMERA TOPIC
			for BodyPart in self.Attributes: #For each bodypart send/create a tf transform
				self.text.write("%f,%f,%f\t" % ((getattr(getattr(data,BodyPart),"x")),getattr(getattr(data,BodyPart),"y"),getattr(getattr(data,BodyPart),"z")))
			self.text.write("%i" % len(self.Attributes))
			self.text.write("\n")
			rospy.loginfo("recording")
		#################################################
		self.WritingFlag = False
	# def tfTransform(self,br,Attribute,BodyPOS):
	# 	#https://docs.python.org/3/library/functions.html#getattr --> GETATTR (Example: BodyPOS/joint_position_spine_mid/x)
	# 	br.sendTransform((getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"),getattr(getattr(BodyPOS,Attribute),"z")), #Translation
	# 		tf.transformations.quaternion_from_euler(0, 0, 0), #Rotation (No need for rotation)
	# 		rospy.Time.now(), #Time at which the transform it collected
	# 		Attribute, #child frame (The given joint)
	# 		"camera")	#parent frame (Started being called "world" Coordinates [0 0 0])
	# 	#rospy.loginfo("//%s X: %s Y: %s Z: %s",Attribute,getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"), getattr(getattr(BodyPOS,Attribute),"z")) #Keep track of values
	def recordCallback(self, data):
		#If the flag is True add 1 to the counter if now is False add another to the counter to close the ROSBAG
		self.User = data
		print("receiving: " + str(self.User.Flag))
		if self.User.Flag == True:
			self.counter=self.counter+1
			print self.counter
		if self.User.Flag == False:
			self.counter=self.counter+1
			print self.counter

	def control_loop(self):
		while not rospy.is_shutdown():
			if self.User.Flag==True:
				#self.BagName = strftime("%Y-%m-%d %H:%M:%S", gmtime()) #Opening bag with the name associated ith the date
				self.BagName = self.User.UserID #Opening bag with the name associated with the USER
				if self.User.Bag == "New":
					self.bag = rosbag.Bag(self.BagPath + str(self.BagName) +".bag","w") # Make sure that you run the code in the directory in which you want to store the bag
					self.text = open(self.BagPath + str(self.BagName) + self.User.Pose +".txt","w") #Create a text file too
					self.text.writelines("%s\t" % position for position in self.Attributes) #Self explanatory line at the beginning of the text
					self.text.write("\n")
					self.text.write("XYZ order\n")
					print "W"
				else:
					self.bag = rosbag.Bag(self.BagPath + str(self.BagName) +".bag","a")
					self.text = open(self.BagPath + str(self.BagName) + self.User.Pose +".txt","w")
					self.text.writelines("%s\t" % position for position in self.Attributes) #Self explanatory line at the beginning of the text
					self.text.write("\n")
					self.text.write("XYZ order\n")
					print "A"
				self.WritingTime=rospy.Time.now() #Adapt time for the ROSBAG what out for delays!!!
				self.OpenBag = True # The bag has been opened START WRITING!

					#Read --> 'r' Write --> 'w' Append --> 'a (http://docs.ros.org/api/rosbag/html/python/)'
				print "inside"
				while self.counter<2:# and self.WritingFlag == True: #not rospy.is_shutdown() and
					z=1
					#time.sleep(1)

					#rospy.loginfo(self.counter)
					#rospy.loginfo(self.Flag)
				print "outside"
				self.OpenBag = False # The bag has been closed STOP WRITING!
				self.bag.close() #Close bag when Ctrl+C is pressed
				self.text.close()
				self.StartingWritingTime = rospy.Time(self.bag.get_end_time()) #It needs to be a Duration object for ROS to interpret
				self.counter = 0
				print "here"

if __name__ == '__main__': #Main function that calls other functions
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('body_data_saver', anonymous=True)
	#jointname = rospy.get_param('~body_tracker') #CONSIDER THIS FOR MORE THAN ONE BODY DETECTED
	n = tf_broadcaster() #Imnstantiate class
	n.control_loop() #Call while loop to manage Ctrl+C
	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin() #OPTIONAL WHEN USING WHILE NOT LOOP
