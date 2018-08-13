#!/usr/bin/env python
import roslib #Ros libraries
import rospy #Python library
import tf #transform library
import rosbag #To store data
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg
import geometry_msgs.msg
from std_msgs.msg import Int32, String, Bool
from time import gmtime, strftime # Library to get the date and time for data saving with ROSBAG
import math



class tf_broadcaster():

	def __init__(self):
		#Subscriber 1 (Skeleton)
		self.sub = rospy.Subscriber(
			"/body_tracker/skeleton",
			body_tracker_msgs.msg.Skeleton,
			self.positionCallback) #Name // Type of message and Callback

		self.BagName = strftime("%Y-%m-%d %H:%M:%S", gmtime()) #Opening bag with the name associated ith the date
		self.bag = rosbag.Bag(str(self.BagName) +".bag","w") # Make sure that you run the code in the directory in which you want to store the bag

	def positionCallback(self, BodyPOS): #Function that is called when the Subscriber/listener receives data
		br = tf.TransformBroadcaster() #tf object creation
		Attributes = [
			"joint_position_head",
			"joint_position_neck",
			#"joint_position_shoulder", #Always 0 by default
			"joint_position_spine_top",
			"joint_position_spine_mid",
			"joint_position_spine_bottom",
			"joint_position_left_shoulder",
			"joint_position_left_elbow",
			"joint_position_left_hand",
			"joint_position_right_shoulder",
			"joint_position_right_elbow",
			"joint_position_right_hand"]

		# for BodyPart in Attributes: #For each bodypart send/create a tf transform
		# 	self.tfTransform(br,BodyPart,BodyPOS)
		##########Saving data into a BAG file############
		Skel = body_tracker_msgs.msg.Skeleton #Object created from message type
		Skel = BodyPOS #Save BodyPOS data in Skel
		self.bag.write("CallbackSkel",Skel) #Write all data on the bag
		#################################################

	# def tfTransform(self,br,Attribute,BodyPOS):
	# 	#https://docs.python.org/3/library/functions.html#getattr --> GETATTR (Example: BodyPOS/joint_position_spine_mid/x)
	# 	br.sendTransform((getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"),getattr(getattr(BodyPOS,Attribute),"z")), #Translation
	# 		tf.transformations.quaternion_from_euler(0, 0, 0), #Rotation (No need for rotation)
	# 		rospy.Time.now(), #Time at which the transform it collected
	# 		Attribute, #child frame (The given joint)
	# 		"camera")	#parent frame (Started being called "world" Coordinates [0 0 0])
	# 	#rospy.loginfo("//%s X: %s Y: %s Z: %s",Attribute,getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"), getattr(getattr(BodyPOS,Attribute),"z")) #Keep track of values

	def control_loop(self):
		while not rospy.is_shutdown():
			a=1
		self.bag.close() #Close bag when Ctrl+C is pressed

if __name__ == '__main__': #Main function that calls other functions
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('body_data_handling', anonymous=True)
	#jointname = rospy.get_param('~body_tracker') #CONSIDER THIS FOR MORE THAN ONE BODY DETECTED
	n = tf_broadcaster() #Imnstantiate class
	n.control_loop() #Call while loop to manage Ctrl+C
	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin() #OPTIONAL WHEN USING WHILE NOT LOOP
