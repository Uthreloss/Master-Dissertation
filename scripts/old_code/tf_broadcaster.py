#!/usr/bin/env python
import roslib #Ros libraries
import rospy #Python library
import tf #transform library
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg

def positionCallback(BodyPOS): #Function that is called when the Subscriber/listener receives data

	br = tf.TransformBroadcaster() #tf object creation
	Attributes = [
		"joint_position_head",
		"joint_position_head",
		"joint_position_neck",
		"joint_position_shoulder",
		"joint_position_spine_top",
		"joint_position_spine_mid",
		"joint_position_spine_bottom",
		"joint_position_left_shoulder",
		"joint_position_left_elbow",
		"joint_position_left_hand",
		"joint_position_right_shoulder",
		"joint_position_right_elbow",
		"joint_position_right_hand"]

	for BodyPart in Attributes: #For each bodypart send/create a tf transform
		tfTransform(br,BodyPart,BodyPOS)

def tfTransform(br, Attribute,BodyPOS):
	#https://docs.python.org/3/library/functions.html#getattr --> GETATTR (Example: BodyPOS/joint_position_spine_mid/x)
	br.sendTransform((getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"),getattr(getattr(BodyPOS,Attribute),"z")), #Translation
		tf.transformations.quaternion_from_euler(0, 0, 0), #Rotation (No need for rotation)
		rospy.Time.now(), #Time at which the transform it collected
		Attribute, #child frame (The given joint)
		"camera")	#parent frame (Started being called "world" Coordinates [0 0 0])
	rospy.loginfo("//%s X: %s Y: %s Z: %s",Attribute,getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"), getattr(getattr(BodyPOS,Attribute),"z")) #Keep track of values


def listener(): #Subscriber node

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('body_data_handling', anonymous=True)
	#jointname = rospy.get_param('~body_tracker') #CONSIDER THIS FOR MORE THAN ONE BODY DETECTED
    rospy.Subscriber("/body_tracker/skeleton", body_tracker_msgs.msg.Skeleton, positionCallback) #Name // Type of message and Callback

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__': #Main function that calls other functions
       listener()
