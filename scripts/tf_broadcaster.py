#!/usr/bin/env python

import roslib #Ros libraries
import rospy #Python library
import tf #ROS Transform library
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg

class tf_broadcaster():

    def __init__(self):
################################CLASS INITIALISATION#############################
		#Subscriber that receives data from the recorded ROSBAGs skeleton topic

        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.positionCallback) #Name // Type of message and Callback


###################################CALLBACK METHOD###################################
    #Function to be run when some data arrives from the ROSBAG

    def positionCallback(self, BodyPOS):
        br = tf.TransformBroadcaster() #tf object instantiation

        # Skeletal body tracker addresses to access joint data
        # Any lines can be commented to toggle joints
        Attributes = [
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

        # Skeletal body tracker gneeral information
        Body_info = [
            "body_id",
            "tracking_status",
            "gesture"]

        for BodyPart in Attributes: #For each bodypart send/create a tf transform

            self.tfTransform(br,BodyPart,BodyPOS) #Method to be send tfs


        for Property in Body_info: #For each additional information attribute show values
            rospy.loginfo("%s: %s",Property,getattr(BodyPOS,Property))

###################################TF METHOD TO SET FRAME REFERENCING###################################

    #Run when data arrives from the ROSBAG

    def tfTransform(self,br,Attribute,BodyPOS):
		#https://docs.python.org/3/library/functions.html#getattr --> GETATTR (Example: BodyPOS/joint_position_spine_mid/x)
        # Extract the x,y and z attribute for each joint and send them as a frame.
        br.sendTransform((getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"),getattr(getattr(BodyPOS,Attribute),"z")), #Translation
            tf.transformations.quaternion_from_euler(0, 0, 0), #Rotation (Not required)
            rospy.Time.now(), #Time at which the transform it collected
            Attribute, #child frame (The given joint)
            "camera")	#parent frame (Started being called "world" Coordinates [0 0 0])
        # Log info being send as tf
        rospy.loginfo("X: %s Y: %s Z: %s //%s",getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"), getattr(getattr(BodyPOS,Attribute),"z"),Attribute) #Keep track of values

################################MAIN SCRIPT#############################

if __name__ == '__main__': #Main function that calls other functions

    rospy.init_node('body_data_handling', anonymous=True) # Node initialisation ANONYMOUS=True to allow different nodes with the same name
    n = tf_broadcaster() #Class instantiation
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped
