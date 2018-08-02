#!/usr/bin/env python
import roslib #Ros libraries
import rospy #Python library
import tf #transform library
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg

class tf_broadcaster():

    def __init__(self):

        #Subscriber
        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.positionCallback) #Name // Type of message and Callback




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

        Body_info = [
            "body_id",
            "tracking_status",
            "gesture"]

        for BodyPart in Attributes: #For each bodypart send/create a tf transform
            self.tfTransform(br,BodyPart,BodyPOS)


        for Property in Body_info: #For each additional info NOT GEOMETRIC print values
            rospy.loginfo("%s: %s",Property,getattr(BodyPOS,Property))

    def tfTransform(self,br,Attribute,BodyPOS):
		#https://docs.python.org/3/library/functions.html#getattr --> GETATTR (Example: BodyPOS/joint_position_spine_mid/x)
        br.sendTransform((getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"),getattr(getattr(BodyPOS,Attribute),"z")), #Translation
            tf.transformations.quaternion_from_euler(0, 0, 0), #Rotation (No need for rotation)
            rospy.Time.now(), #Time at which the transform it collected
            Attribute, #child frame (The given joint)
            "camera")	#parent frame (Started being called "world" Coordinates [0 0 0])
        rospy.loginfo("X: %s Y: %s Z: %s //%s",getattr(getattr(BodyPOS,Attribute),"x"),getattr(getattr(BodyPOS,Attribute),"y"), getattr(getattr(BodyPOS,Attribute),"z"),Attribute) #Keep track of values


if __name__ == '__main__': #Main function that calls other functions
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
    rospy.init_node('body_data_handling', anonymous=True)
	#jointname = rospy.get_param('~body_tracker') #CONSIDER THIS FOR MORE THAN ONE BODY DETECTED
    n = tf_broadcaster()
	# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
