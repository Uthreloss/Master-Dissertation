#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg


class PepperOrbbec():

    def __init__(self):
        #Publish the string for Pepper to reproduce on the Pepper.py subscriber node
        self.pepper_say = rospy.Publisher('pepper/say', String, queue_size=10)

        #Publish a flag for Rosbag to start recoding the topics from the camera
        self.record = rospy.Publisher('pepper/flag', Bool, queue_size=10)

        #Publish pepper awareness disengagement
        self.pepper_engagement = rospy.Publisher('pepper/cmd', String, queue_size=10)
        # rostopic pub /pepper/c std_msgs/String "disengage" #onthe command line

        #Subscriber for the camera detector
        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.position_Callback) #Name // Type of message and Callback
        self.SawIt = False #Flag to know that a person is being tracked
        self.body_status = 4 #Initialise body status with out of scope value
        self.SaidIt = False  #Falg to know if the "Main speech" was pronounced
        #self.pepper_engagement.publish("disengage") #Disable awareness in pepper_say
        #time.sleep(4)

    def position_Callback(self, data):
        self.body_status = data.tracking_status # Update state

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.tracking_status) #Print what you heard

        #Welcome the user once a body is detected
        if data.tracking_status == 3 and self.SawIt== False:
            self.SawIt= not self.SawIt #Swap flag

        #If the body is lost say that it is out of sight
        if data.tracking_status < 3 and self.SawIt== True:
            self.SawIt= not self.SawIt #Swap flag


    def pepper_Orbbec(self):

        rate = rospy.Rate(10)
        # spin() simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            #If you keep seeing the body say that you will record and send Flag

            if self.SaidIt==False and self.SawIt== True:
                #self.pepper_say.publish("Hello, would you like to do some exercise?")
                self.pepper_say.publish("One")
                time.sleep(1)
                self.pepper_say.publish("Two")
                #self.pepper_say.publish("Please, have a sit")
                time.sleep(4)          #while not self.Sitted:
                #input("Press Enter when the person is sitted")
                self.record.publish(True)
                self.pepper_say.publish("Recording")
                time.sleep(4)
                self.SaidIt= not self.SaidIt #Swap flag
            elif self.SaidIt == True and self.SawIt== False:
                self.pepper_say.publish("Doo")
                time.sleep(4)
                self.record.publish(False)
                self.pepper_say.publish("Doo hass")
                time.sleep(4)
                self.pepper_say.publish("Doo hass mish!")
                time.sleep(2)
                self.SaidIt= not self.SaidIt #Swap flag
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('pepper_Orbbec', anonymous=True)
    pa = PepperOrbbec()
    pa.pepper_Orbbec()
    rospy.spin()
