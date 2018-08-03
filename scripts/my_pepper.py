#!/usr/bin/env python
import rospy #General stuff
import time #To set ROSBAG timing
import numpy #To change variables types
import sys #To take argument from the main function (Terminal or launch file)
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from user_msgs.msg import UserData #Essential to avoid problems in message instantiation and subscriber reception
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg


class PepperOrbbec():

    def __init__(self):
        #Publish the string for Pepper to reproduce on the Pepper.py subscriber node
        self.pepper_say = rospy.Publisher('pepper/say',
            String,
            queue_size=10)

        #Publish a flag for Rosbag to start recoding the topics from the camera
        #self.record = rospy.Publisher('pepper/flag', Bool, queue_size=10)
        self.record = rospy.Publisher('pepper/data',
            UserData,
            queue_size=10)

        #Publish pepper awareness disengagement
        self.pepper_engagement = rospy.Publisher('pepper/cmd',
            String,
            queue_size=10)
        # rostopic pub /pepper/c std_msgs/String "disengage" #onthe command line

        #Subscriber for the camera detector
        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.position_Callback) #Name // Type of message and Callback
        self.SawIt = False #Flag to know that a person is being tracked
        self.body_status = 4 #Initialise body status with out of scope value
        self.SaidIt = False  #Falg to know if the "Main speech" was pronounced
        self.User = UserData()
        self.User.UserID = numpy.int64(rospy.get_param('~ID'))#int64
        self.User.Flag = True # By default
        self.User.Bag = str(rospy.get_param('~BagDetails')) #string
        self.positionContainer = [
        "position one",
        "position two",
        "position three"
        ]
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
            rospy.loginfo("ERROR: BODY LOST")


    def pepper_Orbbec(self):

        rate = rospy.Rate(10)
        # spin() simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            #If you keprint pa.User.UserIDep seeing the body say that you will record and send Flag
            #print self.User.UserID
            print self.User.Flag
            #print type(self.User.UserID)
            #print type(self.User.Flag)
            rospy.loginfo("argv[1] = %s and argv[2] = %s",numpy.int64(sys.argv[1]),str(sys.argv[2]))
            if self.SaidIt==False and self.SawIt== True:
                #self.pepper_say.publish("Hello, would you like to do some exercise?")
                ##self.pepper_say.publish("One")
                print 1
                #time.sleep(1)
                ##self.pepper_say.publish("Two")
                #self.pepper_say.publish("Please, have a sit")
                #time.sleep(4)          #while not self.Sitted:
                #input("Press Enter when the person is sitted")
                #SHOW IMAGE OR EXAMPLE OF POSITION
                self.exercise_loop(self.positionContainer[0],"New")
                for position in self.positionContainer[1:]:
                    self.exercise_loop(position,"Current")


                #Send customized message
                #self.User.Flag = self.SawIt
                # self.record.publish(self.User)


                # self.pepper_say.publish("Recording")
                ##time.sleep(4)
                self.SaidIt= not self.SaidIt #Swap flag

            elif self.SaidIt == True and self.SawIt== False:
                self.pepper_say.publish("Doo")
                ##time.sleep(4)
                print 2
                #Send customized message
                #self.User.Flag = False
                #self.record.publish(self.User)

                ##self.pepper_say.publish("Doo hass")
                ##time.sleep(4)
                ##self.pepper_say.publish("Doo hass mish!")
                ##time.sleep(2)
                self.SaidIt= not self.SaidIt #Swap flag
            rate.sleep()
    def exercise_loop(self,position,BagState):
        #show IMAGE
        print "Exercising"
        self.User.Bag = BagState
        self.pepper_say.publish("Please, move your armm to " + str(position))
        #time.sleep(4)          #while not self.Sitted:
        self.pepper_say.publish("Hold it a bit more, please")
        self.User.Flag = True
        self.record.publish(self.User)
        time.sleep(1) #Duration of recording
        self.User.Flag = False
        self.record.publish(self.User)
        self.pepper_say.publish("Release")
        #time.sleep(4)
        self.pepper_say.publish("Now let's go for the new position")
        time.sleep(2)
if __name__ == '__main__':
    if False: #Your length plus 1 (Includes the path to the file)
        rospy.init_node('pepper_Orbbec', anonymous=True)
        pa = PepperOrbbec()
        pa.pepper_Orbbec()
        rospy.spin()
    else:
        while 1:
            rospy.loginfo("usage: Error introducing arguments")
        #rospy.loginfo("%s",sys.argv)
