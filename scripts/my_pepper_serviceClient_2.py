#!/usr/bin/env python
import rospy #General stuff
import time #To set ROSBAG timing
import numpy #To change variables types
import sys #To take argument from the main function (Terminal or launch file)
import os # To create diretories using os.mkdir(path)
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from user_msgs.msg import UserData #Essential to avoid problems in message instantiation and subscriber reception
from user_srv.srv import *# My personal service
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg
import rospkg #To get the path of the Robag directory in the package
import random # for random selection of encouraging comments

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
        #Publish pepper images on Tablet
        self.pepper_display = rospy.Publisher('pepper/display',
            String,
            queue_size=10)

        # rostopic pub /pepper/c std_msgs/String "disengage" #onthe command line

        #Subscriber for the camera detector
        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.position_Callback) #Name // Type of message and Callback

        self.rospack = rospkg.RosPack()
        self.RepoPath = self.rospack.get_path('master_dissertation') +"/experiment_bags/" #Where the bag are saved
        self.SawIt = False #Flag to know that a person is being tracked
        self.body_status = 4 #Initialise body status with out of scope value
        self.SaidIt = False  #Falg to know if the "Main speech" was pronounced
        self.User = UserData()
        self.args = rospy.myargv(argv=sys.argv)
        self.User.UserID = numpy.int64(self.args[1])#int64 ID of the User
        self.Set = numpy.int64(self.args[2])#int64 ID of the User
        self.User.Flag = True # By default
        self.Mode = str(self.args[3]) # the option are "notalive" and "alive"
        self.Pace = "slow" #This variable serves to ease the data collection once the user has familiarised with the positions
        #Just in case that the input is
        if self.Mode != "notalive" and self.Mode != "alive":
            print "MODE ERROR: Not properly written: " + str(self.Mode)
            self.Mode = "alive"
            print "Changed to: alive"
	print self.Mode
        self.User.Bag = "New"
        self.engagement_comments = [
            "Well done!",
            "That's it!",
            "Nicely performed",
            "Keep it up",
            "Good job!",
            "Good work!",
            "That is splendid!",
            "You are doing vey well!",
            "Lovely!",
            "Brilliant!",
            "Kepp going!",
            "Triple a!",
            "Yes, that's it"
        ]
        self.positionContainer = [
            "position one",
            "position two",
            "position three"
        ]
        self.setContainer = [
            "set one",
            "set two",
            "set three",
            "set four"
        ]
        self.ItContainer = [
            "set one",
            "set two",
            "set three",
            "set four",
            "set five",
            "set six",
        ]
        self.ImageContainer = [
        "https://i.imgur.com/S3WCMWd.png",
        "https://i.imgur.com/o5tEwm1.png",
        "https://i.imgur.com/uO0jPfh.png"
        ]
        self.PositionRange = 3
        self.ItRange = 1

    def position_Callback(self, data):
        self.body_status = data.tracking_status # Update state

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.tracking_status) #Print what you heard

        #Welcome the user once a body is detected
        if data.tracking_status == 3 and self.SawIt== False:
            self.User.BodyID = numpy.int64(data.body_id) #In order not to record data from another detected body
            self.SawIt= not self.SawIt #Swap flag
        #If the body is lost say that it is out of sight
        if data.tracking_status < 3 and self.SawIt== True:
            self.SawIt= not self.SawIt #Swap flag
            rospy.loginfo("ERROR: BODY LOST")


    def pepper_Orbbec(self):

        rate = rospy.Rate(10)
        # spin() simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            print self.User.Flag
            if self.SaidIt==False and self.SawIt== True:
            #Conversation LITTLE how are you? etc. TWO DIFFERENT INTERACTION MODES
                #self.pepper_engagement.publish(self.Mode)
                if (self.Set == 1 or self.Set == 3) and self.Mode == "alive":
                    time.sleep(1)
                    self.pepper_say.publish("Hello, welcome to the robot coaching program")
                    time.sleep(5)
                    self.pepper_say.publish("Would you like to do some exercise?")
                    ##self.pepper_say.publish("One")
                    time.sleep(4)
                elif (self.Set == 2 or self.Set == 4) and self.Mode == "alive":
                    time.sleep(1)
                    self.pepper_say.publish("Hello, again!")
                    time.sleep(2)
                    self.pepper_say.publish("I hope you had a restful break")
                    time.sleep(4)
                    self.pepper_say.publish("Are you ready for another round?")
                    ##self.pepper_say.publish("One")
                    time.sleep(4)
                elif (self.Set == 1 or self.Set == 3) and self.Mode == "notalive":
                    time.sleep(1)
                    self.pepper_say.publish("Hello, welcome to the robot coaching program")
                    time.sleep(5)
                    self.pepper_say.publish("Do you like to do some exercise?")
                    ##self.pepper_say.publish("One")
                    time.sleep(4)
                elif (self.Set == 2 or self.Set == 4) and self.Mode == "notalive":
                    time.sleep(1)
                    self.pepper_say.publish("Hello")
                    time.sleep(2)
                    self.pepper_say.publish("Are you ready for another round?")
                    ##self.pepper_say.publish("One")
                    time.sleep(4)
                else:
                    self.pepper_say.publish("Ups!")
                    time.sleep(1)
                    self.pepper_say.publish("I think something is wrong")
                    time.sleep(2)
                    self.pepper_say.publish("Could you please call Daniel?")
                    time.sleep(4)
                    break
                ##self.pepper_say.publish("Two")
                self.pepper_say.publish("Please, remain sitted for the rest of the session")
                time.sleep(6)          #while not self.Sitted:
                #input("Press Enter when the person is sitted")
                for iteration in range(self.ItRange):
                    if iteration == 0:
                        self.pepper_say.publish("We will start with " + self.ItContainer[iteration])
                        time.sleep(3)          #while not self.Sitted:
                    else:
                        self.pepper_say.publish("Now we will do " + self.ItContainer[iteration])
                        self.User.Bag = "Current"
                        time.sleep(3)
                    self.pepper_say.publish("Are you ready?")
                    time.sleep(3)
                    self.pepper_say.publish("Let's go!")
                    time.sleep(1)

                    #SHOW IMAGE OR EXAMPLE OF POSITION
                    if iteration < 1: #<3
                        self.Pace = "slow"
                    else:
                        self.Pace = "quick"

                    if self.SawIt == False:
                        break

                    self.exercise_loop_client(self.positionContainer[0],self.User.Bag,1,self.Set,str(iteration + 1),self.Pace)
                    for posIdx, position in enumerate(self.positionContainer[1:]):

                        if self.SawIt == False:
                            break
                        if self.Mode == "alive":
                            self.pepper_say.publish(random.choice(self.engagement_comments)) #Not for safeguard MODE
                        time.sleep(3)
                        if self.Pace == "slow":
                            self.pepper_say.publish("Let's now go for the new position")
                            time.sleep(1)
                            self.pepper_say.publish("Don't we?")
                            time.sleep(3)
                        else:
                            self.pepper_say.publish("Next one")
                            time.sleep(1)
                        self.exercise_loop_client(position,"Current",posIdx+2,self.Set,str(iteration + 1),self.Pace)
                        ####time.sleep(4)

                    if self.SawIt == False:
                        break

                self.SaidIt= not self.SaidIt #Swap flag

            elif self.SaidIt == True and self.SawIt== False:
                self.pepper_say.publish("I do not see you")
                time.sleep(2)
                self.pepper_say.publish("Could you please call Daniel?")
                time.sleep(4)
                break
                #self.SaidIt= not self.SaidIt #Swap flag

            elif self.SaidIt == True and self.SawIt== True:
                print "Finished!"
                if self.Set==1 or self.Set==3:
                    self.pepper_say.publish("We are done with sequence one")
                    time.sleep(2)
                    if self.Mode == "alive":
                        self.pepper_say.publish("Your work was really good!") # for the solitary MODE
                        time.sleep(3)
                    self.pepper_say.publish("It's time for a break!")
                    time.sleep(2)
                    self.pepper_say.publish("Could you, please, call Daniel?")
                    time.sleep(4)
                else:
                    self.pepper_say.publish("Now we are done")
                    time.sleep(2)
                    self.pepper_say.publish("The exercise session is over")
                    time.sleep(2)

                    if self.Mode == "alive":
                        self.pepper_say.publish("You did really well today!") # for the solitary MODE
                        time.sleep(3)

                    self.pepper_say.publish("Could you, please, fill in the questionnaires?")
                    time.sleep(3)
                    self.pepper_say.publish("They are on the table next to you")
                    time.sleep(4)
                #self.pepper_say.publish("I will now go to sleep")
                #time.sleep(2)
		#self.pepper_engagement.publish("alive")
                #self.pepper_engagement.publish("off")
                break

            rate.sleep()
        #self.pepper_display.publish("hide")
        #self.pepper_engagement.publish("alive")
    def exercise_loop_client(self,position,BagState,CurrentPosition,setNumber,ItNumber,Speed): #Service callback
        #show IMAGE
        print "Exercising"
        if Speed == "slow":
            self.pepper_display.publish(self.ImageContainer[CurrentPosition-1])
            self.pepper_say.publish("Please, move your arm to " + str(position))
            time.sleep(6)          #while not self.Sitted:
            self.pepper_say.publish("Hold it a bit longer, please")
            time.sleep(2)
        else: #A bit quicker
            self.pepper_display.publish(self.ImageContainer[CurrentPosition-1])
            self.pepper_say.publish(str(position)+"!")
            time.sleep(2)          #while not self.Sitted:
            self.pepper_say.publish("Stay put")
            time.sleep(1)
        rospy.wait_for_service('recorder')
        try:
            recorder_service = rospy.ServiceProxy('recorder', UserService)
            BagPath = self.RepoPath + "participant_" + str(self.User.UserID) + "/set_" + str(setNumber)
            WholePath = self.RepoPath + "participant_" + str(self.User.UserID) + "/set_" + str(setNumber) + "/P_" + str(CurrentPosition) + "/P_" + str(CurrentPosition) + "_" + str(ItNumber)
            reply = recorder_service(BagPath,WholePath,self.User.BodyID,BagState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #time.sleep(1) #Pause to give time to manage files in data_storin_flag.py #DO NOT COMMET
        self.pepper_say.publish("Release") #Make sure the participant understands WATCH OUT!
        time.sleep(2)
        self.pepper_display.publish("hide")

if __name__ == '__main__':
    # try: #Your length plus 1 (Includes the path to the file)
    if True:
        rospy.init_node('pepper_Orbbec', anonymous=True)
        pa = PepperOrbbec()
	time.sleep(3)
        #pa.pepper_engagement.publish(pa.Mode)
        pa.pepper_engagement.publish("alive")
	time.sleep(1)
        pa.pepper_engagement.publish("dialogoff")
	time.sleep(1)
        pa.pepper_engagement.publish("hearingoff")
	time.sleep(1)
        pa.pepper_Orbbec()
        rospy.spin()
    # except:
    else:
        print "usage: Error introducing arguments"
        #rospy.loginfo("%s",sys.argv)
