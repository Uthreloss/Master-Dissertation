#!/usr/bin/env python

import rospy #General ros functionalities
import time #To support ROSBAG timing
import numpy #To change variables types
import sys #To take an argument from the main function (Terminal or launch file)
import os # To create diretories using os.mkdir(path) [Storing phase]
from std_msgs.msg import String #Require to send message to control Pepper modules
#from user_msgs.msg import UserData #Created to simplify message instantiation and subscriber reception
from user_srv.srv import *# My personal service
import body_tracker_msgs.msg #Astra Orbbec messages ALL OF THEM BodyTracker & Skeleton check: https://github.com/shinselrobots/body_tracker_msgs/tree/master/msg
import rospkg #To get the path of the Robag directory in the package
import random # for random selection of encouraging comments

class PepperOrbbec():

################################CLASS INITIALISATION#############################
    def __init__(self):
        #Publish the string for Pepper to perform animated speech
        self.pepper_say = rospy.Publisher('pepper/say',
            String,
            queue_size=10)

        #Publish the string for Pepper to control default modules activation
        self.pepper_engagement = rospy.Publisher('pepper/cmd',
            String,
            queue_size=10)
        #Publish images URLs to display on Pepper's Tablet
        self.pepper_display = rospy.Publisher('pepper/display',
            String,
            queue_size=10)

        # Client node initialisation (Wait for the service is ready)

        self.connected = False

        while not self.connected:
            try:
                rospy.wait_for_service('recorder')
                self.recorder_service = rospy.ServiceProxy('recorder', UserService)
                self.connected = True
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                pass

        print(self.connected)

        #Subscriber that receives data from the ORBBEC SDK
        self.sub = rospy.Subscriber(
            "/body_tracker/skeleton",
            body_tracker_msgs.msg.Skeleton,
            self.position_Callback) #Name // Type of message and Callback

    # Global class variables

        self.rospack = rospkg.RosPack() #Initialise ROS function for directory searching
        self.RepoPath = self.rospack.get_path('master_dissertation') +"/experiment_bags/" #Preferred location to save data in the Project

        self.SawIt = False #Flag to know that a person is being tracked (STARTS AS FALSE)
        self.SaidIt = False  #Flag to know if the "Main speech" was pronounced (STARTS AS FALSE)
        #self.User = UserData()

        self.args = rospy.myargv(argv=sys.argv) # rospy adapatation of sys arguments

        self.UserID = numpy.int64(self.args[1])#int64 (ID of the User. Equivalent to the number of the participant)

        self.Set = numpy.int64(self.args[2])#int64 (Number of the set of iterations)

        self.Mode = str(self.args[3]) # Robot engagement mode. "e" or "ne" ("engaging" or "not engaging")

        self.Pace = "slow" #This variable serves to fasten the data collection once the user has familiarised with the positions

        self.Bag = "New" #Variable to set the bag creation preferences

        #If the string of the mode is not correctly received the set "engaging" by default
        if self.Mode != "ne" and self.Mode != "e":
            print "MODE ERROR: Not properly written: " + str(self.Mode)
            self.Mode = "e"
            print "Changed to: engaging"
    	print self.Mode

        # Exercise variables
        self.ItRange = 6 #ItRange repetitions
        self.PositionRange = 3 #Of PositionRange poses

        # Debug mode to remove pause in the HRI
        self.debugMode = 0 #1 disables time delays

        #Compilation of potential engaging comments
        self.engagement_comments = [
            "Well done!",
            "That's it!",
            "Nicely performed",
            "Keep it up!",
            "Good work!",
            "Good pose!",
            "Great job!",
            "Great work!",
            "That is super!",
            "That is splendid!",
            "You are doing vey well!",
            "Lovely!",
            "Brilliant!",
            "Very well done!",
            "Nice!",
            "Bravo",
            "Triple a!",
            "Yes, that's it"
            "Well copied!"
        ]
    #Strings for speech

        #String to be sent to the robot animated speech module
        self.positionContainer = [
            "position one",
            "position two",
            "position three"
        ]
        #String to be sent to the robot animated speech module
        self.setContainer = [
            "set one",
            "set two",
            "set three",
            "set four"
        ]
        #String to be sent to the robot animated speech module
        self.ItContainer = [
            "set one",
            "set two",
            "set three",
            "set four",
            "set five",
            "set six",
        ]

        #URL of the position images to be sent to the robot display module
        self.ImageContainer = [
        "https://i.imgur.com/S3WCMWd.png",
        "https://i.imgur.com/o5tEwm1.png",
        "https://i.imgur.com/uO0jPfh.png"
        ]
################################TIME MANAGEMENT METHOD#############################
    def timeDebugging(self, speechPause, debugMode):
        if debugMode==0:
            time.sleep(speechPause)

###################################CALLBACK METHOD###################################
    #Function to be run when some data arrives from the camera SDK

    def position_Callback(self, data):

        #The data received contains
            #body_tracking_status       Tell the state of the detection: 3 means "fully detected"; 2 means "partially detected";  1 means "lost"
            #body.id                    Number assigned to the detected skeleton

        #self.body_status = data.tracking_status # Update state

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.tracking_status) #Print what you heard

        #Fully detected
        if data.tracking_status == 3 and self.SawIt== False:
            self.BodyID = numpy.int64(data.body_id) # Save body_id for later tracking. Also not to record data from another detected body
            self.SawIt= not self.SawIt #Swap flag False --> True

        #If the body is lost then it is out of sight and the robot expresses this

        if data.tracking_status < 3 and self.SawIt== True:
            self.SawIt= not self.SawIt #Swap flag True --> False
            rospy.loginfo("ERROR: BODY LOST") #For the experimenter awareness

###############################HUMAN-ROBOT INTERACTION#############################

# Main function containing HRI and data storing service calling

    def pepper_HRI(self):

        rate = rospy.Rate(10) #Default Rate loop (optional)



        while not rospy.is_shutdown():

        # If the user was seen and the exercise session has not been delivered yet START the HRI

            if self.SaidIt==False and self.SawIt== True:

            #HRI initialisation depending on set number and "engaging" mode

                # First sets with engagement (FOR NOW THE SAME AS WITHOUT ENGAGEMENT)

                if (self.Set == 1 or self.Set == 3) and self.Mode == "e":
                    self.timeDebugging(1, self.debugMode)
                    self.pepper_say.publish("Hello, welcome to the robot coaching program")
                    self.timeDebugging(5,self.debugMode)
                    self.pepper_say.publish("Would you like to do some exercise?")
                    self.timeDebugging(4,self.debugMode)

                # Second sets with engagement (MORE KIND)

                elif (self.Set == 2 or self.Set == 4) and self.Mode == "e":
                    self.timeDebugging(1,self.debugMode)
                    self.pepper_say.publish("Hello, again!")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("I hope you had a restful break")
                    self.timeDebugging(4,self.debugMode)
                    self.pepper_say.publish("Are you ready for another round?")
                    self.timeDebugging(4,self.debugMode)

                # First sets without engagement (FOR NOW THE SAME AS WITHO ENGAGEMENT)

                elif (self.Set == 1 or self.Set == 3) and self.Mode == "ne":
                    self.timeDebugging(1,self.debugMode)
                    self.pepper_say.publish("Hello, welcome to the robot coaching program")
                    self.timeDebugging(5,self.debugMode)
                    self.pepper_say.publish("Do you want to do some exercise?")
                    self.timeDebugging(4,self.debugMode)

                # Second sets without engagement (MORE DIRECT)

                elif (self.Set == 2 or self.Set == 4) and self.Mode == "ne":
                    self.timeDebugging(1,self.debugMode)
                    self.pepper_say.publish("Hello")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("Are you ready for another round?")
                    self.timeDebugging(4,self.debugMode)

                #If something does not work. Call the experimenter and break the HRI

                else:

                    self.pepper_say.publish("Ups!")
                    self.timeDebugging(1,self.debugMode)
                    self.pepper_say.publish("I think something is wrong")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("Could you please call Daniel?")
                    self.timeDebugging(4,self.debugMode)
                    break

                #After introducing the session the robot asks the user to remain sitted

                self.pepper_say.publish("Please, remain sitted for the rest of the session")
                self.timeDebugging(4,self.debugMode)

                #Make sure that the robot default dialog is disable

                self.pepper_engagement.publish("dialogoff")
            	self.timeDebugging(1,self.debugMode)
                # Send a second time to ensure
                self.pepper_engagement.publish("dialogoff")
            	self.timeDebugging(1,self.debugMode)

                #Recording resting position

                #Special path set and set as New bag
                BagPath = self.RepoPath + "participant_" + str(self.UserID) + "/set_" + str(self.Set) + "_Rest"
                WholePath = self.RepoPath + "participant_" + str(self.UserID) + "/set_" + str(self.Set) + "_Rest"
                BagState = "New"
                self.service_call(BagPath,WholePath,BagState)

                #Main loop to iterate the repetitions of the position 1, 2 and 3

                for iteration in range(self.ItRange):

                    #If the interation is the first one self.Bag = "New" (Initialised as "New")
                    if iteration == 0:
                        self.pepper_say.publish("We will start with " + self.ItContainer[iteration])
                        self.timeDebugging(3,self.debugMode)

                    # Otherwise the ROS bag already exists
                    else:
                        self.pepper_say.publish("Now we will do " + self.ItContainer[iteration])
                        self.Bag = "Current"
                        self.timeDebugging(3,self.debugMode)

                    #Some comments to introduce the current iteration
                    self.pepper_say.publish("Are you ready?")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("Let's go!")
                    self.timeDebugging(1,self.debugMode)

                    # Pace regulation depending on the iteration
                    if iteration < 1: #Only the first iteration is slow
                        self.Pace = "slow"
                    else:
                        self.Pace = "quick"

                    #If the body is lost stop HRI
                    if self.SawIt == False:
                        break

                    # First Pose is recorded out of the loop to better organise robot intervention

                    self.exercise_loop_client(self.positionContainer[0],self.Bag,1,self.Set,str(iteration + 1),self.Pace)

                    # After performing a pose record the robot will say an= random engaging comments if the "engaging" mode was stablished

                    if self.Mode == "e":
                        self.pepper_say.publish(random.choice(self.engagement_comments)) #Not for safeguard MODE
                        self.timeDebugging(3,self.debugMode)

                    # For the remaining poses repeat the data storing

                    for posIdx, position in enumerate(self.positionContainer[1:]):

                        #If the body is lost stop "for" loop
                        if self.SawIt == False:
                            break

                        # Comments according to pace

                        if self.Pace == "slow":
                            self.pepper_say.publish("New position, ready?")
                            self.timeDebugging(4,self.debugMode)
                        else:
                            self.pepper_say.publish("Next one!")
                            self.timeDebugging(1,self.debugMode)

                        # Function to address data storing for the remaining poses

                        self.exercise_loop_client(position,"Current",posIdx+2,self.Set,str(iteration + 1),self.Pace)

                        # After performing a pose record the robot will say an= random engaging comments if the "engaging" mode was stablished

                        if self.Mode == "e":
                            self.pepper_say.publish(random.choice(self.engagement_comments)) #Not for safeguard MODE
                            self.timeDebugging(3,self.debugMode)

                    #If the body is lost stop HRI
                    if self.SawIt == False:
                        break

                #The session is over all iteration of the three poses were performed
                self.SaidIt= not self.SaidIt #Swap flag

            # If the exercise was finished and the user left the robot ask theuser to call the experimenter

            elif self.SaidIt == True and self.SawIt== False:
                self.pepper_say.publish("I do not see you")
                self.timeDebugging(2,self.debugMode)
                self.pepper_say.publish("Could you please call Daniel?")
                self.timeDebugging(4,self.debugMode)
                break

            # If the exercise was performed and the user is still in sight the robot formally finishes the session

            elif self.SaidIt == True and self.SawIt== True:
                print "Finished!"


                # Depending of the set the session is finished or a break is indicated by the robot

                # Set 1 or 3 required break to later perform set 2 or 4 respectively

                if self.Set==1 or self.Set==3:
                    self.pepper_say.publish("We are done with sequence one")
                    self.timeDebugging(2,self.debugMode)

                    # IF the robot was in "engaging" mode, it adds a comment
                    if self.Mode == "e":
                        self.pepper_say.publish("Your work was really good!") # for the solitary MODE
                        self.timeDebugging(3,self.debugMode)

                    self.pepper_say.publish("It's time for a break!")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("Could you, please, call Daniel?")
                    self.timeDebugging(4,self.debugMode)

                # Set 2 or 4 are the last sets and thus the session end is announced

                else:
                    self.pepper_say.publish("Now we are done with sequence two")
                    self.timeDebugging(2,self.debugMode)
                    self.pepper_say.publish("The exercise session is over")
                    self.timeDebugging(2,self.debugMode)

                    # IF the robot was in "engaging" mode, it adds a comment
                    if self.Mode == "e":
                        self.pepper_say.publish("You did really well today!") # for the solitary MODE
                        self.timeDebugging(3,self.debugMode)

                    self.pepper_say.publish("Could you, please, fill in the questionnaires?")
                    self.timeDebugging(3,self.debugMode)
                    self.pepper_say.publish("They are on the table next to Daniel")
                    self.timeDebugging(4,self.debugMode)
                # Break the code as we do not want the robot to repeat the HRI until the user has had a break
                break

            rate.sleep() #Optional

################################PREPARATION FOR RECORDING#############################

    def exercise_loop_client(self,position,BagState,CurrentPosition,setNumber,ItNumber,Speed): #Service callback
        #show IMAGE

        print "Exercising"
        if Speed == "slow":
            self.pepper_display.publish(self.ImageContainer[CurrentPosition-1])
            self.pepper_say.publish("Please, move your arm to " + str(position))
            self.timeDebugging(5,self.debugMode)          #while not self.Sitted:
            self.pepper_say.publish("Hold it a bit longer, please")
            self.timeDebugging(2,self.debugMode)
        else: #A bit quicker
            self.pepper_display.publish(self.ImageContainer[CurrentPosition-1])
            self.pepper_say.publish(str(position)+"!")
            self.timeDebugging(3,self.debugMode)          #while not self.Sitted:
            self.pepper_say.publish("Stay put")
            self.timeDebugging(1,self.debugMode)

        #Path creation to save ROSbag data and Txt

        BagPath = self.RepoPath + "participant_" + str(self.UserID) + "/set_" + str(setNumber)
        WholePath = self.RepoPath + "participant_" + str(self.UserID) + "/set_" + str(setNumber) + "/P_" + str(CurrentPosition) + "/P_" + str(CurrentPosition) + "_" + str(ItNumber)

        self.service_call(BagPath,WholePath,BagState) #Method that calls the service to save the pose data over time

        self.timeDebugging(1,self.debugMode) #Pause to give time to manage files in data_storin_flag.py #DO NOT COMMET
        self.pepper_say.publish("Release") #Make sure the participant understands WATCH OUT!
        self.timeDebugging(2,self.debugMode)
        self.pepper_display.publish("hide")

################################SERVICE#############################
    def service_call(self, BagPath, WholePath, BagState):
        stop = 0;
        try:
            reply = self.recorder_service(BagPath,WholePath,self.BodyID,BagState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "Expect this if the system is stopped using Ctrl+C"
            stop = 1
            pass
        #Stop the whole program if the service was not available
        if stop == 1:
            self.pepper_say.publish("Ups!")
            self.timeDebugging(1,self.debugMode)
            self.pepper_say.publish("I think something is wrong with the service")
            self.timeDebugging(3,self.debugMode)
            self.pepper_say.publish("Could you please call Daniel?")
            self.timeDebugging(4,self.debugMode)
            exit()
################################MAIN SCRIPT#############################
if __name__ == '__main__':

        #Initialise HRI node and call pepper class
        rospy.init_node('pepper_HRI', anonymous=True)
        pa = PepperOrbbec()
    	time.sleep(4) #Pause for the class instantiation to set up in pepper

        # Set the default pepper dialog off
        pa.pepper_engagement.publish("dialogoff")
    	time.sleep(2)

        # Main function to start the HRI
        pa.pepper_HRI() # Main function to start the HRI
        #
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
