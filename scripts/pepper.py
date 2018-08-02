#!/usr/bin/env python

import rospy
import os
import json
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import qi

from naoqi import ALProxy

MOVE_ENABLED = False

#robotIP = "pepper.local"
robotIP = "192.168.0.113"
PORT = 9559

# Location of map file on pepper
#map_path = "/home/nao/.local/share/Explorer/2017-04-27T160526.973Z.explo"
map_path = "/home/nao/.local/share/Explorer/2017-09-08T152701.475Z.explo"

# Hard coded locations
map_home = (-0.4681686460971832, 0.25265851616859436, -1.2069545984268188)
map_chair = (0.4053499698638916, -2.1418206691741943, -2.2219252586364746)
map_kitchen = (-0.9714147448539734, -1.0047649145126343, -1.796849012374878)


class Pepper():

    def __init__(self, ip, port):

        self.reinit = False

        self._robotIP = ip
        self._PORT = port

        self.session = qi.Session()

        self.status = rospy.Publisher('pepper/status', String, queue_size=1)

        self.setup()

        # Stop, reload and relocalize map (requires pepper to be at home when you start this
        #self.navigationProxy.stopLocalization()
        #self.navigationProxy.loadExploration(map_path)
        #self.navigationProxy.relocalizeInMap((0.0,0.0,0.0))
        #self.navigationProxy.startLocalization()

    def check_ping(self):

        response = os.system("ping -c 1 " + self._robotIP)
        # and then check the response...
        if response == 0:
            pingstatus = True
        else:
            pingstatus = False

        return pingstatus


    def setup(self):

        try:
            self.session.connect("tcp://" + self._robotIP + ":" + str(self._PORT))

            self.tabletService = self.session.service("ALTabletService")
            self.tabletTimeoutLength = 60 #seconds
            self.tabletTimeout = time.time()
            self.tabletFlag = False

            self.asr_service = self.session.service("ALAnimatedSpeech")
            self.asr_configuration = {"bodyLanguageMode":"contextual"}

        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + self._PORT + "\" on port " + str(self._PORT) +".\n"
               "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

        try:
            # @TODO see if these need to be converted from ALProxy to session.service calls?

            self.tts = ALProxy("ALTextToSpeech", self._robotIP, self._PORT)

            self.motionProxy  = ALProxy("ALMotion", self._robotIP, self._PORT)
            self.localizationProxy = ALProxy("ALLocalization", self._robotIP, self._PORT)

            self.navigationProxy = ALProxy("ALNavigation", self._robotIP, self._PORT)

            self.baProxy = ALProxy("ALBasicAwareness", self._robotIP, self._PORT)

            self.listeningProxy = ALProxy("ALListeningMovement", self._robotIP, self._PORT)
            self.lifeProxy = ALProxy("ALAutonomousLife", self._robotIP, self._PORT)
            self.dialogProxy = ALProxy("ALDialog", self._robotIP, self._PORT)
            self.system = ALProxy("ALSystem", self._robotIP, self._PORT)

            rospy.loginfo("Connected to Pepper at " + self._robotIP + ":" + str(self._PORT))
            self.reinit = False

        except Exception,e:
            rospy.loginfo("Failed to connect to Pepper, is it on, and is the IP address correct?")
            rospy.loginfo("IP : " + self._robotIP)
            rospy.loginfo("PORT : " + str(self._PORT))
            rospy.loginfo(e)


    def twist_callback(self, data):

        if MOVE_ENABLED:
            self.motionProxy.moveTo(data.linear.x, 0.0, data.angular.z)


    def say_callback(self, data):

        rospy.loginfo("%s", data.data)
        self.say(data.data)


    def display_callback(self, data):

        rospy.loginfo("%s", data.data)
        self.display(data.data)


    def display(self, data):

        #@TODO add videos
        rospy.loginfo ("Displaying: " + data)
        self.tabletTimeout = time.time()
        try:
            # If it ends with an image extension, it's probably one of those
            video_ext = ["mp4"]
            img_ext = ["jpg", "jpeg", "png", "gif"]
            if(data.endswith(tuple(img_ext))):
                r = self.tabletService.showImage(data)
                self.tabletFlag = "image"
            elif(data.endswith(tuple(video_ext))):
                r = self.tabletService.playVideo(data)
                self.tabletFlag = "video"
            else:
                r = self.tabletService.showWebview(data)
                self.tabletFlag = "webview"

            rospy.loginfo(self.tabletFlag)
            rospy.loginfo("Result: " + str(r))

        except Exception,e:
            rospy.loginfo("Pepper TableService failed due to:")
            rospy.loginfo(e)
            self.reinit = True


    def say(self, data):

        rospy.loginfo ("Saying: " + data)

        try:
            #Contextual animated say
            self.asr_service.say(data, self.asr_configuration)
            # Normal say
            #self.tts.say(data)
        except Exception,e:
            rospy.loginfo("Pepper TTS failed due to:")
            rospy.loginfo(e)
            self.reinit = True


    def go_callback(self, data):
        rospy.loginfo("%s", data.data)

        if MOVE_ENABLED:
            if(data.data == "Kitchen"):
                self.navigationProxy.navigateToInMap(map_kitchen)
            if(data.data == "My Armchair"):
                self.navigationProxy.navigateToInMap(map_chair)
            if(data.data == "Home"):
                self.navigationProxy.navigateToInMap(map_home)


    def learnHome(self):

        if MOVE_ENABLED:
            # Learning home.
            ret = self.localizationProxy.learnHome()
            # Check that no problem occurred.
            if ret == 0:
                print "Learning OK"
            else:
                print "Error during learning " + str(ret)


    def goHome(self):

        if MOVE_ENABLED:
            # Go back home.
            ret = self.localizationProxy.goToHome()
            # Check that no problem occurred.
            if ret == 0:
                print "go to home OK"
            else:
                print "error during go to home " + str(ret)


    def nav_callback(self, data):

        dargs = data.data.split()

        try:

            # Basic Movements:
            if(data.data == "turnLeft"):
                self.motionProxy.moveTo(0.0, 0.0, -1.0)
            if(data.data == "turnRight"):
                self.motionProxy.moveTo(0.0, 0.0, 1.0)

            # SLAM:
            if(data.data == "learnHome"):
                self.learnHome()
            if(data.data == "orient"):
                val = self.localizationProxy.getRobotOrientation(True)
                print val
            if(data.data == "goHome"):
                self.goHome()
            if((dargs[0] == "navTo") and (len(dargs) == 3)):
                self.navigationProxy.navigateTo( float(dargs[1]), float(dargs[2]) )
            if((dargs[0] == "moveTo") and (len(dargs) == 4)):
                self.motionProxy.moveTo( float(dargs[1]), float(dargs[2]), float(dargs[3]) )

            # Start exploration mode, find working environment:
            if(data.data == "explore"):
                self.navigationProxy.explore(3.0)
            if(data.data == "stopExplore"):
                self.navigationProxy.stopExploration()
            if(data.data == "saveExplore"):
                path = self.navigationProxy.saveExploration()
                print "saved at: " + path

            # Control localization:
            if(data.data == "stopLocal"):
                self.navigationProxy.stopLocalization()
            if(data.data == "startLocal"):
                self.navigationProxy.stopLocalization()
                self.navigationProxy.loadExploration(map_path)
                self.navigationProxy.relocalizeInMap((0.0,0.0,0.0))
                self.navigationProxy.startLocalization()

            # Use coordinates to go to a location on the map:
            if((dargs[0] == "mapTo") and (len(dargs) == 4)):
                self.navigationProxy.navigateToInMap( (float(dargs[1]), float(dargs[2]), float(dargs[3])) )

            # Load a value for a location:
            if(data.data == "mapGet"):
                val = self.navigationProxy.getRobotPositionInMap()
                print val

        except:
                print "error"


    def cmd_callback(self, data):

        dargs = data.data.split()

        try:
                #STOP dialog responses
                #http://doc.aldebaran.com/2-4/naoqi/interaction/dialog/aldialog-api.html#aldialog-api
                if(data.data == "DialogOff"):
                    self.dialogProxy.stopDialog()
                if(data.data == "DialogOn"):
                    self.dialogProxy.runDialog()

                #http://doc.aldebaran.com/2-1/naoqi/core/autonomouslife_advanced.html#autonomouslife-disabled
                if(data.data == "NotAlive"):
                    self.lifeProxy.setState("safeguard") #Disabled all functionalities. Floffy robot
                if(data.data == "Alive"):
                    self.lifeProxy.setState("solitary") #Comes back into solitary mode and from there it goes to interactive (NEVER GO STRAIGHT)

                if(data.data == "engage"):
                    self.baProxy.startAwareness()
                    self.baProxy.setEnabled(True)
                    self.listeningProxy.setEnabled(True)
                if(data.data == "disengage"):
                    self.baProxy.setEnabled(False)
                    self.baProxy.stopAwareness()
                    self.listeningProxy.setEnabled(False)
                if(data.data == "shutdown"):
                    self.system.shutdown()
        except:
                print "error"


    def update(self):

        rospy.Subscriber("pepper/say", String, self.say_callback)
        rospy.Subscriber("pepper/display", String, self.display_callback)
        rospy.Subscriber("pepper/cmd", String, self.cmd_callback)
        rospy.Subscriber("pepper/nav", String, self.nav_callback)
        rospy.Subscriber("pepper/go", String, self.go_callback)
        rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, self.twist_callback)

        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():

            ### Perform Ping test to check connection is alive:
            if(self.check_ping() == False):
                rospy.loginfo("Pepper not connected")
                self.status.publish("Offline")
            else:
                rospy.loginfo("Pepper alive")
                self.status.publish("Online")

                if(self.tabletFlag != False):

                    elapsed_time = time.time() - self.tabletTimeout
                    if(elapsed_time >= self.tabletTimeoutLength):
                        if(self.tabletFlag == "webview"):
                            self.tabletService.hideWebview()
                        if(self.tabletFlag == "image"):
                            self.tabletService.hideImage()
                        if(self.tabletFlag == "video"):
                            self.tabletService.stopVideo()

                ### Check if a reconnection to the services is needed
                if(self.reinit == True):
                    self.setup()
                    self.reinit = False
                else:
                    rate.sleep()



if __name__ == '__main__':
    rospy.init_node("pepper_controller", anonymous=True)
    pepper = Pepper(robotIP, PORT)
    pepper.update()
    rospy.spin()
