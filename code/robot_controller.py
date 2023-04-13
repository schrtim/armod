#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
Created on Thursday April 13 2023 10:00:00
@author: tims
"""

# Standard libraries
import os
import sys
import numpy as np
import time

# Import ROS, use only std_msg here, as they can be sent between distributions
import rospy
from std_msgs.msg import String

# Option parsers
from optparse import OptionParser
import configparser

# For logging timestamps
from datetime import datetime

# Import Naoqi Modules
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

## Read the configurables from the config file
config = configparser.ConfigParser()
config.read('config.ini')

# If working with the real robot, this needs to be adjusted
NAO_IP = str(config['NAO']['Ip'])
NAO_PORT = int(config['NAO']['Port'])

# Gloabl Variables to store the modules
CommandExecuter = None
memory = None

class CommandExecuterModule(ALModule):
    """Counts seen Faces"""

    def __init__(self,name):

        global config
        # Change for head turning speed:
        self.maxSpeed = float(config['Timing']['NaoMaxSpeed'])

        # Never use whole body for movement, as ARMoD is seating
        self.useWholeBody = False

        # Set Frame to 0 as its the stable torso frame of NAO
        self.frame = 0 #0 - TORSO, 1 - World, 2- Robot

        # Intializie Coordinates with resting position
        self.x = float(config['Experiment']['X'])
        self.y = float(config['Experiment']['Y'])
        self.z = float(config['Experiment']['Z'])

        self.old_x = 0.0
        self.old_y = 0.0
        self.old_z = 0.0

        self.alive = True
        self.resting = True

        self.reaction_times = []

        self.exit_flag = False

        ALModule.__init__(self,name)
        # No need for IP Adress bc PyBroker connected to Naoqi PyBroker

        # Create Proxy ALtts for letting NAO speak
        self.tts = ALProxy("ALTextToSpeech") # No IP Adress or ports needed
        self.posture = ALProxy("ALRobotPosture")
        # The tracker module is for coordinate related movements
        self.tracker = ALProxy( "ALTracker" )
        # Enbales to run pre-installed behaviours
        self.bm = ALProxy("ALBehaviorManager")
        # Enables Playback of soundfiles
        self.framemanager = ALProxy("ALFrameManager")
        self.player = ALProxy('ALAudioPlayer')
        self.playerStop = ALProxy('ALAudioPlayer', True) #Create another proxy as wait is blocking if audioout is remote

        self.led_controller = ALProxy("ALLeds")
        self.flash_duration = 2 # Adjustable
        self.delay_between_trials = 1 # Setbased on Lucas preferences

        global memory
        memory = ALProxy("ALMemory")

        # Set a starting position for the robot here if desired
        # self.posture.goToPosture("Crouch", 1.0)
        # time.sleep(3)

        # ROS Pubs n subs:
        # self.pub_stimulus = rospy.Publisher('stimulus', String, queue_size=0)
        # self.pub_logger = rospy.Publisher('logger', String, queue_size=0)
        # self.pub_result = rospy.Publisher('results', String, queue_size=0)

        # self.sub_keypress = rospy.Subscriber('keypress', String, self.keypressCb)
        # rospy.init_node('nao_cueing', anonymous=True)

    def wait_for_detection_or_intervall(self):
        """
            This function will process the given timestamp of when NAO turned the head.
            Wait time can be adopted by changing the loop boundaries.
            Currently its 100*0.02s = 2s
            in:
            - Naos timestamp in datetime format
            return: None
            Function will wait for 2s or until key is pressed.            
        """
        pass

    def detection_event(self, data):
        pass

    def updateCoordinates(self,x,y,z):
        """Function that simply updates Parameters"""
        self.x = x
        self.y = y
        self.z = z
        self.resting = False

    def onCallLook(self):
        """Lets NAO look to a certain Position"""

        try:
            self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)
        except Exception as excpt:
            print(excpt)
                    
    def flash_eyes(self, color=None):

        sGroup = "FaceLeds"

        if color == "blue":
            duration = self.delay_between_trials
        else:
            duration = self.flash_duration
        p = color

        self.ids = []
        self.leds = ALProxy("ALLeds")
        sGroup = "FaceLeds"

        #id = self.leds.post.fadeRGB(sGroup, float(p[0])/255, float(p[1])/255, float(p[2])/255, duration)
        id = self.leds.post.fadeRGB(sGroup, p, duration)
        self.ids.append(id)
        self.leds.wait(id, 0)
        self.ids.remove(id)

    def set_eyes(self, state=True):

        sGroup = "FaceLeds"
        if state:
            self.led_controller.on(sGroup)
        else:
            self.led_controller.off(sGroup)

# Main Function for NaoPosner Experiment
class ArmodIntegrationClass():

    def __init__(self, naoip, naoport):

        # Name must match variable name
        self.myBroker = ALBroker("myBroker",
        "0.0.0.0",
        0,
        naoip,
        naoport)

        self.CommandExecuter = CommandExecuterModule("CommandExecuter")

        global config
        rp = {}
        x = float(config['Experiment']['X'])
        y = float(config['Experiment']['Y'])
        z = float(config['Experiment']['Z'])

        rp = {"x":x,
              "y":y,
              "z":z}

        self.rest_pose = rp

    def nao_rest(self):
        self.CommandExecuter.tracker.lookAt([self.rest_pose["x"], self.rest_pose["y"], self.rest_pose["z"]], 0, self.CommandExecuter.maxSpeed, False)
        self.CommandExecuter.resting = True

if __name__ == '__main__':
    # # Start a ROS node that will run in parallel to the main loop
    # rospy.init_node('my_node')

    e = ArmodIntegrationClass(NAO_IP, NAO_PORT)

    print("NAO will look somewhere ...")
    e.CommandExecuter.updateCoordinates(2, 1, 3)
    e.CommandExecuter.onCallLook()
    time.sleep(3)
    print("Bring NAO back to resting position ...")
    e.nao_rest()
    e.myBroker.shutdown()
    sys.exit(0)