#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
Created on Thursday April 13 2023 10:00:00
@author: tims
"""

# Standard libraries
import os
import sys

import signal

import numpy as np
import time
import math

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

import tf

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
    """
    This class represents a module for executing commands on a NAO robot.
    It inherits from the ALModule class.
    http://doc.aldebaran.com/2-8/naoqi/core/almodule.html
    """

    def __init__(self,name):
        """
        Initializes a new instance of the CommandExecuterModule class.

        :param name: The name of the module.
        """
        global config
        # Change for head turning speed:
        self.maxSpeed = float(config['Timing']['NaoMaxSpeed'])
        self.simulation = int(config['Experiment']['SimulatedNAO'])

        # Never use whole body for movement, as ARMoD is seating
        self.useWholeBody = False

        # Set Frame to 0 as its the stable torso frame of NAO
        self.frame = 0 #0 - TORSO, 1 - World, 2- Robot

        # Define default Arm for the pointing gestures
        self.effector = "LArm"

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

        # Audio player for playing sound files        
        self.player = ALProxy('ALAudioPlayer')

        # Create another proxy as wait is blocking if audioout is remote
        self.playerStop = ALProxy('ALAudioPlayer', True)

        # LED controller for controlling the LEDs on the robot
        self.led_controller = ALProxy("ALLeds")

         # Duration of LED flash in seconds (adjustable)
        self.flash_duration = 1

        global memory
        # Create a global memory proxy to access the robot's memory
        self.memory = ALProxy("ALMemory")

        # ROS Publisher and subscriber:
        # Subscribe to the 'armod_command' topic to receive commands
        self.sub_keypress = rospy.Subscriber('armod_command', String, self.ros_event_callback)

        # Delay between trials of an experimentself.flash_duration
        rospy.init_node('armod_controll', anonymous=True)

        # Disable Autonomous Living: ROBOT WILL STAND UP!
        # Set the robot's posture to "Crouch" with a speed of 0.8
        # self.posture.goToPosture("Crouch", 0.8)
        
        # Create a proxy for the ALAutonomousLife module
        # self.al = ALProxy("ALAutonomousLife")
        
        # Disable the autonomous life state
        # self.al.setState("disabled")
        
        # Create a proxy for the ALMotion module
        self.motion = ALProxy("ALMotion")
        self.initRestPose()
        # self.localize = ALProxy("ALLocalization")

        # Wake up the robot
        # self.somnus.wakeUp()       

    def command_split(self, cmd):
        """
        Splits a received command string into its components.

        :param cmd: The command string to split.
        :return: A tuple containing the command and its arguments.
        """

        split = cmd.split(",")
        if len(split)>1:
            command = split[0]
            x = float(split[1]) # X-RealSense
            y = float(split[2]) # Y-RealSense
            z = float(split[3])    # Z-RealSense
            return command, x, y, z
        else: return split[0], 0, 0, 0
    
    def ros_event_callback(self, data):
        """
        Callback function for handling ROS events.

        :param data: The data received from the ROS event.
        """
        command, x, y, z = self.command_split(data.data)

        if command == "WRN":
            # print("Let NAO point somewhere ...")

            self.updateCoordinates(x, y, z)
            
            self.onCallLook()
            self.flash_eyes("red")
            self.onCallSay("\\rspd=100\\ \\vol=100\\Carefull you are too close!")
            self.onCallPoint("warn")
            
            # print(x,y,z)
            self.onCallRest()
            self.flash_eyes("white")

        elif command == "AKN+WRN":
            self.updateCoordinates(x, y, z)
            self.flash_eyes("red")
            self.onCallLook()
            self.onCallSay("\\rspd=80\\ \\vol=100\\Pay Attention!")
            self.flash_eyes("white")

            self.flash_eyes("green")
            self.onCallLook()
            self.onCallSay("\\rspd=80\\I have seen you")
            # Uncomment the following line to flash the robot's eyes green
            self.onAffirmNod()
            self.flash_eyes("white")
            
            # print(x,y,z)
            time.sleep(0.6)
            self.onCallRest()

        elif command == "quit":
            self.exit_flag = True
            
        elif command == "AKN":
            self.updateCoordinates(x, y, z)

            self.flash_eyes("green")
            self.onCallLook()
            self.onCallSay("\\rspd=80\\Hello I can see you")
            # Uncomment the following line to flash the robot's eyes green
            self.onAffirmNod()

            self.flash_eyes("white")
            self.onCallRest()

        elif command == "Look":
            # print(x,y,z)
            self.updateCoordinates(x, y, z)
            self.onCallLook()
            # print(x,y,z)
            time.sleep(0.8)
            self.onCallRest()

        else:
            print(data.data)
            if not self.simulation:      
                self.get_pose(verbose=True)      

    def updateCoordinates(self, x, y, z):
        """
        Updates the coordinates of the robot.

        :param x: The new x-coordinate.
        :param y: The new y-coordinate.
        :param z: The new z-coordinate.
        """
        self.x = x
        self.y = y
        self.z = z
        self.resting = False

    def check_FOV_limits(self, vector):
        """
        This function checks if a vector is within the FOV of a robot modelled after human FOV
        The rest position of the robot's head is modulated with the vector [x=1, y=0, z=0].
        The FOV is maximal 200 degrees horizontally and 150 degrees vertically.

        Parameters:
        vector (list): a list of three numbers representing the x, y, and z components of the vector.

        Returns:
        None: prints "Feasible vector" or "Not a feasible vector" depending on the angle between the vector and the rest position vector.

        Explanation:
        The angle between the vector and the rest position vector is the same as the angle between the vector and the x-axis.
        This angle can be used to check both the horizontal and vertical FOV ranges.
        For example, if the angle is 90 degrees, then the vector is perpendicular to the x-axis and lies on the yz-plane.
        This means that it is at the edge of the horizontal FOV range, but it could be anywhere in the vertical FOV range.
        If the angle is 45 degrees, then the vector is halfway between the x-axis and the yz-plane.
        This means that it is within both the horizontal and vertical FOV ranges.
        If the angle is 120 degrees, then the vector is opposite to the x-axis and lies on the negative yz-plane.
        This means that it is outside both the horizontal and vertical FOV ranges.
        """
        
        # calculate the angle between the vector and the rest position vector
        # rest postion vector per default x=1, y=0, z=0
        dot_product = vector[0] * 1 + vector[1] * 0 + vector[2] * 0
        vector_magnitude = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
        rest_magnitude = math.sqrt(1**2 + 0**2 + 0**2)
        angle = math.acos(dot_product / (vector_magnitude * rest_magnitude))
        angle = math.degrees(angle)

        # check if the angle is within the FOV ranges
        if angle <= 100: # horizontal range is +/- 100 degrees from the rest position
            if angle <= 75: # vertical range is +/- 75 degrees from the rest position
                print("Feasible vector")
                return True
            else:
                print("Not a feasible vector")
                return False
        else:
            print("Not a feasible vector")
            return False

    def onCallLook(self):
        """Lets NAO look to a certain Position"""

        valid = self.check_FOV_limits(np.array([self.x, self.y, self.z]))
        if valid:
            try:
                self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)
            except Exception as excpt:
                print(excpt)
        else:
            print("Attempted non valid Gaze")
            return

    def get_pose(self, verbose=False):
        #position = self.motion.getRobotPosition(True)
        ax = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
        ay = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
        az = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value")
        ax = np.rad2deg(ax)
        ay = np.rad2deg(ay)
        az = np.rad2deg(az)

        q = tf.transformations.quaternion_from_euler(ax, ay, az)
        orientation_string = "orientation,"+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])

        self.pub.publish(orientation_string)

        if verbose:
            print("Orientation Euler", (ax, ay, az))
            print("Orientation Quaternion", q)

    def onCallPoint(self, gesture=None):

    # TODO add decision mechanic for arm to point with

        if gesture is None:
            if self.y > 0:
                self.effector = "LArm"
            else:
                self.effector = "RArm"
            try:
                self.tracker.pointAt(self.effector, [self.x, self.y, self.z], self.frame, self.maxSpeed)
            except Exception as excpt:
                print(excpt)
        else:
            if gesture == "warn":
                if self.effector == "LArm":
                    hand = "LHand"
                if self.effector == "RArm":
                    hand = "RHand"
                    
                self.motion.openHand(hand)
                self.tracker.pointAt(self.effector, [self.x, self.y, self.z], self.frame, self.maxSpeed)
                time.sleep(0.4)
                self.motion.closeHand(hand)

    def onCallSay(self, text):
        """
        Lets NAO say the given text.

        :param text: The text for NAO to say.
        """

        try:
            self.tts.say(text)
        except Exception as excpt:
            print(excpt)     

    def flash_eyes(self, color=None):
        """
        Flashes the robot's eyes with the given color.

        :param color: The color to flash the robot's eyes with.
        """

        sGroup = "FaceLeds"

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

    def onAffirmNod(self):
        """Lets NAO nod his head - Exported from Choregraphe"""
        # Choregraphe simplified export in Python.
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.52, 0.66, 0.8, 0.94, 1.08, 1.22, 1.36, 1.48])
        keys.append([-0.413643, -0.0610865, 0.29147, 0.115192, -0.413643, -0.0610865, 0.29147, -0.15708])

        # names.append("HeadYaw")
        # timeand I would like to start working with the perception messages.s.append([0.52, 0.8, 1.08, 1.28, 1.48])
        # keys.append([0, 0, 0, 0, 0])

        try:
            self.tts.say("Hello there!")
            motion = ALProxy("ALMotion")
            motion.angleInterpolation(names, keys, times, True)
        except BaseException as err:
            print(err)

    def initRestPose(self):
        """Code Snippet from Choregraphe
            Lets NAO move back to its original posture
            This posture can easily be defined using choregraphs timeline box
            A defined posture there can be exported as bezier style python code
        """
        # Go back to initial Pose, to prevent unwanted behavior
        # Choregraphe bezier export in Python.
        self.rest_names = list()
        self.rest_times = list()
        self.rest_keys = list()
        self.rest_names.append("HeadPitch")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-0.0153821, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("HeadYaw")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-0.021518, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LElbowRoll")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-1.20568, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LElbowYaw")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-0.490922, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LHand")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.0104001, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LShoulderPitch")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.918824, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LShoulderRoll")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.28835, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("LWristYaw")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.0429101, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RElbowRoll")
        self.rest_times.append([1.2])
        self.rest_keys.append([[1.29934, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RElbowYaw")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.454022, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RHand")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.0388, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RShoulderPitch")
        self.rest_times.append([1.2])
        self.rest_keys.append([[0.91584, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RShoulderRoll")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-0.339056, [3, -0.4, 0], [3, 0, 0]]])
        self.rest_names.append("RWristYaw")
        self.rest_times.append([1.2])
        self.rest_keys.append([[-0.044528, [3, -0.4, 0], [3, 0, 0]]])

    def onCallRest(self):
        self.motion.angleInterpolationBezier(self.rest_names, self.rest_times, self.rest_keys)


# Main Function for NaoPosner Experiment
class ArmodIntegrationClass():
    """
    This class holds the primary functionalties for the ARMoD experiment.
    """

    def __init__(self, naoip, naoport):
        """
        Initializes a new instance of the ArmodIntegrationClass class.

        :param naoip: The IP address of the NAO robot.
        :param naoport: The port number of the NAO robot.
        """

        global config

        # Name must match variable name
        self.myBroker = ALBroker("myBroker",
        "0.0.0.0",
        0,
        naoip,
        naoport)

        self.CommandExecuter = CommandExecuterModule("CommandExecuter")
        self.DEBUG = int(config['Experiment']['Debug'])


        rp = {}
        x = float(config['Experiment']['X'])
        y = float(config['Experiment']['Y'])
        z = float(config['Experiment']['Z'])

        rp = {"x":x,
              "y":y,
              "z":z}

        self.rest_pose = rp

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('Armod Programm will be terminated')
        self.myBroker.shutdown()
        sys.exit(0)


    def nao_head_rest(self):
        """
        Sets the NAO robots Head to its resting position.
        """
        self.CommandExecuter.tracker.lookAt([self.rest_pose["x"], self.rest_pose["y"], self.rest_pose["z"]], 0, self.CommandExecuter.maxSpeed, False)
        self.CommandExecuter.resting = True

if __name__ == '__main__':
    # # Start a ROS node that will run in parallel to the main loop
    # rospy.init_node('my_node')

    # Create a new instance of the ArmodIntegrationClass with the given NAO IP and port (Config.ini)
    e = ArmodIntegrationClass(NAO_IP, NAO_PORT)

    # If debug mode is enabled (Config.ini)
    # Test implementation
    if e.DEBUG:
        print("NAO will look somewhere ...")
        e.CommandExecuter.updateCoordinates(0.5, 1, 0.5)
        e.CommandExecuter.onCallLook()
        time.sleep(3)
        print("Bring NAO back to resting position ...")
        e.nao_head_rest()

        print("Let NAO point somewhere ...")
        e.CommandExecuter.updateCoordinates(0.5, 1, 0.5)
        e.CommandExecuter.onCallPoint()
        time.sleep(3)
        e.CommandExecuter.posture.goToPosture("Sit", 0.8)

        print("Let NAO say something ...")
        e.CommandExecuter.onCallSay("Hello there!")

    try:
        # Keep running until the exit flag is set
        while not e.CommandExecuter.exit_flag:
            if e.CommandExecuter.exit_flag:
                break
            else:
                pass
    except KeyboardInterrupt:
        e.myBroker.shutdown()
        sys.exit(0)