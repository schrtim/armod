#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from darko_perception_msgs.msg import Humans

class ArmodCommand:
    """A class that publishes commands to the Armod robot based on user input and perception data."""

    def __init__(self):
        """Initialize the node, publisher, subscriber and perception data."""
        # Create a ROS node with a unique name
        rospy.init_node('armod_command_publisher', anonymous=True) 

        # Create a publisher  with std_msg type for the armod_command topic 
        # Std_msg are important here as they have to be send between different ROS distors
        self.pub = rospy.Publisher('armod_command', String, queue_size=10) 

        # Create a subscriber for the perception topics
        self.sub_keypress = rospy.Subscriber('/perception/humans', Humans, self.receive_perception_message) 

        # Initialize an empty list for storing the perception data
        # Dictionary also possible if working with detection IDs
        self.pd = [] 

    def send_command(self, command):
        """Publish a command to the armod_command topic as a string"""
        # If the code is modifed this could be any std_msg
        self.pub.publish(command) 

    def receive_perception_message(self, data):
        """Receive and process the perception data from the face_detections topic."""

        self.current_detections = {}

        head = data.header
        for human in data.humans:
            id = human.id
            pose = human.centroid
            twist = human.velocity
            self.current_detections[id] = [head, pose, twist]

    def run(self):
        """Run the main loop of the node.
        
        React on perceptions/detections and send commands to NAO
        TODO: Adapt to DARKO Perception Stack
        """

        while not rospy.is_shutdown(): # loop until the node is shut down
            key = input("Press a key: ") # get user input from keyboard
            if key == 'p': # if the user presses p
                # self.send_command("point,"+str(self.pd[0])+","+str(self.pd[1])+","+str(self.pd[2])) # send a point command with the perception data
                print(self.current_detections)
                # print(self.pd) # print the perception data
            elif key == 'l': # if the user presses l
                # self.send_command("look,"+str(self.pd[0])+","+str(self.pd[1])+","+str(self.pd[2])) # send a look command with the perception data
                # print(self.pd) # print the perception data
                pass
            elif key == 'q': # if the user presses q
                # self.send_command("quit,"+str(0)+","+str(0)+","+str(0)) # send a quit command with zero coordinates
                break # exit the loop
            elif key == 'n': # if the user presses n
                # self.send_command("nod,"+str(self.pd[0])+","+str(self.pd[1])+","+str(self.pd[2])) # send a nod command with the perception data
                pass
            else: # if the user presses any other key
                self.send_command(key) # send the key as a command

if __name__ == '__main__':
    armod_command = ArmodCommand() # create an instance of the ArmodCommand class
    armod_command.run() # run the main loop of the node