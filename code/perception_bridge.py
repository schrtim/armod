#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import rospy
from std_msgs.msg import String
from darko_perception_msgs.msg import Humans
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
import numpy as np

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
        self.listener = tf.TransformListener()

        # Wait for the transform to become available
        self.listener.waitForTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(0), rospy.Duration(4.0))

        # Get the transformation matrix
        (trans, rot) = self.listener.lookupTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(0))
        
        self.trans = np.array(trans)

        # Convert the rotation from quaternion to a 4x4 matrix
        rot_mat = quaternion_matrix(rot)

        # Add the translation to the matrix
        trans_mat = np.identity(4)
        trans_mat[:3, 3] = trans

        # Combine the rotation and translation into a single transformation matrix
        self.TM = np.dot(trans_mat, rot_mat)

        self.current_detections = {}

    def send_command(self, command):
        """Publish a command to the armod_command topic as a string"""
        # If the code is modifed this could be any std_msg
        self.pub.publish(command) 

    def receive_perception_message(self, data):
        """Receive and process the perception data from the face_detections topic."""

        # Count the number of items in the dictionary
        num_items = len(self.current_detections)
        if num_items > 1000:
            self.current_detections = {}

        head = data.header
        for human in data.humans:
            id = human.id
            pose = human.centroid.pose.position
            pose = np.array([pose.x, pose.y, pose.z])
            twist = human.velocity

            # # Wait for the transform to be available
            # self.listener.waitForTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(), rospy.Duration(0))

            # # Create a PoseStamped object from the pose and header
            # pose_stamped = PoseStamped()
            # pose_stamped.header = head
            # pose_stamped.pose = pose.pose
            pose += self.trans
            pose[2] += 0.5

            # Transform the pose
            # pose_transformed = self.listener.transformPose("robot_armod_frame", pose_stamped)

            self.current_detections[id] = [head, pose]#, twist]

    def get_closest_human(self):
        min_abs = 10000
        min_key = 0
        for key, value in self.current_detections.items():
            abs = np.linalg.norm(value[1])
            if abs < min_abs:
                min_abs = abs
                min_key = key
        
        return self.current_detections[min_key][1]

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
                self.get_closest_human()
                # print(self.pd) # print the perception data
            elif key == 'l': # if the user presses l
                # Get the transformation matrix
                (trans, rot) = self.listener.lookupTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(0))
                print(self.TM)

                # Print the translation and rotation
                print("Translation:", trans)
                print("Rotation:", rot)
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