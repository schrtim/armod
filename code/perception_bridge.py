#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import rospy
from std_msgs.msg import String
from darko_perception_msgs.msg import Humans
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
import numpy as np

import configparser
import traceback
import time
import random
from datetime import datetime

class ArmodCommand:
    """A class that publishes commands to the Armod robot based on user input and perception data."""

    def __init__(self):
        """Initialize the node, publisher, subscriber and perception data."""
        # Create a ROS node with a unique name
        rospy.init_node('armod_command_publisher', anonymous=True) 

        config = configparser.ConfigParser()
        config.read('config.ini')
        self.simulation = int(config['Experiment']['SimulatedNAO'])
        self.ack_threshold = int(config['Experiment']['AcknowledgeThreshshold'])
        self.debug = int(config["Experiment"]["Debug"])

        self.detection_fid = "robot_map"

        # Create a publisher  with std_msg type for the armod_command topic 
        # Std_msg are important here as they have to be send between different ROS distors
        self.pub = rospy.Publisher('armod_command', String, queue_size=10) 

        # Create a subscriber for the perception topics
        self.sub_keypress = rospy.Subscriber('/perception/humans', Humans, self.receive_perception_message) 
        self.listener = tf.TransformListener()

        self.recently_acknowledged = {}

        # Wait for the transform to become available

        if not self.simulation:
            # The real thing
            self.listener.waitForTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(0), rospy.Duration(4.0))

            # Get the transformation matrix
            (trans, rot) = self.listener.lookupTransform("robot_armod_frame", "robot_k4a_top_rgb_camera_link", rospy.Time(4.0))
            self.trans = np.array(trans)
            # Convert the rotation from quaternion to a 4x4 matrix
            rot_mat = quaternion_matrix(rot)

            # Add the translation to the matrix
            trans_mat = np.identity(4)
            trans_mat[:3, 3] = trans

            # Combine the rotation and translation into a single transformation matrix
            self.TM = np.dot(trans_mat, rot_mat)
        else:
            self.renew_transform()

        self.current_detections = {}

    def renew_transform(self):
        self.listener.waitForTransform("robot_base_footprint", self.detection_fid,rospy.Time(0), rospy.Duration(4))
        (trans, rot) = self.listener.lookupTransform("robot_base_footprint", self.detection_fid, rospy.Time(0))
        print(trans, rot)
        # trans[0] += 0.3
        # trans[2] += 0.8
        self.trans = np.array(trans)
        # Convert the rotation from quaternion to a 4x4 matrix
        rot_mat = quaternion_matrix(rot)

        self.TF = rot_mat
        # Add the translation to the matrix
        self.TF[:3, 3] = trans
             
    def send_command(self, command):
        """Publish a command to the armod_command topic as a string"""
        # If the code is modifed this could be any std_msg
        self.pub.publish(command) 

    def receive_perception_message(self, data):
        """Receive and process the perception data from the face_detections topic."""
        self.remove_old_acknowledgements()
        head = data.header
        
        if head.frame_id != self.detection_fid:
            print(f"Frame ID of detections changed from: {self.detection_fid} to {head.frame_id} at {datetime.now}")
            self.detection_fid = head.frame_id
            self.renew_transform()

        for human in data.humans:
            id = human.id
            pose = human.centroid.pose.position
            pose = np.array([pose.x, pose.y, pose.z, 1])
            timestamp = datetime.now()

            transformed_point = np.dot(self.TF, pose)
            transformed_point[0]-= 0.3
            transformed_point[2] = transformed_point[2]+(transformed_point[2]*0.8)-0.8
            #print(id, transformed_point)

            self.current_detections[id] = [head, transformed_point, timestamp]
                
        num_items = len(self.current_detections)
        if num_items > 1000:
            self.current_detections = {}

    def remove_old_acknowledgements(self):
        """
        Remove all key-value pairs from the recently_acknowledged dictionary if the values are older than self.ack_threshold seconds.

        This function iterates over the recently_acknowledged dictionary and checks the age of each value. If the age is greater than self.ack_threshold seconds, the corresponding key-value pair is removed from the dictionary.
        """
        current_time = datetime.now()
        keys_to_remove = []
        for cid, timestamp in self.recently_acknowledged.items():
            age = (current_time - timestamp).total_seconds()
            if age > self.ack_threshold:
                keys_to_remove.append(cid)
        for cid in keys_to_remove:
            del self.recently_acknowledged[cid]

    def closest_point(self):
        """
        Find the closest point to [0,0,0] in the current_detections dictionary.

        This function iterates over the current_detections dictionary and calculates the Euclidean distance of each point to [0,0,0]. It keeps track of the point with the minimum distance and returns its id, transformed_point and timestamp.

        Returns:
            tuple: A tuple containing the id, transformed_point and timestamp of the closest point to [0,0,0] in the current_detections dictionary.
        """        
        self.current_time = datetime.now()


        closest_id = None
        min_distance = float('inf')
        for id, points in self.current_detections.items():
            head, transformed_point, timestamp = points
            distance = sum([x**2 for x in transformed_point])**0.5
            if distance < min_distance:
                min_distance = distance
                closest_id = id
        return closest_id, self.current_detections[closest_id][1], timestamp

    def run(self):
        """Run the main loop of the node.
        
        React on perceptions/detections and send commands to NAO
        TODO: Add Random Gazing Engine / Acknowledgement engine
        """
        while not rospy.is_shutdown(): # loop until the node is shut down
            try:
                key = input("Press a key: ") # get user input from keyboard
                
                if key == 'p': # if the user presses p

                    cid, cl, t = self.closest_point()
                    self.send_command("point,"+str(cl[0])+","+str(cl[1])+","+str(cl[2])) # send a nod command with the perception data
                    dt = self.current_time-t
                    print(f"Age of this detection: {dt.seconds}")
                    
                    # print(self.pd) # print the perception data
                elif key == 'l': # if the user presses l

                    cid, cl, t = self.closest_point()
                    self.send_command("look,"+str(cl[0])+","+str(cl[1])+","+str(cl[2])) # send a nod command with the perception data
                    dt = self.current_time-t
                    print(f"Age of this detection: {dt.seconds}")

                elif key == 'q': # if the user presses q
                    i = 0
                    while i < 4:
                        self.send_command("quit,"+str(0)+","+str(0)+","+str(0)) # send a quit command with zero coordinates
                        i += 1
                        time.sleep(0.3)
                    break
                    
                elif key == 'n': # if the user presses n

                    cid, cl, t = self.closest_point()
                    is_acknowledged = cid in self.recently_acknowledged
                    if not is_acknowledged:
                        self.send_command("nod,"+str(cl[0])+","+str(cl[1])+","+str(cl[2])) # send a nod command with the perception data
                        dt = self.current_time-t
                        print(f"Age of this detection: {dt.seconds}")
                        self.recently_acknowledged[cid] = datetime.now()
                    else:
                        print(f"Person ID: {cid}, already acknowledged")

                else: # if the user presses any other key
                    self.send_command(key) # send the key as a command
            except Exception as e:
                traceback.print_exc()

if __name__ == '__main__':
    armod_command = ArmodCommand() # create an instance of the ArmodCommand class
    armod_command.run() # run the main loop of the node