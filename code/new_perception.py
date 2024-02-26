#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import rospy
from std_msgs.msg import String
from darko_perception_msgs.msg import Humans
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
import numpy as np

from visualization_msgs.msg import Marker

import configparser
import traceback
import time
import random
from datetime import datetime

class ArmodPerception:
    def __init__(self) -> None:
        rospy.init_node('armod_perception_commands', anonymous=True)

        self.pub = rospy.Publisher('armod_command', String, queue_size=10) 

        self.sub_perception = rospy.Subscriber('/perception/humans', Humans, self.perception_callback)
        self.sub_smu_marker = rospy.Subscriber('/smu/closest_human_visualization_marker', Marker, self.smu_callback)

        self.detected_humans = {}

        self.nao_base_offset = [0.3, 0, 0.8]
        self.id_stamp = datetime.now()
        print("Intializing class ...")
        time.sleep(5)
        print("Class intialized ...")
        
    def perception_callback(self, data):
        for human in data.humans:
            # Set the default values for the new human id
            self.detected_humans.setdefault(human.id, {"AKN": False, 
                                                    "WRN": False,
                                                    "X": 1.0,
                                                    "Y": 1.0,
                                                    "gtsamp": datetime.now()})
            
            # Update the height for the existing or new human id
            self.detected_humans[human.id]["Height"] = (2*human.centroid.pose.position.z)-0.08

    def get_coordinates(self):
        x = self.detected_humans[self.closest_id]["X"] - self.nao_base_offset[0]
        y = self.detected_humans[self.closest_id]["Y"]
        z = self.detected_humans[self.closest_id]["Height"] - self.nao_base_offset[2]

        return str(x)+","+str(y)+","+str(z)

    def smu_callback(self, data):

        try:
            _ = self.detected_humans[data.id]["AKN"]
        except KeyError:
            print(f"SMU {data.id} id Glitch at: {datetime.now()}")
            return
        # print(f"Closest id: {data.id}, {type(data.id)}")
        self.closest_id = data.id
        self.id_stamp = datetime.now()

        self.detected_humans[self.closest_id]["X"] = data.pose.position.x
        self.detected_humans[self.closest_id]["Y"] = data.pose.position.y

        if "Height" in self.detected_humans[self.closest_id].keys(): 
            # ID already detected, thus has height value assigned to it
            pass
        else:
            self.detected_humans[self.closest_id]["Height"] = 1.7

        if data.color.r:
            self.status = "Warn"
        else:
            self.status = "Ok"

    def run_perception(self):
        rate = rospy.Rate(10)

        dt = 0

        while not rospy.is_shutdown():
            # Get age of closest detection
            age = (datetime.now()-self.id_stamp).seconds

            # If any detection within the last two seconds
            if age < 3:
                # If 2 seconds passed
                if dt >= 20:
                    if self.status == "Warn":
                        if not self.detected_humans[self.closest_id]["WRN"] and not self.detected_humans[self.closest_id]["AKN"]:
                            print(f"Will warn id: {self.closest_id} at time: {datetime.now()}")
                            print(f"Will also acknowledge id: {self.closest_id} at time: {datetime.now()}")
                            self.pub.publish("AKN+WRN,"+self.get_coordinates())
                            self.detected_humans[self.closest_id]["WRN"] = True
                            self.detected_humans[self.closest_id]["AKN"] = True
                            time.sleep(2)
                            dt = 0

                        elif not self.detected_humans[self.closest_id]["WRN"] and self.detected_humans[self.closest_id]["AKN"]:
                            print(f"Will only warn id: {self.closest_id} at time: {datetime.now()} (was already acknowledged)")


                            self.pub.publish("WRN,"+self.get_coordinates())
                            self.detected_humans[self.closest_id]["WRN"] = True
                            time.sleep(2)
                            dt = 0

                        elif self.detected_humans[self.closest_id]["WRN"] and self.detected_humans[self.closest_id]["AKN"]:
                            chance = np.random.randint(1,5) # Implemetning a percentage change
                            if chance != 4: # So 75% Chance of gazing
                                gaze_age = (datetime.now()-self.detected_humans[self.closest_id]["gtsamp"]).seconds
                                if gaze_age > 5:
                                    print(f"Will only look at id: {self.closest_id} at time: {datetime.now()} (AKN and WRN are True)")
                                    self.pub.publish("Look,"+self.get_coordinates())

                                    self.detected_humans[self.closest_id]["gstamp"] = datetime.now()                               
                                    time.sleep(2)
                            dt = 0

                    if self.status == "Ok":
                        if not self.detected_humans[self.closest_id]["AKN"]:
                            print(f"Will acknowledge id: {self.closest_id} at time: {datetime.now()}")
                            self.pub.publish("AKN,"+self.get_coordinates())
                            self.detected_humans[self.closest_id]["AKN"] = True
                            time.sleep(2)
                            dt = 0

                        else:
                            chance = np.random.randint(1,5)
                            if chance%2 == 0:
                                gaze_age = (datetime.now()-self.detected_humans[self.closest_id]["gtsamp"]).seconds
                                if gaze_age > 5:
                                    print(f"Will only look at id: {self.closest_id} at time: {datetime.now()} (was already acknowledged and warned)")
                                    self.pub.publish("Look,"+self.get_coordinates())

                                    self.detected_humans[self.closest_id]["gstamp"] = datetime.now()
                                    time.sleep(2)
                            dt = 0
            dt += 1
            rate.sleep()

if __name__ == '__main__':
    armod = ArmodPerception() # create an instance of the ArmodCommand class
    armod.run_perception() # run the main loop of the node                       

