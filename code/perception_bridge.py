#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class ArmodCommand:
    def __init__(self):
        rospy.init_node('armod_command_publisher', anonymous=True)
        self.pub = rospy.Publisher('armod_command', String, queue_size=10)
        self.sub_keypress = rospy.Subscriber('face_detections', String, self.receive_perception_message)
        self.pd = []

    def send_command(self, command):
        self.pub.publish(command)

    def receive_perception_message(self, data):
        bx, by, bz = data.data.split(",")
        self.pd = [float(bx), float(by), float(bz)/1000]
        

    def run(self):
        while not rospy.is_shutdown():
            key = input("Press a key: ")
            if key == 'p':
                self.send_command("point,"+str(self.pd[0])+","+str(self.pd[1])+","+str(self.pd[2]))
                print(self.pd)
            elif key == 'l':
                self.send_command("look,"+str(self.pd[0])+","+str(self.pd[1])+","+str(self.pd[2]))
                print(self.pd)
            elif key == 'q':
                self.send_command("quit,"+str(0)+","+str(0)+","+str(0))
                break
            else:
                self.send_command(key)

if __name__ == '__main__':
    armod_command = ArmodCommand()
    armod_command.run()