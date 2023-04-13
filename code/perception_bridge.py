#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class ArmodCommand:
    def __init__(self):
        rospy.init_node('armod_command_publisher', anonymous=True)
        self.pub = rospy.Publisher('armod_command', String, queue_size=10)

    def send_command(self, command):
        self.pub.publish(command)

    def run(self):
        while not rospy.is_shutdown():
            key = input("Press a key: ")
            if key == 'p':
                self.send_command("point")
            elif key == 'l':
                self.send_command("look")
            elif key == 'q':
                self.send_command("quit")
                break
            else:
                self.send_command(key)

if __name__ == '__main__':
    armod_command = ArmodCommand()
    armod_command.run()