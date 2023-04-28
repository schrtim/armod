#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String

class ArmodFrameClass:
    def __init__(self):
        rospy.init_node('nao_tf_broadcaster')
        self.br = tf.TransformBroadcaster()
        self.br2 = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        rospy.Subscriber("armod_orientation", String, self.callback)
        self.rate = rospy.Rate(90.0)
        self.q = [0,0,0,1]

    def callback(self, data):
        msg = data.data.split(",")
        self.q = [msg[1], msg[2], msg[3], msg[4]]

    def update_static_transform_broadcaster(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.br.sendTransform(
                    # Use this if X-Axis is aligned with robot_base_foorprint
                (0.3, 0.0, 0.85),
                # Use remaped base_footprint as parent frame
                (self.q[0],   self.q[1],   self.q[2],  self.q[3]),
                now,
                "/robot_armod_frame",
                "/robot_base_footprint")
            self.rate.sleep()

if __name__ == '__main__':
    armod_frame_class = ArmodFrameClass()
    armod_frame_class.update_static_transform_broadcaster()