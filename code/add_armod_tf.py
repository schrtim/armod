#!/usr/bin/env python3

import rospy
import tf

def main():
    rospy.init_node('nao_tf_broadcaster')
    br = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(90.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        br.sendTransform(
                # Use this if X-Axis is aligned with robot_base_foorprint
            (0.45, 0.0, 0.95),
            # Use remaped base_footprint as parent frame
            (0.00000000e+00,   0.00000000e+00,   0.00000000e+00,  1.0),
            now,
            "/robot_armod_frame",
            "/robot_base_footprint")
        rate.sleep()

if __name__ == '__main__':
    main()