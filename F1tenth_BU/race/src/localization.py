#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np
import math
import tf

def listener():
    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
    pub = rospy.Publisher('Car_Pose', Pose2D, queue_size=10)
    while not rospy.is_shutdown():
        t = listener.getLatestCommonTime('/map', '/base_link')
        if listener.canTransform('/map', '/base_link', t):
            position, quaternion = listener.lookupTransform('/map', '/base_link', t)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            msg = Pose2D()
            msg.x = position[0]
            msg.y = position[1]
            msg.theta = rpy[2]
            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("localization")
    listener()
