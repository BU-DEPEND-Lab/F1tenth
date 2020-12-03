#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Pose2D
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header
import numpy as np
import math
import tf

def listener():
    listen = rospy.Subscriber('/vesc/odom', Odometry, _get_Pose)
    rospy.spin()

# Start the node


def _get_Pose(msg):
    pub = rospy.Publisher('Car_Pose', Pose2D, queue_size=10)
    data = msg.pose.pose
    quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    theta = tf.transformations.euler_from_quaternion(quaternion)[2]
    msg = Pose2D()
    msg.x = data.position.x
    msg.y = data.position.y
    msg.theta = theta
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("localization")
    listener()