#!/usr/bin/env python
'''
T
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
from racecar_control.msg import drive_param
goals = [[3,6],[17,25],[10,25]]

kp = 14.0
kd = 0.3
goal_x = goals[0][0]
goal_y = goals[0][1]
prev_error = 0
velocity = 15
index = 0

# Start the node
if __name__ == '__main__':
    rospy.init_node("path_tracking")

    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))

    pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

    while not rospy.is_shutdown():
    	goal_x = goals[index][0]
    	goal_y = goals[index][1]
    	t = listener.getLatestCommonTime('/map', '/base_link')
    	if listener.canTransform('/map', '/base_link', t):
    		position, quaternion = listener.lookupTransform('/map', '/base_link', t)
    		rpy = tf.transformations.euler_from_quaternion(quaternion)
    		goal_angle = math.atan2(goal_y - position[1], goal_x - position[0])
    		#rospy.loginfo("goal_y:%lf,goal_x:%lf", goal_y, goal_x)

    		error = goal_angle - rpy[2]
    		if error > math.pi:
    			error = - 2 * math.pi + error
    		if error < -math.pi:
    			error = 2 * math.pi + error
    		angle = kp * error + kd * (error - prev_error)
    		if angle > 0.7 :
      			angle = 0.7
    		if angle < -0.7 :
      			angle = -0.7
    		prev_error = error
    		rospy.loginfo("yaw:%lf,goal_angle:%lf,error:%lf", rpy[2], goal_angle, error)

    		msg = drive_param()
    		msg.velocity = velocity
    		msg.angle = angle
    		rospy.loginfo("velocity:%lf,angle:%lf",msg.velocity,msg.angle)
    		pub.publish(msg)

    		if math.sqrt(pow(goal_y - position[1],2) + pow(goal_x - position[0],2))<0.5:
    			index = index + 1
    			if index == 3:
    				msg = drive_param()
    				msg.velocity = 0
    				msg.angle = 0
    				pub.publish(msg)
    				rospy.signal_shutdown("the car has reach the goal")
