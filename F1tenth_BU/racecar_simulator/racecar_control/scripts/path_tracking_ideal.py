#!/usr/bin/env python
'''
T
'''
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Pose2D
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
from racecar_control.msg import drive_param
#goals = [[3,6],[17,25],[10,25]]
#goals = [[2.974, 3.661]]
#goals = [[-3.968, 5.635], [-3.960, 9.532], [-3.131, 12.867]] #Gazebo
#goals = [[0.410, -2.703], [2.944, 3.444], [6.689, 4.616], [10.100, 4.947]] #Lidar
#goals = [[-0.2, 1.189], [-3.351, 3.409], [-4.310, 9.536]] #2.323
goals = [[3.5, -4.0], [5.0, 2.0], [3.5, 8.0], [-3.5, 8.0], [-3.5, 4.0], [0.0, 4.0], [0.0, 0.0], [-3.5, 0.0], [-3.5, -3.5]]
kp = 3.0
ki = 0.01
kd = 0.09
goal_x = goals[0][0]
goal_y = goals[0][1]
prev_error = 0
i_error = 0
velocity = 4.0
index = 0

class Pose_listener:
    def __init__ (self):
        self.listener = rospy.Subscriber('/Car_Pose', Pose2D, self.get_Pose)
        self.pose = Pose2D()
        self.pose.x = 1.147
        self.pose.y = 1.364
        self.pose.theta = 2.794
    def get_Pose(self, data):
        self.pose = data


# Start the node
if __name__ == '__main__':
    rospy.init_node("path_tracking")

    pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

    pose_gazebo = Pose_listener()
    count = 0
    while not rospy.is_shutdown():
    	goal_x = goals[index][0]
    	goal_y = goals[index][1]
    	if goals is not None:
    		x = pose_gazebo.pose.x
    		y = pose_gazebo.pose.y
     		yaw = pose_gazebo.pose.theta
    		#goal_angle = math.atan2(goal_y - position[1], goal_x - position[0])
     		goal_angle = math.atan2(goal_y - y, goal_x - x)
     		#print "Pose:", position
    		#rospy.loginfo("Pose:", position)
    		#error = goal_angle - rpy[2]
     		error = goal_angle - yaw
    		if error > math.pi:
    			error = - 2 * math.pi + error
    		if error < -math.pi:
    			error = 2 * math.pi + error
    		if count < 5:
    			i_error += error
    			count += 1
    		else:
    			i_error = 0
    			count = 0
    		angle = kp * error + ki * i_error + kd * (error - prev_error)
    		if angle > 0.7 :
      			angle = 0.7
    		if angle < -0.7 :
      			angle = -0.7
    		if index == 0 :
      			angle = 0

    		prev_error = error
    		rospy.loginfo("x: {}, y: {}".format(x, y))

    		msg = drive_param()
    		msg.velocity = velocity
    		msg.angle = angle
    		rospy.loginfo("index: {}".format(index))
    		pub.publish(msg)

    		if math.sqrt(pow(goal_y - y,2) + pow(goal_x - x,2))<0.5:
    			index = index + 1
    			i_error = 0
    			kp = 3.0
    			ki = 0.01
    			if index == -1:
    				kp = 1.0
    				ki = 0.1
    			if index == 8:
    				msg = drive_param()
    				msg.velocity = 0
    				msg.angle = 0
    				pub.publish(msg)
    				rospy.signal_shutdown("the car has reach the goal")
