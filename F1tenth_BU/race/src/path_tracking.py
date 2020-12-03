#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from race.msg import drive_param
from sensor_msgs.msg import Joy
import math
import tf
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from bspline_path import bspline_planning    
goals = [[-1.5,1.5],[-1.0,4.0],[1.5,5.5],[5,5.5],[100,100]]
#goals = [[-2,2],[1,-0.5],[5,0],[7,1],[100,100]]

kp = 1.4
kd = 0.1
goal_x = goals[0][0]
goal_y = goals[0][1]
prev_error = 0
velocity = 7.5
index = 0
buttons = [0, 0, 0, 0, 0, 0]
axes = [0, 0, 0, 0, 0, 0]
arrived = False
angle_error = 6.5

xx = np.array([-1.5, -1.0, 1.5, 5.0])
yy = np.array([1.5, 4.0, 5.5, 5.5])
sn = 100
cx, cy, cyaw, ck= bspline_planning(xx, yy, sn)
goal = [xx[-1], yy[-1]]

global up
global low
up = 2.0
low = -2.0

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state[0] - icx for icx in cx]
    dy = [state[1] - icy for icy in cy]

    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    dxl = cx[ind] - state[0]
    dyl = cy[ind] - state[1]

    return ind, mind

def control(data):
    global buttons
    global axes 
    buttons = data.buttons
    axes = data.axes

def monitor(data):
    global up
    global low
    values = data.data
    up = values[0]
    low = values[1]
# Start the node
if __name__ == '__main__':
    rospy.init_node("pid_tracking", anonymous=True)
    pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    dis_pub = rospy.Publisher('distance', Float32, queue_size=10)
    r = rospy.Rate(10)

    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
    
    rospy.Subscriber("joy", Joy, control)
    rospy.Subscriber("up_low", Float32MultiArray, monitor)
    violate_flag= False 
    while not rospy.is_shutdown():
    	goal_x = goals[index][0]
    	goal_y = goals[index][1]
    	t = listener.getLatestCommonTime('/map', '/base_link')
    	if listener.canTransform('/map', '/base_link', t):	    
    		position, quaternion = listener.lookupTransform('/map', '/base_link', t)
    		rpy = tf.transformations.euler_from_quaternion(quaternion)

                #calculate the dis_error
                ind, e = calc_nearest_index(position, cx, cy, cyaw)
                dis_pub.publish(e)
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
    		#rospy.loginfo("yaw:%lf,goal_angle:%lf,error:%lf", rpy[2], goal_angle, error)
            
    		msg = drive_param()
                if buttons[1]==1:
    		    msg.velocity = velocity
    		    msg.angle = angle * (-80) + angle_error
                    pub.publish(msg)
    		#if button[1] == 0 or arrived :
    		    #msg.angle = 0 + angle_error
		    #msg.velocity = 0
    		#rospy.loginfo("velocity:%lf,angle:%lf",msg.velocity,msg.angle)
#                if violate_flag:    
#                    rospy.loginfo("violation!!")
                if up < 1.0 or low > 1.0:
                    violate_flag = True
                else:
                    violate_flag = False
                #rospy.loginfo("Pure Pursuit Controller; up:%lf; low:%lf", up, low)

    		if buttons[5] == 1 :
    		    msg.velocity = axes[4] * 15
    		    msg.angle = axes[0] * (-80) + angle_error
                    pub.publish(msg)
                elif buttons[1]==1:
    		    msg.velocity = velocity
    		    msg.angle = angle * (-80) + angle_error
                    pub.publish(msg)
                elif buttons[3]==1 and violate_flag:
    		    msg.velocity = velocity
    		    msg.angle = angle * (-80) + angle_error
                    pub.publish(msg)
                    rospy.loginfo("Pure Pursuit Controller; up:%lf; low:%lf", up, low)
                    if position[0] > -1.50 and position[0] < -1.0:
                        index = 0
                    if position [0] > -1.0 and position[0] < 4 and position[1]>4 and position[1]<5: 
                        index = 2
                    if position [0] >1.5 and position[1]>5.5:
                        index = 3
                elif not violate_flag:
                    if buttons[3] == 1:
                        rospy.loginfo("DDPG Controller; up:%lf; low:%lf", up, low)

                elif buttons[0]==0 and buttons[3] == 0:
                    msg.velocity = 0
                    msg.angle = 0
                    pub.publish(msg)
                if buttons[5] == 0 and arrived:
                    msg.velocity = 0
                    msg.angle = 0
                    pub.publish(msg)
                    rospy.loginfo("goal_y:%lf,goal_x:%lf", goal_y, goal_x)

                if buttons[2] == 1 and arrived:
                    arrived = False
                    voilate_flag = False
                    index = 0
                   	
    		
    		if math.sqrt(pow(goal_y - position[1],2) + pow(goal_x - position[0],2))<0.5 and index == 0:
    			index = index + 1
    			if index == 4:
                            arrived = True    				
    		r.sleep()
    				
    				
