#!/usr/bin/env python

import rospy
from race.msg import drive_param
from sensor_msgs.msg import Joy

global forward
global left
forward = 0
left = 0
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

def control(data):
	global forward
	global left
        button = data.buttons
	axes = data.axes
        if button[1] == 1:
                forward = forward + 1;
        elif button[2] == 1:
                forward = forward - 1;
        if button[4] == 1:
                left = left - 5;
        elif button[5] == 1:
                left = left + 5;
        if button[3] == 1:
                left = 0
                forward = 0
        msg = drive_param()
        msg.velocity = forward 
        msg.angle = left
        pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('keyboard_talker', anonymous=True)
        rospy.Subscriber("joy", Joy, control)
        rospy.spin()
