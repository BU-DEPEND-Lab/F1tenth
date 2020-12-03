#!/usr/bin/env python
import rospy
from race.msg import drive_param
from sensor_msgs.msg import Joy
rospy.init_node('joystick_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)



def control(data):

	button = data.buttons
	axes = data.axes
	
	left = axes[0] * (-80)
	forward = axes[4] * 15
	
	msg = drive_param()
	msg.velocity = forward 
	msg.angle = left
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('joystick_talker', anonymous=True)
	rospy.Subscriber("joy", Joy, control)
	rospy.spin()
    
