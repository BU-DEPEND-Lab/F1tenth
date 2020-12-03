import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Pose2D
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
from racecar_control.msg import drive_param
from std_srvs.srv import Empty
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import os
import numpy as np

rospy.init_node("path_tracking")
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=5)
lineList = []
clean = []
with open('/home/f1/record/review15') as f:
	for line in f:
		if line != '\n':
			lineList.append([float(x) for x in line.split()])
print lineList


msg = drive_param()
msg.angle = 0
msg.velocity = 0
pub.publish(msg)
rospy.sleep(1)
state_msg = ModelState()
state_msg.model_name = 'racecar'
state_msg.pose.position.x = -4.43202682339
state_msg.pose.position.y = -4.211334797
state_msg.pose.position.z = 0.05
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = 0
state_msg.pose.orientation.w = 0
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state( state_msg )

for i in range(5):
    pub.publish(msg)

rospy.sleep(1)

for u in lineList:
	msg.angle = u[0]
	msg.velocity = 5
	pub.publish(msg)
	rospy.sleep(0.05)

msg.angle = 0
msg.velocity = 0
pub.publish(msg)
print "arrived"
