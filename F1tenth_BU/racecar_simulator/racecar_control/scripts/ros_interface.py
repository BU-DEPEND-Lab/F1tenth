import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
from racecar_control.msg import drive_param

# def update(u):
# 	rospy.init_node("path_tracking")
# 	listener = tf.TransformListener()
# 	listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
# 	pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
# 	t = listener.getLatestCommonTime('/map', '/base_link')
# 	if listener.canTransform('/map', '/base_link', t):
# 		position, quaternion = listener.lookupTransform('/map', '/base_link', t)
# 		rpy = tf.transformations.euler_from_quaternion(quaternion)
# 		yaw = rpy[2]
# 		msg = drive_param()
# 		msg.velocity = 10
# 		msg.angle = u[0]
# 		rospy.loginfo("velocity:%lf,angle:%lf",msg.velocity,msg.angle)
# 		pub.publish(msg)
# 		a = np.array([position[0], position[1], yaw]).astype(float)
# 		a = a.tolist()
# 	return a

class ros_interface:
	def __init__(self, u):

		rospy.init_node("path_tracking")
		self.listener = tf.TransformListener()
		self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
		self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
		self.t = self.listener.getLatestCommonTime('/map', '/base_link')
		self.u = u

	def update(u):
		if self.listener.canTransform('/map', '/base_link', self.t):
			position, quaternion = self.listener.lookupTransform('/map', '/base_link', t)
			rpy = tf.transformations.euler_from_quaternion(quaternion)
			yaw = rpy[2]
			msg = drive_param()
			msg.velocity = 10
			msg.angle = u[0]
			rospy.loginfo("velocity:%lf,angle:%lf",msg.velocity,msg.angle)
			self.pub.publish(msg)
			a = np.array([position[0], position[1], yaw]).astype(float)
			a = a.tolist()
		return a
