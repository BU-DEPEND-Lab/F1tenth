#!/usr/bin/env python
"""
Wrap gazebo simulation to gym-like environment
"""

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
import subprocess


class F1tenth:
    def __init__ (self, frequency=0.05, time=20, velocity=0, a=0, init_low=[-5, -4.5, 0], init_high=[-4, -3.5, 0]):
        # car description
        self.frequency = frequency
        self.time = time
        self.velocity = velocity

        # initial region
        self.init_low = init_low
        self.init_high = init_high

        # state information
        self.x = None
        self.y = None
        self.theta = None
        self.initxre = None
        self.inityre = None

        # state-action trajectory
        self.recordx = []
        self.recordy = []
        self.recordtheta = []
        self.recordu = []
        #lidar information
        self.lidarx = None
        self.lidary = None
        self.lidartheta = None

        # initiate control node
        rospy.init_node("path_tracking")

        # start slam
        # os.system("xterm -e 'cd /home/f1/slam_ws && source devel_isolated/setup.bash && roslaunch cartographer_ros demo_backpack_2d_localization.launch    load_state_filename:=${HOME}/track_new.bag.pbstream' &")
        subprocess.call('cd /home/f1/slam_ws && source install_isolated/setup.bash && roslaunch cartographer_ros demo_backpack_2d_localization.launch    load_state_filename:=/home/f1/track_new.bag.pbstream &', shell=True)
        rospy.sleep(3)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))

        # initiate car control
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=5)
        self.gazebo = rospy.Subscriber('/Car_Pose', Pose2D, self.get_Pose)
        self.pose = Pose2D()

    def get_Pose(self, data):
        self.pose = data

    def step(self, u):
        # control step
        msg = drive_param()
        msg.angle = u
        #self.velocity = self.velocity + a * 0.1
        msg.velocity = self.velocity
        self.pub.publish(msg)
        print u
        # obtain new state information
        transform = self.listener.getLatestCommonTime('/map', '/base_link')
        if self.listener.canTransform('/map', '/base_link', transform):
            position, quaternion = self.listener.lookupTransform('/map', '/base_link', transform)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            self.x = self.pose.x
            self.y = self.pose.y
            self.theta = self.pose.theta
            self.recordx.append(self.x)
            self.recordy.append(self.y)
            self.recordtheta.append(self.theta)
            self.recordu.append(u)
            self.lidarx = position[0]
            self.lidary = position[1]
            self.lidartheta = rpy[2]
        rospy.sleep(self.frequency)

    def reset(self, x=None, y=None):
        self.recordx = []
        self.recordy = []
        self.recordtheta = []
        self.recordu = []
        # reset control
        msg = drive_param()
        msg.angle = 0
        msg.velocity = 0
        for i in range(5):
            self.pub.publish(msg)
        os.system("rosnode kill base_link_to_laser cartographer_occupancy_grid_node cartographer_node")
        # reset car state
        state_msg = ModelState()
        state_msg.model_name = 'racecar'
        state_msg.pose.position.x = np.random.uniform(self.init_low[0],
                                                      self.init_high[0])
        state_msg.pose.position.y = np.random.uniform(self.init_low[1],
                                                      self.init_high[1])
        state_msg.pose.position.z = 0.05
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        # initial state
        self.x = state_msg.pose.position.x
        self.y = state_msg.pose.position.y
        self.initxre = self.x
        self.inityre = self.y
        self.theta = 0 # may need to change later
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        for i in range(5):
            self.pub.publish(msg)
        rospy.sleep(1)
        while np.sqrt((self.pose.x - self.x)**2 + (self.pose.y - self.y)**2) > 0.0001 or abs(self.pose.theta - self.theta) > 0.0001:
            print "reset again!"
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
            self.pub.publish(msg)
            rospy.sleep(1)
        # os.system("xterm -e 'cd /home/f1/slam_ws && source devel_isolated/setup.bash && roslaunch cartographer_ros demo_backpack_2d_localization.launch    load_state_filename:=${HOME}/track_new.bag.pbstream' &")
        subprocess.call('cd /home/f1/slam_ws && source install_isolated/setup.bash && roslaunch cartographer_ros demo_backpack_2d_localization.launch    load_state_filename:=/home/f1/track_new.bag.pbstream &', shell=True)
        rospy.sleep(10)
        transform = self.listener.getLatestCommonTime('/map', '/base_link')
        if self.listener.canTransform('/map', '/base_link', transform):
            position, quaternion = self.listener.lookupTransform('/map', '/base_link', transform)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            self.initx = position[0] - 4.7
            self.inity = position[1] - 4
            self.inittheta = rpy[2]
            print self.initx, self.inity
            print self.x, self.y
            print self.theta, self.inittheta


if __name__ == '__main__':
    f = F1tenth()
    f.reset()
