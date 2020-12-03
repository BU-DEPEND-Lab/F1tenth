#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Pose2D
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
from numpy import dot
import math
import tf
from racecar_control.msg import drive_param
import scipy.linalg as la
from cmath import phase, exp

waypoints = [[1.147, 1.364], [-3.968, 5.635], [-3.900, 9.532], [-3.131, 12.867]] #Gazebo
#waypoints = [[0.410, -2.703], [2.944, 3.444], [6.689, 4.616], [10.100, 4.947]] #Lidar

# LQR parameter
Q = np.eye(4)
R = np.eye(1)

# parameters
dt = 0.1
L = 0.5 # Wheel base of the vehicle [m]
max_steer = 0.7
velocity = 5.0

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = dot(dot(A.T, X), A) - dot(dot(dot(dot(dot(dot(A.T, X), B),
            la.inv(R + dot(dot(B.T, X), B))), B.T), X), A) + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn

def dlqr(A, B, Q, R):
    """
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = dot(la.inv(dot(dot(B.T, X), B) + R), dot(dot(B.T, X), A))

    eigVals, eigVecs = la.eig(A - dot(B, K))

    return K, X, eigVals

def lqr_steering_control(x, y, yaw, waypoints, index, pe, pth_e):
    e, th_e, index = calc_error(x, y, yaw, waypoints, index)
    rospy.loginfo("e: %lf, th_e: %lf", e, th_e)

    k = 0
    v = velocity

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = v/L

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((dot(-K, x))[0, 0])

    delta = ff + fb

    return delta, e, th_e, index

def calc_error(x, y ,yaw, waypoints, index):
    """
    Compute distance error and theta error given current state
    """

    min_distance_error = float('inf')
    i = index + 1
    # Vector from closest passed waypoint to the next waypoint
    q = np.array([waypoints[i][0] - waypoints[i-1][0], waypoints[i][1] - waypoints[i-1][1]])
    # Vector from car position to the closest passed waypoint
    c = np.array([x - waypoints[i-1][0], y - waypoints[i-1][1]])
    # Vector from current posiition to teh next waypoint
    togo = np.array([x - waypoints[i][0], y - waypoints[i][1]])
    waypoint_distance = np.linalg.norm(togo)

    if np.linalg.norm(c) == 0:
        # If the car is on the waypoint, return distance error 0
        min_distance_error = 0
    else:
        # Project c to q
        proj_q_c = (np.dot(q, c)/ np.linalg.norm(q)) * (q/ np.linalg.norm(q))
        # Compute the vertical distance from the car to the path
        distance_error = np.linalg.norm(c - proj_q_c)
        min_distance_error = distance_error

    # Compute orientation error
    orientation_vector = exp(1j * yaw)
    q = q[0] + 1j * q[1]
    c = c[0] + 1j * c[1]

    theta_error = phase(q/ orientation_vector)
    if theta_error < 0:
        min_distance_error *= -1

    # Configurable tolerance
    if waypoint_distance < 0.3:
        index = min(index + 1, len(waypoints) - 2)

    return min_distance_error, theta_error, index

class Pose_listener:

    def __init__ (self):
        self.listener = rospy.Subscriber('/Car_Pose_gazebo', Pose2D, self.get_Pose)
        self.pose = Pose2D()
        self.pose.x = 1.147
        self.pose.y = 1.364
        self.pose.theta = 2.794
    def get_Pose(self, data):
        self.pose = data

def closed_loop_prediction(waypoints):
    rospy.init_node("path_tracking")

    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))

    pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    index = 0
    pe = 0
    pth_e = 0

    gazebo_pose = Pose_listener()

    while not rospy.is_shutdown():
        t = listener.getLatestCommonTime('/map', '/base_link')
        if listener.canTransform('/map', '/base_link', t):
            position, quaternion = listener.lookupTransform('/map', '/base_link', t)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            #x = position[0]
            #y = position[1]
            #yaw = rpy[2]
            x = gazebo_pose.pose.x
            y = gazebo_pose.pose.y
            yaw = gazebo_pose.pose.theta
            delta, e, th_e, index = lqr_steering_control(x, y, yaw, waypoints, index, pe, pth_e)
            if delta > max_steer:
                delta = max_steer
            elif delta < -max_steer:
                delta = -max_steer
            pe = e
            pth_e = th_e

            msg = drive_param()
            if np.linalg.norm([x - waypoints[-1][0], y - waypoints[-1][1]]) < 0.3:
                msg.velocity = 0
                rospy.signal_shutdown("the car has reach the goal")
            else:
                msg.velocity = velocity
            msg.angle = -delta
            pub.publish(msg)

if __name__ == '__main__':
    closed_loop_prediction(waypoints)
