"""

Path tracking simulation with rear wheel feedback steering control and PID speed control.

"""
import sys
sys.path.append("~/F1tenth_BU/Path_Tracking/ddpg/")
import gym
import math
import matplotlib.pyplot as plt
import numpy as np
from gym import spaces
from gym.utils import seeding
from bspline_path import bspline_planning
import execnet
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
#from racecar_control.msg import drive_param
from race.msg import drive_param
from sensor_msgs.msg import Joy
import time

Kp = 1.0  # speed propotional gain
# steering control parameter
KTH = 1
KE = 0.5

dt = 0.1  # [s]
L = 0.5  # [m]

show_animation = True
def angle_normalize(x):
    return (((x+np.pi) % (2*np.pi)) - np.pi)

class Rear_Wheel_Path_Tracking_Feedback:

    metadata = {
        'render.modes': ['real_time', 'files'],
        'figure_per_second': 0.0001
    }
    def __init__(self):
        #ros init
        rospy.init_node("path_tracking")
        self.listener = rospy.Subscriber('/Car_Pose', Pose2D, self._get_Pose)
        self.joy = rospy.Subscriber('/joy', Joy, self.control)
        self.buttons = np.zeros(6)
        # self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(20.0))
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
        self.pub_dis_error = rospy.Publisher('distance', Float32, queue_size=10)
        #target trajectory
        self.cx = None
        self.cy = None
        self.cyaw = None
        self.ck = None
#        random generate the target trajectory
#        xx = list(range(0, 100, 10))
#        yy = [0]
#        yy_np = self.np_random.random_integers(0, high = 5, size=(len(xx),))
#        yy.extend(yy_np.tolist())
#        self.cx, self.cy, self.cyaw, self.ck, s = cubic_spline_planner.calc_spline_course(xx, yy, ds = self.dt)

        #iterations
        self.count = 0

        #goal position
        self.goal = None

        #sample rate
        self.dt = 0.05

        #vehicle length
        self.L = 0.5

        #tolerace dis to the goal
        self.goal_dis = 0.3

        #no move check speed
        self.stop_speed = 0.05

        #reach goal
        self.goal_flag = False

        #actual trajectory
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []

        #view the process
        self.view = False

        #reward range
        self.reward_range = None

        #eval_flag
        self.eval_flag = False

        #speed target
        self.sp = None

        self.pose = Pose2D()
        self.pose.x = -3.5
        self.pose.y = -0.5
        self.pose.theta = 0


        #action and observation bound
        action_high = np.array([10., 10.])
        observation_high = np.array([np.inf, np.pi, np.inf,  30/3.6])

        #define action and observation
        self.action_space = spaces.Box(low = -action_high, high = action_high)
        self.observation_space = spaces.Box(low = -observation_high, high = observation_high)

        #random seed
        self.seed()

    def control(self, data):
        self.buttons = data.buttons

    def _get_Pose(self, msg):
        self.pose = msg

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        xx = np.array([-1.5, -1.0, 1.5, 5.0])
        yy = np.array([1.5, 4.0, 5.5, 5.5])
        sn = 100
        self.cx, self.cy, self.cyaw, self.ck= bspline_planning(xx, yy, sn)
        self.goal = [xx[-1], yy[-1]]
        return [seed]

    def step(self, u):
        #get target pose index
        target_ind, e = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw)
        self.pub_dis_error.publish(e)
        drive = drive_param()
        drive.velocity = 7.5#40
        drive.angle = -u[0] * 100 + 6
        if self.buttons[0] == 1 or self.buttons[3] == 1:
            self.pub.publish(drive)
        else:
            drive.velocity = 0

            drive.angle = 0
            self.pub.publish(drive)

        #update the state
        self.state = update(self.state, self.pose.x, self.pose.y, self.pose.theta)
        self.x.append(self.state.x)
        self.y.append(self.state.y)
        self.yaw.append(self.state.yaw)
        self.v.append(self.state.v)

        #next target index and new error
        ind, e = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw)
        th_e = pi_2_pi(self.state.yaw - self.cyaw[ind])

        #new observation
        new_obs = np.array([e, angle_normalize(th_e), self.ck[ind], self.state.v])

        #check goal
        dx = self.state.x - self.goal[0]
        dy = self.state.y - self.goal[1]
        dis = math.sqrt(dx**2 + dy**2)
        #define costs
        costs = 10 * (e**2) + 10 * angle_normalize(th_e)**2 +  1 * (u[0]/ self.state.v - self.ck[ind])**2
        #costs = -self.state.v * np.cos(th_e) + self.state.v * np.sin(th_e) + e **2
        if dis <= self.goal_dis:
            print ("Goal")
            #costs += -1. * (self.goal[0]**2 + self.goal[1]**2)
            self.goal_flag = True

        #record total time
        self.time += self.dt
        rospy.sleep(0.2)
        #print (time.time(), ': ', u[0])
        if self.time > 20:
            costs += 60 * dis**2
            costs = 4*costs
            return new_obs, -costs, True, {"episode": None,
                                           "status":'Unable to goal'}
        costs = 2*costs
        return new_obs, -costs, self.goal_flag, {}

    def render(self, mode):
        self.view = True

    def reset(self):
        self.count += 1
        #plot the result
        if self.view:
            if self.eval_flag:
                plt.figure(num = 2)
            else:
                plt.figure(num = 1)
            plt.cla()
            plt.plot(self.cx, self.cy, "-r", label="spline")
            plt.plot(self.x, self.y, "-g", label="tracking")
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()
            if self.eval_flag:
                dir_eval = '/home/f1/Parameterized_Controller_Synthesis/Path_Tracking/log/eval_results101_us/'
                plt.savefig(dir_eval + 'eval_' + str(int(self.count)) + '.png')
            else:
                dir_train = '/home/f1/Parameterized_Controller_Synthesis/Path_Tracking/log/train_results101_us/'
                plt.savefig(dir_train + 'train_' + str(int(self.count)) + '.png')
        #reset state
        self.state = State(x=-1.5, y=1.5, yaw=1.30, v=0.0)
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.x.append(self.state.x)
        self.y.append(self.state.y)
        self.yaw.append(self.state.yaw)
        self.v.append(self.state.v)

        #random generate the target trajectory
#        xx = list(range(0, 100, 10))
#        yy = [0]
#        yy_np = self.np_random.random_integers(0, high = 5, size=(len(xx),))
#        yy.extend(yy_np.tolist())
#        self.cx, self.cy, self.cyaw, self.ck, s = cubic_spline_planner.calc_spline_course(xx, yy, ds = self.dt)
#      #speed target
        self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, 10/3.6)

       #reset the time
        self.time = 0.0

        #reset the goal
        self.goal_flag = False

        #initialize the observation
        ind, e = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw)
        th_e = pi_2_pi(self.state.yaw - self.cyaw[ind])
        obs = np.array([e, angle_normalize(th_e), self.ck[ind], self.state.v])
        return obs

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, x, y, yaw):

    state.x = x
    state.y = y
    state.yaw = yaw
    state.v = 5

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def rear_wheel_feedback_control(state, cx, cy, cyaw, ck, preind):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
        KTH * abs(v) * th_e - KE * v * math.sin(th_e) * e / th_e

    if th_e == 0.0 or omega == 0.0:
        return 0.0, ind

    delta = math.atan2(L * omega / v, 1.0)
    #  print(k, v, e, th_e, omega, delta)

    return delta, ind


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):

    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    goal_flag = False
    target_ind = calc_nearest_index(state, cx, cy, cyaw)

    while T >= time:
        di, target_ind = rear_wheel_feedback_control(
            state, cx, cy, cyaw, ck, target_ind)
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, di)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            goal_flag = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +
                      ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v, goal_flag


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    #  flg, ax = plt.subplots(1)
    #  plt.plot(speed_profile, "-r")
    #  plt.show()

    return speed_profile


