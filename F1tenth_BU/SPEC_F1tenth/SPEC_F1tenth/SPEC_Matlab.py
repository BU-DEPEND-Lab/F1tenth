import numpy as np
from SPEC_F1tenth.F1tenth import F1tenth
from SPEC_F1tenth.controller_candidate import HJ_controller
from SPEC_F1tenth.collisionDetection import do_polygons_intersect, Rectangle, car_shape
import time
import rospy


"""
Scenario optimization for F1tenth simulation
"""
TARGET_SIZE = 1
TARGET_POSITION = np.array([4, -4])

BARREL_SIZE = 0.25
BARREL_POSITION = np.array([-1, -4])
RIGHT_WALL_POSITION1 = [-5.815, -10]
RIGHT_WALL_POSITION2 = [5.142, -5.591]
RIGHT_WALL_POSITION3 = [-10, -5,91]
RIGHT_WALL_POSITION4 = [-5.815, 10]
RIGHT_WALL_POSITION5 = [5.142, -5.591]
RIGHT_WALL_POSITION6 = [10, 10]
LEFT_WALL_POSITION1 = [3.030, 10]
LEFT_WALL_POSITION2 = [-2.236, -2.374]

def scenario_optimization(car, hj, num=100):
    # define rectangle of target region
    target_rect = Rectangle(TARGET_POSITION-TARGET_SIZE, TARGET_POSITION-TARGET_SIZE)
    # define rectangles of obstacles
    car_rect = car_shape(length=1, width=0.5)
    barrel_rect = Rectangle(BARREL_POSITION-BARREL_SIZE, BARREL_POSITION+BARREL_SIZE)
    left_wall_rect = Rectangle(LEFT_WALL_POSITION2, LEFT_WALL_POSITION1)
    right_wall_rect1 = Rectangle(RIGHT_WALL_POSITION1, RIGHT_WALL_POSITION2)
    right_wall_rect2 = Rectangle(RIGHT_WALL_POSITION3, RIGHT_WALL_POSITION4)
    right_wall_rect3 = Rectangle(RIGHT_WALL_POSITION5, RIGHT_WALL_POSITION6)
    # obstacles = [barrel_rect, left_wall_rect, right_wall_rect1,
    #             right_wall_rect2, right_wall_rect3]
    obstacles = [barrel_rect, left_wall_rect, right_wall_rect1,
                 right_wall_rect2]

    # flags
    intersect_flag = False
    enlarge_barrel = False
    enlarge_wall = False
    shrink_reachable_set = False

    # distance
    dis_barrel = 0.
    dis_wall = 0.
    dis_barrel_old = 0.
    dis_wall_old = 0.

    eng = matlab.engine.start_matlab()

    for i in range(num):
        # obtain car state
        xinit = [car.initx, car.inity, car.inittheta]

        eng.SPEC(0.5, dis_barrel, dis_wall, 0, False)
        
        if hj.empty_flag:
            car_polygon = car_rect.polygon(car.x, car.y, car.theta)
            for idxnum, obstacle in enumerate(obstacles):
                intersect_flag = do_polygons_intersect(car_polygon,
                                                       obstacle.polygon)
                if intersect_flag:
                    if idxnum == 0:
                        enlarge_barrel = True
                    else:
                        enlarge_wall = True
                        wall_idx = idxnum
                    break

            if intersect_flag:
                min_dis = np.inf
                if enlarge_barrel:
                    for state in traj.T:
                        dis = barrel_rect.closest_distance(state[0],
                                                           state[1])
                        if dis < min_dis: min_dis = dis
                    dis_barrel = max(min_dis, dis_barrel_old)
                    dis_barrel_old = dis_barrel
                    print "Hitting the barrel with d_b: {}!".format(dis_barrel)
                    enlarge_barrel = False
                if enlarge_wall:
                    for state in traj.T:
                        dis = obstacles[wall_idx].closest_distance(state[0],
                                                                   state[1])
                        if dis < min_dis: min_dis = dis
                    dis_wall = max(min_dis, dis_wall_old)
                    dis_wall_old = dis_wall
                    print "Hitting the wall with d_b: {}!".format(dis_wall)
                    enlarge_wall = False
                car.reset()
                break

                xstate = [car.x, car.y, car.theta]
                start = time.time()
                traj_u, failure = hj.compute_ctrl(xstate)
                end = time.time()
                duration = end - start
                #print (duration)
                if duration < 0.1:
                	time.sleep(0.1 - duration)
                if failure:
                    print "No feasible controller can be found at this step!"
                    car.reset()
                    return
            car.reset()
        else:
            print "No feasible controller can be found!"
            car.reset()
            return


def run():
    car = F1tenth(velocity=10, time=20)
    hj = HJ_controller(False)
    car.reset()

    # do scenarios optimization
    num = 100 # number of iterations
    ret = scenario_optimization(car, hj, num)

if __name__ == '__main__':
    run()
