import numpy as np
import json
from SPEC_F1tenth.F1tenth import F1tenth
from SPEC_F1tenth.controller_candidate import get_control_sequences
from SPEC_F1tenth.collisionDetection import do_polygons_intersect, Rectangle, car_shape


"""
Scenario optimization for F1tenth simulation
"""
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

def scenario_optimization(car, num=100):
    # define rectangles
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

    # traj_his
    traj_his = []
    sim_his = []

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
    for i in range(num):
        print "iteration" + " " + str(i)
        # obtain car state
        xinit = [car.initx, car.inity, car.inittheta]
        traj, traj_u, traj_tau, empty_flag = get_control_sequences(dis_barrel=dis_barrel,
                                                                   dis_wall=dis_wall,
                                                                   xinit=xinit,
                                                                   visual=False)
        if empty_flag:
            for idx in traj_u[0]:
                car.step(idx)
            traj_his.append(traj.tolist())
            sim = np.vstack((car.recordx, car.recordy, car.recordtheta))
            sim_his.append(sim.tolist())
            car.reset()
        else:
            print "No feasible controller can be found!"
            car.reset()
            break

    print "write the data into data_file.json"
    data = {"model": traj_his, "simulator": sim_his}
    with open("data_file_01.json", "w") as file:
        json.dump(data, file)

def run():
    car = F1tenth(velocity=10, frequency=0.1, time=20)
    car.reset()

    # do scenarios optimization
    num = 10 # number of iterations
    ret = scenario_optimization(car, num)

if __name__ == '__main__':
    run()
