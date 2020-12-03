import matlab.engine
import numpy as np


def get_control_sequences(barrel_size=0.5, dis_barrel=0., dis_wall=0.,
                          xinit=[-4, -4.5, 0.], visual=True):
    """
    call level set for optimal control from specified initial point
    """
    # start matlab
    eng = matlab.engine.start_matlab()

    # doublize xinit
    xinit = matlab.double(xinit)
    dis_barrel = float(dis_barrel)
    dis_wall = float(dis_wall)
    traj, traj_u, traj_tau, empty_flag = eng.F1tenth(barrel_size,
                                                     dis_barrel, dis_wall,
                                                     xinit, visual, nargout=4)

    # change return to numpy.array
    traj = np.asarray(traj)
    traj_u = np.asarray(traj_u)
    traj_tau = np.asarray(traj_tau)

    return traj, traj_u, traj_tau, empty_flag


class HJ_controller(object):
    """
    HJ controller with receding horizon control
    """
    def __init__(self, visual=False):
        # start matlab engine
        self.eng = matlab.engine.start_matlab()

        # render matalb figure or not
        self.visual = False

        # geometrical information
        self.barrel_size = None
        self.dis_barrel = None
        self.dis_wall = None
        self.x = None # current position

    def get_value_map(self, barrel_size, dis_barrel, dis_wall, dis_target=0):
        self.barrel_size = float(barrel_size)
        self.dis_barrel = float(dis_barrel)
        self.dis_wall = float(dis_wall)
        self.dis_target = float(dis_target)

        # obtain computation results
        self.g, self.data, self.dataTraj, self.tau2, self.dCar, self.empty_flag = self.eng.Grid_data(self.barrel_size,
                                                                                                     self.dis_barrel,
                                                                                                     self.dis_wall,
                                                                                                     self.dis_target,
                                                                                                     self.visual, nargout=6)
        # add workspace variables
        self.eng.workspace['g'] = self.g
        self.eng.workspace['data'] = self.data
        self.eng.workspace['dataTraj'] = self.dataTraj
        self.eng.workspace['tau2'] = self.tau2
        self.eng.workspace['dCar'] = self.dCar
        return self.empty_flag

    def compute_traj(self, x):
        # update current state
        self.x = x
        # x_now = matlab.double(self.x)

        # compute trajectory from current state
        traj, traj_u, traj_tau, failure = self.eng.compute_traj(x_now, self.g,
                                                                self.data,
                                                                self.tau2,
                                                                self.visual, nargout=4)
        # traj, traj_u, traj_tau, failure = self.eng.eval("compute_traj(" + str(x) + ", g, data, tau2, false)", nargout=4)

        # change return to numpy.array
        traj = np.asarray(traj)
        traj_u = np.asarray(traj_u)
        traj_tau = np.asarray(traj_tau)

        return traj, traj_u, traj_tau, failure

    def compute_ctrl(self, x):
        self.x = x
        # x_now = matlab.double(self.x)

        # compute optimal control for current state
        # traj_u, failure = self.eng.compute_ctrl(x_now, self.visual, nargout=2)

        traj_u, failure = self.eng.eval("compute_ctrl(" + str(x)  + ", g, data, dataTraj, tau2, dCar, false)", nargout=2)

        # change return to numpy.array
        # traj_u = np.asarray(traj_u)

        return traj_u, failure
