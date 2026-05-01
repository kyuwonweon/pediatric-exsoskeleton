import time

import micropython                                  # type: ignore
from micropython import const                       # type: ignore
import pyb                                          # type: ignore
import json
import math
from lib.config import DeviceParameters

from lib.Robots.RobotInterface import Robot
from lib.Controllers.ControllerInterface import Controller

def sigmoid(x: float) -> float:
    return 2 / (1 + math.exp(-x)) - 1


#### individual instances of controllers
class ctrl_FF_Transp(Controller):
    """
    This class implements the transparent controller build using purelly a feedforward model.
    """
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        super().__init__(robot, file_name, ctrl_name, device_params= device_params, decorator= decorator)

        self._robot_state = self.params[ctrl_name]['ROBOT_STATE']
        self.robot.set_robot_state(self._robot_state)

        self.K = 0.0
        self.B = 0.0
        self.startup_flag = True
        self.t2 = 0

        self.tau_g, self.tau_K, self.tau_b, self.tau_c, self.tau_I = 0.0, 0.0, 0.0, 0.0, 0.0

        self.count = 0

        self.t0 = time.ticks_ms() / 1000  # record start time
        # dynamics
        self.run = False

    def en(self, active=True):
        super().en(active)
        pyb.delay(1000)
        self.robot.set_KD_parameter(self.K, self.B)
        pyb.delay(1000)
        # move to initial pos
        self.robot.set_transition_point(0, 0, 1)
        pyb.delay(1000)
        self.run = True

    def set_gravity_params(self, params):
        if len(params) != 4:
            raise Exception('Invalid number of parameters')
        self.robot.mass = params[0:2]
        self.robot.l_com = params[2:4]

    def set_spring_params(self, params):
        if len(params) != 2:
            raise Exception('Invalid number of parameters')
        self.robot.k_spring = params[0]
        self.robot.eq_angle = params[1]

    def set_viscous_params(self, param):
        self.robot.b = param

    def stic_coul_params(self, params):
        if len(params) != 3:
            raise Exception('Invalid number of parameters')
        self.robot.v_th = max(0.1, params[0])
        self.robot.stic_p = params[1]
        self.robot.coul_p = params[2]

    def set_Inertia_params(self, param):
        if len(param) != 2:
            raise Exception('Invalid number of parameters')
        self.robot.I = param

    def set_params_ctrl(self, params:list[float], index:int)-> bool:

        if bool(params[index+ 0]) and not self.run:
            self.en()
        elif not bool(params[index+ 0]) and self.run:
            self.disable()

        self.robot.set_robot_state(params[index+ 1])
        self.set_gravity_params(params=params[index+ 2: index+6])
        self.set_spring_params(params=params[index+ 6: index+8])
        self.set_viscous_params(param=params[index+ 8])
        self.stic_coul_params(params=params[index+ 9: index+12])
        self.set_Inertia_params(param=params[index+ 12: index+14])

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True


    def get_params_ctrl(self):
        return [
            self.run,
            self.robot.get_robot_state(), # stance or swing
            self.robot.mass[0],
            self.robot.mass[1],
            self.robot.l_com[0],
            self.robot.l_com[1],
            self.robot.k_spring,
            self.robot.eq_angle,
            self.robot.b,
            self.robot.v_th,
            self.robot.stic_p,
            self.robot.coul_p,
            self.robot.I[0],
            self.robot.I[1]
        ]

    def create_report(self, out, index) -> int:
        out[index+0] = self.tau_g
        out[index+1] = self.tau_K
        out[index+2] = self.tau_b
        out[index+3] = self.tau_c
        out[index+4] = self.tau_I
        out[index+5] = (self.tau_g+self.tau_K+self.tau_b+self.tau_c+self.tau_I)
        # nothing to report here
        index += self.rep_ctrl_msg_dim
        # TODO: last assigned to decorator
        # out[self.rep_robot_msg_dim + self.rep_ctrl_msg_dim +0] = ....
        index += self.rep_dec_msg_dim
        return self.robot.create_report(out, index)

    ### ACTUAL CONTROLLERS METHODS
    def _compute_des_torque(self):
        if self.robot.get_position() < 0:  # this by itself should be unfesible but could happen in case of wrong calibrations
            # in negative configurations the gravity component would try to overextending the knee
            return 0.0
        elif self.robot.get_position() < 3:
            # In the joint limit the FF keeps only the "static componensts" to avoid collision
            return self.tau_g + self.tau_K
        else:
            return self.tau_g + self.tau_b + self.tau_K + self.tau_c + self.tau_I

    def _compute_auxiliary(self) ->float:
        self.tau_g = self.robot.get_gravity()
        self.tau_K = self.robot.get_spring_tau()
        self.tau_b = self.robot.get_viscous_tau()
        self.tau_c = self.robot.get_coul_tau()
        self.tau_I = self.robot.get_inertia_tau()
        # TODO: this probably too slow in the controller loop, maybe move in the robot loop
        return 0.0




