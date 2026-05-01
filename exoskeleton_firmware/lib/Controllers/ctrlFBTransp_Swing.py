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
class ctrl_FB_Transp(Controller):
    """
    This class implements the transparent controller build using both feedforward and feedback model for the swing phase.
    This class is specific of the swing phase simply because the robot it is not symmetric for what regards the loadcell placement.
    """
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator= False) -> None:
        super().__init__(robot, file_name, ctrl_name, device_params= device_params, decorator= decorator)

        self._robot_state = self.params[ctrl_name]['ROBOT_STATE']
        self.robot.set_robot_state(self._robot_state)

        self.K = 0.0
        self.B = 0.0
        self.startup_flag = True

        # tau feedback params
        self.tau_int_act =0.0
        self.tau_int_des =0.0
        self.err = 0.0
        self.prev_err =0.0
        self.err_dot = 0.0
        self.err_int =0.0
        self.P_fb = self.params[ctrl_name]['P_fb']
        self.I_fb = self.params[ctrl_name]['I_fb']
        self.D_fb = self.params[ctrl_name]['D_fb']
        self.alpha_test =0.0
        self.tau_fb = 0.0

        self.t0 = time.ticks_ms() / 1000  # record start time
        # dynamics
        self.run = False
        self.enable_FF = False




    def en(self, active=True):
        super().en(active)
        pyb.delay(1000)
        self.robot.set_KD_parameter(self.K, self.B)
        pyb.delay(1000)
        # move to initial pos
        self.robot.set_transition_point(0, 0, 1)
        pyb.delay(1000)
        self.run = True
        self.enable_FF = False


    def set_params_ctrl(self, params:list[float], index:int)-> bool:

        if bool(params[index+ 0]) and not self.run:
            self.en()
        elif not bool(params[index+ 0]) and self.run:
            self.disable()

        self.enable_FF = bool(params[index+ 1])

        self.tau_int_des = params[index+ 2]
        self.P_fb = params[index+ 3]
        self.I_fb = params[index+ 4]
        self.D_fb = params[index+ 5]
        self.alpha_test = params[index+ 6]
        self.robot.filt_loadcell = params[index+ 7]

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True

    def get_params_ctrl(self):
        return [
            self.run,
            self.enable_FF,
            # feeback control params
            self.tau_int_des,
            self.P_fb,
            self.I_fb,
            self.D_fb,
            self.alpha_test,
            self.robot.filt_loadcell
        ]

    def create_report(self, out, index) -> int:
        # Feedback torques
        out[index + 0] = self.P_fb*self.err
        out[index + 1] = self.I_fb*self.err_int
        out[index + 2] = self.D_fb*self.err_dot
        out[index + 3] = self.tau_fb
        index += self.rep_ctrl_msg_dim
        # nothing to report here
        # TODO: last assigned to decorator
        # out[self.rep_robot_msg_dim + self.rep_ctrl_msg_dim +0] = ....
        index += self.rep_dec_msg_dim
        return self.robot.create_report(out, index)


    ### ACTUAL CONTROLLERS METHODS
    def _compute_des_torque(self) -> float:
        tau = 0
        if self.enable_FF and  len(self.decorators)>0:
            tau += self.decorators[0].compute_des_torque()
        tau += self.alpha_test * self.tau_fb
        return tau

    def _compute_auxiliary(self) ->float:
        # update feedforward components
        if len(self.decorators) > 0: self.decorators[0].compute_auxiliary()

        self.tau_int_act = self.robot.get_tau_int_through_loadCell()
        self.prev_err = self.err
        self.err = self.tau_int_des - self.tau_int_act
        self.err_dot = (self.err - self.prev_err) / self.dt
        self.err_int += self.err * self.dt

        self.tau_fb = (self.P_fb * self.err +
                       self.I_fb * self.err_int +
                       self.D_fb * self.err_dot)
        return 0.0




