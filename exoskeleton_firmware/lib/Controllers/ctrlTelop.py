import time

import micropython                                  # type: ignore
from micropython import const                       # type: ignore
import pyb                                          # type: ignore
import json
import math

from lib.config import DeviceParameters

from lib.Robots.RobotInterface import Robot
from lib.Controllers.ControllerInterface import Controller



#### individual instances of controllers
### controller that receive a reference position (and velocity) to follow
class ctrl_Teleop(Controller):
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        super().__init__(robot, file_name, ctrl_name, device_params = device_params, decorator= decorator)

        self.K = self.params[ctrl_name]['K']
        self.B = self.params[ctrl_name]['B']
        self.startup_flag = True
        self.t2 = 0

        self.count = 0

        self.eq_angle = self.params[ctrl_name]["eq_angle"]
        self.eq_velocity = 0.0
        self.t0 = time.ticks_ms() / 1000  # record start time
        self.DT_teleop = self.dt


    def en(self, active=True):
        super().en(active)
        pyb.delay(1000)
        self.robot.set_KD_parameter(self.K, self.B)
        pyb.delay(1000)
        # move to initial pos
        self.robot.set_transition_point(self.eq_angle, 0, 1)
        pyb.delay(1000)
        self.run = True


    def set_KD_parameter(self, K: float, B: float) -> None:
        if not(self.K == K) or not(self.B == B):
            self.K = K
            self.B = B
            self.robot.set_KD_parameter(K, B)


    def set_params_ctrl(self, params:list[float], index:int) -> bool:

        # all bool first
        if bool(params[index+ 0]) and not self.run and self.startup_flag:
            self.en()
        elif bool(params[index+ 0]) and not self.run and not(self.startup_flag):
            self.run = True
        elif not bool(params[index+ 0]) and self.run:
            self.disable()

        # all double ( this is the order rqt sort messages)
        # oscillator parameters
        self.eq_angle = max(0, min(params[index + 1], 90))
        self.eq_velocity = params[index + 2]

        self.set_KD_parameter(params[index+3], params[index + 4])
        self.DT_teleop = max(self.dt, params[index + 5])

        if self.run:     self.robot.set_transition_point(self.eq_angle, self.eq_velocity, self.DT_teleop)
        ### TODO: potentially a portion of the input message is also given to the decorator...

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True

    def get_params_ctrl(self):
        return [
            self.run,
            self.eq_angle,
            self.eq_velocity,
            self.K, self.B,
            self.DT_teleop
        ]


    def create_report(self, out, index) -> int:

        out[index + 0] =  self.eq_angle
        out[index + 1] =  self.eq_velocity
        out[index + 2] =  self.K
        out[index + 3] =  self.B

        index += self.rep_ctrl_msg_dim
        # TODO: last assigned to decorator
        # out [self.rep_robot_msg_dim + self.rep_ctrl_msg_dim +0] = ....
        index += self.rep_dec_msg_dim
        return self.robot.create_report(out, index)


    ### ACTUAL CONTROLLERS METHODS
    def _compute_des_torque(self):
        # enable feedforward components
        if len(self.decorators)>0:
            return self.decorators[0].compute_des_torque()
        else:
            return self.tau_default

    def _compute_auxiliary(self) ->float:

        # update feedforward components
        if len(self.decorators)>0: self.decorators[0].compute_auxiliary()

        return 0.0

    def _execute_des_pose(self) -> float:

        ## FF should do nothing here
        if len(self.decorators) > 0: self.decorators[0].execute_des_pose()

        if self.startup_flag:
            self.robot.set_position_set_point(self.robot.pos)
            self.startup_flag = False


