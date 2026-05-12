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
## Controller oscillator based on sharifi2021adaptive --> using the force as input signal
## The original work uses a NN to learn force
## Here I tried using loadcell forces but probably too noise
class ctrl_Oscillator_Forces(Controller):
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        super().__init__(robot, file_name, ctrl_name,device_params= device_params, decorator= decorator)

        self.K = self.params[ctrl_name]['K']
        self.B = self.params[ctrl_name]['B']
        self.startup_flag = True
        self.t2 = 0

        self.count = 0
        self.residual_work = 0.0

        self.eq_angles = self.params[ctrl_name]["eq_angles"]
        self.ROM = (self.eq_angles[1] - self.eq_angles[0]) / 2
        self.offset = (self.eq_angles[1] + self.eq_angles[0]) / 2

        self.phase = 0
        self.freq_oscillation = self.params[ctrl_name]["freq_oscillator"]
        self.freq_bounds = self.params[ctrl_name]["freq_bounds"]

        self.use_energy = False
        self.t0 = time.ticks_ms() / 1000  # record start time

        self.filtered_ext_torque =0
        self.res_pwr =0.0

        self.alpha_ext_torque = self.params[ctrl_name]["alpha_ext_torque"]
        self.lb = self.params[ctrl_name]["ext_torque_hysteresis_bounds"][0]
        self.ub = self.params[ctrl_name]["ext_torque_hysteresis_bounds"][1]
        self.res_work_scaling_facts = self.params[ctrl_name]["res_work_scaling_factors"]
        self.forget_factor = self.params[ctrl_name]["FORGET_FACTOR"]

        # dynamics
        self.enable_FF = False


    def en(self, active=True):
        super().en(active)
        pyb.delay(1000)
        self.robot.set_KD_parameter(self.K, self.B)
        pyb.delay(1000)
        # move to initial pos
        self.robot.set_transition_point(self.offset, 0, 1)
        pyb.delay(1000)
        self.run = True
        self.enable_FF = False

    @property
    def residual_work_value(self):
        return self.residual_work

    def use_FF_force(self, enable= True):
        self.enable_FF = enable

    def use_energy_freq(self, enable = True):
        self.use_energy = enable

    def set_freq_oscillation(self, freq_oscillation):
        self.freq_oscillation = freq_oscillation

    def set_KD_parameter(self, K: float, B: float) -> None:
        self.robot.set_KD_parameter(K, B)


    def set_params_ctrl(self, params:list[float], index: int) -> bool:

        # all bool first
        if bool(params[index+ 0]) and not self.run and self.startup_flag:
            self.en()
        elif bool(params[index+ 0]) and not self.run and not(self.startup_flag):
            self.run = True
        elif not bool(params[index+ 0]) and self.run:
            self.disable()
        self.use_FF_force(bool(params[index+ 1]))
        self.use_energy_freq(bool(params[index+ 2]))

        # all double ( this is the order rqt sort messages)
        # oscillator parameters
        self.freq_oscillation = params[index+ 3]
        self.eq_angles[0] = params[index+ 4]
        self.eq_angles[1] = params[index+ 5]
        if self.eq_angles[0] > self.eq_angles[1]:
            self.eq_angles[0] = self.eq_angles[1]
            print("Invalid range of motion")
        self.ROM = (self.eq_angles[1] - self.eq_angles[0]) / 2
        self.offset = (self.eq_angles[1] + self.eq_angles[0]) / 2

        self.set_KD_parameter(params[index+ 6], params[index+ 7])
        # self.alpha_ext_torque = params[8]
        self.lb = params[index+ 8]
        self.ub = params[index+ 9]
        self.res_work_scaling_facts[0] = params[index+ 10]
        self.res_work_scaling_facts[1] = params[index+ 11]
        self.forget_factor = params[index+ 12]
        # loadcells
        # self.robot.ratio_jlc = params[14]
        self.robot.filt_loadcell = params[index+ 13]
        self.robot.mass_foot = params[index+ 14]
        self.robot.l_foot  = params[index+ 15]

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True

    def get_params_ctrl(self):
        return [
            self.run,
            self.enable_FF,
            self.use_energy,
            self.freq_oscillation,
            self.eq_angles[0],
            self.eq_angles[1],
            self.K, self.B,
            # self.alpha_ext_torque,
            self.lb,
            self.ub,
            self.res_work_scaling_facts[0],
            self.res_work_scaling_facts[1],
            self.forget_factor,
            # load cell params
            # self.robot.ratio_jlc,
            self.robot.filt_loadcell,
            self.robot.mass_foot,
            self.robot.l_foot,
        ]


    def create_report(self, out, index) -> int:
        out[index +0] = self.robot.get_residual()
        out[index + 1] = self.robot.get_tau_int_through_loadCell()
        out[index + 2] = self.filtered_ext_torque
        out[index + 3] =  self.res_pwr
        out[index + 4] = self.residual_work
        out[index + 5] = self.freq_oscillation
        out[index + 6] = self.phase
        index += self.rep_ctrl_msg_dim

        # TODO: last assigned to decorator
        # out[self.rep_robot_msg_dim + self.rep_ctrl_msg_dim +0] = ....
        index += self.rep_dec_msg_dim
        return self.robot.create_report(out, index)


    ### ACTUAL CONTROLLERS METHODS
    def _compute_des_torque(self):
        # enable feedforward components
        if self.enable_FF and len(self.decorators)>0:
            return self.decorators[0].compute_des_torque()
        else:
            return self.tau_default


    def _compute_auxiliary(self) ->float:

        # update feedforward components
        if len(self.decorators)>0: self.decorators[0].compute_auxiliary()

        # update interaction work even when controller it is not enable
        # compute tau_ext work
        # self.filtered_ext_torque = (
        #         self.alpha_ext_torque * self.robot.get_residual() + (1 - self.alpha_ext_torque) * self.robot.get_tau_int_through_loadCell())
        self.filtered_ext_torque =  self.robot.get_tau_int_through_loadCell()

        # # all this maybe make more sense to include inside a parallel loop of the controller
        if self.filtered_ext_torque < self.lb:
            self.filtered_ext_torque -= self.lb
        elif self.filtered_ext_torque > self.ub:
            self.filtered_ext_torque -= self.ub
        else:
            self.filtered_ext_torque = 0.0
        # lower the forget factor faster it forgot

        self.res_pwr = self.filtered_ext_torque * self.robot.vel * math.pi / 180
        self.residual_work = self.forget_factor * self.residual_work + self.res_pwr * self.dt

        if self.run:
            ## update frequency
            if self.use_energy:
                self.freq_oscillation -= self.res_work_scaling_facts[0] * sigmoid(
                    self.residual_work / self.res_work_scaling_facts[1])
            self.freq_oscillation = max(min(self.freq_oscillation, self.freq_bounds[1]), self.freq_bounds[0])
            self.phase += 2 * math.pi * self.freq_oscillation * self.dt  # TODO: probably the energy shoudl change also the amplitude --> double check this
            self.phase %= 2 * math.pi  # wrap around
            if (0 <= self.phase < math.pi) and self.robot.get_robot_state() == 0:  # stance -->swing
                # update kin model for swing
                self.robot.set_robot_state(1)
            elif (math.pi <= self.phase < 2* math.pi) and self.robot.get_robot_state() == 1: # swing --> stance
                # update stance kin model
                self.robot.set_robot_state(0)
        return 0.0

    def _execute_des_pose(self) -> float:

        ## FF should do nothing here
        if len(self.decorators) > 0: self.decorators[0].execute_des_pose()

        if self.startup_flag:
            self.robot.set_position_set_point(self.robot.pos)
            self.startup_flag = False

        ## During stance phase the equilibrium angle stays to zero
        # namely the first pi of the gait phase is swing
        # the second pi is stance
        if 0 <= self.phase < math.pi: # swing phase
            q_des = self.ROM * math.sin(self.phase) + self.offset
            qdot_des = self.ROM * 2 * math.pi * self.freq_oscillation * math.cos(self.phase)

            if q_des < 0:
                raise Exception("q_des < 0")
            self.robot.set_transition_point(q_des, qdot_des, self.dt)


