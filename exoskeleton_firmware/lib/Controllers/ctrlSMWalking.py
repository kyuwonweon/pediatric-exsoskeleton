import time

import micropython                                  # type: ignore
from micropython import const                       # type: ignore
import pyb                                          # type: ignore
import json
import math

from lib.config import DeviceParameters

from lib.Robots.RobotInterface import Robot
from lib.Controllers.ControllerInterface import Controller

from lib.Controllers.SM_states.EarlySwingState import EarlySwingState
from lib.Controllers.SM_states.LateSwingState import LateSwingState
from lib.Controllers.SM_states.StanceState import StanceState



#### individual instances of controllers
### controller that receive a reference position (and velocity) to follow
class CtrlSMWalking(Controller):
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        super().__init__(robot, file_name, ctrl_name, device_params= device_params, decorator= decorator)

        self.K = self.params[ctrl_name]['K']
        self.B = self.params[ctrl_name]['B']
        self.startup_flag = True
        self.t2 = 0
        self.count = 0

        self.eq_angle = self.params[ctrl_name]["eq_angle"]
        self.eq_velocity = 0.0
        self.t0 = time.ticks_ms() / 1000  # record start time

        self.en()
        self.init_states()

    def init_states(self) -> None:
        self.states = {
                    "EarlySwing" : EarlySwingState(self.robot),
                       "LateSwing" : LateSwingState(self.robot),
                       "Stance" : StanceState(self.robot),
                       }
        # This is the order in which they are defined inside the cfg file
        # TODO: This should be improved (for instance with code for each state)
        self.states_order = ["Stance", "EarlySwing", "LateSwing"]
        self.current_state = "Stance"
        self.state_number = self.states_order.index(self.current_state)
    
    def restart_states(self):
        for k_s in self.states_order:
            self.states[k_s].restart_state()

    def en(self, active=True):
        super().en(active)
        self.robot.set_KD_parameter(0, 0)
        self.robot.set_transition_point(self.eq_angle, 0, 1)
        self.robot.set_KD_parameter(self.K, self.B)
        
        # move to initial pos

    def set_KD_parameter(self, K: float, B: float) -> None:
        if not(self.K == K) or not(self.B == B):
            self.K = K
            self.B = B
            self.robot.set_KD_parameter(K, B)

    def set_params_ctrl(self, params: list[float], index:int) -> bool:

        # all bool first
        if bool(params[index + 0]) and not self.run and self.startup_flag:
            self.restart_states()
            self.current_state = "LateSwing"
            self.state_number = self.states_order.index(self.current_state)

            self.run = True
        elif bool(params[index + 0]) and not self.run and not(self.startup_flag):
            self.restart_states()
            self.current_state = "LateSwing"
            self.state_number = self.states_order.index(self.current_state)

            self.run = True
        elif not bool(params[index + 0]) and self.run:
            # self.disable()
            self.run = False

        # TODO: Update params, use a break word between states params and to identify each state for safety
        # TODO: Update should happen only not when it is not running otherwise it may create some issues with the current state
        # TODO: One option is that state machine don't pass them if running and them pass them as soon as it is off
        # TODO: --> This should be clear though --> maybe sending bad message back
        if self.run:
            print("Running, not passing params update to the states, sending them after stop")
        else:
            i =index + 1
            for k_s in self.states_order:
                n_params= len(self.states[k_s].get_params_state())
                params_s = params[i:i+n_params]
                self.states[k_s].set_params_state(params_s)
                i += n_params

            ### TODO: integrate better
            vel_tresh = params[i]
            i =i+1
            Fz_tresh = params[i]
            print("Vel tresh: ",vel_tresh)
            print("Fz tresh: ", Fz_tresh)
            for k_s in self.states_order:
                self.states[k_s].eps_vel = vel_tresh
                self.states[k_s].Fz_tresh = Fz_tresh

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True


    def get_params_ctrl(self):
        params = [
            self.run,
        ]
        for k_s in self.states_order:
            params += self.states[k_s].get_params_state()
        return params


    def create_report(self, out, index) -> int:
        out[index+0] = self.state_number
        index += self.rep_ctrl_msg_dim
        # nothing to report here
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
        # update decorators components
        if len(self.decorators)>0: self.decorators[0].compute_auxiliary()

        # check if state should change and in case change
        if self.run:
            if self.states[self.current_state].checkConditionForChangingState():
                self.current_state = self.states[self.current_state].getNextState()
                self.state_number = self.states_order.index(self.current_state)

        return 0.0

    def _execute_des_pose(self) -> float:
        if len(self.decorators) > 0: self.decorators[0].execute_des_pose()

        if self.startup_flag:
            ## state handle this
            self.states[self.current_state].handle()

