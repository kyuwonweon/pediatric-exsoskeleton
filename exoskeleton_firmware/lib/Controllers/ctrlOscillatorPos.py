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
## Controller oscillator based on Righetti original works and more recently ronsse2011oscillator
## here we use the position as an observer
class ctrlOscillatorPos(Controller):
    def __init__(self, robot : Robot, file_name, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        super().__init__(robot, file_name, ctrl_name, device_params= device_params, decorator= decorator)

        self.K = self.params[ctrl_name]['K']
        self.B = self.params[ctrl_name]['B']
        self.startup_flag = True
        self.t2 = 0

        self.count = 0
        # current implementation has 2 freq fourier transform so 4 amplitudes (2 sin, 2 cos)
        self.amplitudes_fouriers = self.params[ctrl_name]["amplitudes"]
        self.offset = self.params[ctrl_name]["offset"]

        self.phase = 0
        self.freq_oscillation = self.params[ctrl_name]["freq_oscillator"]
        self.freq_bounds = self.params[ctrl_name]["freq_bounds"]
        self.q_des= 0.0
        self.qdot_des= 0.0
        self.F =0.0


        self.AFO = False # adaptable or not
        self.t0 = time.ticks_ms() / 1000  # record start time
        self.k_freq = self.params[ctrl_name]["k_freq"]
        self.k_phase = self.params[ctrl_name]["k_phase"]
        self.k_amp = self.params[ctrl_name]["k_amp"]

        # dynamics
        self.enable_FF = False


    def en(self, active=True):
        super().en(active)
        pyb.delay(1000)
        self.robot.set_KD_parameter(self.K, self.B)
        pyb.delay(1000)
        # move to initial pos
        self.robot.set_transition_point(self.compute_fourier(0.0), 0, 1)
        pyb.delay(100)
        self.run = True
        self.enable_FF = False

    def compute_fourier(self, phi):
        """
        Compute the Fourier transform of the given phase
        Args:
            phi: phase in the range [0, 2pi]
        Returns:
            angle value in degs
        """
        return (
                self.offset
                + self.amplitudes_fouriers[0] * math.cos(phi)
                + self.amplitudes_fouriers[1] * math.sin(phi)
                + self.amplitudes_fouriers[2] * math.cos(2 * phi)
                + self.amplitudes_fouriers[3] * math.sin(2 * phi)
        )

    def compute_fourier_dphi(self, phi):
        """
        Compute phase derivative of the Fourier-based trajectory
        Args:
            phi: phase in [0, 2pi]
        Returns:
            q_dot / dphi
        """
        return  (
                -self.amplitudes_fouriers[0] * math.sin(phi)
                + self.amplitudes_fouriers[1] * math.cos(phi)
                - 2 * self.amplitudes_fouriers[2] * math.sin(2 * phi)
                + 2 * self.amplitudes_fouriers[3] * math.cos(2 * phi)
        )

    def compute_fourier_dt(self, phi):
        """
        Compute time derivative of the Fourier-based trajectory

        Args:
            phi: phase in [0, 2pi]
        Returns:
            q_dot / dt
        """
        return 2 * math.pi * self.freq_oscillation * self.compute_fourier_dphi(phi)


    def use_FF_force(self, enable= True):
        self.enable_FF = enable

    def set_freq_oscillation(self, freq_oscillation):
        self.freq_oscillation = freq_oscillation

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

        self.use_FF_force(bool(params[index+ 1]))
        self.AFO =  bool(params[index+ 2])

        # all double ( this is the order rqt sort messages)
        # oscillator parameters
        self.freq_oscillation = params[index+ 3]
        self.offset = params[index+ 4]

        self.set_KD_parameter(params[index+ 5], params[index+ 6])

        self.k_freq = params[index+ 7]
        self.k_phase = params[index+ 8]
        self.k_amp = params[index+ 9]

        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True


    def get_params_ctrl(self):
        return [
            self.run,
            self.enable_FF,
            self.AFO,
            self.freq_oscillation,
            self.offset,
            self.K, self.B,
            self.k_freq, self.k_phase, self.k_amp,
        ]


    def create_report(self, out, index) -> int:

        out[index + 0] = self.freq_oscillation
        out[index + 1] = self.phase
        out[index + 2] = self.amplitudes_fouriers[0]
        out[index + 3] = self.amplitudes_fouriers[1]
        out[index + 4] = self.amplitudes_fouriers[2]
        out[index + 5] = self.amplitudes_fouriers[3]
        out[index + 6] = self.q_des
        out[index + 7] = self.qdot_des
        out[index + 8] = self.F

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

        if self.run:

            input_signal = self.robot.get_position()
            observer_input_signal = self.compute_fourier(self.phase)

            self.F =  input_signal - observer_input_signal

            if self.AFO:
                # adaptation
                eps = 1e-6  # avoid division by zero
                # freq reg coefficient
                dq_dphi = self.compute_fourier_dphi(self.phase)
                norm = dq_dphi**2 + eps

                self.freq_oscillation += (
                                        self.k_freq * self.F * dq_dphi / norm
                                         ) * self.dt

                # amplitude update (each amplitude is updated by its basis function)
                self.amplitudes_fouriers[0] += self.k_amp * self.F * math.cos(self.phase) * self.dt
                self.amplitudes_fouriers[1] += self.k_amp * self.F * math.sin(self.phase) * self.dt
                self.amplitudes_fouriers[2] += self.k_amp * self.F * math.cos(2 * self.phase) * self.dt
                self.amplitudes_fouriers[3] += self.k_amp * self.F * math.sin(2 * self.phase) * self.dt

                # bound freq
                self.freq_oscillation = max(min(self.freq_oscillation, self.freq_bounds[1]), self.freq_bounds[0])
                # update phase
                self.phase += (2 * math.pi * self.freq_oscillation
                               + self.k_phase * self.F * dq_dphi / norm) * self.dt
            else:
                self.phase += 2 * math.pi * self.freq_oscillation * self.dt

            self.phase = self.phase % (2 * math.pi)

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
        self.q_des = self.compute_fourier(self.phase)
        self.qdot_des = self.compute_fourier_dt(self.phase)

        if self.q_des < 0 or self.q_des >90:
            raise Exception(f"q_des out of bounds: {self.q_des:.2f}")
        self.robot.set_transition_point(self.q_des, self.qdot_des, self.dt)


