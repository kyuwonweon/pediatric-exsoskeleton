import time

import micropython                                  # type: ignore
from micropython import const                       # type: ignore
import pyb                                          # type: ignore
import json
from lib.Robots.RobotInterface import Robot
from lib.config import DeviceParameters
from lib.ll_common import mode_switch


# SuperClas controller (all other controllers will extend this class)
class Controller:
    def __init__(self, robot:Robot, file_name:str, ctrl_name:str, device_params:DeviceParameters, decorator = False) -> None:
        self.robot = robot
        self.ctrl_name = ctrl_name
        with open(file_name, "r") as f:
            self.params = json.load(f)
        # print(json.dumps(self.params))
        self.run = False

        self.device_parameters = device_params
        self.ctrl_freq = self.device_parameters.TIMER_CONTROLLER['FREQ']
        self.dt = 1.0/self.device_parameters.TIMER_CONTROLLER['FREQ']
        self.tau_default = self.params[ctrl_name]['FEEDFORWARD_TORQUE']

        # reporter dimension of the robot msg and ctrl msg
        self.rep_robot_msg_dim = self.params["reporter"]['dim_robot_msg']
        self.rep_ctrl_msg_dim = self.params["reporter"]['dim_ctrl_msg']
        self.rep_dec_msg_dim = self.params["reporter"]['dim_decorator_msg']

        # input manager msgs
        self.dim_robot_cal_mgs = self.params["input_manager"]['dim_robot_cmd']
        self.dim_ctrl_input_msg = self.params["input_manager"]['dim_ctrl_cmd']

        #header for Input manager
        self.ctrl_header = self.params[self.ctrl_name]['header']

        # a controller can be defined as a decorator or a pure controller,
        # if the controller is defined as decorator its behavior can be borowed without need of rewrite
        self.decorator = decorator
        # if a controller is defined as decorator the timer will be the one of the borrowing controller
        if self.decorator:
            self.timer_controller = None
        else:
            self.timer_controller = pyb.Timer(
                self.device_parameters.TIMER_CONTROLLER['ID'],
                freq = self.ctrl_freq,
                callback=self._controller_callback
            )

        # all controllers potentially has multiple decorators
        self.decorators=[]

    def en(self, active=True):
        if not self.robot.is_enable():
            self.robot.enable(active)

    def disable(self):
        self.run = False

    def create_report(self, out, index) -> int:
        # nothing to report here
        index += self.rep_ctrl_msg_dim
        # nothing to report here
        index += self.rep_dec_msg_dim
        return self.robot.create_report(out, index)

    def calibrate_robot(self, msg):
        self.robot.calibrate(msg)

    def get_params_ctrl(self):
        pass

    def set_params_ctrl(self, params:list[float], index:int)-> bool:
        index += self.dim_ctrl_input_msg
        if not self.robot.calibrate(params, index): return False
        return True

    def add_decorator(self, ctrl: Controller):
        """
        :param ctrl: a controller object defined as a decorator.
        To define a controller as a decorator:
        ctrl = Controller(robot, decorator = False)
        moreover some controller are not accepted as decorator to see the decorators
        accepted inside the params.json file.
        Args:
            ctrl:
        Returns:
        """
        # check controller is defined as a decorator (namelly no secondary loop running)
        if not ctrl.decorator:
            raise Exception("Only possible to call if ctrl is defined as a decorator")
        # check that the name of the controller is inside the possible decorators the controller support
        if not ctrl.ctrl_name in self.params[self.ctrl_name]["decorators"]:
            raise Exception("These two controllers can't be merged")
        self.decorators.append(ctrl)

    # private internal methods that describe the behavior of the contorller
    # these are the methods that should be overwritten
    def _execute_des_pose(self) -> None:
        pass  # pose (position and velocity are not comulative so the actual action is executed inside here without returns

    def _compute_des_torque(self)-> float:
        return 0.0
    ## this one is to compute auxiliary measures that can be indipendent by the three above
    # or can be used by more than one, default return 0.0
    def _compute_auxiliary(self)->float:
        return 0.0

    ### THere is a public version that can be called only if the controller is used as a decorator
    def execute_des_pose(self) -> None:
        if self.decorator:
            self._execute_des_pose()
        else:
            raise Exception("Only possible to call if method is defined as a decorator")
    def compute_des_torque(self) -> float:
        if self.decorator:
            return self._compute_des_torque()
        else:
            raise Exception("Only possible to call if method is defined as a decorator")
    def compute_auxiliary(self) -> float:
        if self.decorator:
            return self._compute_auxiliary()
        else:
            raise Exception("Only possible to call if method is defined as a decorator")


    def _controller_callback(self, timer_object):

        self._compute_auxiliary()

        tau_des = self._compute_des_torque()
        if self.run:
            self.robot.set_tau_offset(tau_des)
            self._execute_des_pose()
        else:
            self.robot.set_tau_offset(self.tau_default)

