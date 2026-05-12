import pyb                                          # type: ignore
import json
import micropython                                  # type: ignore
from micropython import const                       # type: ignore

### const for Robot
_GRAVITY= const(9.81)

class Robot:
    def __init__(self, robot_file,
                 robot_name,
                 device_parameters
                 ):

        with open(robot_file, "r") as f:
            self.params = json.load(f)

        self.device_parameters = device_parameters

        # robot parameter
        self.pos =0.0  # degs
        self.vel = 0.0
        self.acc = 0.0
        self.transmission = 100
        self.run = False

        # proportional parameters loop:
        self.K = self.params[robot_name]["K_initial"]
        self.B = self.params[robot_name]["B_initial"]
        self.K_bounds = self.params[robot_name]['K_bounds']
        self.B_bounds = self.params[robot_name]['B_bounds']

        # these are for acc computation
        self.robot_frequency = self.device_parameters.TIMER_ROBOT['FREQ']

        self.robot_name = robot_name

        # reporter dimension of the robot msg
        self.rep_robot_msg_dim = self.params["reporter"]['dim_robot_msg']

        # calibration msg has dimension defined in the json file
        self.dim_robot_cal_mgs = self.params["input_manager"]['dim_robot_cmd']

    ###########################
    ### PUBLIC METHODS

    # getter functions
    def get_position(self):
        return self.pos
    def get_velocity(self):
        return self.vel
    def get_acceleration(self):
        return self.acc
    def get_transmission(self):
        return self.transmission

    def is_enable(self):
        return self.run

    def enable(self, active):
        pass

    # create report specific of the interface
    def create_report(self, out, index) -> int:

        # all robot report has dimension defined in params file
        # first three are dedicated to:
        # 0 - position
        # 1 - velocity
        # 2 - acceleration

        out[index] = self.get_position()
        out[index+1] = self.get_velocity()
        out[index+2] = self.get_acceleration()
        return index+3


    def calibrate(self, msg:list[float], index) -> bool:
        """
        This is the unique calibration method accessible from aux classes, internally this function
        manage all the possible calibration procedures.
        Args:
            msg: int
        Returns:
            None
        """
        return True

    #####################
    ### PRIVATE

    # Timer function
    def _start_timer(self):
        self.timer_update_robot = pyb.Timer(
            self.device_parameters.TIMER_ROBOT['ID'],
            freq=self.device_parameters.TIMER_ROBOT['FREQ'],
            callback=self._update_callback
        )


    # update functions
    def _update_position(self, static =False):
        pass

    def _update_velocity(self):
        pass

    def _update_acceleration(self):
        pass

    def _compute_transmission(self):
        pass


    def _update_K(self):
        pass

    def _update_callback(self, timer_object):
        pass


