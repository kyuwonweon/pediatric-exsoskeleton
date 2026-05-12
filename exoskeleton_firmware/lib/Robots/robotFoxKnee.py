import pyb                                          # type: ignore
import json
import math

from lib.Hardware.CAN import FSR, CANCommunication, LoadCell
from lib.Hardware.imu_streamer import ImuStreamer
from lib.Hardware.servoMotor import Servo
import micropython                                  # type: ignore

from micropython import const                       # type: ignore
from lib.ll_common.const_dict import CONST_DICT as const_dict

from lib.Robots.RobotInterface import Robot
import time

from lib.ll_common.error_flag import print_error

### const for Robot
_GRAVITY= const(9.81)



class FoxKnee(Robot):
    def __init__(self, robot_file,
                 robot_name, controller_board, device_type,
                 device_parameters
                 ):
        super().__init__(robot_file, robot_name, device_parameters)

        # Servo params specifics of Novanta (all other params are defined in the super class)
        self.Kpp = 0.0
        self.Kpv = 0.0
        self.Kpt = 0.0

        # gravity
        self.tau_g = 0
        self.mass = self.params[robot_name]["mass"]  # 2x thigh and shank masses in order[kg]
        self.l_com = self.params[robot_name]["l_com"]  # 2x distance CoM wrt q1 joint [m] --> this convention is only momentary (until no more joints are added)
        self.mass_foot = self.params[robot_name]["mass_foot"] # this measure is for the loadcell and computes only torque due to the part lower to the loadcell(no motor component)
        self.l_foot = self.params[robot_name]["l_foot"]
        # spring
        self.tau_K = 0
        self.k_spring = self.params[robot_name]["K_spring"]  # [Nm/degs]
        self.eq_angle = self.params[robot_name]["eq_angle"]  # [degs]
        # viscous force
        self.tau_b = 0
        self.b = self.params[robot_name]["b"]
        # couloumb force
        self.tau_c = 0
        self.v_th = self.params[robot_name]["v_th"]
        self.stic_p = self.params[robot_name]["stic_p"]
        self.coul_p = self.params[robot_name]["coul_p"]
        # inertial Force
        self.tau_I = 0
        self.I = self.params[robot_name]["I"] # inertia for each link wrt the q1 joint
        self.tau_offset = self.params[robot_name]["tau_offset"]

        # load cell params
        self.My_offset = self.params[robot_name]["My_offset"]
        # Momentum at the load cell must be rescaled to be compared with momentum at the joint
        self.ratio_jlc =  self.params[robot_name]["ratio_jlc"] # ratio between the distance joint and loadcell and loadcell pos contact
        self.filt_loadcell = self.params[robot_name]["filt_loadcell"]

        # robot state (0 = stance, 1, swing)
        self._robot_state =1

        # residual computation
        self.p_hat = 0.0
        self.res = 0.0

        canComm = CANCommunication()
        # load cells
        self.loadCell = LoadCell(canComm.can, fifoID= 1, bank =1)
        self.load_cell_available = self.loadCell.setup()
        self.load_cell_Fz = 0
        self.load_cell_My = 0
        self.tau_int_loadcell = 0

        # FSR
        self.FSR = FSR(canComm.can, fifoID = 0, bank =0)
        self.FSR_available = self.FSR.setup()
        self.FSR_values = [0.0]*16
        self.FSR_a_params= [1.0]*16
        self.FSR_b_params= [1e-3]*16

        # imu sensor
        self.imu_str = ImuStreamer()
        self.imu_sag_angle_knee = 0
        # self.imu_offset_calibration =0

        # servo
        current_limit = self.params["servo"]["CURRENT_LIMIT"] # TODO: move in servo
        self._initialize_servo(controller_board, device_type, current_limit)

        # update timer
        self._start_timer()



    ###########################
    ### PUBLIC METHODS

    ### GETTER FUNCTIONS ###
    def get_thigh_pos(self):
        return self.imu_sag_angle_knee - self.pos
    def get_theta_des(self):
        return self.servo.position_commanded * const_dict['KNEE_JOINT_ENC_COEF']
    def get_torque_des(self):
        return self.servo.torque_commanded
    def get_torque_act(self):
        return self.servo.actual_torque
    def get_robot_state(self):
        return self._robot_state
    def get_gravity(self):
        return self.tau_g
    def get_inertia_tau(self):
        return self.tau_I
    def get_spring_tau(self):
        return self.tau_K
    def get_viscous_tau(self):
        return self.tau_b + self.tau_offset
    def get_coul_tau(self):
        return self.tau_c
    def get_residual(self):
        return self.res
    def get_loadCell(self):
        return self.load_cell_Fz, self.load_cell_My
    def get_FSR_sum(self):
        foot_force=0.0
        for i in range(len(self.FSR_values)):
            foot_force += self.FSR_a_params[i] * math.exp(self.FSR_b_params[i] * self.FSR_values[i])
        return foot_force
    
    def get_current(self):
        return self.servo.get_current()
    def get_tempetature(self):
        return self.servo.get_temperature()

    def get_tau_int_through_loadCell(self):
        return self.tau_int_loadcell

    def create_report(self, out, index) -> int:
        new_index = super().create_report(out, index)

        # all robot report has dimension defined in params file
        # first three are filled in the interface and dedicated to:
        # 0 - position
        # 1 - velocity
        # 2 - acceleration
        # ---- locally defined ----
        # 3 - loadcell Fz
        # 4 - loadcell My
        # 5 - thigh positionll
        out[new_index]= self.get_loadCell()[0]
        out[new_index + 1] = self.get_loadCell()[1]
        out[new_index + 2] = self.get_thigh_pos()
        out[new_index + 3] = self.get_FSR_sum()
        # current and temperature
        out[new_index +4] = self.get_current()
        out[new_index +5] = self.get_tempetature()
        return index + self.rep_robot_msg_dim

    #### INITIALIZE FUNCTIONS

    def enable(self, active):
        self.servo.enable(active)
        self.loadCell.enable()
        self.FSR.enable()
        pyb.delay(1000)
        # default set the servo in PVT mode
        self.servo.change_PVT_profile()
        self.run = True

    ### CALIBRATION FUNCTIONS: Need some improvement
    # currently need to place the leg horizontally
    def calibrate_IMU(self):
        print("Calibrating IMU")
        self.imu_str.reset_orientation()
        # self.imu_offset_calibration = self.imu_sag_angle_knee

    def calibrate_loacell(self):
        print("Load cell calibration")
        self.My_offset = - self.load_cell_My # convention sign same of Kukuctabak TRO

    def calibrate(self, msg:list[float], index) -> bool:
        """
        This is the unique calibration method accessible from aux classes, internally this function
        manage all the possible calibration procedures.
        Args:
            msg:  list[int], by default all zero
                if index 0 ==1 --> imu calibration
                elif index 2 ==1 --> load-cell calibration[int]
        Returns:
            None
        """

        if int(msg[index+0]) == 1:
            self.calibrate_IMU()
        elif int(msg[index+1]) == 1:
            self.calibrate_loacell()
        return True

    #########################
    #### setter functions
    def set_position_set_point(self, position):
        # position should be in degs
        position=  int(position / const_dict['KNEE_JOINT_ENC_COEF'])
        self.servo.set_pos_loop_set_point(position)

    def set_transition_point(self, pos, vel = 0.0, time = 1.0):
        """
        THis function set a position in the servo (specific for PVT profiler)
        Args:
            pos (float): The angle in degrees.
            vel (float): The velocity in degs/s. This is the arriving velocity. By default is zero.
            time (float): The time in seconds. For interpolation profiler this is not used.
                        The one used for this mode is the product between the exp and the mantissa one.
        Returns:
            None
        """
        pos = int(pos / const_dict['KNEE_JOINT_ENC_COEF'])
        # Convert degs/s to rev/s
        vel = vel/360
        self.servo.append_execute_buffer(pos, vel, time)

    def set_tau_offset(self, tau=0.0):
        self.servo.set_tau_offset(tau)

    def set_KD_parameter(self, K: float, B: float) -> None:
        """
        Function to set the stiffness and damping parameter.
            :param K: Stiffness coefficient. The value must be in the range [0, 2]. If value exceeds the range the bound value will be applied.
            :param B: Damping coefficient. The value must be in the range [0, 0.1]. If value exceeds the range the bound value will be applied.
            :return: None
        """
        if K < self.K_bounds[0] or K > self.K_bounds[1]:
            print(f'KD parameter must be in the range [{self.K_bounds[0]}, {self.K_bounds[1]}]. Buonded value applied')
            K = min(max(self.K_bounds[0], K), self.K_bounds[1])
        if B < self.B_bounds[0] or B > self.B_bounds[1]:
            print(f'Damping parameter must be in the range [{self.B_bounds[0]}, {self.B_bounds[1]}]. Buonded value applied')
            B = min(max(self.B_bounds[0], B), self.B_bounds[1])
        print('K = ', K)
        print('D = ', B)
        self.K = K
        self.B = B

    def set_robot_state(self, state):
        if state == 0 or state == 1 :
            self._robot_state = state
        else:
            raise ValueError('State must be 0 or 1.')

    #####################
    ### PRIVATE

    ## Initialize functions
    def _initialize_servo(self, controller_board, device_type, current_limit):
        self.servo = Servo(controller_board, device_type, current_limit)

    # update functions
    def _update_position(self, static =False):
        # function that return and update the value of position
        if static:
            pos = self.servo.position(static) * const_dict['KNEE_JOINT_ENC_COEF']
        else:
            pos = self.servo.position * const_dict['KNEE_JOINT_ENC_COEF']
        self.pos = (pos + 180) % 360 - 180

    def _update_velocity(self):
        # function that update and return the value of vel
        self.vel = self.servo.velocity * 360.0

    def _update_acceleration(self):
        self.acc = self.servo.acceleration * 360.0

    def _update_residual(self):
        # this is updated only if needed
        K = 30
        tau = self.servo.torque_commanded

        # real momentum
        if self._robot_state ==0: ## stance
            p = self.I[0] * self.vel
        elif self._robot_state ==1: ## swing
            p = self.I[1] * self.vel
        # observer dynamics (Euler integration)
        if self.pos < 2: # close to the physical block the moment is due purelly to the physical block
            dp_hat = 0.0 - K * (self.p_hat - p)
        else:
            dp_hat = tau - self.tau_g - self.tau_K - self.tau_b - K * (self.p_hat - p)
        dt = 1 / self.robot_frequency
        self.p_hat += dp_hat * dt

        # external torque estimate
        self.res = K * (self.p_hat - p)

    # gravity function during stance phase
    def _compute_gravity_stance(self):
        sin_theta= math.sin( (self.imu_sag_angle_knee - self.pos) * math.pi/180)
        self.tau_g= self.mass[0] * self.l_com[0] * _GRAVITY * sin_theta

    # gravity function during swing phase
    def _compute_gravity_swing(self):
        sin_theta= math.sin(self.imu_sag_angle_knee * math.pi/180)
        self.tau_g= self.mass[1] * self.l_com[1] * _GRAVITY * sin_theta

    def _compute_gravity(self):
        if self._robot_state ==0: ## stance
            self._compute_gravity_stance()
        elif self._robot_state ==1: ## swing
            self._compute_gravity_swing()

    def _compute_spring_force(self):
        self.tau_K = self.k_spring*(self.pos - self.eq_angle)

    def _compute_viscous_force(self):
        self.tau_b = self.b * self.vel

    def _compute_stic_coul_force(self):
        sign_vel = 0
        if self.vel < 0: sign_vel= -1
        elif self.vel > 0: sign_vel = 1
        # Smooth transition factor between stiction and Coulomb
        transition = math.tanh(self.vel / self.v_th)
        # Blend stiction and Coulomb smoothly
        self.tau_c = self.stic_p * sign_vel * (1 - abs(transition)) \
                     + self.coul_p * sign_vel * abs(transition)

    def _compute_Inertial_force(self):
        if self._robot_state ==0: ## stance
            self.tau_I = self.I[0] * self.acc
        elif self._robot_state ==1: ## swing
            self.tau_I = self.I[1] * self.acc

    def _compute_loadCell(self):
        # lc = self.loadCell.get_value()
        self.load_cell_Fz, new_My = self.loadCell.load_cell_Fz, self.loadCell.load_cell_My
        ## filtering of My
        self.load_cell_My = (1-self.filt_loadcell) *  self.load_cell_My +  self.filt_loadcell * new_My

    def _compute_FSR(self):
        self.FSR_values = self.FSR.get_value()

    def _compute_tau_int_through_loadCell(self):
        # Remove foot gravity component
        sin_theta = math.sin(self.imu_sag_angle_knee * math.pi / 180)
        tau_g_foot = self.mass_foot * self.l_foot * _GRAVITY * sin_theta
        My_load_cell = - self.load_cell_My - tau_g_foot - self.My_offset
        self.tau_int_loadcell =  (self.ratio_jlc + 1) * My_load_cell


    def _update_imu(self):
        # TODO check
        self.imu_sag_angle_knee = self.imu_str.get_roll_pitch_yaw() #- self.imu_offset_calibration


    def _compute_transmission(self):
        '''
        Calculates the transmission ratio for the Fox Knee from the current joint angle in degrees
        '''

        terms = const_dict['FOX_TRANSMISSION_CALCULATION_TERMS']

        gamma_prime = math.radians(self.pos - terms[0])

        self.transmission = (
                terms[1] * math.sin(gamma_prime)
                / math.sqrt(terms[2] + terms[3] * math.cos(gamma_prime))
        )


    def _update_K(self):
        if self.B > 0.3:
            self.B = 0.3
        # self.Kpv = 360.0 * self.B / self.n  # motor velocity
        self.Kpv = 360.0 * self.B  # joint velocity
        # Avoid division by zero when Kpv is zero
        if self.Kpv == 0.0:
            self.Kpp = 0.0
        # The normal controller case:
        else:
            self.Kpp = (
                    self.K * const_dict['KNEE_JOINT_ENC_COEF'] / self.Kpv
            )
        self.Kpt = (
                1.0 / (self.transmission * const_dict['TORQUE_CONSTANT_LARGE_MOTOR'])
        )
        self.servo.set_Kpp(self.Kpp)
        self.servo.set_Kpv(self.Kpv)
        self.servo.set_Kpt(self.Kpt)


    def _update_callback(self, timer_object):
        if self.run:
            # update kin
            self._update_position()
            self._update_velocity()
            self._update_acceleration()
            self._compute_transmission()
            self._update_K()

            # dynamics
            self._compute_gravity()
            self._compute_spring_force()
            self._compute_viscous_force()
            self._compute_stic_coul_force()
            self._compute_Inertial_force()

            # residual
            self._update_residual()

            # update sensors
            self._compute_loadCell()
            # self._compute_FSR()
            self._compute_tau_int_through_loadCell()
            self._update_imu()


