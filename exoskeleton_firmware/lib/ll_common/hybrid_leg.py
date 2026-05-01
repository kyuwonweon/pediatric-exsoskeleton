#############################################################################
# @file       hybrid_leg.py
#
# @details    The main application code for the hybrid leg
#
# @author     Frank Ursetta, Chandler Clark and Chrestien Ilaya
#############################################################################
#

#
#   IMPORTS
#

import array
import os
import gc
import math
import time

import pyb                              # type: ignore
import micropython                      # type: ignore
import machine                          # type: ignore
from uctypes import addressof           # type: ignore

from lib import config
from lib import tunable_settings

from . import mode_switch
from . import assembler_functions
from . import controller
from . import novanta
from . import LPS25HB
from . import BNO085
from . import routing
from . import ble_advertising
from . import bleComms
from . import state_machine
from . import logger
from . import upy_float
from . import upy_spi
from . import can_sender
from . import CAN_PDCP_Streaming_Message
from . import fts
from . import error_flag
from . import nvic
from . import hardware_manager
from . import caps_manager
from . import led_buzzer_handler
from . import caps_command_handler
from . param_dict import PARAM_DICT
from . flag_dict import FLAG_DICT
from . const_dict import CONST_DICT

print('IMPORTING HYBRID LEG')

class HybridLeg:
    # ========================================================================
    # INITIALIZATION AND SETUP
    # ------------------------------------------------------------------------
    # ========================================================================
    def __init__(self):
        self.initialize_imported_modules()
        self.initialize_device_parameters()
        gc.collect()

        self.import_pyb_sm()
        self.print_device_info()
        self.log_sm_channels()
        self.initialize_managers()
        self.set_watchdog_memory_view()
        gc.collect()

        # Active tab of phone app
        self.app_tab = 1  

    def initialize_imported_modules(self):
        self.config = config
        self.tunable_settings = tunable_settings
        self.assembler_functions = assembler_functions
        self.controller = controller
        self.novanta = novanta
        self.LPS25HB = LPS25HB
        self.BNO085 = BNO085
        self.fts = fts
        self.routing = routing
        self.ble_advertising = ble_advertising
        self.bleComms = bleComms
        self.state_machine = state_machine
        self.logger = logger
        self.upy_float = upy_float
        self.upy_spi = upy_spi
        self.can_sender = can_sender
        self.nvic = nvic
        self.CAN_PDCP_Streaming_Message = CAN_PDCP_Streaming_Message
        self.error_flag = error_flag
        self.PARAM_DICT = PARAM_DICT
        self.FLAG_DICT = FLAG_DICT
        self.CONST_DICT = CONST_DICT

    def initialize_device_parameters(self):
        """
        Determine the device type based on the mode switch position.
        For device_id:
        1: hybrid knee
        2: polycentric ankle
        3: polycentric ankle only
        8: fox knee
        """
        # Defined in boot.py
        self.selector = mode_switch.selector
        # Grab device parameters based on the device_id
        self.device_parameters = (
            self.config.DeviceParameters(self.selector.device_id)
        )

    def initialize_managers(self):
        """
        Initialize all system managers and handler instances.
        """
        # ====================================================================
        # Hardware Manager Initialization
        # --------------------------------------------------------------------
        # This creates instances of BrakingCircuit, Buzzer,
        # Everest(Servo), IMU, CAN bus, etc.
        self.hardware_manager = hardware_manager.HardwareManager(self)
        self.hardware_manager.initialize_all_hardware()

        # ====================================================================
        # CAPS Manager Initialization
        # --------------------------------------------------------------------
        # This creates CAPS channel handlers for incoming parameter data,
        # incoming custom commands, and outgoing streaming data.
        self.caps_manager = caps_manager.CapsManager(self)
        self.caps_manager.initialize_caps_handlers()

        # Cache FTS output availability for ISR use
        self.fts_output_enabled = hasattr(self, 'can_out_fts_data1')

        # Handler scripts for caps commands and LED/buzzer patterns
        self.caps_command = caps_command_handler.CapsCommandHandler(self)
        self.led_buzzer = led_buzzer_handler.LedBuzzerHandler(self)

    def import_pyb_sm(self):
        """
        Creates the necessary state machine variables if
        the device is a primary controller
        """
        if self.device_parameters.PRIMARY_CONTROLLER:
            self.sm_channels_current = array.array('f', [0] * 24)
            self.sm_channels_current_and_scale = array.array('i',
                    [addressof(self.sm_channels_current),
                     addressof(self.CONST_DICT['SM_JOINT_SCALE_PARAMS'])]
                )
            sm_channels_max = array.array('f', [0] * 24)
            sm_channels_min = array.array('f', [0] * 24)
            sm_channels_slope = array.array('f', [0] * 24)
            # Load python state machine file saved in main directory
            file_name = r"uPythonStateMachine.psm"
            self.sm_loaded = False
            if file_name in os.listdir():
                gc.collect()
                self.sm = self.state_machine.StateMachine(
                    self.sm_channels_current,
                    sm_channels_max,
                    sm_channels_min,
                    sm_channels_slope,
                    file_name
                )
                self.sm_loaded = True
                print('state machine loaded')
            else:
                print('state machine file not found')
        else:
            self.sm_loaded = False
            print('Device not primary, state machine not loaded')

    def print_device_info(self):
        """
        Prints device information to the console
        """
        device_names = {
            self.config.HYBRID_KNEE: "HYBRID KNEE",
            self.config.FOX_KNEE: "FOX KNEE",
            self.config.POLYCENTRIC_ANKLE: "POLYCENTRIC ANKLE",
            self.config.POLYCENTRIC_ANKLE_ONLY: "POLYCENTRIC ANKLE ONLY"
        }

        device_type = self.device_parameters.device
        device_label = device_names.get(device_type, None)

        if device_label:
            print('Active Device Configuration:         ',
                  device_label)
            # # BLE setup only for knee devices
            # if device_type in [self.config.HYBRID_KNEE, self.config.FOX_KNEE]:
            #     self.ble = self.bleComms.ubluetooth.BLE()
            #     self.b = self.bleComms.BLEComms(self.ble, device_label)
            print("Rotation direction:                  ",
                  self.device_parameters.ROTATION_DIRECTION)
            print("Degree limit in flexion:             ",
                  self.device_parameters.DEG_LIMIT_FLEX)
            print("Degree limit in extension:           ",
                  self.device_parameters.DEG_LIMIT_EXT)
        else:
            print('NO VALID DEVICE SPECIFIED IN CONFIG.PY')

    def log_sm_channels(self):
        """
        Sets up logging for the state machine channels
        if the device is a primary controller
        """
        if self.device_parameters.PRIMARY_CONTROLLER:
            self.logFile = self.logger.Logger(self.sm_channels_current, 2)

    def set_watchdog_memory_view(self):
        """
        Sets the memory view for the watchdog timer to monitor
        """

        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            self.PARAM_DICT['WATCHDOG_IMP_MESSAGE'] = \
                memoryview(self.CAPS2.data)
        else:        # Ankle
            self.PARAM_DICT['WATCHDOG_IMP_MESSAGE'] = \
                memoryview(self.CAPS6.data)


    # ========================================================================
    # UTILITY AND HELPER FUNCTIONS
    # ------------------------------------------------------------------------
    # ========================================================================

    def report_value(self, name, value):
        """
        Sets format for reporting values
        """
        # print('{: <36}{}'.format(name.upper(), value))
        print(name, value)

    def report_status(self, message):
        """
        Reports status message
        """
        print(message)

    def alarm_clock(self, duration):
        """
        Function that acts as an alarm clock. It should return True when
        a specified amount of time in microseconds has passed and otherwise
        is False.
        """

        if self.PARAM_DICT['ALARM_START_TIME'] == 0:
            self.PARAM_DICT['ALARM_START_TIME'] = time.ticks_us()

        if time.ticks_diff(time.ticks_us(),
                            self.PARAM_DICT['ALARM_START_TIME']
                            ) > duration:
            self.PARAM_DICT['ALARM_START_TIME'] = 0
            return True
        else:
            return False

    def barometer_read(self):
        """
        Reads the barometer and calculates log(barometer value)
        Output is saved in LOG_TAYLOR_COEFS[0]
        """
        self.PARAM_DICT['BAR_BYTE_VALS'][1] = self.barometer.readPress()
        self.assembler_functions.ass_log(
            self.CONST_DICT['LOG_TAYLOR_COEFS'],
            self.PARAM_DICT['BAR_BYTE_VALS']
        )


    # ========================================================================
    # KINEMATIC CALCULATIONS
    # ------------------------------------------------------------------------
    # ========================================================================

    def calculate_hybrid_knee_transmission(self, joint_angle):
        '''Calculates the transmission ratio for the hybrid knee.

        This function calculates the transmission ratio for the hybrid knee as
        a function of its joint angle. The kinematic parameters for the hybrid
        knee are brought in from the constants dictionary.

        Parameters:
            joint_angle (float): The joint angle in degrees.

        Returns:
            float: The transmission ratio.
        '''

        const_dict = self.CONST_DICT
        const_crank_helper_array_0 = const_dict['CRANK_HELPER_ARRAY'][0]
        const_pivot_bar_length_squared = const_dict['PIVOT_BAR_LENGTH'] ** 2
        const_cvt_angle_alpha = const_dict['CVT_ANGLE_ALPHA']
        const_crank_val = const_dict['CVT_ARM_LENGTH']
        const_knee_gear_ratio = const_dict['KNEE_GEAR_RATIO']
        x = (joint_angle + const_cvt_angle_alpha) * math.pi / 180
        sin_x = math.sin(x)
        cos_x = math.cos(x)

        crank_val_sin_x_minus_crank_helper_array_0 = (
            const_crank_val
            * sin_x
            - const_crank_helper_array_0
        )

        numerator = (
            const_crank_val
            * (
                sin_x
                - cos_x
                * (
                    crank_val_sin_x_minus_crank_helper_array_0
                    / math.sqrt(
                        const_pivot_bar_length_squared
                        - crank_val_sin_x_minus_crank_helper_array_0 ** 2)
                    )
                )
        )

        return (numerator * 2 * math.pi * const_knee_gear_ratio
                / const_dict['ROLLVIS_SCREW_LEAD'])

    def calculate_fox_knee_transmission(self, joint_angle):
        '''Calculates the transmission ratio for the Fox knee.
        
        This function calculates the transmission ratio for the Fox knee as a
        function of its joint angle. The kinematic parameters for the Fox knee
        are brought in from the constants dictionary.
        
        Parameters:
            joint_angle (float): The joint angle in degrees.
            
        Returns:
            float: The transmission ratio.
        '''

        terms = self.CONST_DICT['FOX_TRANSMISSION_CALCULATION_TERMS']

        gamma_prime = math.radians(joint_angle - terms[0])

        return (
            terms[1] * math.sin(gamma_prime)
            / math.sqrt(terms[2] + terms[3] * math.cos(gamma_prime))
        )


    # ========================================================================
    # MOTOR CONTROL AND SERVO MANAGEMENT
    # ------------------------------------------------------------------------
    # ========================================================================

    def update_control_parameters(self):
        '''Updates the control parameters for the large motor controller
        for all device types (hybrid knee, fox knee, and polycentric ankle).

        This method updates the position control gains and torque bias for the
        large motor controller. The gains are based on the K and B values from
        CAPS, but scaled appropriately for the current transmission ratio and
        the motor constant.

        Parameters:
            None

        Returns:
            None
        '''

        CONST_DICT = self.CONST_DICT
        PARAM_DICT = self.PARAM_DICT
        controller_parameters = self.everest_pb.controller_parameters

        K = controller_parameters[12]
        B = controller_parameters[1]

        # Limit B to safe range for now. May increase based on testing.
        if B > 0.6:
            B = 0.6

        k_t = CONST_DICT['TORQUE_CONSTANT_LARGE_MOTOR']
        n = PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'][0]
        
        # Device-specific encoder coefficient
        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            c = CONST_DICT['KNEE_JOINT_ENC_COEF']  # encoder deg/count
        elif (
            self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
            or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
        ):
            c = CONST_DICT['ANKLE_JOINT_ENC_COEF']
        
        # Device-specific KPV calculation
        if self.device_parameters.device == self.config.HYBRID_KNEE:
            PARAM_DICT['EVEREST_KPV'] = 360.0 * B / n
        elif self.device_parameters.device == self.config.FOX_KNEE:
            PARAM_DICT['EVEREST_KPV'] = 360.0 * B  # No division by n for Fox knee
        elif (
            self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
            or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
        ):
            PARAM_DICT['EVEREST_KPV'] = 360.0 * B / n
        
        # Common KPP calculation
        if B == 0.0:
            PARAM_DICT['EVEREST_KPP'] = 0.0
        else:
            PARAM_DICT['EVEREST_KPP'] = c * K / PARAM_DICT['EVEREST_KPV']
        
        # Common KPT calculation
        PARAM_DICT['EVEREST_KPT'] = 1 / (n * k_t)

        # Device-specific set point calculation
        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            PARAM_DICT['EVEREST_SET_POINT'] = int(controller_parameters[10] / c)
        elif (
            self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
            or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
        ):
            # Convert set point angle into encoder counts using polynomial
            try:
                self.assembler_functions.calculate_polynomial(
                    PARAM_DICT['EVEREST_SET_POINT'], 
                    controller_parameters[10], 
                    self.CONST_DICT['P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS_INVERTED'], 
                    len(self.CONST_DICT['P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS_INVERTED'])
                )
            except Exception as e:
                print('update control parameters polycentric ankle error')

        # Common torque bias calculation
        PARAM_DICT['EVEREST_TORQUE_BIAS'] = controller_parameters[19]

    def idle_brake_mode(self):
        """
        Sets the device to idle brake mode
        """

        # Disabling large motor driving
        self.FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG'] = False

        # Set parameters related to everest and dynamic braking
        everest_enabled = self.FLAG_DICT['EVEREST_OPERATION_IS_ENABLED']
        not_deactivating = not self.FLAG_DICT['DEACTIVATING_EVEREST_FLAG']
        
        if everest_enabled and not_deactivating:
            # Indicates everest should be deactivated
            self.FLAG_DICT['DEACTIVATING_EVEREST_FLAG'] = True

        # Checking to see if free braking should be applied
        if not self.FLAG_DICT['EVEREST_OPERATION_IS_ENABLED']:
            if (
                self.device_parameters.device == self.config.HYBRID_KNEE
                or self.device_parameters.device == self.config.FOX_KNEE
            ):
                dynamic_brake = int(
                    self.PARAM_DICT['DYNAMIC_BRAKE_PERCENTAGE'][0]
                )
                caps_brake_command = int(
                    self.PARAM_DICT['BRAKE_PERCENTAGE'][0]
                )
                
                if dynamic_brake > caps_brake_command:
                    # Apply output of the dynamic brake equation
                    self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] = (
                        dynamic_brake
                    )
                else:
                    # Apply the output of the CAPS brake percentage
                    self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] = (
                        caps_brake_command
                    )
                
                # Limit dynamic braking percentage to 100
                if self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] > 100:
                    self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] = 100

                # If the brake flag is not active
                if not self.FLAG_DICT['DYNAMIC_BRAKE_STATE']:
                    # Setting braking to 0 - completely free swinging
                    self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] = 0

                # Applying braking percentage
                self.braking_circuit_pwm_ch.pulse_width_percent(
                    self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE']
                )

    def large_motor_active_mode(self):
        """
        Sets the device to large motor active mode
        """

        # print('everest driving mode')
        # Turning off braking only applies to the hybrid_knee
        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            # Indicating zero braking
            self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE'] = 0
            # Setting PWM value for braking to zero - no braking
            self.braking_circuit_pwm_ch.pulse_width_percent(
                self.PARAM_DICT['APPLIED_BRAKE_PERCENTAGE']
            )
            
        # If the everest is properly activated, drive main motor
        if self.FLAG_DICT['EVEREST_OPERATION_IS_ENABLED']:
            # Set drive flag to True to start driving main motor
            self.FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG'] = True
        elif not self.FLAG_DICT['ACTIVATING_EVEREST_FLAG']:
            # Raise flag to indicate everest should be activated
            self.FLAG_DICT['ACTIVATING_EVEREST_FLAG'] = True

    def activate_everest(self):
        '''activate_everest(): brings Everest to active state by transitioning
        its states from READY_TO_SWITCH_ON to SWITCHED_ON, then to
        OPERATION_ENABLED'''

        '''
        TODO: assess whether everest_failure_flag and activating_everest_flag
        are still needed.'''

        # print('Attempting to activate Everest', time.ticks_us())

        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            # Disable braking fets - this must be done before activation
            self.braking_circuit_fet_enable(0)
        # If everest not active or being activated, then activate
        self.everest_enable(1)
        self.FLAG_DICT['STO_ALARM_ENABLED'] = True

        # Create a 5ms of time to allow for the sto lines to go high 
        # before attempting operation enable. It returns True after 5ms 
        # has passed, otherwise False.
        if self.alarm_clock(5000):
            # this call does the actual work for changing the Everest to
            # OPERATION_ENABLE. It returns 0 for success, otherwise, it 
            # returns the status word:
            self.PARAM_DICT['EVEREST_ERROR'] = (
                self.servo.operation_enable_cyclic()
            )
            
            if not self.PARAM_DICT['EVEREST_ERROR']:
                self.FLAG_DICT['EVEREST_FAILURE_FLAG'] = False
                self.FLAG_DICT['EVEREST_OPERATION_IS_ENABLED'] = True
                print('Everest successfully activated')
            else:
                # Indicate that an error has occurred
                self.FLAG_DICT['EVEREST_FAILURE_FLAG'] = True
                print('Everest failed to activate')
                
            self.FLAG_DICT['STO_ALARM_ENABLED'] = False
                
        # Indicate the end of the everest activation process
        self.FLAG_DICT['ACTIVATING_EVEREST_FLAG'] = False

    def deactivate_everest(self):
        '''deactivate_everest(): brings Everest to inactive state by 
        transitioning its states from OPERATION_ENABLED directly to 
        READY_TO_SWITCH_ON'''

        '''
        TODO: assess whether everest_failure_flag and
        deactivating_everest_flag are still needed.'''

        # print('Attempting to deactivate Everest')

        self.PARAM_DICT['EVEREST_ERROR'] = 1

        # this call does the actual work for changing the Everest to
        # OPERATION_DISABLE. It returns 0 for success, otherwise, it 
        # returns the status word:
        self.PARAM_DICT['EVEREST_ERROR'] = (
            self.servo.operation_disable_cyclic()
        )

        if not self.PARAM_DICT['EVEREST_ERROR']:
            print('Everest successfully deactivated')
            # Setting everest sto1 low immediately - Prevents a scheduling 
            # conflict from preventing an everest shut down
            self.everest_enable(0)
            if (
                self.device_parameters.device == self.config.HYBRID_KNEE
                or self.device_parameters.device == self.config.FOX_KNEE
            ):
                # keeping STO lines consistent
                self.braking_circuit_fet_enable(1)
            self.FLAG_DICT['EVEREST_FAILURE_FLAG'] = False
            self.FLAG_DICT['EVEREST_OPERATION_IS_ENABLED'] = False
        else:
            # Indicate that an error has occurred
            self.FLAG_DICT['EVEREST_FAILURE_FLAG'] = True

        # Indicate the end of the everest deactivation process
        self.FLAG_DICT['DEACTIVATING_EVEREST_FLAG'] = False


    # ========================================================================
    # INTERRUPT SERVICE ROUTINES (ISRs)
    # ------------------------------------------------------------------------
    # ========================================================================

    @micropython.native
    def isr_joint_controller(self, timer):
        """
        ISR for joint controller for all leg devices.

        This function handles signal exchange with the servo, calculating variables
        that use these values, and managing servo activation/deactivation for
        hybrid knee, fox knee, and polycentric ankle devices.
        
        Parameters:
            timer (machine.Timer): The timer that triggered the ISR.
            
        Returns:
            None
        """
        self.timing_pin(1)
        tim_0 = time.ticks_us()

        # Cache the PARAM_DICT and FLAG_DICT at the start of the function
        PARAM_DICT = self.PARAM_DICT
        FLAG_DICT = self.FLAG_DICT

        try:
            # -----------------------------------------------------------------
            #   Operation of servo in active state
            # -----------------------------------------------------------------
            if(
                not FLAG_DICT['ACTIVATING_EVEREST_FLAG']
                and not FLAG_DICT['DEACTIVATING_EVEREST_FLAG']
                and not FLAG_DICT['STO_ALARM_ENABLED']
            ):
                self.servo.send_receive_cyclic()

                # Read in the cyclic channel values, do any signal conditioning
                # necessary, and store them for subsequent use.

                # -------------------------------------------------------------
                #   JOINT POSITION (common to all devices)
                # -------------------------------------------------------------
                temp_int = self.servo.channel_poci.actual_position
                
                # Handle rollunder for Hybrid knee, Fox knee and Polycentric ankle
                res = self.CONST_DICT['JOINT_ENC_RESOLUTION']
                if (temp_int >> (res - 1)) & 1:     # handle rollunder
                    temp_int = temp_int - (1 << res)
                
                PARAM_DICT['MAIN_JOINT_ENCODER_VALUE'][0] = temp_int

                # -------------------------------------------------------------
                #   JOINT ANGLE CALCULATION (device-specific)
                # -------------------------------------------------------------
                if (
                    self.device_parameters.device == self.config.HYBRID_KNEE
                    or self.device_parameters.device == self.config.FOX_KNEE
                ):
                    # Direct calculation for knee devices
                    PARAM_DICT['JOINT_ANGLE'][0] = (
                        PARAM_DICT['MAIN_JOINT_ENCODER_VALUE'][0]
                        * self.CONST_DICT['KNEE_JOINT_ENC_COEF']
                    )
                elif (
                    self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
                    or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
                ):
                    # Polynomial calculation for ankle devices
                    self.assembler_functions.calculate_polynomial(
                        PARAM_DICT['JOINT_ANGLE'], 
                        PARAM_DICT['MAIN_JOINT_ENCODER_VALUE'], 
                        self.CONST_DICT['P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS'], 
                        len(self.CONST_DICT['P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS'])
                    )

                # -------------------------------------------------------------
                #   VELOCITY HANDLING (device-specific)
                # -------------------------------------------------------------
                if self.device_parameters.device == self.config.FOX_KNEE:
                    # Fox knee directly assigns to joint speed array
                    PARAM_DICT['JOINT_SPEED_ARRAY'][0] = (
                        self.servo.channel_poci.actual_velocity * 360.0
                    )  # rev/s -> deg/s
                else:
                    # Hybrid knee and polycentric ankle use motor_velocity object
                    self.motor_velocity.val = (
                        self.servo.channel_poci.actual_velocity * 360.0
                    )  # rev/s -> deg/s

                # -------------------------------------------------------------
                #   REFERENCE CURRENT (common to all devices)
                # -------------------------------------------------------------
                PARAM_DICT['LARGE_MOTOR_REFERENCE_CURRENT'][0] = (
                    self.servo.channel_poci.current_quadrature_value
                )

                # -------------------------------------------------------------
                #   STATUS WORD (common to all devices)
                # -------------------------------------------------------------
                PARAM_DICT['EVEREST_STATUS_WORD'] = (
                    self.servo.channel_poci.status_word
                )

                # -------------------------------------------------------------
                #   LAST ERROR (common to all devices)
                # -------------------------------------------------------------
                PARAM_DICT['EVEREST_LAST_ERROR'] = (
                    self.servo.channel_poci.last_error
                )

                # -------------------------------------------------------------
                #   BUS VOLTAGE (common to all devices)
                # -------------------------------------------------------------
                PARAM_DICT['BATTERY_VOLTS'][0] = (
                    self.servo.channel_poci.bus_voltage_value
                )

                # -------------------------------------------------------------
                #   MOTOR TEMPERATURE (Fox knee only)
                # -------------------------------------------------------------
                if self.device_parameters.device == self.config.FOX_KNEE:
                    PARAM_DICT['MOTOR_TEMP'][0] = (
                        self.servo.channel_poci.motor_temperature_value
                    )

                # -------------------------------------------------------------
                #   FTS FORCE/MOMENT (Fox knee only, when FTS is present)
                # -------------------------------------------------------------
                if self.device_parameters.device == self.config.FOX_KNEE:
                    if hasattr(self, 'fts_sensor'):
                        self.fts_sensor.get_force_moment()
                        PARAM_DICT['FTS_FZ'][0] = self.fts_sensor.fz
                        PARAM_DICT['FTS_MY'][0] = self.fts_sensor.my
                        if abs(PARAM_DICT['FTS_FZ'][0]) > 500:
                            PARAM_DICT['FTS_FZ_SPIKE'][0] = self.fts_sensor.fz
                            PARAM_DICT['FTS_MY_SPIKE'][0] = self.fts_sensor.my
                            PARAM_DICT['FTS_FZ_RAW_SPIKE'][0] = self.fts_sensor.ch1_voltage
                            PARAM_DICT['FTS_MY_RAW_SPIKE'][0] = self.fts_sensor.ch2_voltage
                            PARAM_DICT['FTS_CH1_RAW_SPIKE'][0] = self.fts_sensor.ch1_raw[0]
                            PARAM_DICT['FTS_CH2_RAW_SPIKE'][0] = self.fts_sensor.ch2_raw[0]
                            FLAG_DICT['FTS_SPIKE_FLAG'] = True
                            # print('fts spike!!!!!!!!!!!!!!!!')

                # -------------------------------------------------------------
                #   CALCULATE TRANSMISSION RATIO (device-specific)
                # -------------------------------------------------------------
                if self.device_parameters.device == self.config.HYBRID_KNEE:
                    PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'][0] = (
                        self.calculate_hybrid_knee_transmission(
                            PARAM_DICT['JOINT_ANGLE'][0]
                        )
                    )
                elif self.device_parameters.device == self.config.FOX_KNEE:
                    PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'][0] = (
                        self.calculate_fox_knee_transmission(
                            PARAM_DICT['JOINT_ANGLE'][0]
                        )
                    )
                elif (
                    self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
                    or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
                ):
                    self.assembler_functions.calculate_polynomial(
                        PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'], 
                        PARAM_DICT['JOINT_ANGLE'], 
                        self.CONST_DICT['P2_TRANSMISSION_POLYNOMIAL_COEFFICIENTS'], 
                        len(self.CONST_DICT['P2_TRANSMISSION_POLYNOMIAL_COEFFICIENTS'])
                    )

                # -------------------------------------------------------------
                #   CALCULATE JOINT SPEED (device-specific)
                #   (Fox knee joint speed is calculated in velocity section)
                # -------------------------------------------------------------
                if self.device_parameters.device != self.config.FOX_KNEE:
                    self.assembler_functions.compute_joint_velocity(
                        PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'],
                        self.motor_velocity.adr,
                        PARAM_DICT['JOINT_SPEED_ARRAY']
                    )
                
                # -------------------------------------------------------------
                #   UPDATE CONTROL GAINS and SEND CONTROL VALUES TO SERVO
                #   (Polycentric ankle has this section commented out)
                # -------------------------------------------------------------
                if (
                    self.device_parameters.device == self.config.HYBRID_KNEE
                    or self.device_parameters.device == self.config.FOX_KNEE
                ):
                    # Polycentric ankle has this section commented out.
                    # We can remove the if statement altogether if we decide to
                    # implement control parameters for the polycentric ankle.
                    self.update_control_parameters()

                    channel_pico = self.servo.channel_pico
                    channel_pico.position_loop_kp = PARAM_DICT['EVEREST_KPP']
                    channel_pico.velocity_loop_kp = PARAM_DICT['EVEREST_KPV']
                    channel_pico.torque_loop_kp = PARAM_DICT['EVEREST_KPT']
                    channel_pico.position_set_point = PARAM_DICT['EVEREST_SET_POINT']
                    channel_pico.torque_loop_input_offset = PARAM_DICT['EVEREST_TORQUE_BIAS']


                self.servo.send_receive_cyclic()

                # Clear exception
                self.error_flag.clear('servo_cyclic_exception')

            # -----------------------------------------------------------------
            #   Everest activation
            # -----------------------------------------------------------------
            elif FLAG_DICT['ACTIVATING_EVEREST_FLAG']:
                self.activate_everest()

            # -----------------------------------------------------------------
            #   Everest deactivation
            # -----------------------------------------------------------------
            elif FLAG_DICT['DEACTIVATING_EVEREST_FLAG']:
                self.deactivate_everest()

        except Exception as e:
            self.error_flag.activate('servo_cyclic_exception')

        # ---------------------------------------------------------------------
        # Impedance control calculation (Common to all devices)
        # ---------------------------------------------------------------------
        self.everest_pb.controller_call(
            PARAM_DICT['JOINT_ANGLE'],
            PARAM_DICT['JOINT_SPEED_ARRAY']
        )

        # ---------------------------------------------------------------------
        # Calculate current command (Common to all devices)
        # ---------------------------------------------------------------------
        PARAM_DICT['LARGE_MOTOR_CURRENT_COMMAND'][0] = (
            self.everest_pb.output_array[0]
            / PARAM_DICT['LARGE_MOTOR_TRANSMISSION_RATIO'][0]
            * self.device_parameters.ROTATION_DIRECTION
        )

        # ---------------------------------------------------------------------
        # Driving the large motor (Polycentric Ankle only)
        # ---------------------------------------------------------------------
        if (
            self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
            or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
        ):
            if FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG']:
                self.servo.channel_pico.current_quadrature_set_point = (
                    PARAM_DICT['LARGE_MOTOR_CURRENT_COMMAND'][0]
                )
            else:
                self.servo.channel_pico.current_quadrature_set_point = 0

            # Adjust the neutral position of the ankle before it is sent to CAPS
            PARAM_DICT['JOINT_ANGLE'][0] = (
                PARAM_DICT['JOINT_ANGLE'][0] - PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0]
            )

        # ---------------------------------------------------------------------
        #   Dynamic braking calculations for free swing
        #   (Hybrid knee and Fox knee only)
        # ---------------------------------------------------------------------
        if (
            self.device_parameters.device == self.config.HYBRID_KNEE
            or self.device_parameters.device == self.config.FOX_KNEE
        ):
            if self.components.BrakingCircuit.is_enabled():
                # If there is a dynamic brake percentage from CAPS, and
                # the motor brake flag is off, braking should be applied
                if (
                    not FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG']
                    and PARAM_DICT['BRAKE_PERCENTAGE'][0] > 0
                ):
                    FLAG_DICT['DYNAMIC_BRAKE_STATE'] = True
                    if (
                        # Brake equation only non-zero if joint angle is less than
                        # the brake angle specified by CAPS
                        PARAM_DICT['JOINT_ANGLE'][0] < PARAM_DICT['BRAKE_ANGLE'][0]
                        # Brake equation only non-zero if the brake velocity value
                        # from CAPS is positive. A zero value will lead to an
                        # undefined value
                        and PARAM_DICT['BRAKE_RATE'][0] > 0
                    ):
                        PARAM_DICT['DYNAMIC_BRAKE_PERCENTAGE'][0] = (
                            200 + (-200 / (
                                1 + math.exp(
                                    -PARAM_DICT['JOINT_ANGLE'][0]
                                    / PARAM_DICT['BRAKE_RATE'][0]
                            )))
                        )
                    else:
                        # By setting this value to zero, it ensures that it will be
                        # less than the brake percentage from CAPS
                        PARAM_DICT['DYNAMIC_BRAKE_PERCENTAGE'][0] = 0
                else:
                    PARAM_DICT['DYNAMIC_BRAKE_PERCENTAGE'][0] = 0
                    FLAG_DICT['DYNAMIC_BRAKE_STATE'] = False

        # Timing and cleanup
        tim_1 = time.ticks_us()
        PARAM_DICT['CONTROLLER_TIMER'] = time.ticks_diff(tim_1, tim_0)      
        self.timing_pin(0)

    def tick_imu(self, timer):
        """
        ISR for IMU communication
        """
        self.timing_pin(1)

        try:
            tim_0 = time.ticks_us()
            self.assembler_functions.imu_isr(self.imu_isr_addresses)
            
            # Calculate barometric pressure from raw pressure array
            raw_pressure = (
                (self.imu.raw_pressure_array[1] << 16) + 
                self.imu.raw_pressure_array[0]
            )
            self.PARAM_DICT['BAR_BYTE_VALS'][1] = self.imu.q_to_float_32(
                raw_pressure, 20
            )
            
            # Calculate altitude from barometric pressure
            pressure = self.PARAM_DICT['BAR_BYTE_VALS'][1]
            self.PARAM_DICT['ALTITUDE'] = (
                (0.0044 * pressure * pressure) - 
                (17.175 * pressure) + 
                12877.0
            )

            # Send IMU CAN messages
            if self.FLAG_DICT['CAN_RESTART_FLAG']:
                self.can_out_mes_imu1.seq_num_restart_flag = True
                self.can_out_mes_imu2.seq_num_restart_flag = True
            self.can_out_mes_imu1.send()
            self.can_out_mes_imu2.send()

            tim_1 = time.ticks_us()
            imu_timer = time.ticks_diff(tim_1, tim_0)
            if imu_timer > self.PARAM_DICT['MAX_IMU_TIMER']:
                self.PARAM_DICT['MAX_IMU_TIMER'] = imu_timer

            self.error_flag.clear('tick_imu_exception')

        except Exception as e:
            self.error_flag.activate('tick_imu_exception')

        self.timing_pin(0)

    def tick_caps(self, timer):
        """
        ISR for CAPS communication for all leg devices.
        """
        self.timing_pin(1)
        tim_0 = time.ticks_us()

        try:

            self.can.info(self.PARAM_DICT['CAN_INFO_HOLDER'])
            if (
                self.PARAM_DICT['CAN_INFO_HOLDER'][4] > 
                self.PARAM_DICT['CAN_BUS_OFF_COUNTER']
            ):
                self.FLAG_DICT['CAN_RESTART_FLAG'] = True
                self.PARAM_DICT['CAN_BUS_OFF_COUNTER'] = (
                    self.PARAM_DICT['CAN_INFO_HOLDER'][4]
                )
                if self.PARAM_DICT['CAN_BUS_OFF_COUNTER'] >= 100:
                    self.error_flag.activate('can_bus_repeatedly_failed')
            else :
                self.FLAG_DICT['CAN_RESTART_FLAG'] = False


            if (
                self.device_parameters.device == self.config.HYBRID_KNEE
                or self.device_parameters.device == self.config.FOX_KNEE
            ):
                if not self.PARAM_DICT['WIFI_COUNTER'] % 2:
                    if self.FLAG_DICT['CAN_RESTART_FLAG']:
                        self.can_out_mes1.seq_num_restart_flag = True

                    # Send CAN message associated with the hybrid knee angle,
                    # velocity and current
                    self.can_out_mes1.send()

                if not self.PARAM_DICT['WIFI_COUNTER'] % 2:
                    if self.FLAG_DICT['CAN_RESTART_FLAG']:
                        self.can_out_mes3.seq_num_restart_flag = True
                    # Send CAN message associated with the hybrid knee
                    # braking percentage
                    self.can_out_mes3.send()

                    # Send FTS data if streaming is active
                    if self.fts_output_enabled and self.FLAG_DICT['FTS_STREAM_FLAG']:
                        if self.FLAG_DICT['CAN_RESTART_FLAG']:
                            self.can_out_fts_data1.seq_num_restart_flag = True
                            self.can_out_fts_data2.seq_num_restart_flag = True
                        self.can_out_fts_data1.send()
                        self.can_out_fts_data2.send()

                if self.PARAM_DICT['WIFI_COUNTER'] % 2:

                    if self.sm_loaded:
                        # active_state_value = sm.active_state[0]
                        self.PARAM_DICT['ACTIVE_STATE'][0] = float(
                            self.sm.active_state[0]
                        )

                    # The timer flag is set to True in main if the device is a
                    # hybrid knee, it's streaming and aps_communication_check_counter
                    # is within a range
                    if self.FLAG_DICT['CAPS_TIMER_FLAG']:
                        # If the caps_communication_value is 0, change it to 1000
                        # and save the time it happens as self.PARAM_DICT['CAPST1']
                        if (
                            self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] == 0
                            and not self.FLAG_DICT['CAPS_REPLY_FLAG']
                        ):
                            # Any value above 0 would work, 1000 chosen to make
                            # viewable on signal viewer - This is for the initial
                            # value transition
                            self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] = 1000
                            # Saving the time when the
                            # self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] is
                            # changed to 1000
                            self.PARAM_DICT['CAPST1'] = time.ticks_us()

                        # If a reply has been received from CAPS change the
                        # self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0]
                        if self.FLAG_DICT['CAPS_REPLY_FLAG']:
                            # Evaluate the time between when a signal was sent to
                            # CAPS and when a reply was received from CAPS
                            self.PARAM_DICT['CAPS_TICK_TIME'] = time.ticks_diff(
                                self.PARAM_DICT['CAPST2'],
                                self.PARAM_DICT['CAPST1']
                            )
                            self.PARAM_DICT['CAPS_COMM_CHECK_COUNTER'] = (
                                self.PARAM_DICT['CAPS_COMM_CHECK_COUNTER'] + 1
                            )
                            # Print the time between the ticks
                            print('Comm time', self.PARAM_DICT['CAPS_TICK_TIME'])
                            # If the response time is greater than 350,000us turn on
                            # the buzzer, this is done in main.py
                            if abs(self.PARAM_DICT['CAPS_TICK_TIME']) > 350000:
                                # Flag used to indicate that CAPS communication
                                # is slow
                                self.FLAG_DICT['CAPS_SLOW_PING_FLAG'] = True
                                # Print the values related to the slow communication
                                print(
                                    'Slow CAPS Communication!!!!!!!!!!',
                                    self.PARAM_DICT['CAPST2'],
                                    self.PARAM_DICT['CAPST1']
                                )
                            # If CAPS has replied, flip the communication value to
                            # cause another response from CAPS
                            if self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] == 1000:
                                # Set the communication value to 2000 if it is
                                # currently 1000
                                self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] = 2000
                            else:
                                # Set the communication value to 1000 if it is
                                # currently 2000
                                self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] = 1000
                            # Saving the time when the
                            # self.PARAM_DICT['CAPS_COMMUNICATION_VALUE'][0] is changed
                            self.PARAM_DICT['CAPST1'] = time.ticks_us()
                            # Ensures the value saved in CAPS12.data is reset
                            self.CAPS12.zero()
                            # Reset the self.FLAG_DICT['CAPS_REPLY_FLAG']
                            self.FLAG_DICT['CAPS_REPLY_FLAG'] = False

                    if self.error_flag.error_code_to_CAPS():
                        self.can_send.cpy_mess_into_can_send(
                            self.error_flag.ec_can,
                            self.error_flag.ec_NID
                        )
                    if self.FLAG_DICT['CAN_RESTART_FLAG']:
                        self.can_out_mes2.seq_num_restart_flag = True
                    # Sending the CAN message
                    self.can_out_mes2.send()

            if (
                self.device_parameters.device == self.config.POLYCENTRIC_ANKLE
                or self.device_parameters.device == self.config.POLYCENTRIC_ANKLE_ONLY
            ):
                if not self.PARAM_DICT['WIFI_COUNTER'] % 2:
                    if self.FLAG_DICT['CAN_RESTART_FLAG']:
                        self.can_out_mes5.seq_num_restart_flag = True
                    self.can_out_mes5.send()

                if self.PARAM_DICT['WIFI_COUNTER'] % 2:
                    if self.error_flag.error_code_to_CAPS():
                        self.can_send.cpy_mess_into_can_send(
                            self.error_flag.ec_can,
                            self.error_flag.ec_NID
                        )
                    # Empty can message
                    # self.can_out_mes6.send()

            self.PARAM_DICT['WIFI_COUNTER'] = (
                (self.PARAM_DICT['WIFI_COUNTER'] + 1) % 1000
            )

            self.error_flag.clear('tick_caps_exception')

        except Exception as e:
            self.error_flag.activate('tick_caps_exception')

        tim_1 = time.ticks_us()
        self.PARAM_DICT['CAPS_COM_TIMER'] = time.ticks_diff(tim_1, tim_0)
        self.timing_pin(0)

    # ========================================================================
    # COMMUNICATION AND DATA HANDLING
    # ------------------------------------------------------------------------
    # ========================================================================

    # Start adding some structure to BLE parser
    # Lets make the message[0]=0x01 mean a CAN message to be initiated
    def parse_ble_message(self, message_value):
        if message_value[0] == 0x01:
            if message_value[1] == 0x00:                 # Zero the load cell and tare the IMU
                print('Zeroing loadcell via Bluetooth')
                zero_load = bytearray(1)
                zero_load[0] = 0x52
                self.can.send(zero_load[0], 0x140)
            elif message_value[1] == 0x01:               # Reset the wifi
                print('Resetting wifi via Bluetooth')
                reset_network = bytearray(6)
                reset_network[0] = 0x04
                reset_network[1] = 0x7C
                reset_network[2] = 0xFE
                reset_network[3] = 0x01
                reset_network[4] = 0x02
                reset_network[5] = 0x01
                self.can.send(reset_network, 0x1F0)
            elif message_value[1] == 0x02:               # Home CVT
                print('Homing CVT via Bluetooth')
                self.FLAG_DICT['HOME_CVT_FLAG'] = True
            elif message_value[1] == 0x03:               # Rehome Encoder
                print('Rehoming encoder via Bluetooth')
                self.FLAG_DICT['REHOME_MAIN_JOINT_ENCODER_FLAG'] = True
            elif message_value[1] == 0x04:               # Rehome Encoder
                print('Zero IMU')
                self.FLAG_DICT['ZERO_IMU_FLAG'] = True
        elif message_value[0] == 0x02:                   # This will indicat the tab we are on.
            if message_value[1] == 0x01:                 # Zero the load cell and tare the IMU
                self.app_tab = 1
            elif message_value[1] == 0x02:                 # Zero the load cell and tare the IMU
                self.app_tab = 2
            elif message_value[1] == 0x03:                 # Zero the load cell and tare the IMU
                self.app_tab = 3
            elif message_value[1] == 0x04:                 # Zero the load cell and tare the IMU
                self.app_tab = 4
