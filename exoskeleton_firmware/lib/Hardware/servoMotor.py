import pyb                                          # type: ignore

from micropython import const                       # type: ignore
from lib.ll_common.const_dict import CONST_DICT as const_dict
from lib.ll_common.novanta_constants import NovantaConstants as NOV
from lib.ll_common import upy_spi
from lib.ll_common import nvic
from lib.ll_common import novanta

from lib import config
from lib.ll_common import mode_switch


### const for Servo
BOARD_EVEREST_BREAKOUT = 0
BOARD_LL_CONTROL_BOARD_VB_1_1 = 1   # Pyboard SF6
BOARD_PORTENTA_ADAPTER = 2          # Portenta H7 adapter for VB_1_1
BOARD_LL_CONTROL_BOARD_VB_2_0 = 3   # Portenta H7
BOARD_LL_CONTROL_BOARD_VB_3_0 = 4   # Portenta H7 Fox Knee

DEVICE_HYBRID_KNEE = 0
DEVICE_FOX_KNEE = 1
DEVICE_CUBE_MARS_BENCHTOP = 2

class Servo: # servo constructor/decorator
    def __init__(self, _CONTROLLER_BOARD,
                 _DEVICE_TYPE, CURRENT_LIMIT):

        # -----------------------------------------------------------------------------
        #   DEFINE BOARD-SPECIFIC CONSTANTS
        # -----------------------------------------------------------------------------
        # TODO: All this can be cleaned by using different subclasses
        if _CONTROLLER_BOARD == BOARD_EVEREST_BREAKOUT:
            print('Using Everest breakout board...')
            _NOVANTA_SPI_BUS = 1
            _NOVANTA_SPI_PRESCALER = 8
            # Everest max is 50 MHz
            # _NOVANTA_SPI_BAUDRATE       = 50000000

            _NOVANTA_SPI_SELECT_PIN = 'W7'  # SPI1-SS1
            _NOVANTA_INTERRUPT_PIN = 'W33'

            _NOVANTA_BOOT_SYNC1_PIN = 'W34'
            _NOVANTA_RESET_PIN = 'W25'

        #   For control board vB_1_1:
        elif _CONTROLLER_BOARD == BOARD_LL_CONTROL_BOARD_VB_1_1:
            print('Using ll-control-board vB_1_1...')
            _NOVANTA_SPI_BUS = 1
            _NOVANTA_SPI_PRESCALER = 4
            # Everest max is 50 MHz
            _NOVANTA_SPI_BAUDRATE = 50000000

            _NOVANTA_SPI_SELECT_PIN = 'W7'  # SPI1-SS1
            _NOVANTA_INTERRUPT_PIN = 'W24'

            _NOVANTA_BOOT_SYNC1_PIN = 'W11'
            _NOVANTA_RESET_PIN = 'W22'

        #   For Portenta adapter board:
        elif _CONTROLLER_BOARD == BOARD_PORTENTA_ADAPTER:
            print('Using Portenta adapter board...')
            _NOVANTA_SPI_BUS = 2
            # _NOVANTA_SPI_PRESCALER      = 4
            # Everest max is 50 MHz
            _NOVANTA_SPI_BAUDRATE = 50000000

            _NOVANTA_SPI_SELECT_PIN = 'I0'  # SPI1-SS1
            _NOVANTA_INTERRUPT_PIN = 'E3'

            _NOVANTA_BOOT_SYNC1_PIN = 'A4'
            _NOVANTA_RESET_PIN = 'D5'

        #   For control board vB_2_0:
        elif _CONTROLLER_BOARD == BOARD_LL_CONTROL_BOARD_VB_2_0:
            print('Using ll-control-board vB_2_0...')
            _NOVANTA_SPI_BUS = 2
            # Everest max is 50 MHz. 50 MHz is prescaler of 8 for Portenta H7
            _NOVANTA_SPI_BAUDRATE = 50000000
            # _NOVANTA_SPI_PRESCALER      = 8

            _NOVANTA_SPI_SELECT_PIN = 'J2_36'  # SPI1-SS1 I0
            _NOVANTA_INTERRUPT_PIN = 'J2_54'  # E3

            _NOVANTA_BOOT_SYNC1_PIN = 'J2_22'  # A4
            _NOVANTA_RESET_PIN = 'J2_52'

        #   For control board vB_3_0:
        elif _CONTROLLER_BOARD == BOARD_LL_CONTROL_BOARD_VB_3_0:
            print('Using ll-control-board vB_3_0...')
            _NOVANTA_SPI_BUS = 2
            # Everest max is 50 MHz. 50 MHz is prescaler of 8 for Portenta H7
            _NOVANTA_SPI_BAUDRATE = 50000000
            # _NOVANTA_SPI_PRESCALER      = 8

            _NOVANTA_SPI_SELECT_PIN = 'J2_36'  # SPI1-SS1 I0
            _NOVANTA_INTERRUPT_PIN = 'J2_54'  # E3

            _NOVANTA_BOOT_SYNC1_PIN = 'J2_22'  # A4
            _NOVANTA_RESET_PIN = 'J2_52'

        # Set STO lines to same state for hybrid knee
        if _CONTROLLER_BOARD == BOARD_LL_CONTROL_BOARD_VB_1_1:
            sto_1 = pyb.Pin('W49', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)
            dbb_enable = pyb.Pin('W54', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)

            sto_1.value(1)
            dbb_enable.value(0)

        elif _CONTROLLER_BOARD == BOARD_PORTENTA_ADAPTER:
            sto_1 = pyb.Pin('H6', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)
            dbb_enable = pyb.Pin('G7', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)

            sto_1.value(1)
            dbb_enable.value(0)

        elif _CONTROLLER_BOARD == BOARD_LL_CONTROL_BOARD_VB_2_0:
            sto_1 = pyb.Pin('J2_68', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)
            dbb_enable = pyb.Pin('J2_65', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_UP)

            sto_1.value(1)
            dbb_enable.value(0)

        #
        #   INITIALIZE SPI
        #

        self.spi = upy_spi.upySPI(
        # self.spi = pyb.SPI(
            _NOVANTA_SPI_BUS
        )
        self.spi.init(
            mode=pyb.SPI.CONTROLLER,
            baudrate=_NOVANTA_SPI_BAUDRATE,
            # prescaler=_NOVANTA_SPI_PRESCALER,
            polarity=0,
            phase=1,
            bits=8,
            #   crc=0x1021,
            firstbit=pyb.SPI.MSB
        )
        print(self.spi)

        #
        #   INITIALIZE EVEREST
        #
        config_dict = {
            NOV.POSITION_LOOP_KP: 0.0,                          # Control parameter
            NOV.POSITION_LOOP_KI: 0.0,
            NOV.POSITION_LOOP_KD: 0.0,
            NOV.POSITION_LOOP_KFFA: 0.0,
            NOV.POSITION_LOOP_KFFV: 0.0,
            NOV.POSITION_LOOP_KD_FILTER: 0.0,
            NOV.POSITION_FOLLOWING_ERROR_OPTION_CODE: 1,
            NOV.POSITION_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
            NOV.POSITION_LOOP_MIN_OUTPUT: -200.0,             # Default is -200.0
            NOV.POSITION_LOOP_MAX_OUTPUT: 200.0,              # Default is 200.0

            NOV.VELOCITY_LOOP_KP: 0.0,                          # Control parameter
            NOV.VELOCITY_LOOP_KI: 0.0,
            NOV.VELOCITY_LOOP_KD: 0.0,
            NOV.VELOCITY_LOOP_KFFA: 0.0,
            NOV.VELOCITY_LOOP_KD_FILTER: 0.0,
            NOV.VELOCITY_FOLLOWING_ERROR_OPTION_CODE: 1,
            NOV.VELOCITY_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
            NOV.VELOCITY_LOOP_MIN_OUTPUT: -200.0,
            NOV.VELOCITY_LOOP_MAX_OUTPUT: 200.0,

            NOV.TORQUE_LOOP_KP: 0.0,                            # Control parameter
            NOV.TORQUE_LOOP_KI: 0.0,

            # Controls the frequency of the power stage, as well as position
            # and velocity control loops. The latter may be read via
            # NOV.POSITION_AND_VELOCITY_CONTROL_LOOP_RATE, which work out as:
            # 0 = 10 kHz
            # 1 = 20 kHz
            # 2 = 25 kHz
            NOV.POWER_STAGE_FREQUENCY: 1,

            # NOV.CONTROL_LOOPS_FEEDBACK_OPTIONS: 19,        # normally 27. vel feedback disabled = 19

            NOV.OPERATION_MODE: NOV.OPERATION_MODE_POSITION,
            # NOV.OPERATION_MODE: NOV.PROFILER_CYCLIC_SYNCHRONOUS_POSITION,
            # NOV.INTERPOLATION_TIME_MANTISSA: 3.0,
            # NOV.INTERPOLATION_TIME_EXPONENT: 1.0,

            NOV.POSITIONING_OPTION_CODE: 3 << 6,            # 0 is default, 3 is shortest to target
            # NOV.POSITIONING_OPTION_CODE: 0,

            #
            #   SET ERROR OPTION CODE FOR STO
            #
            # This allows for the servo to tolerate the sto lines toggling off
            # when in the SWITCHED_ON or OPERATION_ENABLED state. If STO is
            # toggled low in these modes, the servo will go to the FAULT state.
            # However, by setting this register to 1, performing transition #15
            # will work to recover from the fault state. If this option code is
            # left at it's default value of 0, recovering from this type of
            # fault requires power-cycling the servo.
            NOV.STO_ERROR_OPTION_CODE: 1,

            #
            #   SET FEEDBACK RUNAWAY ERROR OPTION CODE
            #
            #   This masks faults arising from missed encoder or hall-effect
            #   counts. Uncomment the next line line if this error begins
            #   occurring due to excessive signal noise or motor speed. Default
            #   is 0
            # NOV.FEEDBACK_RUNAWAY_ERROR_OPTION_CODE: 1,            # Default is 0

            #
            #   LIMIT CURRENT COMMANDS WHEN DEBUGGING
            #
            NOV.CURRENT_QUADRATURE_LOOP_MIN_OUT: -1.0 * CURRENT_LIMIT,
            NOV.CURRENT_QUADRATURE_LOOP_MAX_OUT: CURRENT_LIMIT,
            NOV.MAX_CURRENT: CURRENT_LIMIT,

            #
            #   I2T CURRENT PROTECTION
            #

            # 0=disable power stage, 1=do nothing, 2=slow down ramp, 3=quick stop ramp
            NOV.USER_I2T_ERROR_OPTION_CODE: 1,
            NOV.PEAK_CURRENT: CURRENT_LIMIT,  # Amps
            NOV.PEAK_CURRENT_TIME: 1_000_000,  # ms
        }

        # ------------------------------------------------------------------
        #
        #   FOX KNEE SPECIFIC CONFIGURATION
        #
        # ------------------------------------------------------------------
        if _DEVICE_TYPE == DEVICE_CUBE_MARS_BENCHTOP:
            config_dict.update({
                NOV.POSITION_FEEDBACK_SENSOR: NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_POLARITY: 1, # TODO check current direction
                NOV.MIN_POSITION_RANGE_LIMIT: 0,
                NOV.MAX_POSITION_RANGE_LIMIT: 2 ** 12,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_PROTOCOL: 0x0,  # 0 = Biss-C # default 1 --> SSI
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_SIZE: 25,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_BAUDRATE: 1000,  # kHz
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_POLYNOMIAL: 0x43,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_NUMBER_OF_BITS: 6,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_SEED: 0x0,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_BITS: 12,
                NOV.PRIMARY_ABSOLUTE_SLAVE_1_SINGLE_TURN_BITS: 12,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_START_BIT: 1,
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_TYPE: 0,  # 0=raw, 3=BiSS-C BP3, 4=BiSS-C BP3 gray
                # NOV.PRIMARY_ABSOLUTE_SLAVE_1_ERROR_TOLERANCE: 0,  # Errors are ignored
                NOV.POSITION_WINDOW: 13,  # counts (default is 0)
                NOV.POSITION_WINDOW_TIME: 0,  # ms (int) - default 0

                NOV.VELOCITY_FEEDBACK_SENSOR: NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1,
                NOV.VELOCITY_THRESHOLD: 1.0 / 360.0,  # rev/s default=1.0
                NOV.VELOCITY_THRESHOLD_TIME: 0,

                NOV.VELOCITY_FEEDBACK_FILTER_1_TYPE: 1,  # Configured 1
                NOV.VELOCITY_FEEDBACK_FILTER_1_FREQUENCY: 200,  # Configured 200
                NOV.VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR: 0.7071,
                NOV.VELOCITY_FEEDBACK_FILTER_1_GAIN: 0.0,

                NOV.VELOCITY_REFERENCE_FILTER_1_TYPE: 1,
                NOV.VELOCITY_REFERENCE_FILTER_1_FREQUENCY: 200,
                NOV.VELOCITY_REFERENCE_FILTER_1_Q_FACTOR: 0.7071,
                NOV.VELOCITY_REFERENCE_FILTER_1_GAIN: 0.0,

                NOV.AUXILIAR_FEEDBACK_SENSOR: NOV.FEEDBACK_INCREMENTAL_ENCODER_1,
                NOV.INCREMENTAL_ENCODER_1_RESOLUTION: 4096,
            })
            cyclic_poci_registers=[ # ATTENTION: Max number of registers supported is 15
                NOV.ACTUAL_VELOCITY,                                # (4 bytes)
                NOV.ACTUAL_POSITION,
                NOV.CURRENT_QUADRATURE_A_CONTROL_LOOP_COMMAND,      # (4 bytes)
                NOV.CURRENT_QUADRATURE_VALUE,                       # (4 bytes)
                NOV.MOTOR_TEMPERATURE_VALUE,    # motor temperature (float, C)
                # NOV.STATUS_WORD,                                    # (2 bytes)
                # NOV.LAST_ERROR,                                     # (4 bytes)
                # NOV.POSITION_CONTROL_LOOP_ERROR,                    # (4 bytes)
                NOV.POSITION_LOOP_CONTROL_COMMAND,
                # NOV.VELOCITY_LOOP_CONTROL_COMMAND,
                # NOV.BUS_VOLTAGE_VALUE,                               # (4 bytes)
                NOV.POSITION_DEMAND,
                NOV.TORQUE_LOOP_CONTROL_COMMAND,
                # NOV.INTERPOLATION_DATA_RECORD_STATUS,
                # NOV.COMMUTATION_ANGLE_VALUE,    # doesn't work in cyclic mode
                NOV.INCREMENTAL_ENCODER_1_VALUE,
                NOV.ACTUAL_TORQUE,
            ]
            self.current_name = 'current_quadrature_a_control_loop_command'

            cyclic_pico_registers=[
                NOV.POSITION_SET_POINT,                             # (4 bytes)
                NOV.INTERPOLATION_DATA_RECORD_POSITION_INPUT,       # INT32 [counts] --> different from [encoder counts]
                NOV.INTERPOLATION_DATA_RECORD_VELOCITY_INPUT,       # FLOAT [rev/sec]
                NOV.INTERPOLATION_DATA_RECORD_TIME_INPUT,           # UINT16 [millisec]
                NOV.INTERPOLATION_DATA_RECORD_INTEGRITY_CHECK,      # UINT16
                NOV.POSITION_LOOP_KP,                               # (4 bytes)
                NOV.POSITION_LOOP_KD,
                NOV.VELOCITY_LOOP_KP,                               # (4 bytes)
                NOV.TORQUE_LOOP_KP,                                 # (4 bytes)
                NOV.TORQUE_LOOP_INPUT_OFFSET,                       # (4 bytes)
                # NOV.CURRENT_QUADRATURE_SET_POINT,
                NOV.CONTROL_WORD,
                NOV.OPERATION_MODE
            ]


        self._servo = novanta.Everest(
            self.spi,
            _NOVANTA_RESET_PIN,
            _NOVANTA_BOOT_SYNC1_PIN,
            _NOVANTA_SPI_SELECT_PIN,
            _NOVANTA_INTERRUPT_PIN,
            cyclic_pico_registers=cyclic_pico_registers,
            cyclic_poci_registers=cyclic_poci_registers,
            config_dict=config_dict,
            use_hardware_crc=True,
            # use_hardware_crc=False,
            verbose=True
        )

        self.profiler = "P"

        nvic.nvic_set_prio(self._servo.interrupt_pin.line() + 6, 5-1)

        if self._servo.read_int(NOV.ACTUAL_POSITION) > 2048:
            print('=============================')
            print('WARNING: JOINT ENCODER > 2048')
            print('=============================')

        self.config = config
        self.selector = mode_switch.selector  # defined in boot.py
        self.device_parameters = (
            self.config.DeviceParameters(self.selector.device_id)
        )

        self.timer_servo = pyb.Timer(
            self.device_parameters.TIMER_SERVO['ID'],
            freq=self.device_parameters.TIMER_SERVO['FREQ'],
            callback=self._servo_callback
        )

        self.pos =0
        self.pos_des =0
        self.pos_incremental_encoder =0
        self.vel =0
        self.tau_des= 0
        self.tau_actual = 0
        self.current =0
        self.temperature =0
        self.counter_for_temp =0 # reading temp each instance can be heavy so one for sec
        self.run = False

        self.reference_position = 0

        # computation of the accelleration
        self.prev_vel = 0.0
        self.acc = 0.0
        self.dt = 1 / self.device_parameters.TIMER_SERVO['FREQ']


    @property
    def position(self, static=False):
        if static:
            return self._servo.read_uint(NOV.ACTUAL_POSITION)
        else:
            return self.pos
    @property
    def position_incremental_encoder(self):
        return self.pos_incremental_encoder
    @property
    def position_commanded(self):
        return self.pos_des
    @property
    def torque_commanded(self):
        return self.tau_des
    @property
    def actual_torque(self):
        return self.tau_actual
    @property
    def velocity(self):
        return self.vel
    @property
    def acceleration(self):
        return self.acc
    
    def get_current(self):
        return self.current
    def get_temperature(self):
        return self.temperature

    def get_profiler(self):
        return self.profiler

    def enable(self, active):
        self.zero()
        self.reference_position = self._servo.read_uint(NOV.ACTUAL_POSITION)
        if self.reference_position > 2048:
            print("Current position: ", self.reference_position)
            raise ValueError("Robot out of range: move manually and then restart!")

        self._servo.channel_pico.set_idle()
        self._servo.state_transition(2)
        pyb.delay(100)
        self._servo.state_transition(3)
        pyb.delay(300)

        if active:
            self._servo.state_transition(4)
            pyb.delay(100)

        self._servo.enable_cyclic_mode()
        print('COMMUNICATION MODE =', self._servo.communication_mode)
        self.run = True

    def zero(self):
        self._servo.set_register(
            NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET,
            0
        )
        ## CubeMars is actually Absolute
        # self._servo.set_register(
        #     NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET,
        #     -1 * self._servo.read_uint(NOV.ACTUAL_POSITION)
        # )

    def change_PVT_profile(self):
        """
        Function to change to PVT profile.
        Position velocity time (PVT) behaves in a similar fashion to IP.
        In this mode, the Set-point manager and interpolation buffer allow the user to provide the drive with sets of
        final position, velocity, and movement duration. Interpolation time mantissa and magnitude order will
        not be necessary for this mode because interpolation time will be extracted from the time introduced into the
        buffer of the set-point manager directly. These sets of points are then fed to the set-point manager,
        which will use the PVT profiler to generate 5th-degree polynomial trajectories that reach each position
        set-point at the specified velocity in the given time duration.
        Returns:
            None
        """
        self.profiler = "PVT"

        # clear the buffer
        # TODO: check why stucked
        # try:
        #     self._servo.write_uint(NOV.INTERPOLATION_DATA_RECORD_FORCE_CLEAR, 1)
        #     pyb.delay(10)
        # except Exception as e:
        #     print("Warning: could not clear buffer:", e)
        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6  # check exa
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7  # check exa
        pyb.delay(20)
        #Set the Operation mode to PROFILER_BUFFERED_PVT Profile
        self._servo.channel_pico.operation_mode = NOV.PROFILER_BUFFERED_PVT #0xB4
        pyb.delay(20)
        self._servo.channel_pico.control_word = 15  # check exa
        pyb.delay(20)
        print("Changed OPERATION_MODE to PVT profiler")

    def change_to_Position_profile(self):
        """
        Function to profiler to position profile.
        """
        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7
        pyb.delay(20)
        self._servo.channel_pico.operation_mode = NOV.OPERATION_MODE_POSITION  # Position Profile
        pyb.delay(20)
        self._servo.channel_pico.control_word = 15

        self.profiler = "P"
        print("Drive is now in POSITION_PROFILE mode.")

    def clear_buffer(self):

        self._servo.channel_pico.control_word = 0
        pyb.delay(30)
        self._servo.channel_pico.control_word = 6  # check exa
        pyb.delay(20)
        self._servo.channel_pico.control_word = 7  # check exa
        pyb.delay(20)
        # clear the buffer
        try:
            self._servo.write_uint(NOV.INTERPOLATION_DATA_RECORD_FORCE_CLEAR, 1)
            pyb.delay(10)
        except Exception as e:
            print("Warning: could not clear buffer:", e)

        self._servo.channel_pico.control_word = 15  # check exa
        pyb.delay(20)

    def set_pos_loop_set_point(self, position : int):
        """This is uded only in full position mode.
        Double check what it is the current servo profiler"""
        self.reference_position = position

    def append_execute_buffer(self, angle: int, vel= 0.0, dt = 1) -> None:
        """
        Append element to the buffer and update the control word to execute it
        Args:
            angle(int): The angle in counts.
            vel (float): The velocity in revs/s. This is the arriving velocity. By default is zero.
            dt (float): The time in seconds. For interpolation profiler this is not used.
                        The one used for this mode is the product between the exp and the mantissa one.
        Returns:
            None
        """
        # Write the required input:
        # 0x20C1 - Interpolation data record - Position input
        # 0x20C2 - Interpolation data record - Velocity input
        # 0x20C3 - Interpolation data record - Time input  --> this is not used in the interpolated position mode
        #                                                   and replaced by the one defined by the time mantissa and exponent
        self._servo.channel_pico.interpolation_data_record_position_input = angle
        self._servo.channel_pico.interpolation_data_record_velocity_input = vel
        self._servo.channel_pico.interpolation_data_record_time_input = max(1,  int(dt*1000))
        # Increment the integrity check value by 1 (0x20C4 - Interpolation data record integrity check).
        #  When the drive detects an increment of 1 in the integrity check,
        #  it will save the configured values in the position, velocity, and time inputs.
        self._servo.channel_pico.interpolation_data_record_integrity_check += 1
        pyb.delay(1)
        self._servo.channel_pico.control_word = 31

    def _update_position(self):
        self.pos = self._servo.channel_poci.actual_position
        self.pos_incremental_encoder = self._servo.channel_poci.incremental_encoder_1_value
    def _update_pos_desired(self):
        self.pos_des = self._servo.channel_poci.position_loop_control_command
    def _update_torque_commanded(self):
        self.tau_des = self._servo.channel_poci.torque_loop_control_command
    def _update_actual_torque(self):
        self.tau_actual =self._servo.channel_poci.actual_torque
    def _update_velocity(self):
        self.prev_vel = self.vel
        self.vel = self._servo.channel_poci.actual_velocity
    def _update_acceleration(self):
        prev_acc = self.acc
        curr_acc = (self.vel - self.prev_vel) / self.dt
        alpha =0.05
        self.acc = alpha * curr_acc + (1 - alpha) * prev_acc
    def _update_current_temp(self):
        self.current =  self._servo.channel_poci.current_quadrature_value
        self.counter_for_temp += 1
        if self.counter_for_temp==300:
            self.temperature = self._servo.channel_poci.motor_temperature_value
            self.counter_for_temp =0
    def _set_pos_loop_set_point(self):
        self._servo.channel_pico.position_set_point = self.reference_position


    def _servo_callback(self, _):
        if self.run:
            self._servo.send_receive_cyclic()
            self._update_position()
            self._update_pos_desired()
            self._update_actual_torque()
            self._update_torque_commanded()
            self._update_velocity()
            self._update_acceleration()
            self._update_current_temp()
            self._set_pos_loop_set_point()
            self._servo.send_receive_cyclic()


    def set_tau_offset(self, tau=0.0):
        self._servo.channel_pico.torque_loop_input_offset = tau

    def set_Kpp(self, Kpp):
        # position loop K
        self._servo.channel_pico.position_loop_kp = Kpp
    
    def set_Kpd(self, Kpd):
        # position loop derivative component
        self._servo.channel_pico.position_loop_kd = Kpd

    def set_Kpv(self, Kpv):
        # velocity loop K
        self._servo.channel_pico.velocity_loop_kp = Kpv

    def set_Kpt(self, Kpt):
        # torque loop K
        self._servo.channel_pico.torque_loop_kp = Kpt