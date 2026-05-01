#############################################################################
# @file       const_dict.py
#
# @details    Code that contains all the constants used in main.py and hybrid_leg.py
#
# @author     Frank Ursetta
#############################################################################

#
#   IMPORTS
#
import pyb                                          # type: ignore
import array
from micropython import const                       # type: ignore
import math
import gc

from . import mode_switch
from .novanta_constants import NovantaConstants as NOV

print('IMPORTING CONSTANT DICTIONARY')

# -----------------------------------------------------------------------------
#   Use mode switch to determine what kind of device this is.
# -----------------------------------------------------------------------------
selector = mode_switch.selector     # defined in boot.py

# -----------------------------------------------------------------------------
#
#   CONST_DICT
#
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
#   Universal Constants
# -----------------------------------------------------------------------------
CONST_DICT = {
    'RED_LED'                               : pyb.LED(1),
    'GREEN_LED'                             : pyb.LED(2),
    'BLUE_LED'                              : pyb.LED(3),
    'TAY_COEFS_SIN'                         : array.array('f', [6, 120, 5040, 362880, 0.8726646260, .01745329]),        # Taylor series coefficients for sine calcualation
    'TAY_COEFS_COS'                         : array.array('f', [2, 24, 720, 40320, 0.8726646260, .01745329, 1]),        # Taylor series coefficients for cosine calcualation
    'TAY_COEFS_ATAN'                        : array.array('f', [.3333333, .2, .142857, 57.2958]),                       # Taylor series coefficients for arctan calcualation
    'TAY_COEFS_ASIN'                        : array.array('f', [0.16666667, 0.075, 0.044642857, 57.2958, 1.570796]),    # Taylor series coefficients for arcsine calcualation
    'LOG_TAYLOR_COEFS'                      : array.array('f', [0.0, 0.5, 0.333333333, 0.25, -0.0001185, 1.0]),         # Taylor series coefficients for log calcualation
    'CRANK_HELPER_ARRAY'                    : array.array('f', [20.8675, 10000, 7.33038, 0]),                           # Array values used to calculate the transmission ratio for the hybrid knee [hinge to lead screw distance, scaler, gear ratio x pi, 0]
    'FLOAT_COMPARISON_VALUE'                : 0.001,                                                 # Used as a static value to check for 0 values from CAPS

    # Zero payload messages for CAPS k,b,e:
    'JOINT_ZERO_PAYLOAD_MSG'                : (0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF),  # in an 8-byte message these values are in indices 1 through 6

    # Constants for LED control
    'OFF'                                   : [0, (0, 0, 0)],
    'RED'                                   : [1, (1, 0, 0)],
    'GREEN'                                 : [2, (0, 1, 0)],
    'BLUE'                                  : [3, (0, 0, 1)],
    'YELLOW'                                : [4, (1, 1, 0)],
    'MAGENTA'                               : [5, (1, 0, 1)],
    'TURQUOISE'                             : [6, (0, 1, 1)],
    'WHITE'                                 : [7, (1, 1, 1)],

    'BLE_SLEEP_TIME'                        : 100,
    'SM_JOINT_SCALE_PARAMS'                 : array.array('f', [36, 500, 1]),
    'CAPS_COMM_CHECK_LIMIT'                 : 20,                               # Value that controls the duration of the CAPS comminication is checked
    'ERROR_BUZZ_INTERVAL'                   : 200,                              # Interval time in between buzzing (in ms)}

    # The following register values will be set during Everest initialization
    # in novanta.py. Some of these values are initial values for cyclic
    # registers. They should be read and written using the property methods
    # provided by the Everest class initialization (see comments below), and
    # these entries should not be used as variables in the code.

    'EVEREST_CONFIG_DICT': {
        NOV.POSITION_LOOP_KP: 0.0,                          # Control parameter
        NOV.POSITION_LOOP_KI: 0.0,
        NOV.POSITION_LOOP_KD: 0.0,
        NOV.POSITION_LOOP_KFFA: 0.0,
        NOV.POSITION_LOOP_KFFV: 0.0,
        NOV.POSITION_LOOP_KD_FILTER: 0.0,
        NOV.POSITION_FOLLOWING_ERROR_OPTION_CODE: 1,
        NOV.POSITION_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
        NOV.POSITION_LOOP_MIN_OUTPUT: -200.0,
        NOV.POSITION_LOOP_MAX_OUTPUT: 200.0,

        NOV.VELOCITY_LOOP_KP: 0.0,                          # Control parameter
        NOV.VELOCITY_LOOP_KI: 0.0,
        NOV.VELOCITY_LOOP_KD: 0.0,
        NOV.VELOCITY_LOOP_KFFA: 0.0,
        NOV.VELOCITY_LOOP_KD_FILTER: 0.0,
        NOV.VELOCITY_FOLLOWING_ERROR_OPTION_CODE: 1,
        NOV.VELOCITY_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE: 1,     # Needed?
        
        NOV.VELOCITY_LOOP_MAX_OUTPUT: 200.0,                # Maximum output of the velocity loop in Nm
        NOV.VELOCITY_LOOP_MIN_OUTPUT: -200.0,               # Minimum output of the velocity loop in Nm

        NOV.TORQUE_LOOP_KP: 0.0,                            # Control parameter 
        NOV.TORQUE_LOOP_KI: 0.0,

        # Controls the frequency of the power stage, as well as position
        # and velocity control loops. The latter may be read via
        # NOV.POSITION_AND_VELOCITY_CONTROL_LOOP_RATE, which work out as:
        # 0 = 10 kHz
        # 1 = 20 kHz
        # 2 = 25 kHz
        NOV.POWER_STAGE_FREQUENCY: 1,

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
    }
}

# -----------------------------------------------------------------------------
#   Constants for the Hybrid Knee
# -----------------------------------------------------------------------------
if selector.device_name == 'HYBRID KNEE':

    #
    #   CONFIGURE KINEMATIC GEOMETRY, ENCODER RESOLUTIONS, AND MOTOR CONSTANTS
    #

    CONST_DICT.update({
        'CVT_ANGLE_ALPHA'       : 50,            # degrees
        'CVT_ARM_LENGTH'        : 25,            # mm
        'PIVOT_BAR_LENGTH'      : 105,           # mm
        'ROLLVIS_SCREW_LEAD'    : 2,             # mm
        'KNEE_GEAR_RATIO'       : 35 / 15,       # mm
        'JOINT_ENC_RESOLUTION'  : 12,            # bits
        'KNEE_JOINT_ENC_COEF'   : 360 / 2**12,   # deg/count
        'MAIN_JOINT_HOMING_WINDOW' : 2**12 / 360,  # ticks for one degree of movement
        'TORQUE_CONSTANT_LARGE_MOTOR': -0.0135,  # Nm/A (EC22, 120W)
    })

    #
    #   CONFIGURE FEEDBACK SENSORS
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1
    )
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_POLARITY] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MIN_POSITION_RANGE_LIMIT] = 0
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_POSITION_RANGE_LIMIT] = 4096
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_INCREMENTAL_ENCODER_1
    )

    #
    #   CONFIGURE CURRENT LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_CURRENT] = 60.0                               # Max instantaneous current 
    
    #
    #   CONFIGURE I2T LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.USER_I2T_ERROR_OPTION_CODE] = 1                   # 0 = disable power stage, 1 = do nothing, 2 = slow down ramp, 3 = quick stop ramp
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT] = 60.0                              # Max instantaneous peak current for I2T calculation
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT_TIME] = 1000000                      # 1,000,000ms

    #
    #   CONFIGURE OPERATION MODE
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.OPERATION_MODE] = (
        NOV.OPERATION_MODE_POSITION
    )
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITIONING_OPTION_CODE] = 3 << 6

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.OPERATION_MODE] = NOV.PROFILER_CYCLIC_SYNCHRONOUS_POSITION
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.INTERPOLATION_TIME_MANTISSA] = 3.0
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.INTERPOLATION_TIME_EXPONENT] = 1.0

    #
    #   SET FILTERS
    #

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_FILTER_1_TYPE] = 0
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_FILTER_1_FREQUENCY] = 100

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_TYPE] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_FREQUENCY] = 200
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR] = 0.7071
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_GAIN] = 0.0

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_TYPE] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_FREQUENCY] = 200
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_Q_FACTOR] = 0.7071
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_GAIN] = 0.0

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW] =  13  # counts
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW_TIME] =  0  # ms

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD] = 1.0  # rev/s
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD_TIME] = 0  # ms

    #
    #   CONFIGURE CYCLIC REGISTERS
    #

    # Adding `NOV.REGISTER_NAME` to these lists will cause the register to be
    # sent/received cyclically to/from the Everest servo. The Everest
    # initialization code will automatically add properties for
    # `servo.channel_[pico|poci].register_name` where "register_name" will be
    # the same as the name of the NOV constant in lowercase.

    CONST_DICT['EVEREST_CYCLIC_PICO_REGISTERS'] = [
        NOV.POSITION_SET_POINT,                     # (int32, counts)
        NOV.POSITION_LOOP_KP,                       # (float)
        NOV.VELOCITY_LOOP_KP,                       # (float)
        NOV.TORQUE_LOOP_KP,                         # (float)
        NOV.TORQUE_LOOP_INPUT_OFFSET,               # (float)
    ]

    CONST_DICT['EVEREST_CYCLIC_POCI_REGISTERS'] = [
        NOV.ACTUAL_VELOCITY,            # motor velocity (float, rev/s)
        NOV.ACTUAL_POSITION,            # joint encoder (int32, counts)
        NOV.STATUS_WORD,                # status word (uint16)
        NOV.LAST_ERROR,                 # last error (int32)
        NOV.BUS_VOLTAGE_VALUE,          # bus voltage (float, V)
        # motor current (float, A):
        NOV.CURRENT_QUADRATURE_VALUE
    ]

# -----------------------------------------------------------------------------
#   Constants for the Fox Knee
# -----------------------------------------------------------------------------
if selector.device_name == 'FOX KNEE':

    #
    #   CONFIGURE KINEMATIC GEOMETRY, ENCODER RESOLUTIONS, AND MOTOR CONSTANTS
    #

    A = const(22.0)                             # (mm)
    B = math.sqrt(91**2 + 2**2)                 # (mm)
    ALPHA = const(91.26)                        # (deg)
    THETA_OFFSET = const(65.0)                  # (deg)
    SCREW_LEAD = const(2.0)                     # (mm)

    CONST_DICT.update({
        # Terms to optimize the calculation of the transmission ratio for the
        # Fox Knee
        'FOX_TRANSMISSION_CALCULATION_TERMS' : array.array(
            'f', 
            [
                ALPHA + THETA_OFFSET,
                -A * B * 2 * math.pi / SCREW_LEAD,
                A ** 2 + B ** 2,
                2 * A * B,
            ]
        ),
        'JOINT_ENC_RESOLUTION'  : 18,               # bits
        'KNEE_JOINT_ENC_COEF'   : 360 / 2**18,      # deg/count
        'MAIN_JOINT_HOMING_WINDOW' : 2**18 / 360,   # ticks for one degree of movement
        'TORQUE_CONSTANT_LARGE_MOTOR': -0.035,      # Nm/A
        'FTS_TRANSIENT_REJECT_THRESHOLD': 50000    # raw ADC counts; max delta per sample before rejection
    })

    #
    #   CONFIGURE FEEDBACK SENSORS
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1
    )
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_POLARITY] = 0
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MIN_POSITION_RANGE_LIMIT] = 0
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_POSITION_RANGE_LIMIT] = 2 ** 18
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_PROTOCOL] = 0x0          # 0=BiSS-C; 1=SSI; 2=Reserved; 3=Serial Synchronous 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_SIZE] = 26         # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_BAUDRATE] = 1000         # kHz
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_POLYNOMIAL] = 0x43   # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_NUMBER_OF_BITS] = 6  # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_CRC_SEED] = 0x0          # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_BITS] = 18      # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_SINGLE_TURN_BITS] = 18   # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_POSITION_START_BIT] = 8  # Celera docs
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_FRAME_TYPE] = 0          # 0=Raw; 1=Raw gray; 2=Zettlex SSI1; 3=BiSS-C BP3; 4=BiSS-C BP3 gray
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_ERROR_TOLERANCE] = 0     # Errors are ignored
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1
    )

    #
    #   CONFIGURE CURRENT LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_CURRENT] = 60.0                               # Max instantaneous current                  
    
    #
    #   CONFIGURE I2T LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.USER_I2T_ERROR_OPTION_CODE] = 1                   # 0 = disable power stage, 1 = do nothing, 2 = slow down ramp, 3 = quick stop ramp
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT] = 60.0                              # Max instantaneous peak current for I2T calculation
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT_TIME] = 1000000                      # 1,000,000ms
    
    #
    #   CONFIGURE OPERATION MODE
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.OPERATION_MODE] = (
        NOV.OPERATION_MODE_POSITION
    )
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITIONING_OPTION_CODE] = 3 << 6

    #
    #   SET FILTERS
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_TYPE] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_FREQUENCY] = 200
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR] = 0.7071
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_GAIN] = 0.0

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_TYPE] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_FREQUENCY] = 200
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_Q_FACTOR] = 0.7071
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_GAIN] = 0.0

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW] =  0  # counts
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW_TIME] =  0  # ms

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD] = 1.0 / 360.0  # rev/s
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD_TIME] = 0  # ms

    #
    #   CONFIGURE CYCLIC REGISTERS
    #

    # Adding `NOV.REGISTER_NAME` to these lists will cause the register to be
    # sent/received cyclically to/from the Everest servo. The Everest
    # initialization code will automatically add properties for
    # `servo.channel_[pico|poci].register_name` where "register_name" will be
    # the same as the name of the NOV constant in lowercase.

    CONST_DICT['EVEREST_CYCLIC_PICO_REGISTERS'] = [
        NOV.POSITION_SET_POINT,                     # (int32, counts)
        NOV.POSITION_LOOP_KP,                       # (float)
        NOV.VELOCITY_LOOP_KP,                       # (float)
        NOV.TORQUE_LOOP_KP,                         # (float)
        NOV.TORQUE_LOOP_INPUT_OFFSET,               # (float, Nm)
        # NOV.DYNAMIC_BRAKE_OPTION_CODE,              # (int16, 0=disable, 1=enable)
    ]

    CONST_DICT['EVEREST_CYCLIC_POCI_REGISTERS'] = [
        NOV.ACTUAL_VELOCITY,            # joint velocity (float, rev/s)
        NOV.ACTUAL_POSITION,            # joint encoder (int32, counts)
        NOV.STATUS_WORD,                # status word (uint16)
        NOV.LAST_ERROR,                 # last error (int32)
        # NOV.CURRENT_LOOP_STATUS,        # contains the current loop status (uint16)
        # NOV.VELOCITY_LOOP_STATUS,       # contains the velocity loop status (uint16)
        # NOV.POSITION_LOOP_STATUS,       # contains the position loop status (uint16)
        NOV.BUS_VOLTAGE_VALUE,          # bus voltage (float, V)
        NOV.MOTOR_TEMPERATURE_VALUE,    # motor temperature (float, C)
        # motor current (float, A):
        NOV.CURRENT_QUADRATURE_VALUE,
        NOV.PRIMARY_ABSOLUTE_SLAVE_1_FULL_FRAME
    ]

# -----------------------------------------------------------------------------
#   Constants for the Polycentric Ankle
# -----------------------------------------------------------------------------
if (
    selector.device_name == 'POLYCENTRIC ANKLE'
    or selector.device_name == 'POLYCENTRIC ANKLE ONLY'
):

    CONST_DICT.update({
        # Polynomial coefficients for calculating the transmission ratio from
        # shank angle (deg) for P2 ankle:
        'P2_TRANSMISSION_POLYNOMIAL_COEFFICIENTS'   : array.array(
            'f',
            [6.59357730e+02, -1.02870085e+01, -1.85692769e+00, 2.34262842e-02, 2.99307242e-03, -2.49702545e-05, -2.50648140e-06]
        ),

        # Polynomial coefficients for calculating joint angle from encoder
        # signal:
        'P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS'  : array.array(
            'f',
            [-8.65273E-02, -2.93956E-02, 7.27465E-06, -1.54225E-08]
        ),
        # Polynomial coefficients for calculating encoder values from the
        # joint angle
        'P2_JOINT_ENCODER_POLYNOMIAL_COEFFICIENTS_INVERTED' : array.array(
            'f',
            [0.56610141, -33.7858617, 0.1585951, 0.01196084]
        ),
        
        'JOINT_ENC_RESOLUTION'  : 12,               # bits
        'ANKLE_JOINT_ENC_COEF'  : 0.0828528,        # deg/count based on a linearization of the joint angle to encoder counts
        # 'ANKLE_JOINT_ENC_COEF'  : 360 / 2**12,      # deg/count based on encoder resolution
        'MAIN_JOINT_HOMING_WINDOW' : 2**12 / 360,  # ticks for one degree of movement
        
        'TORQUE_CONSTANT_LARGE_MOTOR': 0.0137       # Nm/A
    })

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1
    )
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PRIMARY_ABSOLUTE_SLAVE_1_POLARITY] = 1
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MIN_POSITION_RANGE_LIMIT] = 0
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_POSITION_RANGE_LIMIT] = 4096

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_SENSOR] = (
        NOV.FEEDBACK_INCREMENTAL_ENCODER_1
    )
    
    #
    #   CONFIGURE CURRENT LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.MAX_CURRENT] = 25.0                               # Max instantaneous current
    
    #
    #   CONFIGURE I2T LIMITS
    #
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.USER_I2T_ERROR_OPTION_CODE] = 1                   # 0 = disable power stage, 1 = do nothing, 2 = slow down ramp, 3 = quick stop ramp
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT] = 25.0                              # Max instantaneous peak current for I2T calculation
    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.PEAK_CURRENT_TIME] = 1000000                      # 1,000,000ms

    #
    #   CONFIGURE OPERATION MODE
    #

    CONST_DICT['EVEREST_CONFIG_DICT'][NOV.OPERATION_MODE] = (
        NOV.OPERATION_MODE_CURRENT
    )
    
    #CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITIONING_OPTION_CODE] = 3 << 6
    
    #
    #   SET FILTERS
    #

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_FILTER_1_TYPE] = 0
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_FEEDBACK_FILTER_1_FREQUENCY] = 100

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_TYPE] = 1
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_FREQUENCY] = 200
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR] = 0.7071
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_FEEDBACK_FILTER_1_GAIN] = 0.0

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_TYPE] = 1
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_FREQUENCY] = 200
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_Q_FACTOR] = 0.7071
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_REFERENCE_FILTER_1_GAIN] = 0.0

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW] =  13  # counts
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.POSITION_WINDOW_TIME] =  0  # ms

    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD] = 1.0  # rev/s
    # CONST_DICT['EVEREST_CONFIG_DICT'][NOV.VELOCITY_THRESHOLD_TIME] = 0  # ms

    # See comment in hybrid knee section above about the following.
    # CONST_DICT['EVEREST_CYCLIC_PICO_REGISTERS'] = [
    #     NOV.POSITION_SET_POINT,                     # (int32, counts)
    #     NOV.POSITION_LOOP_KP,                       # (float)
    #     NOV.VELOCITY_LOOP_KP,                       # (float)
    #     NOV.TORQUE_LOOP_KP,                         # (float)
    #     NOV.TORQUE_LOOP_INPUT_OFFSET,               # (float)
    # ]

    CONST_DICT['EVEREST_CYCLIC_PICO_REGISTERS'] = [
        NOV.CURRENT_QUADRATURE_SET_POINT                     # (int32, counts)
    ]

    CONST_DICT['EVEREST_CYCLIC_POCI_REGISTERS'] = [
        NOV.ACTUAL_VELOCITY,                            # motor velocity (float, rev/s)
        NOV.ACTUAL_POSITION,                            # joint encoder (int32, counts)
        NOV.CURRENT_QUADRATURE_VALUE,
        # NOV.CURRENT_QUADRATURE_A_CONTROL_LOOP_COMMAND,  # motor current (float, A)
        NOV.STATUS_WORD,                                # status word (uint16)
        NOV.LAST_ERROR,                                 # last error (int32)
        NOV.BUS_VOLTAGE_VALUE,                          # bus voltage (float, V)
    ]

gc.collect()
