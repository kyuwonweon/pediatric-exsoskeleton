'''
The following declare constants that are defined by Novanta's specifications.
Implementation-specific constants are defined in their respective class(es).
'''

'''
╔══════════════════════════════════════════════════════════════════════════════════════════════╗

 NOVANTA STATE DIAGRAM

╚══════════════════════════════════════════════════════════════════════════════════════════════╝
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓   ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃Power                                                       ┃   ┃Fault         ┌──[13]───     ┃
┃Disabled                                                    ┃   ┃              ▼              ┃
┃                 ┌─────────────────────┐                    ┃   ┃   ┌─────────────────────┐   ┃
┃                 │                     │                    ┃   ┃   │   FAULT REACTION    │   ┃
┃                 │        START        │                    ┃   ┃   │       ACTIVE        │   ┃
┃                 │                     │                    ┃   ┃   │                     │   ┃
┃                 └─────────────────────┘                    ┃   ┃   │  SW = 0b x0xx 1111  │   ┃
┃                            │                               ┃   ┃   └─────────────────────┘   ┃
┃                           [0]                              ┃   ┃              │              ┃
┃                            │                               ┃   ┃            [14]             ┃
┃                            ▼                               ┃   ┃         (automatic)         ┃
┃                 ┌─────────────────────┐                    ┃   ┃              │              ┃
┃                 │    NOT READY TO     │                    ┃   ┃              ▼              ┃
┃                 │      SWITCH ON      │                    ┃   ┃   ┌─────────────────────┐   ┃
┃                 │                     │                    ┃   ┃   │        FAULT        │   ┃
┃                 │  SW = 0b x0xx 0000  │                    ┃   ┃   │                     │   ┃
┃                 └─────────────────────┘                    ┃   ┃   │  SW = 0b x0xx 1000  │   ┃
┃                            │                               ┃   ┃   │                     │   ┃
┃                           [1]                              ┃   ┃   └─────────────────────┘   ┃
┃                       (automatic)                          ┃   ┃              │              ┃
┃                            │                               ┃   ┃              │              ┃
┃                            ▼                               ┃   ┃      [15]    │              ┃
┃                 ┌─────────────────────┐◀────────────────────────────CW = 128──┘              ┃
┃                 │      SWITCH ON      │                    ┃   ┃                             ┃
┃                 │      DISABLED       │                    ┃   ┃                             ┃
┃    ┌───────────▶│                     │◀───────────────┐   ┃   ┃                             ┃
┃    │            │  SW = 0b x1xx 0000  │                │   ┃   ┃                             ┃
┃    │            └─────────────────────┘◀───────┐       │   ┃   ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
┃    │            │                     ▲        │       │   ┃
┃    │           [2]                    │        │       │   ┃    NOTE:
┃    │          CW=6                   [7]       │       │   ┃    ----------------
┃    │            │                   CW=0       │       │   ┃    CW: control word
┃    │            ▼                     │        │       │   ┃    SW: status word
┃    │            ┌─────────────────────┐        │       │   ┃
┃    │            │      READY TO       │        │       │   ┃
┃    │            │      SWITCH ON      │        │       │   ┃
┃    │    ┌──────▶│                     │        │       │   ┃
┃    │    │       │  SW = 0b x01x 0001  │        │       │   ┃
┃    │    │       └─────────────────────┘        │       │   ┃
┃    │    │       │                     ▲        │       │   ┃
┗━━━━│━━━━│━━━━━━━│━━━━━━━━━━━━━━━━━━━━━│━━━━━━━━│━━━━━━━│━━━┛
     │    │       │                     │        │       │
     │    │       │                     │        │       │
┏━━━━│━━━━│━━━━━━━│━━━━━━━━━━━━━━━━━━━━━│━━━━━━━━│━━━━━━━│━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃    │    │       │                     │        │       │                       Power┃
┃    │    │       │                     │        │       │                     Enabled┃
┃    │    │      [3]                   [6]       │       │                            ┃
┃    │    │     CW=7                  CW=6       │       │                            ┃
┃    │    │       │                     │      [10]    [12]                           ┃
┃    │    │       │                     │      CW=0    CW=0                           ┃
┃    │   [8]      ▼                     │        │       │                            ┃
┃    │  CW=6      ┌─────────────────────┐        │       │                            ┃
┃    │    │       │     SWITCHED ON     │        │       │                            ┃
┃    │    │       │                     │        │       │                            ┃
┃   [9]   │       │  SW = 0b x01x 0011  │────────┘       └───────┐                    ┃
┃  CW=0   │       │                     │                        │                    ┃
┃    │    │       └─────────────────────┘                        │                    ┃
┃    │    │       │                     ▲                        │                    ┃
┃    │    │      [4]                    │                        │                    ┃
┃    │    │     CW=15                  [5]                       │                    ┃
┃    │    │       │                   CW=7                       │                    ┃
┃    │    │       ▼                     │                        │                    ┃
┃    │    └───────┌─────────────────────┐───[11]─────▶┌─────────────────────┐         ┃
┃    │            │      OPERATION      │   CW=2      │     QUICK STOP      │         ┃
┃    │            │       ENABLE        │ (or 11)     │       ACTIVE        │         ┃
┃    │            │                     │             │                     │         ┃
┃    │            │  SW = 0b x01x 0111  │             │  SW = 0b x00x 0111  │         ┃
┃    └────────────└─────────────────────┘             └─────────────────────┘         ┃
┃                                                                                     ┃
┃                                                                                     ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
'''

class NovantaConstants:
    """
    Constants defined by Novanta's specifications.

    These constants are used to define the state diagram, register addresses,
    and other values that are used to communicate with Novanta's servo drives.
    """

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   COMMAND CODES
    #
    #   All commands are 3 bits. The header of each Rx and Tx transmission
    #   contains a command code.
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    #   controller -> servo
    CMD_GET_INFO = 0b000
    CMD_READ = 0b001
    CMD_WRITE = 0b010

    #   servo -> controller
    CMD_ACK = 0b011
    CMD_INFO_ERROR = 0b100       # not in documentation
    CMD_READ_ERROR = 0b101
    CMD_WRITE_ERROR = 0b110

    #   used by both
    CMD_IDLE = 0b111

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   REGISTER INFO VALUES
    #
    #   One each of 'ACCESS', 'TYPE', and 'CYCLIC' are returned when
    #   information about a specific register is requested.
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ACCESS_READ_ONLY = 3
    ACCESS_WRITE_ONLY = 5
    ACCESS_READ_WRITE = 7

    TYPE_INT16 = 0
    TYPE_UINT16 = 1
    TYPE_INT32 = 2
    TYPE_UINT32 = 3
    TYPE_FLOAT = 4
    TYPE_STRING = 5

    CYCLIC_CONFIG_ONLY = 0
    CYCLIC_TX = 1                # servo -> controller
    CYCLIC_RX = 2                # controller -> servo

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   STATE DIAGRAM IDENTIFIERS          # (transition number)
    #   numbers are used as identifiers and are also sent in data field
    #   when updating control word to change state. negative numbers aren't
    #   utilized by control word
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    STATE_NOT_READY_TO_SWITCH_ON = -1    # 0
    STATE_SWITCH_ON_DISABLED = 0         # 1, 7, 9, 10, 12
    STATE_READY_TO_SWITCH_ON = 6 << 48   # 2, 6, 8
    STATE_SWITCHED_ON = 7 << 48          # 3, 5
    STATE_OPERATION_ENABLE = 15 << 48    # 4
    STATE_QUICK_STOP = 2 << 48           # 11
    STATE_FAULT_REACTION = -2
    STATE_FAULT = -3
    STATE_FAULT_RESET = 128 << 48        # 15 FAULT -> SWITCH_ON_DISABLED

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   OPERATION MODES
    #   see documentation for register 0x014 (Operation mode)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    OPERATION_MODE_VOLTAGE = 0
    OPERATION_MODE_CURRENT_AMPLIFIER = 1
    OPERATION_MODE_CURRENT = 2
    OPERATION_MODE_VELOCITY = 3
    OPERATION_MODE_POSITION = 4
    OPEPRATION_MODE_TORQUE = 5

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   FEEDBACK TYPES
    #   see documentation for registers:
    #       0x360 (velocity feedback sensor)
    #       0x361 (position feedback sensor)
    #       0x362 (auxiliary feedback sensor)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    FEEDBACK_NONE = 0
    FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_1 = 1
    # 2 is reserved
    FEEDBACK_INTERNAL_GENERATOR = 3
    FEEDBACK_INCREMENTAL_ENCODER_1 = 4
    FEEDBACK_DIGITAL_HALLS = 5
    FEEDBACK_SECONDARY_ABSOLUTE_SLAVE_1 = 6
    FEEDBACK_PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN = 7
    FEEDBACK_INCREMENTAL_ENCODER_2 = 8

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   PROFILER MODES
    #   see documentation for register 0x014 (Operation mode)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    #   Primary modes
    PROFILER_NONE = 0
    PROFILER_TRAPEZOIDAL = 1
    PROFILER_LINEAR_INTERPOLATOR = 2
    PROFILER_5TH_DEGREE_POLY = 3
    PROFILER_S_CURVE = 4

    #   Modes with options
    PROFILER_POSITION = 0x14
    PROFILER_POSITION_S_CURVE = 0x44
    PROFILER_CYCLIC_SYNCHRONOUS_POSITION = 0x24
    PROFILER_INTERPOLATED_POSITION = 0xA4
    PROFILER_BUFFERED_PVT = 0xB4
    PROFILER_VELOCITY = 0x13
    PROFILER_CYCLIC_SYNCHRONOUS_VELOCITY = 0x23
    PROFILER_SYNCHRONOUS_CURRENT = 0x22
    PROFILER_HOMING = 0x113

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   REGISTER ADDRESSES
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    LAST_ERROR = 0x00F
    CONTROL_WORD = 0x010
    STATUS_WORD = 0x011
    OPERATION_MODE = 0x014
    OPERATION_MODE_DISPLAY = 0x015
    VOLTAGE_QUADRATURE_SET_POINT = 0x018
    VOLTAGE_DIRECT_SET_POINT = 0x019
    CURRENT_QUADRATURE_SET_POINT = 0x01A
    CURRENT_DIRECT_SET_POINT = 0x01B
    CURRENT_A_SET_POINT = 0x01C
    CURRENT_B_SET_POINT = 0x01D
    POSITION_SET_POINT = 0x020
    VELOCITY_SET_POINT = 0x021
    TORQUE_SET_POINT = 0x022
    POSITIONING_OPTION_CODE = 0x024
    PROFILER_MAX_VELOCITY = 0x025
    PROFILER_MAX_ACCELERATION = 0x026
    PROFILER_MAX_DECELERATION = 0x027
    ACTUAL_TORQUE = 0x029
    ACTUAL_POSITION = 0x030
    ACTUAL_VELOCITY = 0x031
    AUXILIAR_FEEDBACK_VALUE = 0x033
    BUS_VOLTAGE_CURRENT_COUNTS = 0x037
    CURRENT_A_VALUE = 0x038
    CURRENT_B_VALUE = 0x039
    CURRENT_C_VALUE = 0x03A
    CURRENT_QUADRATURE_VALUE = 0x03B
    CURRENT_DIRECT_VALUE = 0x03C
    CURRENT_A_COUNTS = 0x03D
    CURRENT_B_COUNTS = 0x03E
    CURRENT_C_COUNTS = 0x03F
    COMMUTATION_ANGLE_VALUE = 0x040
    REFERENCE_ANGLE_VALUE = 0x041
    PRIMARY_ABSOLUTE_SLAVE_1_POSITION = 0x04A
    PRIMARY_ABSOLUTE_SLAVE_1_FULL_FRAME = 0x04B
    GENERATOR_VALUE = 0x050
    SECONDARY_ABSOLUTE_SLAVE_1_POSITION = 0x051
    TORQUE_DEMAND = 0x053
    SECONDARY_ABSOLUTE_SLAVE_1_FULL_FRAME = 0x054
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_FULL_FRAME = 0x055
    POWER_STAGE_TEMPERATURE_3_VALUE = 0x05D
    POWER_STAGE_TEMPERATURE_3_COUNTS = 0x05E
    BUS_VOLTAGE_VALUE = 0x060
    POWER_STAGE_TEMPERATURE_1_VALUE = 0x061
    POWER_STAGE_TEMPERATURE_1_COUNTS = 0x062
    MOTOR_TEMPERATURE_VALUE = 0x063
    POWER_STAGE_TEMPERATURE_2_VALUE = 0x064
    POWER_STAGE_TEMPERATURE_2_COUNTS = 0x066
    POWER_STAGE_MAX_ACTUAL_TEMPERATURE = 0x067
    VOLTAGE_PHASE_A = 0x068
    VOLTAGE_PHASE_B = 0x069
    VOLTAGE_PHASE_C = 0x06A
    VOLTAGE_DIRECT_COMMAND = 0x06E
    VOLTAGE_QUADRATURE_COMMAND = 0x06F
    VOLTAGE_QUADRATURE_DEMAND = 0x070
    VOLTAGE_DIRECT_DEMAND = 0x071
    CURRENT_QUADRATURE_DEMAND = 0x072
    CURRENT_DIRECT_DEMAND = 0x073
    CURRENT_ACTUAL_VALUE = 0x076
    CURRENT_COMMAND_VALUE = 0x077
    POSITION_DEMAND = 0x078
    VELOCITY_DEMAND = 0x079
    ACCELERATION_DEMAND = 0x07A
    VELOCITY_LOOP_INPUT_OFFSET = 0x07B
    CURRENT_QUADRATURE_LOOP_INPUT_OFFSET = 0x07C
    CURRENT_DIRECT_LOOP_INPUT_OFFSET = 0x07D
    TORQUE_LOOP_INPUT_OFFSET = 0x080
    ANALOG_INPUT_1_COUNTS = 0x081
    ANALOG_INPUT_1_VALUE = 0x082
    ANALOG_2_COUNTS = 0x083
    ANALOG_2_VALUE = 0x084
    GATE_VOLTAGE_VALUE = 0x086
    MAIN_LOGIC_SUPPLY_VOLTAGE = 0x089
    IO_LEVEL_SUPPLY_VOLTAGE = 0x08A
    MCU_LOGIC_SUPPLY_VOLTAGE = 0x08B
    ANALOG_OUTPUT_1_VALUE = 0x08D
    POSITION_LOOP_CONTROL_COMMAND = 0x096
    VELOCITY_LOOP_CONTROL_COMMAND = 0x097
    CURRENT_QUADRATURE_A_CONTROL_LOOP_COMMAND = 0x098
    CURRENT_DIRECT_B_CONTROL_LOOP_COMMAND = 0x099
    RATED_TORQUE = 0x09A
    TORQUE_LOOP_CONTROL_COMMAND = 0x09B
    INTERPOLATION_DATA_RECORD_POSITION_INPUT = 0x0C1
    INTERPOLATION_DATA_RECORD_VELOCITY_INPUT = 0x0C2
    INTERPOLATION_DATA_RECORD_TIME_INPUT = 0x0C3
    INTERPOLATION_DATA_RECORD_INTEGRITY_CHECK = 0x0C4
    INTERPOLATION_DATA_RECORD_STATUS = 0x0C5
    RATED_CURRENT = 0x100
    PEAK_CURRENT = 0x101
    PEAK_CURRENT_TIME = 0x102
    MOTOR_POLE_PAIRS = 0x106
    MOTOR_TEMPERATURE_SENSOR_RESISTANCE = 0x108             # undocumented
    MOTOR_TEMP_PROTECTION_THRESHOLD = 0x109
    MOTOR_TEMPERATURE_SENSOR_BETA = 0x10C                   # undocumented
    MOTOR_TEMPERATURE_SENSOR_EXTERNAL_RESISTANCE = 0x10D    # undocumented
    MOTOR_TEMPERATURE_SENSOR_VOLTAGE_REFERENCE = 0x110
    INTERNAL_SHUNT_ENABLE_VOLTAGE = 0x120
    INTERNAL_SHUNT_DISABLE_VOLTAGE = 0x121
    EXTERNAL_SHUNT_ENABLE_VOLTAGE = 0x122
    EXTERNAL_SHUNT_DISABLE_VOLTAGE = 0x123
    BRAKE_ACTIVATION_VOLTAGE_PERCENTAGE = 0x124
    BRAKE_HOLDING_VOLTAGE_PERCENTAGE = 0x125
    DELAY_BEFORE_RELEASE_BRAKE = 0x126
    DELAY_AFTER_ENABLE_BRAKE = 0x127
    ACTIVATION_BRAKE_TIME = 0x128
    BRAKE_OVERRIDE = 0x129
    BRAKE_ACTIVATION_CURRENT = 0x12A
    BRAKE_HOLDING_CURRENT = 0x12B
    BRAKE_CURRENT_CONTROL_KP = 0x12C
    BRAKE_CURRENT_CONTROL_KI = 0x12D
    BRAKE_CONTROL_MODE = 0x12E
    BRAKE_CURRENT_ERROR_WINDOW = 0x12F
    BRAKE_CURRENT_ERROR_TIMEOUT = 0x130
    BRAKE_MAX_CURRENT = 0x131
    BRAKE_CURRENT_ACTUAL_VALUE = 0x133
    BRAKE_CURRENT_FEEDBACK_SOURCE = 0x134
    BRAKE_CURRENT_COMMAND = 0x141
    COMMUTATION_MODULATION = 0x14F
    COMMUTATION_ANGLE_OFFSET = 0x150
    COMMUTATION_FEEDBACK_SENSOR = 0x151
    REFERENCE_ANGLE_OFFSET = 0x152
    REFERENCE_FEEDBACK_SENSOR = 0x153
    PHASING_MODE = 0x154
    MAX_CURRENT_ON_PHASING_SEQUENCE = 0x155
    PHASING_TIMEOUT = 0x156
    PHASING_ACCURACY = 0x157
    RESTART_ALL_FEEDBACKS = 0x17B
    SET_FEEDBACK_POWER_SUPPLY_STATUS = 0x17C
    POWER_STAGE_MAXIMUM_DUTY_CYCLE = 0x1C0
    PROFILER_LATCHING_MODE = 0x1C8
    VELOCITY_THRESHOLD = 0x1D6
    VELOCITY_THRESHOLD_TIME = 0x1D7
    VELOCITY_FOLLOWING_ERROR_WINDOW = 0x1DC
    VELOCITY_FOLLOWING_ERROR_TIMEOUT = 0x1DD
    MIN_POSITION_RANGE_LIMIT = 0x1DE
    MAX_POSITION_RANGE_LIMIT = 0x1DF
    MAX_CURRENT = 0x1E0
    POSITION_CONTROL_LOOP_ERROR = 0x1E2
    VELOCITY_CONTROL_LOOP_ERROR = 0x1E3
    TORQUE_CONTROL_LOOP_ERROR = 0x1E4
    CURRENT_QUADRATURE_CONTROL_LOOP_ERROR = 0x1E5
    CURRENT_DIRECT_CONTROL_LOOP_ERROR = 0x1E6
    MAX_VELOCITY = 0x1E8
    MIN_POSITION = 0x1EA
    MAX_POSITION = 0x1EB
    POSITION_FOLLOWING_ERROR_WINDOW = 0x1EC
    POSITION_FOLLOWING_ERROR_TIMEOUT = 0x1ED
    FOLLOWING_ERROR = 0x1EE
    INTERPOLATION_TIME_MANTISSA = 0x1EF
    INTERPOLATION_TIME_EXPONENT = 0x1F0
    POSITION_WINDOW = 0x1F1
    POSITION_WINDOW_TIME = 0x1F2
    VELOCITY_WINDOW = 0x1F3
    VELOCITY_WINDOW_TIME = 0x1F4
    QUICK_STOP_DECELERATION = 0x1F5
    INTERPOLATION_DATA_RECORD_FORCE_CLEAR = 0x1F6
    INTERPOLATION_BUFFER_SIZE = 0x1F7
    INTERPOLATION_BUFFER_NUMBER_OF_ELEMENTS = 0x1F8
    INTERPOLATION_BUFFER_MAXIMUM_SIZE = 0x1F9
    COMMAND_SOURCE_1 = 0x2E0
    COMMAND_SOURCE_2 = 0x2E1
    ANALOG_INPUT_1_GAIN = 0x2E8
    ANALOG_INPUT_1_OFFSET = 0x2E9
    ANALOG_INPUT_2_GAIN = 0x2EC
    ANALOG_INPUT_2_OFFSET = 0x2ED
    ENABLED_FEEDBACKS_MASK = 0x35F
    VELOCITY_FEEDBACK_SENSOR = 0x360
    POSITION_FEEDBACK_SENSOR = 0x361
    AUXILIAR_FEEDBACK_SENSOR = 0x362
    POSITION_TO_VELOCITY_SENSOR_RATIO = 0x364
    PRIMARY_ABSOLUTE_SLAVE_1_PROTOCOL = 0x36F
    PRIMARY_ABSOLUTE_SLAVE_1_FRAME_SIZE = 0x370
    PRIMARY_ABSOLUTE_SLAVE_1_ERROR_TOLERANCE = 0x371
    DEPRECATED_PRIMARY_ABSOLUTE_SLAVE_1_WAIT_CYCLES = 0x372
    PRIMARY_ABSOLUTE_SLAVE_1_POLARITY = 0x373
    PRIMARY_ABSOLUTE_SLAVE_1_FRAME_TYPE = 0x374
    PRIMARY_ABSOLUTE_SLAVE_1_POSITION_BITS = 0x375
    PRIMARY_ABSOLUTE_SLAVE_1_SINGLE_TURN_BITS = 0x376
    PRIMARY_ABSOLUTE_SLAVE_1_POSITION_START_BIT = 0x377
    SECONDARY_ABSOLUTE_SLAVE_1_FRAME_SIZE = 0x378
    SECONDARY_ABSOLUTE_SLAVE_1_ERROR_TOLERANCE = 0x379
    DEPRECATED_SECONDARY_ABSOLUTE_SLAVE_1_WAIT_CYCLES = 0x37A
    SECONDARY_ABSOLUTE_SLAVE_1_POLARITY = 0x37B
    SECONDARY_ABSOLUTE_SLAVE_1_FRAME_TYPE = 0x37C
    SECONDARY_ABSOLUTE_SLAVE_1_POSITION_BITS = 0x37D
    SECONDARY_ABSOLUTE_SLAVE_1_SINGLE_TURN_BITS = 0x37E
    SECONDARY_ABSOLUTE_SLAVE_1_POSITION_START_BIT = 0x37F
    GENERATOR_MODE = 0x380
    GENERATOR_FREQUENCY = 0x381
    GENERATOR_GAIN = 0x382
    GENERATOR_OFFSET = 0x383
    GENERATOR_CYCLE_NUMBER = 0x384
    GENERATOR_REARM = 0x385
    PRIMARY_ABSOLUTE_SLAVE_1_BAUDRATE = 0x386
    SECONDARY_ABSOLUTE_SLAVE_1_BAUDRATE = 0x387
    INCREMENTAL_ENCODER_1_RESOLUTION = 0x388
    INCREMENTAL_ENCODER_1_POLARITY = 0x389
    INCREMENTAL_ENCODER_1_FILTER = 0x38A
    INCREMENTAL_ENCODER_1_VALUE = 0x38B
    INCREMENTAL_ENCODER_2_RESOLUTION = 0x38C
    INCREMENTAL_ENCODER_2_POLARITY = 0x38D
    INCREMENTAL_ENCODER_2_FILTER = 0x38E
    INCREMENTAL_ENCODER_2_VALUE = 0x38F
    DIG_HALL_POLARITY = 0x390
    DIG_HALL_FILTER = 0x391
    DIG_HALL_POLE_PAIRS = 0x392
    DIG_HALL_VALUE = 0x393
    FEEDBACK_RUNAWAY_SOURCE = 0x394
    BISS_C_FEEDBACKS_IN_CHAIN = 0x395
    PRIMARY_ABSOLUTE_MAXIMUM_REFRESH_RATE = 0x398
    SECONDARY_ABSOLUTE_MAXIMUM_REFRESH_RATE = 0x399
    PRIMARY_ABSOLUTE_ACTUAL_REFRESH_RATE = 0x39A
    SECONDARY_ABSOLUTE_ACTUAL_REFRESH_RATE = 0x39B
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_FRAME_SIZE = 0x400
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_ERROR_TOLERANCE = 0x401
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_POLARITY = 0x403
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_FRAME_TYPE = 0x404
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_POSITION_BITS = 0x405
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_SINGLE_TURN_BITS = 0x406
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_POSITION_START_BIT = 0x407
    PRIMARY_ABSOLUTE_SLAVE_1_CRC_POLYNOMIAL = 0x408
    PRIMARY_ABSOLUTE_SLAVE_1_CRC_NUMBER_OF_BITS = 0x409
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_CRC_POLYNOMIAL = 0x40A
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_CRC_NUMBER_OF_BITS = 0x40B
    PRIMARY_ABSOLUTE_SLAVE_1_CRC_SEED = 0x40C
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_CRC_SEED = 0x40D
    PRIMARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET = 0x438
    PRIMARY_ABSOLUTE_SLAVE_2_DAISY_CHAIN_POSITION_OFFSET = 0x439
    SECONDARY_ABSOLUTE_SLAVE_1_POSITION_OFFSET = 0x43A
    TORQUE_CONSTANT = 0x43B
    SECONDARY_ABSOLUTE_SLAVE_1_PROTOCOL = 0x43C
    SECONDARY_ABSOLUTE_SLAVE_1_CRC_POLYNOMIAL = 0x43D
    SECONDARY_ABSOLUTE_SLAVE_1_CRC_SEED = 0x43E
    SECONDARY_ABSOLUTE_SLAVE_1_CRC_NUMBER_OF_BITS = 0x43F
    HOMING_MODE = 0x450
    HOMING_OFFSET = 0x451
    HOMING_TIMEOUT = 0x452
    HOMING_LIMIT_SEARCH_SPEED = 0x453
    HOMING_ZERO_SEARCH_SPEED = 0x454
    POSITIVE_HOMING_SWITCH = 0x455
    NEGATIVE_HOMING_SWITCH = 0x456
    HOMING_INDEX_PULSE_SOURCE = 0x457
    CURRENT_LOOP_RATE = 0x458
    CURRENT_SCALING_GAIN_FACTOR = 0x4FF
    CURRENT_QUADRATURE_LOOP_KP = 0x500
    CURRENT_QUADRATURE_LOOP_KI = 0x501
    CURRENT_QUADRATURE_LOOP_MAX_OUT = 0x502
    CURRENT_QUADRATURE_LOOP_MIN_OUT = 0x503
    DEPRECATED_CURRENT_QUADRATURE_LOOP_KR = 0x504
    CURRENT_DIRECT_LOOP_KP = 0x505
    CURRENT_DIRECT_LOOP_KI = 0x506
    CURRENT_DIRECT_LOOP_MAX_OUT = 0x507
    CURRENT_DIRECT_LOOP_MIN_OUT = 0x508
    DEPRECATED_CURRENT_DIRECT_LOOP_KR = 0x509
    VELOCITY_LOOP_KP = 0x50A
    VELOCITY_LOOP_KI = 0x50B
    VELOCITY_LOOP_KD = 0x50C
    VELOCITY_LOOP_KD_FILTER = 0x50D
    VELOCITY_LOOP_MAX_OUTPUT = 0x50E
    VELOCITY_LOOP_MIN_OUTPUT = 0x50F
    CONTROL_LOOPS_FEEDBACK_OPTIONS = 0x510
    POSITION_LOOP_KP = 0x511
    POSITION_LOOP_KI = 0x512
    POSITION_LOOP_KD = 0x513
    POSITION_LOOP_KD_FILTER = 0x514
    POSITION_LOOP_MAX_OUTPUT = 0x515
    POSITION_LOOP_MIN_OUTPUT = 0x516
    CURRENT_LOOP_STATUS = 0x517
    VELOCITY_LOOP_STATUS = 0x518
    POSITION_LOOP_STATUS = 0x519
    STO_STATUS = 0x51A
    VELOCITY_LOOP_INTEGRAL_RESET = 0x51B
    POSITION_LOOP_INTEGRAL_RESET = 0x51C
    VELOCITY_LOOP_INTEGRAL_VALUE = 0x51D
    POSITION_LOOP_INTEGRAL_VALUE = 0x51E
    MAXIMUM_VOLTAGE_BUS_UTILIZATION = 0x51F
    POSITION_AND_VELOCITY_LOOP_RATE = 0x520
    POWER_STAGE_FREQUENCY = 0x521
    CONTROL_LOOPS_OPTION_CODE = 0x522
    TORQUE_LOOP_KP = 0x523
    TORQUE_LOOP_KI = 0x524
    TORQUE_LOOP_MAX_OUTPUT = 0x527
    TORQUE_LOOP_MIN_OUTPUT = 0x528
    POSITION_FEEDBACK_FILTER_1_TYPE = 0x530
    POSITION_FEEDBACK_FILTER_1_FREQUENCY = 0x531
    POSITION_FEEDBACK_FILTER_1_Q_FACTOR = 0x532
    POSITION_FEEDBACK_FILTER_1_GAIN = 0x533
    POSITION_FEEDBACK_FILTER_2_TYPE = 0x538
    POSITION_FEEDBACK_FILTER_2_FREQUENCY = 0x539
    POSITION_FEEDBACK_FILTER_2_Q_FACTOR = 0x53A
    POSITION_FEEDBACK_FILTER_2_GAIN = 0x53B
    POSITION_REFERENCE_FILTER_1_TYPE = 0x540
    POSITION_REFERENCE_FILTER_1_FREQUENCY = 0x541
    POSITION_REFERENCE_FILTER_1_Q_FACTOR = 0x542
    POSITION_REFERENCE_FILTER_1_GAIN = 0x543
    POSITION_REFERENCE_FILTER_2_TYPE = 0x548
    POSITION_REFERENCE_FILTER_2_FREQUENCY = 0x549
    POSITION_REFERENCE_FILTER_2_Q_FACTOR = 0x54A
    POSITION_REFERENCE_FILTER_2_GAIN = 0x54B
    VELOCITY_FEEDBACK_FILTER_1_TYPE = 0x550
    VELOCITY_FEEDBACK_FILTER_1_FREQUENCY = 0x551
    VELOCITY_FEEDBACK_FILTER_1_Q_FACTOR = 0x552
    VELOCITY_FEEDBACK_FILTER_1_GAIN = 0x553
    VELOCITY_FEEDBACK_FILTER_2_TYPE = 0x558
    VELOCITY_FEEDBACK_FILTER_2_FREQUENCY = 0x559
    VELOCITY_FEEDBACK_FILTER_2_Q_FACTOR = 0x55A
    VELOCITY_FEEDBACK_FILTER_2_GAIN = 0x55B
    VELOCITY_REFERENCE_FILTER_1_TYPE = 0x560
    VELOCITY_REFERENCE_FILTER_1_FREQUENCY = 0x561
    VELOCITY_REFERENCE_FILTER_1_Q_FACTOR = 0x562
    VELOCITY_REFERENCE_FILTER_1_GAIN = 0x563
    VELOCITY_REFERENCE_FILTER_2_TYPE = 0x568
    VELOCITY_REFERENCE_FILTER_2_FREQUENCY = 0x569
    VELOCITY_REFERENCE_FILTER_2_Q_FACTOR = 0x56A
    VELOCITY_REFERENCE_FILTER_2_GAIN = 0x56B
    CURRENT_FEEDBACK_FILTER_1_TYPE = 0x570
    CURRENT_FEEDBACK_FILTER_1_FREQUENCY = 0x571
    CURRENT_FEEDBACK_FILTER_1_Q_FACTOR = 0x572
    CURRENT_FEEDBACK_FILTER_1_GAIN = 0x573
    CURRENT_FEEDBACK_FILTER_2_TYPE = 0x578
    CURRENT_FEEDBACK_FILTER_2_FREQUENCY = 0x579
    CURRENT_FEEDBACK_FILTER_2_Q_FACTOR = 0x57A
    CURRENT_FEEDBACK_FILTER_2_GAIN = 0x57B
    CURRENT_REFERENCE_FILTER_1_TYPE = 0x580
    CURRENT_REFERENCE_FILTER_1_FREQUENCY = 0x581
    CURRENT_REFERENCE_FILTER_1_Q_FACTOR = 0x582
    CURRENT_REFERENCE_FILTER_1_GAIN = 0x583
    CURRENT_REFERENCE_FILTER_2_TYPE = 0x588
    CURRENT_REFERENCE_FILTER_2_FREQUENCY = 0x589
    CURRENT_REFERENCE_FILTER_2_Q_FACTOR = 0x58A
    CURRENT_REFERENCE_FILTER_2_GAIN = 0x58B
    VELOCITY_LOOP_KFFA = 0x590
    POSITION_LOOP_KFFV = 0x591
    POSITION_LOOP_KFFA = 0x592
    MAP_OUTPUT_1 = 0x5FC
    MAP_OUTPUT_2 = 0x5FD
    MAP_OUTPUT_3 = 0x5FE
    MAP_OUTPUT_4 = 0x5FF
    DIGITAL_INPUTS_VALUE = 0x600
    DIGITAL_OUTPUTS_VALUE = 0x601
    DIGITAL_OUTPUTS_SET_VALUE = 0x602
    DIGITAL_OUTPUTS_POLARITY = 0x603
    DIGITAL_INPUTS_POLARITY = 0x604
    POSITIVE_SWITCH_LIMIT = 0x608
    NEGATIVE_SWITCH_LIMIT = 0x609
    QUICK_STOP_INPUT = 0x60A
    QUICK_STOP_OPTION_CODE = 0x60B
    EXTERNAL_ERROR_SIGNAL_REACTION = 0x60D
    USER_I2T_ERROR_OPTION_CODE = 0x60F
    POSITION_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE = 0x610
    VELOCITY_FEEDBACK_OUT_OF_LIMITS_ERROR_OPTION_CODE = 0x611
    POSITION_FOLLOWING_ERROR_OPTION_CODE = 0x612
    POWER_STAGE_USER_OVER_TEMPERATURE_ERROR_OPTION_CODE = 0x613
    MOTOR_OVER_TEMPERATURE_ERROR_OPTION_CODE = 0x614
    POWER_STAGE_USER_UNDER_TEMPERATURE_ERROR_OPTION_CODE = 0x615
    POWER_STAGE_USER_OVER_VOLTAGE_ERROR_OPTION_CODE = 0x616
    POWER_STAGE_USER_UNDER_VOLTAGE_ERROR_OPTION_CODE = 0x617
    EXTERNAL_FAULT_OPTION_CODE = 0x618
    HALLS_SEQUENCE_ERROR_OPTION_CODE = 0x619
    FEEDBACK_RUNAWAY_ERROR_OPTION_CODE = 0x61A
    OVER_CURRENT_WITHOUT_CURRENT_CONTROL_ERROR_OPTION_CODE = 0x61B
    COMMUNICATIONS_WATCHDOG_ERROR_OPTION_CODE = 0x61C
    VELOCITY_FOLLOWING_ERROR_OPTION_CODE = 0x61D
    HALT_OPTION_CODE = 0x620
    HALT_INPUT = 0x621
    LOCAL_CONTROL_ACTIVATION = 0x622
    LOCAL_CONTROL_OPERATION_ENABLE_INPUT = 0x623
    FAULT_RESET_INPUT = 0x624
    FAULT_REACTION_TIMEOUT = 0x62B
    DYNAMIC_BRAKE_OPTION_CODE = 0x62E
    OVER_VOLTAGE_SENSITIVITY = 0x62F
    USER_OVER_VOLTAGE_LEVEL = 0x630
    USER_UNDER_VOLTAGE_LEVEL = 0x631
    USER_OVER_TEMPERATURE_LEVEL = 0x632
    USER_UNDER_TEMPERATURE_LEVEL = 0x633
    USER_OVER_TEMPERATURE_WARNING_LEVEL = 0x634
    USER_UNDER_TEMPERATURE_WARNING_LEVEL = 0x635
    STO_ERROR_OPTION_CODE = 0x63B
    SYNCHRONIZATION_SIGNAL_CONFIGURATION = 0x641
    COMMUNICATIONS_WATCHDOG_WINDOW = 0x642
    SYNCHRONIZATION_SIGNAL_FREQUENCY = 0x643
    SYNCHRONIZATION_SIGNAL_PLL_FILTER_CUTOFF_FREQUENCY = 0x644
    SYNCHRONIZATION_SIGNAL_PLL_PHASE = 0x645
    SYNCHRONIZATION_SIGNAL_PLL_KP = 0x646
    ERROR_TOTAL_NUMBER = 0x64D
    ERROR_LIST_INDEX_REQUEST = 0x64E
    ERROR_LIST_REQUESTED_CODE = 0x64F
    CRC_OUTPUT_MOTION_CORE = 0x6D1
    STORE_ALL = 0x6DB
    RESTORE_ALL = 0x6DC
    BOOT_MODE = 0x6DE
    VENDOR_ID = 0x6E0
    PRODUCT_CODE = 0x6E1
    REVISION_NUMBER = 0x6E2
    SOFTWARE_VERSION = 0x6E4
    INGENIA_URL = 0x6E5
    SERIAL_NUMBER = 0x6E6

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   REGISTER VALUE CONSTANTS
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    STORE_ALL_PASSWORD = 0x65766173


    @classmethod
    def get_register_name(cls, address):
        """Provides the name of the register given the address

        Parameters:
            address (int): the address of the register

        Returns:
            str: the name of the register
        """
        for name, adr in cls.__dict__.items():
            if adr == address:
                return name
        return None
