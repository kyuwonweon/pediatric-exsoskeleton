####################################################################################################
#   @file
#
#   @brief      Driver for BNO085 IMU chip which provides data via SPI
#
#   @author     Jim Lipsey (JHL)
####################################################################################################

import array
import time

import pyb                                          # type: ignore
import micropython                                  # type: ignore
from micropython import const                       # type: ignore


from . import nvic

print('IMPORTING BNO085')

_REPORT_TIMING                                                              = const(False)

# -----------------------------------------------------------------------------
#   CONSTANTS FOR SH2
#   (SH2  = SENSOR HUB VERSION 2 - DATA FORMAT PROTOCOL)
#   (SHTP = SENSOR HUB TRANSPORT PROTOCOL - DATA TRANSMISSION PROTOCOL)
#
#   These are constants to implement manufacturer's protocol specification
# -----------------------------------------------------------------------------
#   CHANNEL INDENTIFIERS
_CHANNEL_EXECUTABLE                                                          = const(0x1)
_CHANNEL_SENSOR_HUB_CONTROL                                                  = const(0x2)
_CHANNEL_INPUT_SENSOR_REPORTS                                                = const(0x3)
_CHANNEL_WAKE_INPUT_SENSOR_REPORTS                                           = const(0x4)
_CHANNEL_GYRO_ROTATION_VECTOR                                                = const(0x5)

#   COMMANDS
_COMMAND_REPORT_ERRORS                                                       = const(0x01)
_COMMAND_COUNTER                                                             = const(0x02)
_SUBCOMMAND_GET_COUNTS                                                       = const(0x00)
_SUBCOMMAND_CLEAR_COUNTS                                                     = const(0x01)
_COMMAND_TARE                                                                = const(0x03)  # Tare command usage described in supplemental document
_SUBCOMMAND_TARE_NOW                                                         = const(0x00)
_SUBCOMMAND_PERSIST_TARE                                                     = const(0x01)
_SUBCOMMAND_SET_TARE_REORIENTATION                                           = const(0x02)
_COMMAND_INITIALIZE                                                          = const(0x04)
_UNSOLICITED_INITIALIZE                                                      = const(0x84)
_COMMAND_SAVE_DCD                                                            = const(0x06)
_COMMAND_MOTION_ENGINE_CALIBRATE                                             = const(0x07)
_COMMAND_CONFIGURE_PERIODIC_DCD_SAVE                                         = const(0x09)
_COMMAND_GET_OSCILLATOR_TYPE                                                 = const(0x0A)
_COMMAND_CLEAR_AND_RESET_DCD                                                 = const(0x0B)
_COMMAND_TURNTABLE_CALIBRATION                                               = const(0x0C)
_SUBCOMMAND_START_TURNTABLE_CALIBRATION                                      = const(0x00)
_SUBCOMMAND_FINISH_TURNTABLE_CALIBRATION                                     = const(0x01)
_COMMAND_BOOTLOADER                                                          = const(0x0D)
_SUBCOMMAND_BOOTLOADER_OPERATING_MODE_REQUEST                                = const(0x00)
_SUBCOMMAND_BOOTLOADER_STATUS_REQUEST                                        = const(0x01)
_COMMAND_INTERACTIVE_CALIBRATION                                             = const(0x0E)

#   REPORT IDENTIFIERS
#   pyboard is host, BNO085 is hub
_REPORT_PRODUCT_ID_REQUEST                                                   = const(0XF9)  # host ->  hub
_REPORT_PRODUCT_ID_RESPONSE                                                  = const(0XF8)  # hub  ->  host
_REPORT_FRS_WRITE_REQUEST                                                    = const(0XF7)  # host ->  hub
_REPORT_FRS_WRITE_DATA                                                       = const(0XF6)  # host ->  hub
_REPORT_FRS_WRITE_RESPONSE                                                   = const(0XF5)  # hub  ->  host
_REPORT_FRS_READ_REQUEST                                                     = const(0XF4)  # host ->  hub
_REPORT_FRS_READ_RESPONSE                                                    = const(0xF3)  # hub  ->  host
_REPORT_COMMAND_REQUEST                                                      = const(0XF2)  # host ->  hub (used with command identifiers listed above)
_REPORT_COMMAND_RESPONSE                                                     = const(0XF1)  # hub  ->  host (used with command identifiers listed above)
_REPORT_GET_FEATURE_REQUEST                                                  = const(0XFE)  # host ->  hub
_REPORT_SET_FEATURE_COMMAND                                                  = const(0XFD)  # host ->  hub
_REPORT_GET_FEATURE_RESPONSE                                                 = const(0XFC)  # hub  ->  host
_REPORT_FORCE_SENSOR_FLUSH                                                   = const(0xF0)  # host ->  hub
_REPORT_SENSOR_FLUSH_COMPLETED                                               = const(0xEF)  # hub  ->  to host

_REPORT_TIMESTAMP                                                            = const(0xFB)  # BNO085
_REPORT_RAW_ACCELEROMETER                                                    = const(0x14)  # BNO085
_REPORT_ACCELEROMETER                                                        = const(0x01)  # BNO085
_REPORT_LINEAR_ACCELERATION                                                  = const(0x04)  # BNO085
_REPORT_GRAVITY                                                              = const(0x06)  # BNO085
_REPORT_RAW_GYROSCOPE                                                        = const(0x15)  # BNO085
_REPORT_GYROSCOPE_CALIBRATED                                                 = const(0x02)  # BNO085
_REPORT_GYROSCOPE_UNCALIBRATED                                               = const(0x07)  # BNO085
_REPORT_RAW_MAGNETOMETER                                                     = const(0x16)  # BNO085
_REPORT_MAGNETIC_FIELD_CALIBRATED                                            = const(0x03)  # BNO085
_REPORT_MAGNETIC_FIELD_UNCALIBRATED                                          = const(0x0F)  # BNO085
_REPORT_ROTATION_VECTOR                                                      = const(0x05)  # BNO085
_REPORT_GAME_ROTATION_VECTOR                                                 = const(0x08)  # BNO085
_REPORT_GEOMAGNETIC_ROTATION_VECTOR                                          = const(0x09)  # BNO085
_REPORT_PRESSURE                                                             = const(0x0A)  # BNO085 via external sensor
_REPORT_AMBIENT_LIGHT                                                        = const(0x0B)  # BNO085 via external sensor
_REPORT_HUMIDITY                                                             = const(0x0C)  # BNO085 via external sensor
_REPORT_PROXIMITY                                                            = const(0x0D)
_REPORT_TEMPERATURE                                                          = const(0x0E)  # BNO085 via external sensor
_REPORT_RESERVED                                                             = const(0x17)
_REPORT_TAP_DETECTOR                                                         = const(0x10)  # BNO085
_REPORT_STEP_DETECTOR                                                        = const(0x18)  # BNO085
_REPORT_STEP_COUNTER                                                         = const(0x11)  # BNO085
_REPORT_SIGNIFICANT_MOTION                                                   = const(0x12)  # BNO085
_REPORT_STABILITY_CLASSIFIER                                                 = const(0x13)  # BNO085
_REPORT_SHAKE_DETECTOR                                                       = const(0x19)  # BNO085
_REPORT_FLIP_DETECTOR                                                        = const(0x1A)
_REPORT_PICKUP_DETECTOR                                                      = const(0x1B)
_REPORT_STABILITY_DETECTOR                                                   = const(0x1C)  # BNO085
_REPORT_PERSONAL_ACTIVITY_CLASSIFIER                                         = const(0x1E)  # BNO085
_REPORT_SLEEP_DETECTOR                                                       = const(0x1F)
_REPORT_TILT_DETECTOR                                                        = const(0x20)
_REPORT_POCKET_DETECTOR                                                      = const(0x21)
_REPORT_CIRCLE_DETECTOR                                                      = const(0x22)
_REPORT_HEART_RATE_MONITOR                                                   = const(0x23)
_REPORT_ARVR_STABILIZED_ROTATION_VECTOR                                      = const(0x28)  # BNO085
_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR                                 = const(0x29)  # BNO085
_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR                                      = const(0x2A)  # BNO085
_REPORT_MOTION_REQUEST                                                       = const(0x2B)  # BNO085 (called gyro rotation vector prediction in datasheet?)

#   METADATA RECORD IDENTIFIERS
_FRS_METADATA_RECORD_RAW_ACCELEROMETER                                       = const(0XE301)
_FRS_METADATA_RECORD_ACCELEROMETER                                           = const(0XE302)
_FRS_METADATA_RECORD_LINEAR_ACCELERATION                                     = const(0XE303)
_FRS_METADATA_RECORD_GRAVITY                                                 = const(0XE304)
_FRS_METADATA_RECORD_RAW_GYROSCOPE                                           = const(0XE305)
_FRS_METADATA_RECORD_GYROSCOPE_CALIBRATED                                    = const(0XE306)
_FRS_METADATA_RECORD_GYROSCOPE_UNCALIBRATED                                  = const(0XE307)
_FRS_METADATA_RECORD_RAW_MAGNETOMETER                                        = const(0XE308)
_FRS_METADATA_RECORD_MAGNETIC_FIELD_CALIBRATED                               = const(0XE309)
_FRS_METADATA_RECORD_MAGNETIC_FIELD_UNCALIBRATED                             = const(0XE30A)
_FRS_METADATA_RECORD_ROTATION_VECTOR                                         = const(0XE30B)
_FRS_METADATA_RECORD_GAME_ROTATION_VECTOR                                    = const(0XE30C)
_FRS_METADATA_RECORD_GEOMAGNETIC_ROTATION_VECTOR                             = const(0XE30D)
_FRS_METADATA_RECORD_PRESSURE                                                = const(0XE30E)
_FRS_METADATA_RECORD_AMBIENT_LIGHT                                           = const(0XE30F)
_FRS_METADATA_RECORD_HUMIDITY                                                = const(0XE310)
_FRS_METADATA_RECORD_PROXIMITY                                               = const(0XE311)
_FRS_METADATA_RECORD_TEMPERATURE                                             = const(0XE312)
_FRS_METADATA_RECORD_TAP_DETECTOR                                            = const(0XE313)
_FRS_METADATA_RECORD_STEP_DETECTOR                                           = const(0XE314)
_FRS_METADATA_RECORD_STEP_COUNTER                                            = const(0XE315)
_FRS_METADATA_RECORD_SIGNIFICANT_MOTION                                      = const(0XE316)
_FRS_METADATA_RECORD_STABILITY_CLASSIFIER                                    = const(0XE317)
_FRS_METADATA_RECORD_SHAKE_DETECTOR                                          = const(0XE318)
_FRS_METADATA_RECORD_FLIP_DETECTOR                                           = const(0XE319)
_FRS_METADATA_RECORD_PICKUP_DETECTOR                                         = const(0XE31A)
_FRS_METADATA_RECORD_STABILITY_DETECTOR                                      = const(0XE31B)
_FRS_METADATA_RECORD_PERSONAL_ACTIVITY_CLASSIFIER                            = const(0XE31C)
_FRS_METADATA_RECORD_SLEEP_DETECTOR                                          = const(0XE31D)
_FRS_METADATA_RECORD_TILT_DETECTOR                                           = const(0XE31E)
_FRS_METADATA_RECORD_POCKET_DETECTOR                                         = const(0XE31F)
_FRS_METADATA_RECORD_CIRCLE_DETECTOR                                         = const(0XE320)
_FRS_METADATA_RECORD_HEART_RATE_MONITOR                                      = const(0XE321)
_FRS_METADATA_RECORD_ARVR_STABILIZED_ROTATION_VECTOR                         = const(0XE322)
_FRS_METADATA_RECORD_ARVR_STABILIZED_GAME_ROTATION_VECTOR                    = const(0XE323)
_FRS_METADATA_RECORD_GYRO_INTEGRATED_ROTATION_VECTOR                         = const(0XE324)
_FRS_METADATA_RECORD_MOTION_REQUEST                                          = const(0XE325)

#   CONFIGURATION RECORD IDENTIFIERS
#   AGM is ACCELEROMETER, GYROSCOPE, AND MAGNETOMETER
#   SRA is SCREEN ROTATION ACCELEROMETER
_FRS_CONFIGURATION_RECORD_STATIC_CALIBRATION_AGM                             = const(0X7979)
_FRS_CONFIGURATION_RECORD_NOMINAL_CALIBRATION_AGM                            = const(0X4D4D)
_FRS_CONFIGURATION_RECORD_STATIC_CALIBRATION_SRA                             = const(0X8A8A)
_FRS_CONFIGURATION_RECORD_NOMINAL_CALIBRATION_SRA                            = const(0X4E4E)
_FRS_CONFIGURATION_RECORD_DYNAMIC_CALIBRATION                                = const(0X1F1F)
_FRS_CONFIGURATION_RECORD_MOTIONENGINE_POWER_MANAGEMENT                      = const(0XD3E2)
_FRS_CONFIGURATION_RECORD_SYSTEM_ORIENTATION                                 = const(0X2D3E)
_FRS_CONFIGURATION_RECORD_PRIMARY_ACCELEROMETER_ORIENTATION                  = const(0X2D41)
_FRS_CONFIGURATION_RECORD_GYROSCOPE_ORIENTATION                              = const(0X2D46)
_FRS_CONFIGURATION_RECORD_MAGNETOMETER_ORIENTATION                           = const(0X2D4C)
_FRS_CONFIGURATION_RECORD_AR_VR_STABILIZATION_ROTATION_VECTOR                = const(0X3E2D)
_FRS_CONFIGURATION_RECORD_AR_VR_STABILIZATION_GAME_ROTATION_VECTOR           = const(0X3E2E)
_FRS_CONFIGURATION_RECORD_SIGNIFICANT_MOTION_DETECTOR_CONFIGURATION          = const(0XC274)
_FRS_CONFIGURATION_RECORD_SHAKE_DETECTOR_CONFIGURATION                       = const(0X7D7D)
_FRS_CONFIGURATION_RECORD_MAXIMUM_FUSION_PERIOD                              = const(0XD7D7)
_FRS_CONFIGURATION_RECORD_SERIAL_NUMBER                                      = const(0X4B4B)
_FRS_CONFIGURATION_RECORD_ENVIRONMENTAL_SENSOR_PRESSURE_CALIBRATION          = const(0X39AF)
_FRS_CONFIGURATION_RECORD_ENVIRONMENTAL_SENSOR_TEMPERATURE_CALIBRATION       = const(0X4D20)
_FRS_CONFIGURATION_RECORD_ENVIRONMENTAL_SENSOR_HUMIDITY_CALIBRATION          = const(0X1AC9)
_FRS_CONFIGURATION_RECORD_ENVIRONMENTAL_SENSOR_AMBIENT_LIGHT_CALIBRATION     = const(0X39B1)
_FRS_CONFIGURATION_RECORD_ENVIRONMENTAL_SENSOR_PROXIMITY_CALIBRATION         = const(0X4DA2)
_FRS_CONFIGURATION_RECORD_ALS_CALIBRATION                                    = const(0XD401)
_FRS_CONFIGURATION_RECORD_PROXIMITY_SENSOR_CALIBRATION                       = const(0XD402)
_FRS_CONFIGURATION_RECORD_STABILITY_DETECTOR_CONFIGURATION                   = const(0XED85)
_FRS_CONFIGURATION_RECORD_USER_RECORD                                        = const(0X74B4)
_FRS_CONFIGURATION_RECORD_MOTIONENGINE_TIME_SOURCE_SELECTION                 = const(0XD403)
_FRS_CONFIGURATION_RECORD_GYRO_INTEGRATED_ROTATION_VECTOR_CONFIGURATION      = const(0XA1A2)


# -----------------------------------------------------------------------------
#   CONSTANTS FOR CLASS BNO085
#
#   These are constants related to our driver implementation
# -----------------------------------------------------------------------------
_SPI_ACTIVE                                                                 = const(0)          # SPI enable pin state
_SPI_INACTIVE                                                               = const(1)
_WAKE_ASSERT                                                                = const(False)
_WAKE_DEASSERT                                                              = const(True)

_HEADER_BUFFER_SIZE                                                         = const(4)
_CARGO_BUFFER_SIZE                                                          = const(272)        # Used to receive buffers that don't go to ISR. largest response is startup response.
_TIMESTAMP_BUFFER_SIZE                                                      = const(5)
_USER_RECORD_BUFFER_SIZE                                                    = const(4)

# Provides quaternion orientation
_GYRO_INTEGRATED_ROTATION_REPORT_BUFFER_SIZE                                = const(14)
_GYRO_INTEGRATED_ROTATION_REPORT_DATA_OFFSET                                = const(0)

# Alternate way to receive quaternion orientation
_ROTATION_REPORT_BUFFER_SIZE                                                = const(14 - 1)     # Subtract 1 because we read the first byte to identify the report type
_ROTATION_REPORT_DATA_OFFSET                                                = const(3)

# Alternate way to receive quaternion orientation
_GAME_ROTATION_REPORT_BUFFER_SIZE                                           = const(12 - 1)     # ibid
_GAME_ROTATION_REPORT_DATA_OFFSET                                           = const(3)

# Provides linear acceleration as xyz
_LINEAR_ACCELERATION_REPORT_BUFFER_SIZE                                     = const(10 - 1)     # ibid
_LINEAR_ACCELERATION_REPORT_DATA_OFFSET                                     = const(3)

# Provides angular velocity about xyz
_GYROSCOPE_CALIBRATED_REPORT_BUFFER_SIZE                                    = const(10 - 1)     # ibid
_GYROSCOPE_CALIBRATED_REPORT_DATA_OFFSET                                    = const(3)

# Provides pressure data from BME280 peripheral
_PRESSURE_REPORT_BUFFER_SIZE                                                = const(8 - 1)      # ibid
_PRESSURE_REPORT_DATA_OFFSET                                                = const(3)

_RESET_TIMEOUT                                                              = const(5000)       # (ms) - how long to wait for a reset (reboot) to be confirmed


# -----------------------------------------------------------------------------
#
#   CLASS BNO085
#
# -----------------------------------------------------------------------------
class BNO085():
    # -------------------------------------------------------------------------
    #   __INIT__()
    # -------------------------------------------------------------------------
    def __init__(self,
                 spi_object,
                 h_csn_pin_id,
                 ps0_wake_pin_id,
                 ps1_pin_id,
                 bootn_pin_id,
                 nrst_pin_id,
                 intn_pin_id,
                 verbose=False,
                 report_type=_REPORT_ROTATION_VECTOR,
                 report_interval=10000,                     # microseconds
                 interrupt_priority=6):

        # --
        # timing variables for profiling code that is commented out in ISR:
        self.isr_timer = 0
        self.isr_timer_max = 0
        self.isr_timer_avg = 0
        self.isr_counter = 0
        # --

        # pre-allocate a reference, so schedule() may be used in the ISR to call a function that
        # allocates heap in pseudo-real-time
        self.process_buffer_reference = self.process_buffer

        self.verbose = verbose
        self.spi = spi_object
        self.report_interval = report_interval

        # status booleans
        self.reset_confirmed = False
        self.sending_data = False
        self.buffers_in_use = False

        # buffer for reading and writing to user record
        self.user_record_buffer = bytearray(_USER_RECORD_BUFFER_SIZE)

        #
        #   CONFIGURE I/O PINS
        #
        self.spi_enable_pin = pyb.Pin(h_csn_pin_id, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)
        self.ps0_wake_pin   = pyb.Pin(ps0_wake_pin_id, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)
        self.ps1_pin        = pyb.Pin(ps1_pin_id, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)
        self.bootn_pin      = pyb.Pin(bootn_pin_id, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)                   # bootloader is entered if bootn low at reset
        self.nrst_pin       = pyb.Pin(nrst_pin_id, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)                    # active low
        self.intn_pin       = pyb.ExtInt(intn_pin_id, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, self.isr)

        nvic.nvic_set_prio(self.intn_pin.line() + 6, interrupt_priority)

        # Set ps0/wake and ps1 True to enable SPI on BNO085. After hard_reset(), ps0/wake
        # is then used to wake IMU chip from sleep
        self.ps0_wake_pin.value(True)
        self.ps1_pin.value(True)

        # If bootn is False after hard_reset(), IMU enters firmware update mode.
        self.bootn_pin.value(True)

        # Deactivate SPI until it's needed
        self.spi_disable()

        #
        #   SET UP SENSOR REPORTS AND BUFFERS
        #

        try:                                                        # report_type is a list, if a single int is passed, put it in a one-element list
            self.report_type = [item for item in report_type]
        except TypeError:
            self.report_type = [report_type]

        self.raw_quaternion_array = array.array('i', [0, 0, 0, 0])
        self.raw_gyroscope_array = array.array('i', [0, 0, 0])
        self.raw_acceleration_array = array.array('i', [0, 0, 0])
        self.raw_pressure_array = array.array('i', [0, 0])

        # create buffers to receive SPI sensor traffic
        self.header_buffer = bytearray(_HEADER_BUFFER_SIZE)
        self.cargo_buffer = bytearray(_CARGO_BUFFER_SIZE)      # only used for traffic that is not processed by ISR
        self.timestamp_buffer = bytearray(_TIMESTAMP_BUFFER_SIZE)
        self.report_ID_buffer = bytearray(1)
        self.report_ID = 0                                          # extracted value to use in if/then chains

        for entry in self.report_type:
            # create buffer for rotation report
            if entry == _REPORT_ROTATION_VECTOR:
                self.rotation_report_buffer = bytearray(_ROTATION_REPORT_BUFFER_SIZE)
                self.rotation_report_data_offset = _ROTATION_REPORT_DATA_OFFSET
            elif entry == _REPORT_GAME_ROTATION_VECTOR:
                self.rotation_report_buffer = bytearray(_GAME_ROTATION_REPORT_BUFFER_SIZE)
                self.rotation_report_data_offset = _GAME_ROTATION_REPORT_DATA_OFFSET
            elif entry == _REPORT_GYRO_INTEGRATED_ROTATION_VECTOR:
                self.rotation_report_buffer = bytearray(_GYRO_INTEGRATED_ROTATION_REPORT_BUFFER_SIZE)
                self.rotation_report_data_offset = _GYRO_INTEGRATED_ROTATION_REPORT_DATA_OFFSET
            # create buffer for gyroscope report
            elif entry == _REPORT_GYROSCOPE_CALIBRATED:
                self.gyroscope_report_buffer = bytearray(_GYROSCOPE_CALIBRATED_REPORT_BUFFER_SIZE)
                self.gyroscope_report_data_offset = _GYROSCOPE_CALIBRATED_REPORT_DATA_OFFSET
            # create buffer for acceleration report
            elif entry == _REPORT_LINEAR_ACCELERATION:
                self.acceleration_report_buffer = bytearray(_LINEAR_ACCELERATION_REPORT_BUFFER_SIZE)
                self.acceleration_report_data_offset = _LINEAR_ACCELERATION_REPORT_DATA_OFFSET
            # create buffer for pressure report
            elif entry == _REPORT_PRESSURE:
                self.pressure_report_buffer = bytearray(_PRESSURE_REPORT_BUFFER_SIZE)
                self.pressure_report_data_offset = _PRESSURE_REPORT_DATA_OFFSET
            else:
                print('BNO085 UNSUPPORTED RECORD TYPE REQUESTED')

        #
        #   REBOOT IMU
        #
        self.hard_reset()
        pyb.delay(200)

        #
        #   READ SENSOR METADATA
        #
        for entry in self.report_type:
            self.metadata_record = SensorMetadataRecord()
            if entry == _REPORT_ROTATION_VECTOR:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_ROTATION_VECTOR)
            elif entry == _REPORT_GAME_ROTATION_VECTOR:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_GAME_ROTATION_VECTOR)
            elif entry == _REPORT_GYRO_INTEGRATED_ROTATION_VECTOR:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_GYRO_INTEGRATED_ROTATION_VECTOR)
            elif entry == _REPORT_GYROSCOPE_CALIBRATED:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_GYROSCOPE_CALIBRATED)
            elif entry == _REPORT_LINEAR_ACCELERATION:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_LINEAR_ACCELERATION)
            elif entry == _REPORT_PRESSURE:
                self.request_sensor_metadata_record(_FRS_METADATA_RECORD_PRESSURE)
            time.sleep_ms(250)

        #
        #   INITIATE DATA STREAMING
        #
        # for i in range(5):
        for entry in self.report_type:
            self.begin_streaming(entry, self.report_interval)
            time.sleep_ms(250)

    # --------------------------------------------------------------------------
    #   BEGIN_STREAMING()
    # --------------------------------------------------------------------------
    def begin_streaming(self, report_id, report_period):
        '''Begin streaming data. Report period is in microseconds.'''

        if self.verbose:
            print()
            print('--------------------------------------------------')
            print('BNO085 INITIALIZING STREAMING FOR REPORT ID: ', hex(report_id))
            print('--------------------------------------------------')

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 21                                      # Payload length LSB (21 total bytes, includes header)
        self.header_buffer[1] = 0                                       # Payload length MSB
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        self.cargo_buffer[0] = _REPORT_SET_FEATURE_COMMAND
        self.cargo_buffer[1] = report_id
        self.cargo_buffer[2] = 0                                        # Feature flags
        self.cargo_buffer[3] = 0                                        # Change sensitivity LSB
        self.cargo_buffer[4] = 0                                        # Change sensitivity MSB
        self.cargo_buffer[5] = report_period & 0xFF                     # Report interval LSB (microseconds)
        self.cargo_buffer[6] = (report_period >> 8) & 0xFF
        self.cargo_buffer[7] = (report_period >> 16) & 0xFF
        self.cargo_buffer[8] = (report_period >> 24) & 0xFF             # Report interval MSB
        self.cargo_buffer[9] = 0                                        # Batch interval LSB
        self.cargo_buffer[10] = 0
        self.cargo_buffer[11] = 0
        self.cargo_buffer[12] = 0                                       # Batch interval MSB
        self.cargo_buffer[13] = 0                                       # Sensor-specific configuration word LSB
        self.cargo_buffer[14] = 0
        self.cargo_buffer[15] = 0
        self.cargo_buffer[16] = 0                                       # Sensor-specific configuration word MSB

        self.send_data()

    # --------------------------------------------------------------------------
    #   TARE()
    # --------------------------------------------------------------------------
    def tare(self, axes=0b111, basis=0):
        '''tare(axes=0b111, basis=0x0)
        axes is a bitmap with
            bit 0 = x
            bit 1 = y
            bit 2 = z
        basis determines which rotation vector to use as a basis for tare:
            0: Rotation vector
            1: Gaming rotation vector
            2: Geomagnetic rotation vector
            3: Gyro-integrated rotation vector
            4: ARVR-stabilized rotation vector
            5: ARVR-stabilized game rotation vector'''

        if self.verbose:
            print('-----------------------------')
            print('BNO085 REQUESTING SENSOR TARE')
            print('-----------------------------')
            print('axes =', axes)
            print('basis =', basis)
            print()

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 0x10                            # Payload length LSB (16 bytes)
        self.header_buffer[1] = 0x0                             # Payload length MSB
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        self.cargo_buffer[0] = _REPORT_COMMAND_REQUEST
        self.cargo_buffer[1] = 0x0                              # sequence number
        self.cargo_buffer[2] = _COMMAND_TARE
        self.cargo_buffer[3] = _SUBCOMMAND_TARE_NOW
        self.cargo_buffer[4] = axes
        self.cargo_buffer[5] = basis
        # bytes 6-11 are reserved

        self.send_data()

        while self.sending_data | self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.header_buffer[0] = 0x10                            # Payload length LSB (16 bytes)
        self.header_buffer[1] = 0x0                             # Payload length MSB
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        self.cargo_buffer[0] = _REPORT_COMMAND_REQUEST
        self.cargo_buffer[1] = 0x1                              # sequence number
        self.cargo_buffer[2] = _COMMAND_TARE
        self.cargo_buffer[3] = _SUBCOMMAND_PERSIST_TARE
        # bytes 4-11 are reserved

        self.send_data()

    # --------------------------------------------------------------------------
    #   SET_ORIENTATION(QUATERNION=[])
    # --------------------------------------------------------------------------
    '''Sets IMU orientation using quaternion. If no quaternion is supplied,
    defaults to [0.0, 0.0, -sqrt(2)/2, -sqrt(2)/2] which results in calculated
    pitch spanning -90 to 90 degrees when this orienation is set for hybrid knee.

    To reset to default orientation, call this function with
    [1.0, 0.0, 0.0, 0.0], or call reset_orientation()'''

    def set_orientation(self, quaternion=[0.0, 0.0, -.7071067811865476, -.7071067811865476]):
        if self.verbose:
            print('-----------------------------')
            print('BNO085 SETTING ORIENTATION')
            print('-----------------------------')
            print()

        w = self.float_to_q(quaternion[0], 14)
        x = self.float_to_q(quaternion[1], 14)
        y = self.float_to_q(quaternion[2], 14)
        z = self.float_to_q(quaternion[3], 14)

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 0x10                            # Payload length LSB (16 bytes)
        self.header_buffer[1] = 0x0                             # Payload length MSB
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        self.cargo_buffer[0] = _REPORT_COMMAND_REQUEST
        self.cargo_buffer[1] = 0x0                              # sequence number
        self.cargo_buffer[2] = _COMMAND_TARE
        self.cargo_buffer[3] = _SUBCOMMAND_SET_TARE_REORIENTATION
        self.cargo_buffer[4] = w & 0xFF
        self.cargo_buffer[5] = (w >> 8) & 0xFF
        self.cargo_buffer[6] = x & 0xFF
        self.cargo_buffer[7] = (x >> 8) & 0xFF
        self.cargo_buffer[8] = y & 0xFF
        self.cargo_buffer[9] = (y >> 8) & 0xFF
        self.cargo_buffer[10] = z & 0xFF
        self.cargo_buffer[11] = (z >> 8) & 0xFF

        self.send_data()

    # --------------------------------------------------------------------------
    #   RESET_ORIENTATION()
    # --------------------------------------------------------------------------
    def reset_orientation(self):
        self.set_orientation([1.0, 0.0, 0.0, 0.0])

    # --------------------------------------------------------------------------
    #   ENABLE()
    # --------------------------------------------------------------------------
    def enable(self):
        self.nrst_pin.high()

    # --------------------------------------------------------------------------
    #   DISABLE()
    # --------------------------------------------------------------------------
    def disable(self):
        self.nrst_pin.low()

    # --------------------------------------------------------------------------
    #   HARD_RESET()
    # --------------------------------------------------------------------------
    def hard_reset(self):

        if self.verbose:
            print()
            print('-------------------------------')
            print('BNO085 HARD RESET')

        self.disable()
        pyb.delay(1)
        self.enable()

        reset_timer = 0
        timeout = _RESET_TIMEOUT
        while (not self.reset_confirmed) and timeout:
            pyb.delay(1)
            reset_timer = reset_timer + 1
            timeout = timeout - 1
        if timeout:
            self.reset_confirmed = True
            if self.verbose:
                print('Reset confirmed after ', reset_timer, ' ms.')
        else:
            if self.verbose:
                print('Reset did not occur.')
        if self.verbose:
            print('-------------------------------')

    # --------------------------------------------------------------------------
    #   Q_TO_FLOAT_16()
    # --------------------------------------------------------------------------
    def q_to_float_16(self, input, q_point):
        '''BNO085 represents floats as 16-bit, Q-formatted numbers. The
        Q-point is specified for each type of datapoint, and can be
        determined from the FRS metadata reports. This function accepts two
        integers: the 16-bit Q-formatted number and the q-point. It returns a
        signed float.'''

        # 1. Convert input to signed int using two's complement
        # 2. Divide by 2^q_point

        sign_bit = input >> 15                              # 0->positive; 1->negative

        if sign_bit:
            return (input - 0x10000) / (2 ** q_point)       # 0x10000 = 1 << 16

        else:
            return input / (2 ** q_point)

    # --------------------------------------------------------------------------
    #   Q_TO_FLOAT_32()
    # --------------------------------------------------------------------------
    def q_to_float_32(self, input, q_point):
        '''BNO085 represents floats Q-formatted numbers. The Q-point is
        specified for each type of datapoint, and can be determined from the FRS
        metadata reports. This function accepts two integers: the 32-bit
        Q-formatted number and the q-point. It returns a signed float.'''

        # 1. Convert input to signed int using two's complement
        # 2. Divide by 2^q_point

        sign_bit = input >> 31                              # 0->positive; 1->negative

        if sign_bit:
            return (input - 0x100000000) / (2 ** q_point)   # 0x100000000 = 1 << 32

        else:
            return input / (2 ** q_point)

    # --------------------------------------------------------------------------
    #   FLOAT_TO_Q()
    # --------------------------------------------------------------------------
    def float_to_q(self, input, q_point):
        '''Represent a float as a 16-bit, Q-formatted int.'''

        if input < 0.0:
            return int(input * (2 ** q_point)) + 0x10000
        else:
            return int(input * (2 ** q_point))

    # --------------------------------------------------------------------------
    #   PROCESS_BUFFER()
    #   NOTE: The ISR processes data streamed from the sensors. All other cargo
    #   is processes here, which happens outside the ISR
    # --------------------------------------------------------------------------
    def process_buffer(self, _):

        number_of_bytes = (self.header_buffer[1] << 8) | self.header_buffer[0] - 4  # size of just the cargo buffer (subtracting 4 bytes for the header that's already been read)
        if number_of_bytes < 0:
            number_of_bytes = 0

        #
        #   CASES TO DO NOTHING
        #
        if (
            ((self.cargo_buffer[0] == _REPORT_COMMAND_RESPONSE)
                and (self.cargo_buffer[2] == _UNSOLICITED_INITIALIZE))
            or self.cargo_buffer[0] == 0xFF                                     # empty SPI sometimes interpreted as all 0xFF (on file save, reset, etc.)
        ):
            pass

        #
        #   CONFIRM DATA STREAM REQUEST
        #
        elif self.cargo_buffer[0] == _REPORT_GET_FEATURE_RESPONSE:
            if self.verbose:

                change_sensitivity = self.q_to_float_16((self.cargo_buffer[4] << 8) | self.cargo_buffer[3], 0)             # report-specific. TODO: extract from FRS report
                report_interval = (self.cargo_buffer[8] << 24) | (self.cargo_buffer[7] << 16) | (self.cargo_buffer[6] << 8) | self.cargo_buffer[5]
                batch_interval = (self.cargo_buffer[12] << 24) | (self.cargo_buffer[11] << 16) | (self.cargo_buffer[10] << 8) | self.cargo_buffer[9]
                configuration_word = (self.cargo_buffer[16] << 24) | (self.cargo_buffer[15] << 16) | (self.cargo_buffer[14] << 8) | self.cargo_buffer[13]

                print()
                print('----------------------------------------------------')
                print('BNO085 NOW STREAMING SENSOR DATA FOR REPORT ID:', hex(self.cargo_buffer[1]))
                print('----------------------------------------------------')
                print(('{:<25}0x{:02,x}').format('Feature Report ID:', self.cargo_buffer[1]))
                print(('{:<25}0b{:08,b}').format('Feature Flags:', self.cargo_buffer[2]))
                print(('{:<25}{}').format('Change Sensitivity:', change_sensitivity))
                print(('{:<25}{}').format('Report Interval:', report_interval))
                print(('{:<25}{}').format('Batch Interval:', batch_interval))
                print(('{:<25}{}').format('Configuration Word:', configuration_word))
                print()

        #
        #   PROCESS STARTUP MESSAGE
        #
        #   TAG-LENGTH-VALUE (see SH-2 SHTP Reference Manual [1000-3600], page 5; and Sensor Hub Transport Protocol
        #   [1000-3535], section 5.2)
        elif(
            self.cargo_buffer[0] == 0x0
            and self.header_buffer[1] != 0
        ):
            if self.verbose:
                advertisement_tag_name = {
                    0x00:  'Reserved',
                    0x01:  'GUID',
                    0x02:  'MaxCargoPlusHeaderWrite',
                    0x03:  'MaxCargoPlusHeaderRead',
                    0x04:  'MaxTransferWrite',
                    0x05:  'MaxTransferRead',
                    0x06:  'NormalChannel',
                    0x07:  'WakeChannel',
                    0x08:  'AppName',
                    0x09:  'ChannelName',
                    0x80:  'SHTP Version',
                    0x81:  'SHTP Report Lengths'                 # can also be SHTP over UART timeout? docs unclear
                }

                print()
                print('----------------------')
                print('BNO085 STARTUP MESSAGE')
                print('----------------------')

                i = 1
                while i < number_of_bytes:
                    tag_id = self.cargo_buffer[i]
                    length = self.cargo_buffer[i + 1]
                    if(                             # value is a string
                        tag_id == 8                 # AppName
                        or tag_id == 9              # ChannelName
                        or tag_id == 0x80           # GUID
                    ):
                        value = self.cargo_buffer[i + 2: i + 2 + length - 1].decode()
                        print(('{:<25}{}').format(advertisement_tag_name[tag_id], value))

                    elif(
                        tag_id == 1                 # GUID
                        or tag_id == 2              # MaxCargoPlusHeaderWrite
                        or tag_id == 3              # MaxCargoPlusHeaderRead
                        or tag_id == 4              # MaxTransferWrite
                        or tag_id == 5              # MaxTransferRead
                        or tag_id == 6              # NormalChannel
                        or tag_id == 7              # WakeChannel
                    ):
                        integer_value = 0
                        for j, byte in enumerate(self.cargo_buffer[i + 2: i + 2 + length]):
                            integer_value += byte << (j * 8)
                        value = str(integer_value)
                        print(('{:<25}{}').format(advertisement_tag_name[tag_id], value))

                    elif (tag_id == 0x81            # Report Lengths
                          and False):               # This is a lot of stuff to print out, even when self.verbose is True
                        print()
                        print(advertisement_tag_name[tag_id], end=':\n')
                        print('ID      bytes')
                        print('-------------')
                        output_string = '0x{:02x}:\t{:d}'
                        for j in range(length / 2):
                            print(output_string.format(self.cargo_buffer[i + 2 + (2 * j)], self.cargo_buffer[i + 2 + (2 * j) + 1]))
                    i = i + 2 + length

        #
        #   PROCESS FRS RECORD BEING
        #
        elif self.cargo_buffer[0] == _REPORT_FRS_READ_RESPONSE:
            data_length = (self.cargo_buffer[1] & 0b11110000) >> 4              # result will be either 1 or 2
            word_offset = (self.cargo_buffer[3] << 8) + self.cargo_buffer[2]    # value is in words
            status = self.cargo_buffer[1] & 0b1111
            FRS_type = (self.cargo_buffer[13] << 8) + self.cargo_buffer[12]

            #
            #   PROCESS A READ OPERATION FOR THE USER RECORD
            #
            if FRS_type == _FRS_CONFIGURATION_RECORD_USER_RECORD:
                self.clear_buffer(self.user_record_buffer)
                for i in range(data_length):
                    for j in range(4):
                        self.user_record_buffer[j] = self.cargo_buffer[4 + (4 * i) + j]

            #
            #   PROCESS A SENSOR METADATA RECORD
            #
            else:
                for i in range(data_length):
                    self.metadata_record.raw_report.append(bytearray(4))
                    for j in range(4):
                        self.metadata_record.raw_report[word_offset + i][3 - j] = self.cargo_buffer[4 + (4 * i) + j]
                if(
                    status == 3                                                     # record complete
                    and self.verbose
                ):
                    self.metadata_record.parse_record(FRS_type)

        #
        #   IF A WRITE REQUEST ACKNOWLEDGEMENT, SEND THE USER RECORD DATA
        #
        elif self.cargo_buffer[0] == _REPORT_FRS_WRITE_RESPONSE:

            status = self.cargo_buffer[1] & 0b1111

            if status == 4:                                     # write mode entered or ready

                self.clear_buffer(self.header_buffer)
                self.clear_buffer(self.cargo_buffer)

                self.header_buffer[0] = 12
                self.header_buffer[1] = 0x0
                self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
                self.header_buffer[3] = 0

                self.cargo_buffer[0] = _REPORT_FRS_WRITE_DATA
                self.cargo_buffer[1] = 0                        # reserved
                self.cargo_buffer[2] = 0                        # offset LSB
                self.cargo_buffer[3] = 0                        # offset MSB

                for i in range(len(self.user_record_buffer)):
                    self.cargo_buffer[4 + i] = self.user_record_buffer[i]

                self.send_data()

        #
        #   EMPTY ACKNOWLEDGEMENT
        #   happens after reset and occasionally when file operations are performed
        #
        elif (number_of_bytes <= 1):
            pass

        #
        #   ANY OTHER REPORT
        #
        else:
            if self.verbose:
                print('HEADER')
                print('-----------------------')
                print('[0]', hex(self.header_buffer[0]), '- Length LSB')
                print('[1]', hex(self.header_buffer[1]), '- Length MSB')
                print('[2]', hex(self.header_buffer[2]), '- Channel')
                print('[3]', hex(self.header_buffer[3]), '- Sequence number')
                print()

                print('CARGO [', number_of_bytes, ']')
                print('-----------------------')
                if number_of_bytes > 0:
                    for i, byte_value in enumerate(self.cargo_buffer[0:number_of_bytes]):
                        print(i, '\t', byte_value, '\t', hex(byte_value), '\t', chr(byte_value))
                print()

        self.buffers_in_use = False

    # --------------------------------------------------------------------------
    #   CLEAR_BUFFER()
    # --------------------------------------------------------------------------
    def clear_buffer(self, buffer):
        '''clear_buffer(buffer_to_be_cleared). NOTE: this is optimized for speed
        and cannot function in ISR. To change this, uncomment alternate
        algorithm in definition below.'''

        # buffer = b'\x00' * len(buffer)

        for i in range(len(buffer)):
            buffer[i] = 0x0

    # --------------------------------------------------------------------------
    #   SPI_ENABLE()
    # --------------------------------------------------------------------------
    def spi_enable(self):
        '''Set SPI enable pin to make bus active'''
        self.spi_enable_pin.value(_SPI_ACTIVE)

    # --------------------------------------------------------------------------
    #   SPI_DISABLE()
    # --------------------------------------------------------------------------
    def spi_disable(self):
        '''Set SPI enable pin to make bus inactive'''
        self.spi_enable_pin.value(_SPI_INACTIVE)

    # --------------------------------------------------------------------------
    #   REQUEST_SENSOR_METADATA_RECORD()
    # --------------------------------------------------------------------------
    def request_sensor_metadata_record(self, metadata_record_id):
        '''Request FRS report. Refer to SH-2 Reference Manual [1000-3625],
        page 44 .'''

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 12
        self.header_buffer[1] = 0x0
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        self.cargo_buffer[0] = _REPORT_FRS_READ_REQUEST

        self.cargo_buffer[4] = metadata_record_id & 0xFF
        self.cargo_buffer[5] = (metadata_record_id >> 8) & 0xFF

        self.send_data()

    # --------------------------------------------------------------------------
    #   REQUEST_USER_RECORD()
    # --------------------------------------------------------------------------
    def read_user_record_buffer(self):
        '''Request user record data be from host to hub sent. The record sent in
        reply is parsed in process_buffer()'''

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 12
        self.header_buffer[1] = 0x0
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        # cargo_buffer bytes 1-3, 6, and 7 are reserved
        self.cargo_buffer[0] = _REPORT_FRS_READ_REQUEST
        self.cargo_buffer[4] = _FRS_CONFIGURATION_RECORD_USER_RECORD & 0xFF
        self.cargo_buffer[5] = (_FRS_CONFIGURATION_RECORD_USER_RECORD >> 8) & 0xFF

        self.send_data()

    # --------------------------------------------------------------------------
    #   WRITE_USER_RECORD_BUFFER()
    # --------------------------------------------------------------------------
    def write_user_record(self):
        '''Request opportunity to send data to be stored in user record. Once
        acknowledgement is received, actual data transmission is processed in
        'process_buffer().'''

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        data_size = len(self.user_record_buffer) >> 2                                   # size in in 4-byte words

        self.header_buffer[0] = 12
        self.header_buffer[1] = 0x0
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        # cargo_buffer bytes 1, 6, and 7 are reserved
        self.cargo_buffer[0] = _REPORT_FRS_WRITE_REQUEST
        self.cargo_buffer[2] = data_size & 0xFF                                         # size LSB in 4-byte words
        self.cargo_buffer[3] = (data_size >> 8) & 0xFF                                  # size MSB in 4-byte words
        self.cargo_buffer[4] = _FRS_CONFIGURATION_RECORD_USER_RECORD & 0xFF
        self.cargo_buffer[5] = (_FRS_CONFIGURATION_RECORD_USER_RECORD >> 8) & 0xFF

        self.send_data()

    # --------------------------------------------------------------------------
    #   DELETE_USER_RECORD()
    # --------------------------------------------------------------------------
    def delete_user_record(self):
        '''Delete the user record entry on the bno085'''

        while self.buffers_in_use:
            pyb.delay(1)
        self.buffers_in_use = True

        self.clear_buffer(self.header_buffer)
        self.clear_buffer(self.cargo_buffer)

        self.header_buffer[0] = 12
        self.header_buffer[1] = 0x0
        self.header_buffer[2] = _CHANNEL_SENSOR_HUB_CONTROL
        self.header_buffer[3] = 0x0

        # cargo_buffer bytes 1, 6, and 7 are reserved
        self.cargo_buffer[0] = _REPORT_FRS_WRITE_REQUEST
        self.cargo_buffer[2] = 0                                                          # size LSB in 4-byte words
        self.cargo_buffer[3] = 0                                                          # size MSB in 4-byte words
        self.cargo_buffer[4] = _FRS_CONFIGURATION_RECORD_USER_RECORD & 0xFF
        self.cargo_buffer[5] = (_FRS_CONFIGURATION_RECORD_USER_RECORD >> 8) & 0xFF

        self.send_data()

    # --------------------------------------------------------------------------
    #   SEND_DATA()
    # --------------------------------------------------------------------------
    def send_data(self):
        '''Actual send commands are in the ISR. This method pulls the wake pin low, which causes the
        BNO085 to trigger the interrupt. It also sets a boolean so the ISR knows what to do.'''

        # The IMU will set nrst_pin active after being woken to signal it
        # can receive data. self.sending_data is used so the ISR knows it
        # should send data
        self.sending_data = True                        # for ISR to know to send the buffers
        self.ps0_wake_pin.value(_WAKE_ASSERT)      # waking the BNO085, which will then initiate the interrupt

    # --------------------------------------------------------------------------
    #   RECEIVE_DATA()
    # --------------------------------------------------------------------------
    @micropython.native
    def receive_data(self):
        # NOTE: this is called by the ISR and needs to be ISR-safe
        #
        # buffers are not cleared before being read, as this requires time
        # and buffer sizes are set to be the same as the amount being read, with
        # the exception of cargo_buffer, which is cleared each time it is
        # processed.

        header_buffer = self.header_buffer          # create local ref for speed
        self.spi_enable()
        self.spi.readinto(header_buffer)

        if(
            header_buffer[2] == _CHANNEL_INPUT_SENSOR_REPORTS
        ):
            self.spi.readinto(self.timestamp_buffer)
            self.spi.readinto(self.report_ID_buffer)
            self.report_ID = self.report_ID_buffer[0]
            report_ID = self.report_ID              # create local ref for speed

            if report_ID == _REPORT_GYROSCOPE_CALIBRATED:
                self.spi.readinto(self.gyroscope_report_buffer)
            elif report_ID == _REPORT_LINEAR_ACCELERATION:
                self.spi.readinto(self.acceleration_report_buffer)
            elif(
                report_ID == _REPORT_ROTATION_VECTOR
                or report_ID == _REPORT_GAME_ROTATION_VECTOR
            ):
                self.spi.readinto(self.rotation_report_buffer)
            elif report_ID == _REPORT_PRESSURE:
                self.spi.readinto(self.pressure_report_buffer)

        elif(
            header_buffer[2] == _CHANNEL_GYRO_ROTATION_VECTOR
        ):
            self.report_ID = _REPORT_GYRO_INTEGRATED_ROTATION_VECTOR
            self.spi.readinto(self.rotation_report_buffer)
        else:
            self.spi.readinto(self.cargo_buffer)

        self.spi_disable()

    # --------------------------------------------------------------------------
    #   ISR(LINE)
    # --------------------------------------------------------------------------
    @micropython.native
    def isr(self, line):
        if _REPORT_TIMING:
            tim0 = time.ticks_us()

        #
        #   CONFIRM A RESET
        #
        if not self.reset_confirmed:
            self.reset_confirmed = True         # in case this is a reset event

        #
        #   SEND DATA
        #
        # elif self.sending_data:
        #     micropython.schedule(self.process_sent_message_reference, 0)
        elif self.sending_data:                               # Unsuppress to retare the knee
            self.ps0_wake_pin.value(_WAKE_DEASSERT)
            self.spi_enable()
            self.spi.write(self.header_buffer)
            self.spi.write(self.cargo_buffer)
            self.spi_disable()
            self.sending_data = False
            self.buffers_in_use = False

        #
        #   RECEIVE DATA
        #
        else:
            if self.buffers_in_use:
                return                                          # BNO085 will try again by reasserting interrupt in 10ms
            self.receive_data()
            report_ID = self.report_ID                          # Create local reference for speed optimization
            channel_ID = self.header_buffer[2]                  # ibid

        #
        #   PROCESS SENSOR REPORTS
        #   All other received cargo is run through self.process_buffer(), which runs outside of ISR
        #
            if(
                channel_ID == _CHANNEL_INPUT_SENSOR_REPORTS
                or channel_ID == _CHANNEL_GYRO_ROTATION_VECTOR
            ):
                if(
                    report_ID == _REPORT_GYROSCOPE_CALIBRATED
                ):
                    a = self.raw_gyroscope_array
                    b = self.gyroscope_report_buffer
                    c = self.gyroscope_report_data_offset

                    a[0] = (b[1 + c] << 8) | b[0 + c]           # x
                    a[1] = (b[3 + c] << 8) | b[2 + c]           # y
                    a[2] = (b[5 + c] << 8) | b[4 + c]           # z
                elif(
                    report_ID == _REPORT_LINEAR_ACCELERATION
                ):
                    a = self.raw_acceleration_array
                    b = self.acceleration_report_buffer
                    c = self.acceleration_report_data_offset

                    a[0] = (b[1 + c] << 8) | b[0 + c]           # x
                    a[1] = (b[3 + c] << 8) | b[2 + c]           # y
                    a[2] = (b[5 + c] << 8) | b[4 + c]           # z
                elif(
                    report_ID == _REPORT_ROTATION_VECTOR
                    or report_ID == _REPORT_GAME_ROTATION_VECTOR
                    or report_ID == _REPORT_GYRO_INTEGRATED_ROTATION_VECTOR
                ):
                    a = self.raw_quaternion_array
                    b = self.rotation_report_buffer
                    c = self.rotation_report_data_offset

                    a[0] = (b[7 + c] << 8) | b[6 + c]           # w (real)
                    a[1] = (b[1 + c] << 8) | b[0 + c]           # x (i)
                    a[2] = (b[3 + c] << 8) | b[2 + c]           # y (j)
                    a[3] = (b[5 + c] << 8) | b[4 + c]           # z (k)
                elif(
                    report_ID == _REPORT_PRESSURE
                ):
                    a = self.raw_pressure_array
                    b = self.pressure_report_buffer
                    c = self.pressure_report_data_offset

                    a[0] = (b[1 + c] << 8) | b[0 + c]
                    a[1] = (b[3 + c] << 8) | b[2 + c]

            else:
                self.buffers_in_use = True
                micropython.schedule(self.process_buffer_reference, 0)

        if _REPORT_TIMING:
            tim1 = time.ticks_us()
            self.isr_timer = time.ticks_diff(tim1, tim0)
            self.isr_counter += 1
            if self.isr_timer > self.isr_timer_max:
                self.isr_timer_max = self.isr_timer
            self.isr_timer_avg = (self.isr_timer_avg * (self.isr_counter - 1) + self.isr_timer) // self.isr_counter

    def reset_isr_timing(self):
        self.isr_timer = 0
        self.isr_timer_max = 0
        self.isr_timer_avg = 0
        self.isr_counter = 0

    def report_timing(self):
        print('ISR TIMING:\t', self.isr_timer_avg, '(avg)\t', self.isr_timer_max, '(max)')


# -----------------------------------------------------------------------------
#
#   CLASS SENSORMETADATARECORD
#
# -----------------------------------------------------------------------------
class SensorMetadataRecord():
    def __init__(self):

        # The raw report consists of a minimum of 11 4-byte words
        self.raw_report = []
        self.sensor_type_ID = 0

    def word_to_int(self, word_index):
        '''Converts a word in the raw report to an integer.'''
        return_value = 0
        for i in range(4):
            return_value += self.raw_report[word_index][3 - i] << (8 * i)
        return return_value

    def halfword_to_int(self, word_index, offset):
        '''Converts half a word in the raw report to an integer. Offset is 0 or 1.'''
        return_value = 0
        for i in range(2):
            return_value += self.raw_report[word_index][3 - 2 * offset - i] << (8 * i)
        return return_value

    def int_to_signed_int(self, input):
        '''Converts 16-bit unsigned integer to signed integer'''
        if not(input >> 15):                # if MSb is zero, return value as-is
            return input
        elif input == 0b1 << 15:            # limiting case
            return -1 * (0xFFFF >> 1)
        else:                               # process input to a negative number using two's complement
            return -1 * (((0xFFFF >> 1) ^ input) + 1)

    def report_value(self, name, value):
        print('{:.<40}{:}'.format(name.upper(), value))

    def parse_record(self, sensor_type_ID):
        print()
        self.sensor_type_ID = sensor_type_ID
        self.version = self.word_to_int(0)
        self.motion_engine_version = self.version & 0xFF
        self.motion_hub_version = (self.version >> 8) & 0xFF
        self.sensor_hub_version = (self.version >> 16) & 0xFF
        self.revision = self.halfword_to_int(3, 1)
        self.FIFO_reserved_count = self.halfword_to_int(5, 1)
        self.FIFO_maximum_count = self.halfword_to_int(5, 0)
        self.minimum_period = self.word_to_int(4)
        self.vendor_ID_length = self.halfword_to_int(6, 1)
        self.batch_buffer_bytes = self.halfword_to_int(6, 0)
        self.q_point_1 = self.int_to_signed_int(self.halfword_to_int(7, 0))
        self.q_point_2 = self.int_to_signed_int(self.halfword_to_int(7, 1))
        self.q_point_3 = self.int_to_signed_int(self.halfword_to_int(8, 1))
        self.sensor_specific_metadata_length = self.halfword_to_int(8, 0) >> 2          # number of words
        self.maximum_period = self.word_to_int(9)

        if self.sensor_specific_metadata_length > 0:
            self.sensor_specific_metadata = bytearray(0)
            for word in self.raw_report[10: 10 + self.sensor_specific_metadata_length]:
                for i in range(4):
                    self.sensor_specific_metadata.append(word[i])
        else:
            self.sensor_specific_metadata = ''

        self.vendor_ID = str()
        for word in self.raw_report[10 + self.sensor_specific_metadata_length:]:
            for i in range(4):
                self.vendor_ID += chr(word[3 - i])

        self.print_formatted_report()

    def print_formatted_report(self):
        print('--------------------------')
        print('BNO085 FRS METADATA REPORT')
        print('--------------------------')
        self.report_value('sensor type ID', hex(self.sensor_type_ID))
        self.report_value('motion engine version', hex(self.motion_engine_version))
        self.report_value('motion hub version', hex(self.motion_hub_version))
        self.report_value('sensor hub version', hex(self.sensor_hub_version))
        self.report_value('revision', self.revision)
        self.report_value('FIFO_reserved_count', self.FIFO_reserved_count)
        self.report_value('FIFO_maximum_count', self.FIFO_maximum_count)
        self.report_value('minimum_period', self.minimum_period)
        self.report_value('vendor_ID_length', self.vendor_ID_length)
        self.report_value('batch_buffer_bytes', self.batch_buffer_bytes)
        self.report_value('q_point_1', self.q_point_1)
        self.report_value('q_point_2', self.q_point_2)
        self.report_value('q_point_3', self.q_point_3)
        self.report_value('sensor_specific_metadata_length', self.sensor_specific_metadata_length)
        self.report_value('maximum_period', self.maximum_period)
        self.report_value('sensor_specific_metadata', self.sensor_specific_metadata)
        self.report_value('vendor id', self.vendor_ID)

    def print_raw_report(self):
        for i, word in enumerate(self.raw_report):
            print('[', i, ']\t\t', end='')
            for single_byte in word:
                print('0x{:02X} '.format(single_byte), end='')
            print()
