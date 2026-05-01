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

import pyb  # type: ignore
import micropython  # type: ignore
import machine  # type: ignore
from uctypes import addressof  # type: ignore

from lib import config

from lib.ll_common import mode_switch
from lib.ll_common import assembler_functions
from lib.ll_common import BNO085
from lib.ll_common import routing
from lib.ll_common import upy_spi
from lib.ll_common.param_dict import PARAM_DICT
from lib.ll_common.flag_dict import FLAG_DICT



class ImuStreamer:
    def __init__(self):

        self.config = config
        self.assembler_functions = assembler_functions
        self.BNO085 = BNO085
        self.routing = routing
        self.upy_spi = upy_spi
        self.PARAM_DICT = PARAM_DICT
        self.FLAG_DICT = FLAG_DICT


        self.selector = mode_switch.selector  # defined in boot.py
        self.device_parameters = (
            self.config.DeviceParameters(self.selector.device_id)
        )

        self.components = self.config.Board().get_components()

        self.timer_imu = pyb.Timer(
            self.device_parameters.TIMER_IMU['ID'],
            freq=self.device_parameters.TIMER_IMU['FREQ']
        )

        self.spi_instances = [None] * (4 + 1)

        self.setIMUVariables()
        self.imu.set_orientation()
        self.imu.tare()



    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Define IMU related variables
    def setIMUVariables(self):
        if self.components.IMU.is_enabled():

            # IMU arrays
            self.imu_data = array.array('f',
                                        [0, 0, 0, 0, 0, 0, 0, 0])  # [thigh, shank, ax, ay, az, gyrox, gyroy, gyroz]
            self.imu_qpoint = array.array('f', [2 ** 8, 2 ** 9])

            # Values required to calculate the arctan2 to convert the IMU values into thigh and shank angles.
            self.arc_tan2_taylor_coefs = array.array(
                'f',
                [
                    0,  # ┌        ┐
                    0.3333333,  # │ TAYLOR │
                    0.2,  # │ SERIES │
                    0.142857,  # │ PARAMS │
                    0.111111,  # └        ┘
                    57.2958,  # 180 / pi
                    (1 / (1 << 14)),  # formatting value
                    2.0,  # used to evaluate if -1 < angle < 1
                    0.0,  # save output of preliminary calc.
                    1.0,  # ¯\_(ツ)_/¯
                    1.570796,  # pi / 2
                    0,  # numerator of val
                    0  # denominator of val
                ]
            )

            # SPI speed depends on processor being used
            if self.device_parameters.PROCESSOR == 'STM32H747':  # Portenta
                baudrate = 12000000  # 12MHz (scales to 6.25MHz)
            elif self.device_parameters.PROCESSOR == 'STM32F767':  # Pyboard SF6
                baudrate = 4000000  # 4MHz (scales to 3.375MHz)

            # self.spi_instances[components.IMU.SPI.bus_number] = pyb.SPI(
            self.spi_instances[self.components.IMU.SPI.bus_number] = self.upy_spi.upySPI(
                self.components.IMU.SPI.bus_number
            )
            self.spi_instances[self.components.IMU.SPI.bus_number].init(
                mode=pyb.SPI.MASTER,
                baudrate=baudrate,
                polarity=1,
                phase=1,
                bits=8,
                firstbit=pyb.SPI.MSB
            )

            print('BNO SPI:')
            print(self.spi_instances[self.components.IMU.SPI.bus_number])

            self.interrupt_priorities = 5

            self.imu = self.BNO085.BNO085(
                self.spi_instances[self.components.IMU.SPI.bus_number],
                self.components.IMU.SPI.chip_select,
                self.components.IMU.protocol_select_pin_0,
                self.components.IMU.protocol_select_pin_1,
                self.components.IMU.bootloader_mode_select,
                self.components.IMU.reset,
                self.components.IMU.data_is_ready,
                report_interval=20000,  # microseconds (5000 us is 200Hz) (20000 us is 50Hz)
                report_type=[
                    0x05,  # BNO085._REPORT_ROTATION_VECTOR,
                    0x04,  # BNO085._REPORT_LINEAR_ACCELERATION,
                    0x02,  # BNO085._REPORT_GYROSCOPE_CALIBRATED,
                    0x0A  # BNO085._REPORT_PRESSURE (will default to 44000 us minimum period)
                ],
                verbose=False,
                interrupt_priority=self.interrupt_priorities)

            self.PARAM_DICT['BAR_BYTE_VALS'][0] = self.imu.q_to_float_32(
                (self.imu.raw_pressure_array[1] << 16) + self.imu.raw_pressure_array[0], 20)

        else:
            print("  ... Skipping 'IMU'")
            _BNO_NRST_PIN = 'W25'  # this should be defined under bno085 in the master self.config system
            self.nrst_pin = pyb.Pin(_BNO_NRST_PIN, pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_UP)  # active low
            self.nrst_pin.value(False)


    def reset_orientation(self):
        self.imu.set_orientation()
        self.imu.tare()


    def get_roll_pitch_yaw(self):
        # Convert byte values to floats
        w = self.imu.q_to_float_16(self.imu.raw_quaternion_array[0], 14)
        x = self.imu.q_to_float_16(self.imu.raw_quaternion_array[1], 14)
        y = self.imu.q_to_float_16(self.imu.raw_quaternion_array[2], 14)
        z = self.imu.q_to_float_16(self.imu.raw_quaternion_array[3], 14)

        # Compute norm
        norm = math.sqrt(w * w + x * x + y * y + z * z)

        # Avoid division by zero
        if norm > 0:
            w /= norm
            x /= norm
            y /= norm
            z /= norm

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / math.pi

        R22 = (1 - 2*(x*x + y*y))
        R22 = max(-1.0, min(1.0, R22))  #
        sag_angle = math.asin(R22) * 180.0 /math.pi
        # sag_angle = -sag_angle +90

        arcsin_argument = 2 * (w * y - z * x)
        arcsin_argument = max(-1.0, min(1.0, arcsin_argument))  #
        pitch = math.asin(arcsin_argument) * 180 / math.pi
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / math.pi

        return sag_angle

