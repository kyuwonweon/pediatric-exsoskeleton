#############################################################################
# @file       mode_switch.py
#
# @details    Code for evaluating the status of the mode switch soon after
#             boot to determine the device.
#
# @author     Frank Ursetta
#############################################################################

#
#   IMPORTS
#

import machine                                      # type: ignore
import sys

from config import PROCESSOR


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
#   class ModeSwitch()
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class ModeSwitch():
    '''
    ModeSwitch() is used to determine the device type based on the position of
    the mode switch.

    Parameters:
        None

    Returns:
        None

    Provides:
        Class attributes:
            id_from_position_dict (dict): Dictionary to map the switch position
            to the device ID.

            name_from_id_dict (dict): Dictionary to map the device ID to the
            device name.
        
        Instance attributes:
            device_id (int): The device ID based on the mode switch position.

            device_name (str): The device name based on the device ID.

        NOTE: Following power up, once instantiated, update() should be called a
        second time after a delay to allow the switch to settle.
    '''

    id_from_position_dict = {
        0: 0,                   # UNKNOWN
        1: 1,                   # HYBRID KNEE
        2: 1,                   # HYBRID KNEE
        3: 2,                   # POLYCENTRIC ANKLE
        4: 3,                   # POLYCENTRIC ANKLE ONLY
        5: 0,                   # UNKNOWN
        6: 0,                   # UNKNOWN
        7: 0,                   # UNKNOWN
        8: 8,                   # FOX_KNEE
        9: 0                    # UNKNOWN
    }

    name_from_id_dict = {
        0: 'UNKNOWN',
        1: 'HYBRID KNEE',
        2: 'POLYCENTRIC ANKLE',
        3: 'POLYCENTRIC ANKLE ONLY',
        8: 'FOX KNEE'
    }

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   __init__()
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def __init__(self):

        if PROCESSOR == 'STM32H747':
            self.switch_pin = machine.ADC(machine.Pin.cpu.C3_C)     # Portenta
        elif PROCESSOR == 'STM32F767':
            self.switch_pin = machine.ADC('W43')                    # Pyboard

        self.update()

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   update()
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def update(self):

        #
        # Read the switch position and determine the device
        #

        switch_reading = self.switch_pin.read_u16() >> 4

        if switch_reading > 3800: switch_position = 0
        elif switch_reading > 2900: switch_position = 4
        elif switch_reading > 2500: switch_position = 2
        elif switch_reading > 2200: switch_position = 1
        elif switch_reading > 1900: switch_position = 6
        elif switch_reading > 1800: switch_position = 5
        elif switch_reading > 1600: switch_position = 3
        elif switch_reading > 1400: switch_position = 7
        elif switch_reading > 1100: switch_position = 8
        elif switch_reading > 800: switch_position = 9
        else: switch_position = -1

        self.switch_reading = switch_reading
        self.switch_position = switch_position
        self.device_id = self.id_from_position_dict[self.switch_position]
        self.device_name = self.name_from_id_dict[self.device_id]
