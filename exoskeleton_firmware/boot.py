import os
import sys
import pyb                              # type: ignore
import micropython                      # type: ignore
from micropython import const           # type: ignore

micropython.alloc_emergency_exception_buf(150)


_LL_DEVICE = const(True)
# _LL_DEVICE = const(False)

pyb.country('US')                       # ISO 3166-1 Alpha-2 code, eg US, GB


processor = None
if sys.implementation._machine == 'Arduino Portenta H7 with STM32H747':
    processor = 'STM32H747'
else:
    processor = 'STM32F767'

'''
From first powering on a device with an Everest servo, the two STO lines must
alway be in agreement, and change together within a ~1 second window. sto_1 is
controlled directly. sto_2 is the inverse of the dynamic braking board fet
enable pin. The startup process takes longer than this window to get through
the initialization of these pins. To ensure they don't spend too long in
differing states, the pin states are set here and then the pin objects deleted
(as they'll be replaced during the proper initialization sequence).
'''

# -----------------------------------------------------------------------------
#
# Import mode switch to determine what kind of device this is. Set STO pins for
# Everest devices.
#
# -----------------------------------------------------------------------------
if _LL_DEVICE:
    from lib.ll_common import mode_switch

    selector = mode_switch.ModeSwitch()
    pyb.delay(100)              # Allow mode switch to settle after reboot.
    selector.update()           # Re-read the mode switch position.
    mode_switch.selector = selector

    if (
        selector.device_id == 1         # Hybrid knee
        or selector.device_id == 2      # Polycentric ankle
        or selector.device_id == 8      # Fox knee
    ):
        # -------------------------------------------------------------------------
        #   STM H747 (includes Portenta H7)
        # -------------------------------------------------------------------------
        if processor == 'STM32H747':

            print("Booting for Portenta H7...")

            pyb.freq(400000000)

            sto_1 = pyb.Pin('J2_68', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_NONE)
            dbb_enable = pyb.Pin('J2_65', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_DOWN)

            sto_1(1)
            dbb_enable(0)

        # -------------------------------------------------------------------------
        #   STM F767 (includes Pyboard SF6)
        # -------------------------------------------------------------------------
        elif processor == 'STM32F767':

            print("Booting for Pyboard SF6...")

            pyb.freq(216000000)

            sto_1 = pyb.Pin('W49', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_NONE)
            dbb_enable = pyb.Pin('W54', mode=pyb.Pin.OUT, pull=pyb.Pin.PULL_DOWN)

            sto_1(1)
            dbb_enable(0)

        # delete the pin objects. They'll be properly instantiated later:
        del sto_1
        del dbb_enable

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
#   Configure remaining parameters
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
try:
    sd = pyb.SDCard()                   # Calling pyboard SD card sd
    os.mount(sd, '/sd')                 # Mounting SD card on pyboard
    sd_mounted = True
    print('SD Card mounted')
except Exception as e:
    sd_mounted = False
    if processor == 'STM32F767':
        print('!! SD Card not mounted !!')
    pass

# Enabling USB with both VCP (virtual COM port) interface and MSC (mass storage
# device) interface:
if sd_mounted:
    pyb.usb_mode('VCP+MSC', port=1, msc=(pyb.Flash(), pyb.SDCard()))
else:
    pyb.usb_mode('VCP+MSC', port=1, msc=(pyb.Flash(),))

RED_LED = pyb.LED(1)                    # Instantiate red LED
RED_LED.on()                            # Turn on powerup LED

pyb.main('main.py')
