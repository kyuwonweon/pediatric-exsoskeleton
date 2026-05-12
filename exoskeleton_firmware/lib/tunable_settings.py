#############################################################################
# @file       tunable_settings.py
#
# @details    This file is contains settings that are tunable
#
# @author     Frank Ursetta
#############################################################################
#
# Absolute encoders flexion and extension values for main knee joint
# reading from joint_encoder @ flexion limit.
# NOTE: These 2 values must be updated if abs encoder is reinstalled.

import micropython                          # type: ignore

print('IMPORTING TURNABLE SETTINGS')

ENC_LIMIT_FLEX = micropython.const(2234)
ENC_LIMIT_EXT = micropython.const(3463)

DEVICE_NAME                         = 'FOX_LEG_TAILS'
