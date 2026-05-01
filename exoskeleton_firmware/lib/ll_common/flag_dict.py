import array
import gc

print('IMPORTING FLAG DICTIONARY')

FLAG_DICT = {
    'CRITICAL_BATTERY'                  : False,
    'LARGE_MOTOR_STATE'                 : False,        # Indicates if the main large motor is driving or stopped. It is dependant on receiving a positive non-zero stiffness from CAPS.
    'DYNAMIC_BRAKE_STATE'               : False,        # Indicates when the dynamic brake should be engaged. This is only applicable to the hybrid knee.
    'LARGE_MOTOR_DRIVE_FLAG'            : False,        # Flag to indicate if the large motor should be driven.
    'STREAM_FLAG'                       : False,        # Flag to indicate the device is streaming.
    'CAPS_STOP_STREAM_FLAG'             : False,        # Flag to indicate if a stop streaming message has been received in can_class.py
    'CAPS_START_STREAM_FLAG'            : False,        # Flag to indicate if a start streaming message has been received in can_class.py
    'CAPS_TIMER_FLAG'                   : False,        # Flag to track when the caps timer tracking should start (only upon streaming)
    'CAPS_REPLY_FLAG'                   : False,        # Flag to track if CAPS has replied to the timer message
    'CAPS_SLOW_PING_FLAG'               : False,        # Flag to indicate if CAPS replies are slow
    'WATCHDOG_CAPS_SENDING'             : False,        # to check if we're receiving impedance parameters from CAPS'
    'WATCHDOG_CAPS_ZERO_IMP_RECEIVED'   : True,         # to check if we received a zero impedance message or not
    'LOW_BATTERY_FLAG'                  : False,        # Flag to indicate if the battery is low
    'CAPS_BUZZER_SCHEDULED_FLAG'        : False,        # Flag to indicate if a CAPS buzzer is scheduled
    'REHOME_MAIN_JOINT_ENCODER_FLAG'    : False,        # Flag to indicate the CAN message to home the main joint encoder has been received
    'ZERO_IMU_FLAG'                     : False,        # Flag to indicate the CAN message to zero the IMU has been received
    'INVERT_SHANK_THIGH_FLAG'           : False,        # Flag to indicate the CAN message to invert the shank/thigh has been received
    'ANKLE_OFFSET_FLAG'                 : False,        # Flag to indicate the CAN message to set the ankle offset has been received
    'ACTIVATING_EVEREST_FLAG'           : False,        # Flag to indicate the that the Everest is being activated
    'DEACTIVATING_EVEREST_FLAG'         : False,        # Flag to indicate the that the Everest is being deactivated
    'EVEREST_FAILURE_FLAG'              : False,        # Flag to indicate the that the Everest has failed
    'EVEREST_OPERATION_IS_ENABLED'      : False,        # Flag to indicate the that the Everest is enabled
    'CAPS_STOP_STREAM_LEG_STIFF_FLAG'   : False,        # Flag to indicate when CAPS has been stopped but the knee should remain stiff
    'CAN_RESTART_FLAG'                  : False,        # Flag to indicate that the CAN bus has been restarted
    'RESET_CONTROLLER_FLAG'             : False,        # Flag to indicate the CAN message to reset the controller has been received
    'MAIN_JOINT_ENCODER_HOMED_FLAG'     : True,         # Flag to indicate the main joint has been homed
    'FTS_STREAM_FLAG'                   : True,         # Flag to indicate FTS CAN streaming is active (independent of STREAM_FLAG)
    'STO_ALARM_ENABLED'                 : False,        # Flag to indicate the STO lines have started transitioning and everest cyclic shouldn't run until alarm clock has finished
    'ZERO_FTS_FLAG'                     : False,        # Flag to indicate the CAN message to zero the FTS has been received
    'FTS_SPIKE_FLAG'                    : False         # Flag to indicate when a spike in the FTS readings has been detected
}

gc.collect()