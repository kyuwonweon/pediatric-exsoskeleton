import array
import gc

print('IMPORTING PARAM DICTIONARY')

PARAM_DICT = {
    'ACTIVE_STATE'                      : array.array('f',[0.0]),
    'ANKLE_K_PLACEHOLDER'               : array.array('f',[0.0]),
    'ANKLE_B_PLACEHOLDER'               : array.array('f',[0.0]),
    'ANKLE_EQ_PLACEHOLDER'              : array.array('f',[0.0]),
    'ANKLE_NEUTRAL_OFFSET'              : array.array('f',[0.0]),           
    'ENC_DEG_COEF'                      : array.array('f',[0.0]),           # Value used as offset in mapping encoder counts to degrees
    'ENC_DEG_OFFSET'                    : array.array('f',[0.0]),           # Value that maps encoder counts to degrees
    'JOINT_ANGLE'                       : array.array('f',[0.0]),           # Angle in degrees of the main joint
    'BATTERY_VOLTS'                     : array.array('f',[1000.0]),        # Battery voltage received from everest
    'EVEREST_ERROR'                     : 0,                                # Error value from everest
    'CAPS_COMMUNICATION_VALUE'          : array.array('f',[0.0]),           # Variable used to check timing of communication with CAPS
    'BRAKE_PERCENTAGE'                  : array.array('f',[0.0]),           # Braking percentage as received from CAPS - This is not necessarily the braking applied which is dependant on state
    'BRAKE_ANGLE'                       : array.array('f',[120.0]),         # Angle from CAPS at which to start applying braking
    'BRAKE_RATE'                        : array.array('f',[-1000.0]),       # Rate from CAPS at which to start applying braking
    'BRAKE_PERCENTAGE_CAPS'             : array.array('f',[0.0]),           # Setting braking percentage for output to CAPS
    'EVEREST_KPP'                       : 0.0,                              # NOV.POSITION_LOOP_KP
    'EVEREST_KPV'                       : 0.0,                              # NOV.VELOCITY_LOOP_KP
    'EVEREST_KPT'                       : 0.0,                              # NOV.TORQUE_LOOP_KP
    'EVEREST_SET_POINT'                 : 0,                                # NOV.POSITION_SET_POINT (counts)
    'EVEREST_TORQUE_BIAS'               : 0.0,                              # NOV.TORQUE_LOOP_INPUT_OFFSET
    'DYNAMIC_BRAKE_PERCENTAGE'          : array.array('f',[0.0]),           # Braking percentage as calculated by the dynamic braking function
    'APPLIED_BRAKE_PERCENTAGE'          : 0,                                # The actual braking percentage being applied to the braking circuit
    'LARGE_MOTOR_REFERENCE_CURRENT'     : array.array('f',[0.0]),           # Array to hold the reference current from the everest for the large motor
    'LARGE_MOTOR_CURRENT_COMMAND'       : array.array('f',[0.0]),           # the value of torque to be commanded to the large motor driver
    'LARGE_MOTOR_TRANSMISSION_RATIO'    : array.array('f',[1.0]),           # This is the calculated transmission ratio for the large motor
    'SUBJECT_BUTTON_STATE'              : array.array('f',[0.0]),           # Value to indicate the safety switch state - used to initialize stair climbing
    'MAIN_JOINT_ENCODER_VALUE'          : array.array('f',[0.0]),           # main joint (absolute) encoder value
    'ENDSTOP_LIMIT_ARRAY'               : array.array('f',[0.0, 0.0]),      # Array used to pass to assembler eq_offset_and_limiter for protection checking
    'FOB_VAL_ARRAY'                     : array.array('f', [0.0]),          # Array for saving the key fob value'
    'FOB_CALC_ARRAY'                    : array.array('f', [100.0, 5.0]),   # Used for scaling the key fob value correctly
    'BAR_BYTE_VALS'                     : array.array('f', [0.0, 0.0]),     # Array to hold output values from the barometer. The first value is the startup barometer reading, the second is the current barometer reading
    'CRITICAL_BATTERY_BUZZER_COUNTER'   : 800,                              # Starting frequency for battery buzzer
    'CRITICAL_BATTERY_BUZZER_THRESHOLD' : array.array('f', [99.0]),         # Voltage threshold for critical battery buzzer; this value will correct itself based on the battery voltage at startup
    'JOINT_SPEED_ARRAY'                 : array.array('f', [0]),            # the main joint speed calculated from the incremental encoder velocity and transmission array. Only applicable to large motor
    'JOINT_SPEED_PEAK_ARRAY'            : array.array('f', [0]),            # the main joint speed peak. Only applicable to large motor #notused?
    'CONTROLLER_TIMER'                  : 0,                                # the duration of the controller loop
    'MAX_CONTROLLER_TIMER'              : 0,                                # the maximum duration of the controller loop
    'WIFI_COUNTER'                      : 0,                                # Used to track and stagger messages to CAPS
    'CAPS_TICK_TIME'                    : 0,                                # Used to track the time it takes to send a message to CAPS and receive a reply
    'CAPST1'                            : 0,                                # the time for caps message sending
    'CAPST2'                            : 0,                                # the time for caps message sending
    'CAPS_COMM_CHECK_COUNTER'           : 0,                                # Value to control the timing of the caps communicaiton check - this is incremented at the end of main.py
    'CAPS_COM_TIMER'                    : 0,                                # The time for caps communication interrupt
    'SEND_MESSAGE_TIMER'                : 0,                                # The time for sending a message
    'MAX_IMU_TIMER'                     : 0,                                # The max time it takes to read the imu
    'WATCHDOG_COUNTER'                  : 0,                                # watchdog timer, increments if timed-out
    'WATCHDOG_IMP_MESSAGE'              : 0,                                # the last impedance parameter payload from either CAPS2 or CAPS6
    'WATCHDOG_TIME_PASSED'              : 0,                                # how much time has passed since last impedance parameter message was received
    'PRINT_TIME_INTERVAL'               : 0,                                # Time between print statements
    'FREE_TIME_0'                       : 0,                                # the free time
    'FREE_TIME_1'                       : 0,                                # the free time
    'FREE_TIME_TOTAL'                   : 0,                                # The total free time
    'ALARM_START_TIME'                  : 0,                                # The time when the alarm starts
    'MAIN_LOOP_START_TIME'              : 0,                                # The time when the main loop starts
    'ALTITUDE'                          : 0,                                # The altitude in meters based on the barometer reading
    'ISR_TIMER0'                        : 0,                                # The first timer for the interrupt
    'ISR_TIMER1'                        : 0,                                # The second timer for the interrupt
    'ISR_TIME'                          : 0,                                # The time between each call of interrupt
    'CAN_CALLBACK_TIMER'                : 0,                                # The time for the CAN callback
    'MOTOR_TEMP'                        : array.array('f', [0.0]),          # The temperature of the motor
    'CAN_BUS_OFF_COUNTER'               : 0,                                # The counter for the CAN bus off
    'CAN_INFO_HOLDER'                   : [0, 0, 0, 0, 0, 0, 0, 0],         # 8-entry list to store information from can.info()
    'FTS_FZ'                            : array.array('f', [0.0]),          # The force in the Z direction from the FTS sensor
    'FTS_MY'                            : array.array('f', [0.0]),          # The moment in the Y direction from the FTS sensor
    'EVEREST_LAST_ERROR'                : 0,                                # The last error value from everest
    'EVEREST_STATUS_WORD'               : 0,                                # The status word from everest
    'FTS_FZ_SPIKE'                      : array.array('f', [0.0]),          # The force in the Z direction from the FTS sensor
    'FTS_MY_SPIKE'                      : array.array('f', [0.0]),          # The moment in the Y direction from the FTS sensor
    'FTS_FZ_RAW_SPIKE'                  : array.array('f', [0.0]),          # The force in the Z direction from the FTS sensor
    'FTS_MY_RAW_SPIKE'                  : array.array('f', [0.0]),          # The moment in the Y direction from the FTS sensor
    'FTS_CH1_RAW_SPIKE'                 : array.array('i', [0]),          # The raw value from channel 1 of the FTS sensor
    'FTS_CH2_RAW_SPIKE'                 : array.array('i', [0])           # The raw value from channel 2 of the FTS sensor
    
}

gc.collect()