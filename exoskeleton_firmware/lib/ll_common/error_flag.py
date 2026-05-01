import time            # type: ignore

print('IMPORTING ERROR_FLAG')

'''
            Signal Viewer Guide in CAPS:
Knee Error Channel: 36 | Ankle Error Channel: 37
------------------------------------------------
5       --->> watchdog_triggered
4       --->> tick_caps_exception
3       --->> tick_imu_exception
2.5     --->> everest_error
2       --->> everest_warning
1.5     --->> main_loop_exception
1       --->> servo_cyclic_exception
0       --------------
-1      --->> can_fifo0_exception
-1.5    --->> can_fifo0_msg_lost
-2      --->> can_fifo1_exception
-2.5    --->> can_fifo1_msg_lost
-3      --->> can_ack_failed
-3.5    --->> can_bus_repeatedly_failed
-5      --------------
-------------------------------------------------
'''

ERROR_FLAG = {                             # [Flag State,    CAPS Signal,   Buzz/Not Buzz]
    'watchdog_triggered':                   [False,                 5,              True],          # watchdog_triggered
    'tick_caps_exception':                  [False,                 4,              False],         # tick_caps
    'tick_imu_exception':                   [False,                 3,              False],         # tick_imu
    'everest_error':                        [False,                 2.5,            True],          # everest errored out
    'everest_warning':                      [False,                 2,              False],         # everest warning (not critical enough to buzz)
    'main_loop_exception':                  [False,                 1.5,            False],         # main timer loop error
    'servo_cyclic_exception':               [False,                 1,              False],         # everest cyclic operation failure
    'can_fifo0_exception':                  [False,                 -1,             False],         # error receiving can message in fifo0
    'can_fifo0_msg_lost':                   [False,                 -1.5,           False],         # lost a can message in fifo0
    'can_fifo1_exception':                  [False,                 -2,             False],         # error receiving can message in fifo1
    'can_fifo1_msg_lost':                   [False,                 -2.5,           False],         # lost a can message in fifo1
    'can_ack_failed':                       [False,                 -3,             False],         # failed in sending acknowledgement message
    'can_bus_repeatedly_failed':            [False,                 -3.5,           False],         # CAN bus restarted multiple times
}

# For faster dictionary checking, we will create a subset of the error flags that are buzzable
BUZZABLE_ERROR_FLAG = {key: ERROR_FLAG[key] for key in ERROR_FLAG if ERROR_FLAG[key][2]}

ec_index = 0
error_count = 0
ec_start_time = time.ticks_ms()
ec_index_time = time.ticks_ms()
ec_hex = {0: (0x7F, 0xFF)}     # Dictionary to store the hex values of the error codes
ec_zero_msg_sent = False
channel_initialized = False
error_channel_zeroed = False


def initialize_error_channel(channel, refresh_interval=200):
    '''
    This checks the active device and initializes its corresponding error channel.
    This should be called in the main.py file.
    '''
    global num_entries
    global error_codes
    global error_refresh_interval
    global ec_display_time
    global ec_NID
    global ec_can
    global ec_hex
    global channel_initialized

    num_entries = len(ERROR_FLAG)
    error_codes = [0] * num_entries
    error_refresh_interval = refresh_interval  # ms
    ec_display_time = error_refresh_interval / (num_entries + 1)  # ms

    ec_NID = 0x1E8       # This is the debug NID
    ec_can = bytearray(5)
    ec_can[0] = 0x04
    ec_can[1] = 0x50
    ec_can[2] = channel

    # Compute the corresponding hex values for the error codes
    for key in ERROR_FLAG:
        ec_hex[ERROR_FLAG[key][1]] = convert_to_can(ERROR_FLAG[key][1])

    channel_initialized = True


def convert_to_can(error_code):
    val = int(error_code * 6553.5) + 0x7FFF
    val = max(0, min(val, 0xFFFF))

    return val >> 8, val & 0xFF


def activate(key, dictionary=ERROR_FLAG):
    if key in dictionary:
        dictionary[key][0] = True
    else:
        print('Error_flag[key] not found')


def clear(key, dictionary=ERROR_FLAG):
    if key in dictionary:
        dictionary[key][0] = False
    else:
        print('Error_flag[key] not found')


def contains_true(dictionary=ERROR_FLAG):
    for key in dictionary:
        if dictionary[key][0]:
            return True
    return False


def check_if_buzz(dictionary=BUZZABLE_ERROR_FLAG):
    for key in dictionary:
        if dictionary[key][0]:
            return True
    return False


def print_error(dictionary=ERROR_FLAG):
    header_printed = False  # Flag to track if header is printed or not
    for key in dictionary:
        if dictionary[key][0]:
            if not header_printed:
                print("=== Error Flags ===")
                header_printed = True
            print("->", key)


def update_error_codes(dictionary=ERROR_FLAG):
    global error_codes
    global error_count
    index = 0
    for key in dictionary:
        if dictionary[key][0]:
            error_codes[index] = dictionary[key][1]
            index += 1
    error_count = index


def clear_error_codes():
    global error_codes
    for i in range(num_entries):
        error_codes[i] = 0


def error_code_to_CAPS(dictionary=ERROR_FLAG):
    '''
    This updates ec_can, the bytearray to send over CAN to update the debug channel
    Returns True if there is an error code to send, else returns False.
    '''
    global ec_display_time, error_count
    global ec_index
    global ec_start_time, ec_index_time
    global ec_can
    global ec_zero_msg_sent

    if channel_initialized:         # This ensures ec_can[2] is set to correct device channel
        if time.ticks_diff(time.ticks_ms(), ec_start_time) >= error_refresh_interval:
            clear_error_codes()
            update_error_codes(dictionary)
            ec_display_time = error_refresh_interval / (error_count + 1)
            ec_start_time = time.ticks_ms()
            ec_index = 0
        if ec_index < num_entries:
            if time.ticks_diff(time.ticks_ms(), ec_index_time) >= ec_display_time:
                ec = error_codes[ec_index]
                ec_can[3], ec_can[4] = ec_hex[ec]
                ec_index_time = time.ticks_ms()
                ec_index += 1
                if ec == 0 and not ec_zero_msg_sent:
                    ec_zero_msg_sent = True
                    return True
                elif ec != 0:
                    ec_zero_msg_sent = False
                    return True
        else:
            ec_index = 0
        return False
