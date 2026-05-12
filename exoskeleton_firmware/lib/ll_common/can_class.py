#############################################################################
# @file       can_class.py
#
# @details    The application for reading and processing CAN traffic
#
# @author     Levi Hargrove, Frank Ursetta and Chrestien Ilaya
#############################################################################
##

#
#   IMPORTS
#

import array
import time

import pyb                                          # type: ignore
import micropython                                  # type: ignore
import stm                                          # type: ignore
from . import assembler_functions

print('IMPORTING CAN_CLASS')


class canClass():

    def __init__(self, hybrid_leg):
        self.hybrid_leg                             = hybrid_leg
        self.fbuf1 = bytearray(8)
        self.fmv1 = memoryview(self.fbuf1)
        self.tv1 = [0, 0, 0, 0, self.fmv1]
        self.can_pos_read_time = time.ticks_us()
        self.can_callback_time = 0
        self.watchdog_timer0 = 0
        self.fbuf = bytearray(8)
        self.fmv = memoryview(self.fbuf)
        self.tv = [0, 0, 0, 0, self.fmv]
        self.can_pos_read_time = 0
        # self.float_comp = array.array('f', [0.001])             # Minimum stiffnes required to enable the large motor controller
        # self.large_motor_float_comp = array.array('f', [0.1])   # Value used minimum stiffness before zero stiffness is enabled for the large motor controller
        # self.POLY_ANKLE_MIN_DAMPING = array.array('f', [0.1])   # Minimum damping value for the polycentric ankle. Used to prevent accidental instability
        self.mode_bit = micropython.const(0x100)                # Mode bit in CAN SID is 0b'001 0000 0000'
        self.can_ack_list = [0,                                 # List of bytearrays to use for acknowledging CAN messages of different lengths
                             bytearray(2),
                             bytearray(3),
                             bytearray(4),
                             bytearray(5),
                             bytearray(6),
                             bytearray(7),
                             bytearray(8),
                             bytearray(8)]
        self.can_ack_id = 0

        # --------------------------------------------------------------------
        #   FILTERS FOR HYBRID KNEE AND FOX KNEE
        # --------------------------------------------------------------------
        if (self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.HYBRID_KNEE
            or self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.FOX_KNEE
            ):

            #
            #   F767 (Pyboard) uses conventional CAN
            #
            if self.hybrid_leg.device_parameters.PROCESSOR == 'STM32F767':
                self.hybrid_leg.can.setfilter(0, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS1_ID, 0xFF, self.hybrid_leg.CAPS2_ID, 0xFF))   # CAPS1, CAPS2
                self.hybrid_leg.can.setfilter(1, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS9_ID, 0xFF, self.hybrid_leg.CAPS12_ID, 0xFF))  # CAPS9, CAPS12
                self.hybrid_leg.can.setfilter(2, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS7_ID, 0xFF, self.hybrid_leg.CAPS8_ID, 0xFF))   # CAPS7, CAPS8
                self.hybrid_leg.can.setfilter(3, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS10_ID, 0xFF, self.hybrid_leg.CAPS11_ID, 0xFF)) # CAPS10, CAPS11
                self.hybrid_leg.can.setfilter(4, pyb.CAN.MASK16, 1, (self.hybrid_leg.LOAD_CELL1_DATA1_ID, 0xFF, self.hybrid_leg.LOAD_CELL1_DATA2_ID, 0xFF))   # loadcell
            #
            #   H747 uses FDCAN
            #
            elif self.hybrid_leg.device_parameters.PROCESSOR == 'STM32H747':
                # self.hybrid_leg.can.setfilter(0, pyb.CAN.DUAL, 0, (self.hybrid_leg.CAPS1_ID, self.hybrid_leg.CAPS2_ID))
                # self.hybrid_leg.can.setfilter(2, pyb.CAN.DUAL, 0, (self.hybrid_leg.CAPS9_ID, self.hybrid_leg.CAPS12_ID))
                # self.hybrid_leg.can.setfilter(3, pyb.CAN.DUAL, 0, (self.hybrid_leg.CAPS7_ID, self.hybrid_leg.CAPS8_ID))
                # self.hybrid_leg.can.setfilter(4, pyb.CAN.DUAL, 0, (self.hybrid_leg.CAPS10_ID, self.hybrid_leg.CAPS11_ID))
                # self.hybrid_leg.can.setfilter(5, pyb.CAN.DUAL, 1, (self.hybrid_leg.LOAD_CELL1_DATA1_ID, self.hybrid_leg.LOAD_CELL1_DATA2_ID))

                self.hybrid_leg.can.setfilter(0, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS1_ID, 0xFF))
                self.hybrid_leg.can.setfilter(1, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS2_ID, 0xFF))
                self.hybrid_leg.can.setfilter(2, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS7_ID, 0xFF))
                self.hybrid_leg.can.setfilter(3, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS8_ID, 0xFF))
                self.hybrid_leg.can.setfilter(4, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS9_ID, 0xFF))
                self.hybrid_leg.can.setfilter(5, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS10_ID, 0xFF))
                self.hybrid_leg.can.setfilter(6, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS11_ID, 0xFF))
                self.hybrid_leg.can.setfilter(7, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS12_ID, 0xFF))
                self.hybrid_leg.can.setfilter(8, pyb.CAN.MASK, 1, (self.hybrid_leg.LOAD_CELL1_DATA1_ID, 0xFF))
                self.hybrid_leg.can.setfilter(9, pyb.CAN.MASK, 1, (self.hybrid_leg.LOAD_CELL1_DATA2_ID, 0xFF))
                if hasattr(self.hybrid_leg, 'fts_nid_control'):
                    self.hybrid_leg.can.setfilter(10, pyb.CAN.MASK, 1, (self.hybrid_leg.fts_nid_control, 0xFF))

        # --------------------------------------------------------------------
        #   FILTERS FOR POLYCENTRIC ANKLE
        # --------------------------------------------------------------------
        elif (
            self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.POLYCENTRIC_ANKLE
            or self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.POLYCENTRIC_ANKLE_ONLY
        ):
            #
            #   F767 (Pyboard) uses conventional CAN
            #
            if self.hybrid_leg.device_parameters.PROCESSOR == 'STM32F767':
                self.hybrid_leg.can.setfilter(0, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS5_ID, 0xFF, self.hybrid_leg.CAPS6_ID, 0xFF))   # CAPS5, CAPS6
                self.hybrid_leg.can.setfilter(1, pyb.CAN.MASK16, 0, (self.hybrid_leg.CAPS13_ID, 0xFF, self.hybrid_leg.CAPS13_ID, 0xFF))  # CAPS13, CAPS13
            #
            #   H747 uses FDCAN
            #
            elif self.hybrid_leg.device_parameters.PROCESSOR == 'STM32H747':
                self.hybrid_leg.can.setfilter(0, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS5_ID, 0xFF))
                self.hybrid_leg.can.setfilter(1, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS6_ID, 0xFF))
                self.hybrid_leg.can.setfilter(2, pyb.CAN.MASK, 0, (self.hybrid_leg.CAPS13_ID, 0xFF))

        # --------------------------------------------------------------------
        #   INITIALIZE CAN CALLBACK FUNCTIONS:
        # --------------------------------------------------------------------

        #
        #   F767
        #
        if self.hybrid_leg.device_parameters.PROCESSOR == 'STM32F767':
            self.hybrid_leg.can.rxcallback(0, self.can_call_fifo0_F7)
            if hybrid_leg.device_parameters.PRIMARY_CONTROLLER:
                self.hybrid_leg.can.rxcallback(1, self.can_call_fifo1_F7)

        #
        #   H747
        #
        elif self.hybrid_leg.device_parameters.PROCESSOR == 'STM32H747':
            self.hybrid_leg.can.rxcallback(0, self.can_call_fifo0_H7)
            if hybrid_leg.device_parameters.PRIMARY_CONTROLLER:
                self.hybrid_leg.can.rxcallback(1, self.can_call_fifo1_H7)

        # --------------------------------------------------------------------
        #   SET UP CALLBACK DICTIONARY
        # --------------------------------------------------------------------
        self.can_function_dict = {
            self.hybrid_leg.CAPS1_ID: self.process_CAPS_Setting,
            self.hybrid_leg.CAPS5_ID: self.process_CAPS_Setting,
            self.hybrid_leg.CAPS7_ID: self.process_CAPS_Setting,
            self.hybrid_leg.CAPS9_ID: self.process_CAPS_Message,
            self.hybrid_leg.CAPS11_ID: self.process_CAPS_Setting,
            self.hybrid_leg.CAPS10_ID: self.process_CAPS_Buzzer,
            self.hybrid_leg.CAPS2_ID: self.process_CAPS_Streaming,
            self.hybrid_leg.CAPS6_ID: self.process_CAPS_Streaming,
            self.hybrid_leg.CAPS8_ID: self.process_CAPS_Streaming,
            self.hybrid_leg.CAPS12_ID: self.process_CAPS_Message,
            self.hybrid_leg.CAPS13_ID: self.process_CAPS_Message
        }

        # Setting up the CAPS channels that are relevant for the primary controller device
        if self.hybrid_leg.device_parameters.PRIMARY_CONTROLLER:
            self.can_function_dict[self.hybrid_leg.LOAD_CELL1_DATA1_ID] = self.process_LOAD_CELL_DATA
            self.can_function_dict[self.hybrid_leg.LOAD_CELL1_DATA2_ID] = self.process_LOAD_CELL_DATA

        # FTS control NID (Fox Knee with FTS installed)
        if hasattr(self.hybrid_leg, 'fts_nid_control'):
            self.can_function_dict[self.hybrid_leg.fts_nid_control] = self.process_FTS_Setting

    # CAPS Buzzer messages coming from CAPS10
    def process_CAPS_Buzzer(self, CAN_ID):
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        self.check_if_ack(active_channel)
        assembler_functions.cpy_can(0, active_channel.data, self.fbuf)
        if active_channel.data[0] == 0x54:                                                           # Checking for message from CAPS signifying state change and giving corresponding tone
            # print('CAPS buzzer received in main')
            if active_channel.payload_length > 0 and ((active_channel.data[1] << 8) | active_channel.data[2]) > 0:              # Getting beep frequency stored in CAPS10 data:
                self.hybrid_leg.led_buzzer.buzzer_parameters[0] = 0                                       # Clear buzzer array [0] value to ensure that a "main loop" buzzer in queue isn't played
                self.hybrid_leg.led_buzzer.stop_buzzer()                                                                # Stop the buzzer if a new CAPS buzzer signal is received
                if not self.hybrid_leg.FLAG_DICT['CAPS_BUZZER_SCHEDULED_FLAG']:                                            # If flag is false, a beep request has not already been sent from CAPS
                    self.hybrid_leg.FLAG_DICT['CAPS_BUZZER_SCHEDULED_FLAG'] = True                                              # Setting to True to indicate that a beep request has been received from CAPS
                else:
                    print('buzzer flag still true, waiting on scheduler')
            else:
                active_channel.zero()                                                           # Clear CAPS10 messages to prevent going back into this a second time
                # print('invalid CAPS buzzer value, payload length =', hybrid_leg.CAPS10.data, hybrid_leg.CAPS10.payload_length)

    # CAPS Settings come in on CAPS_IDS 1, 3, 5, 7, 11
    def process_CAPS_Setting(self, CAN_ID):
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        self.check_if_ack(active_channel)
        assembler_functions.cpy_can(0, active_channel.data, self.fbuf)                                      # Getting data from the buffer and putting it into the active_channel.data
        # Checking for CAPS stop/start streaming messages
        if active_channel.data[0] == 0x51 and active_channel.payload_length == 3:
            self.hybrid_leg.FLAG_DICT['CAPS_STOP_STREAM_FLAG'] = True
        if not self.hybrid_leg.FLAG_DICT['STREAM_FLAG']:
            if active_channel.data[0] == 0x50 and active_channel.payload_length == 2:
                self.hybrid_leg.FLAG_DICT['CAPS_START_STREAM_FLAG'] = True

        # Checking for additional backup motors disable messages
        if active_channel.data[0] != 0x88 and active_channel.data[0] != 0x80:                                # If the the first item in the data array is not 0x88 or 0x80 return
            return
        else:
            # If the the first item in the data array is 0x88 and the second is 0x01, then the motor disable message has been received from CAPS. Stop the large motor by
            # making the large_motor_state flag False
            if active_channel.data[0] == 0x88 and active_channel.data[1] == 0x01:
                self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_STATE'] = False
                # If the stiff leg CAPS stop streaming message has been received, but  motor disabled button is then pressed, it's necessary to go through clear the
                # CAPS_STOP_STREAM_LEG_STIFF_FLAG in order force the code to go through the CAPS_START_STREAM_FLAG logic and clear the CAPS and controller values
                self.hybrid_leg.FLAG_DICT['CAPS_STOP_STREAM_LEG_STIFF_FLAG'] = False

            # if the first item in the data array is 0x80 and the device is the primary controller, then process the fob array
            elif active_channel.data[0] == 0x80 and self.hybrid_leg.device_parameters.PRIMARY_CONTROLLER:
                assembler_functions.fob_value_calc(self.hybrid_leg.PARAM_DICT['FOB_VAL_ARRAY'], active_channel.data[1], active_channel.data[2], self.hybrid_leg.PARAM_DICT['FOB_CALC_ARRAY'])
                assembler_functions.mov_from_to(self.hybrid_leg.PARAM_DICT['FOB_VAL_ARRAY'], 0, self.hybrid_leg.sm_channels_current, 4 * 7)
                return

    # Processing of messages from CAN Message Generator and ping script, CAPS9, CAPS12, CAPS13
    def process_CAPS_Message(self, CAN_ID):
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        self.check_if_ack(active_channel)
        assembler_functions.cpy_can(0, active_channel.data, self.fbuf)

        ####################################################################################################
        # !! This CAPS12 section can probably be a separate function
        # Function to process CAPS12 data. CAPS12 is for communication control message from CAPS - Used for ping script that checks for slow CAPS streaming
        if CAN_ID == self.hybrid_leg.CAPS12_ID:
            self.hybrid_leg.PARAM_DICT['CAPST2'] = time.ticks_us()                                 # Save the time when the response was received - for evaluation in hybrid_leg.py
            self.hybrid_leg.FLAG_DICT['CAPS_REPLY_FLAG'] = True                                     # Set flag inticating reply from CAPS was received
            return
        ####################################################################################################
        # Check custom CAN messages from CAPS (CAPS9 or CAPS13)
        if active_channel.payload_length == 1:
            if active_channel.data[0] == 0x52:                                                      # Message from CAN message generator to rehome the absolute encoder
                self.hybrid_leg.FLAG_DICT['REHOME_MAIN_JOINT_ENCODER_FLAG'] = True                  # Set flag to rehome main joint encoder
            if self.hybrid_leg.components.IMU.is_enabled():
                if active_channel.data[0] == 0x53:                                                  # Message from CAN message generator to zero the IMU
                    self.hybrid_leg.FLAG_DICT['ZERO_IMU_FLAG'] = True                               # Set flag to zero the IMU
                if self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.FOX_KNEE:     # Only allow inversion on the FOX knee
                    if active_channel.data[0] == 0x56:                                              # Message from CAN message generator to invert the shank/thigh
                        self.hybrid_leg.FLAG_DICT['INVERT_SHANK_THIGH_FLAG'] = True                 # Set flag to invert the shank/thigh
            if active_channel.data[0] == 0x55:                                                      # Message from CAN message generator to indicate that the device should remain stiff while CAPS isn't streaming
                self.hybrid_leg.FLAG_DICT['CAPS_STOP_STREAM_LEG_STIFF_FLAG'] = True                 # Set flag to keep motors enabled
            if active_channel.data[0] == 0x0B:                                                      # Message from CAN message generator to reset the controller
                self.hybrid_leg.FLAG_DICT['RESET_CONTROLLER_FLAG'] = True                           # Set flag to do a soft reset of the controller   
            if active_channel.data[0] == 0x54:                                                      # Message from CAN message generator to zero the FTS
                self.hybrid_leg.FLAG_DICT['ZERO_FTS_FLAG'] = True                                   # Set flag to zero the FTS
            active_channel.zero()                                                                   # Clear CAPS message to prevent going back into this a second time
            return

        ####################################################################################################
        # PDCP Command Section
        if active_channel.data[0] == 0x03:                                                          # PDCP Command for querying a value
            # can_obj.can_ack(active_channel, 2)                                                    # Status code 2 for an unknown function code
            pass
        elif active_channel.data[0] == 0x04:                                                        # PDCP Command for setting a value
            if (self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.HYBRID_KNEE
                or self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.FOX_KNEE
            ):
                if active_channel.payload_length == 4:                                              # Message from safety switch to transmit signal to CAPS
                    if active_channel.data[3] == 0:
                        self.hybrid_leg.PARAM_DICT['SUBJECT_BUTTON_STATE'][0] = 3.0
                    else:
                        self.hybrid_leg.PARAM_DICT['SUBJECT_BUTTON_STATE'][0] = 1.0
                    active_channel.zero()                                                           # Clear CAPS message to prevent going back into this a second time
                    return
            else:
                if active_channel.payload_length == 5:
                    if active_channel.data[1] == 0x50:                                              # Message from CAN message generator to place an offset in ankle neutral angle
                        # can_obj.can_ack(active_channel, 1)                                        # Acknowledge CAPS message, Status code = 1 for success
                        self.hybrid_leg.FLAG_DICT['ANKLE_OFFSET_FLAG'] = True                       # Set flag to place an offset in ankle angle
            # can_obj.can_ack(active_channel, 2)

    # Function to process load cell data (handles both DATA1 and DATA2)
    def process_LOAD_CELL_DATA(self, CAN_ID):
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        active_channel.process_can_message(self.fbuf1)

    # FTS start/stop streaming (separate from process_CAPS_Setting to avoid
    # triggering main actuator stream start/stop)
    def process_FTS_Setting(self, CAN_ID):
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        self.check_if_ack(active_channel)
        assembler_functions.cpy_can(0, active_channel.data, self.fbuf1)
        self.hybrid_leg.FLAG_DICT['FTS_STREAM_FLAG'] = True

    # Function to process CAPS2 data. CAPS2 data is for knee control (used for impedance parameters)
    def process_CAPS_Streaming(self, CAN_ID):
        # print('process_CAPS_Streaming')
        self.watchdog_timer0 = time.ticks_ms()                                                                                     # Reset watchdog timer, data has been recieved

        # Take the data in the buffer, scale it then load it into the appropriate addresses, in this case CAPS2_outmap, which corresponds to everest_pb.controller_parameters
        active_channel = self.hybrid_leg.CAN_INPUT_DICT[CAN_ID]
        active_channel.process_can_message(self.fbuf)
        self.execute_flag_logic(CAN_ID)

    def execute_flag_logic(self, CAN_ID):
        self.hybrid_leg.PARAM_DICT['ISR_TIMER0'] = time.ticks_us()
        self.hybrid_leg.PARAM_DICT['ISR_TIME'] = time.ticks_diff(self.hybrid_leg.PARAM_DICT['ISR_TIMER0'], self.hybrid_leg.PARAM_DICT['ISR_TIMER1'])
        self.hybrid_leg.PARAM_DICT['ISR_TIMER1'] = self.hybrid_leg.PARAM_DICT['ISR_TIMER0']

        K = 12                                          # controller_parameters[12] is joint stiffness
        B = 1                                           # controller_parameters[1] is joint damping
        if CAN_ID == self.hybrid_leg.CAPS2_ID:
            if (
                self.hybrid_leg.everest_pb.controller_parameters[K] > 0.001         # Minimum stiffness required to enable the large motor to enable
                and self.hybrid_leg.FLAG_DICT['MAIN_JOINT_ENCODER_HOMED_FLAG']      # If the main joint has been homed, then enable the motor
            ):
                self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_STATE'] = True
                if self.hybrid_leg.everest_pb.controller_parameters[K] < 0.1:       # If it is less than .1 set the stiffness to 0
                    self.hybrid_leg.everest_pb.controller_parameters[K] = 0
            else:
                self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_STATE'] = False
                self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG'] = False
                # If the stiff leg CAPS stop streaming message has been received, but  motor disabled button is then pressed, it's necessary to go through clear the
                # CAPS_STOP_STREAM_LEG_STIFF_FLAG in order force the code to go through the CAPS_START_STREAM_FLAG logic and clear the CAPS and controller values
                self.hybrid_leg.FLAG_DICT['CAPS_STOP_STREAM_LEG_STIFF_FLAG'] = False
        elif CAN_ID == self.hybrid_leg.CAPS6_ID:
            if (
                self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.POLYCENTRIC_ANKLE
                or self.hybrid_leg.device_parameters.device == self.hybrid_leg.config.POLYCENTRIC_ANKLE_ONLY
            ):
                if self.hybrid_leg.everest_pb.controller_parameters[B] < 0.1:
                    self.hybrid_leg.everest_pb.controller_parameters[B] = 0.1

                self.eq_offset_and_limiter(self.hybrid_leg.PARAM_DICT['ANKLE_NEUTRAL_OFFSET'][0], self.hybrid_leg.PARAM_DICT['ANKLE_EQ_PLACEHOLDER'][0], self.hybrid_leg.everest_pb.controller_parameters, self.hybrid_leg.PARAM_DICT['ENDSTOP_LIMIT_ARRAY'])

                if self.hybrid_leg.everest_pb.controller_parameters[K] > 0.001:
                    self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_STATE'] = True
                else:
                    self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_STATE'] = False                                                                               # Change the state of the motor to false - changes the motor state as identified in the man loop
                    self.hybrid_leg.FLAG_DICT['LARGE_MOTOR_DRIVE_FLAG'] = False
                    # If the stiff leg CAPS stop streaming message has been received, but  motor disabled button is then pressed, it's necessary to go through clear the
                    # CAPS_STOP_STREAM_LEG_STIFF_FLAG in order force the code to go through the CAPS_START_STREAM_FLAG logic and clear the CAPS and controller values
                    self.hybrid_leg.FLAG_DICT['CAPS_STOP_STREAM_LEG_STIFF_FLAG'] = False

    # Function of unknown usage
    def process_DEFAULT_ID(self, CAN_ID):
        pass

    # Setting up the fifo0 to get CAN traffic off the bus, identify its can_id
    # and call the appropriate dictionary function to store the data
    def can_call_fifo0_H7(self, bus, reason):
        try:
            tim0 = time.ticks_us()

            if reason == 1:
                # print('can fifo0 full')
                pass

            elif reason == 2:
                print('can fifo0 msg lost')
                self.hybrid_leg.error_flag.activate('can_fifo0_msg_lost')

            else:
                self.hybrid_leg.error_flag.clear('can_fifo0_msg_lost')

            # messages = stm.mem8[self.RXFOS] & 0b1111111
            # messages = stm.mem32[stm.FDCAN1 + stm.FDCAN_RXF0S] & 0b1111111
            messages = self.hybrid_leg.can.any(0)
            while messages:
                # print('processing messages')
                self.hybrid_leg.can.recv(0, self.tv)
                can_id = self.tv[0] & 0xFF

                # This returns None if can_id is not in dictionary
                method = self.can_function_dict.get(can_id, None)
                if method:
                    method(can_id)
                else:
                    self.process_DEFAULT_ID(can_id)
                    
                # Query number of messages remaining in fifo
                messages = self.hybrid_leg.can.any(0)
                # messages = messages - 1

            # Ensure new msg interrupt line for FIFO0 is enabled:
            stm.mem32[stm.FDCAN1 + stm.FDCAN_IE] |= 0b1

            # Reset boolean to False if function ran successfully
            self.hybrid_leg.error_flag.clear('can_fifo0_exception')
            tim1 = time.ticks_us()
            self.hybrid_leg.PARAM_DICT['CAN_CALLBACK_TIMER'] = time.ticks_diff(tim1, tim0)

        # Setting error code for exception found in fifo0
        except Exception as e:
            self.hybrid_leg.error_flag.activate('can_fifo0_exception')

    # Setting up the fifo1 to get CAN traffic off the bus, identify its can_id
    # and call the appropriate dictionary function to store the data
    def can_call_fifo1_H7(self, bus, reason):
        try:
            # tim0 = time.ticks_us()

            if reason == 1:
                # print('can fifo1 full')
                pass

            elif reason == 2:
                print('can fifo1 msg lost')
                self.hybrid_leg.error_flag.activate('can_fifo1_msg_lost')

            else:
                self.hybrid_leg.error_flag.clear('can_fifo1_msg_lost')

            messages = self.hybrid_leg.can.any(1)
            while messages:
                self.hybrid_leg.can.recv(1, self.tv1)
                can_id = self.tv1[0] & 0xFF

                # returns None if can_id is not in dictionary
                method = self.can_function_dict.get(can_id, None)
                if method:
                    method(can_id)
                else:
                    self.process_DEFAULT_ID(can_id)

                messages = self.hybrid_leg.can.any(1)

            # Ensure new msg interrupt line for FIFO1 is enabled:
            stm.mem32[stm.FDCAN1 + stm.FDCAN_IE] |= 0b10000

            # Reset boolean to False if function ran successfully
            self.hybrid_leg.error_flag.clear('can_fifo1_exception')
            # tim1 = time.ticks_us()
            # self.can_callback_time = time.ticks_diff(tim1, tim0)

        # Setting error code for exception found in fifo0
        except Exception as e:
            self.hybrid_leg.error_flag.activate('can_fifo1_exception')

    # Setting up the fifo to get CAN traffic off the bus, identify its can_id
    # and call the appropriate dictionary function to store the data
    def can_call_fifo0_F7(self, bus, reason):
        try:
            # tim0 = time.ticks_us()

            if reason == 1:
                # print('can fifo1 full')
                pass

            elif reason == 2:
                self.hybrid_leg.error_flag.activate('can_fifo0_msg_lost')
                print('can fifo1 msg lost')

            else:
                self.hybrid_leg.error_flag.clear('can_fifo0_msg_lost')

            while self.hybrid_leg.can.any(0):
                self.hybrid_leg.can.recv(0, self.tv)
                can_id = self.tv[0] & 0xFF

                # Returns None if can_id is not in dictionary:
                method = self.can_function_dict.get(can_id, None)
                if method:
                    method(can_id)
                else:
                    self.process_DEFAULT_ID(can_id)

            # Reset boolean to False if function ran successfully
            self.hybrid_leg.error_flag.clear('can_fifo0_exception')

            # tim1 = time.ticks_us()
            # self.can_callback_time = time.ticks_diff(tim1, tim0)

        # Setting error code for exception found in fifo1
        except Exception as e:
            self.hybrid_leg.error_flag.activate('can_fifo0_exception')

    # Setting up the fifo to get CAN traffic off the bus, identify its can_id
    # and call the appropriate dictionary function to store the data
    def can_call_fifo1_F7(self, bus, reason):
        try:
            # tim0 = time.ticks_us()

            if reason == 1:
                # print('can fifo1 full')
                pass

            elif reason == 2:
                self.hybrid_leg.error_flag.activate('can_fifo1_msg_lost')
                print('can fifo1 msg lost')

            else:
                self.hybrid_leg.error_flag.clear('can_fifo1_msg_lost')

            while self.hybrid_leg.can.any(1):
                self.hybrid_leg.can.recv(1, self.tv1)
                can_id = self.tv1[0] & 0xFF

                # Returns None if can_id is not in dictionary:
                method = self.can_function_dict.get(can_id, None)
                if method:
                    method(can_id)
                else:
                    self.process_DEFAULT_ID(can_id)

            # Reset boolean to False if function ran successfully
            self.hybrid_leg.error_flag.clear('can_fifo1_exception')

            # tim1 = time.ticks_us()
            # self.can_callback_time = time.ticks_diff(tim1, tim0)

        # Setting error code for exception found in fifo1
        except Exception as e:
            self.hybrid_leg.error_flag.activate('can_fifo1_exception')

    def eq_offset_and_limiter(self, neutral_offset, ankle_eq_placeholder, controller_parameters, endstop_limit_array):
        offset = neutral_offset + ankle_eq_placeholder
        if offset > endstop_limit_array[0]:
            offset = endstop_limit_array[0]
        if offset < endstop_limit_array[1]:
            offset = endstop_limit_array[1]
        controller_parameters[10] = offset

    def check_if_ack(self, can_channel):
        # Grab the data length of the latest message received
        can_channel.payload_length = len(self.tv[4])
        # If CAN msg has its mode bit turned on, the message should be acknowledged
        if self.tv[0] & self.mode_bit == self.mode_bit:
            can_channel.can_ack_flag = True
            can_channel.received_id = self.tv[0]

    def can_ack(self, can_channel, status_code):
        if can_channel.can_ack_flag:
            # Grab the correct CAN ID to respond with by taking the received ID and bringing its mode bit to 0
            self.can_ack_id = can_channel.received_id - self.mode_bit
            # Acknowledgement messages can respond with status codes of:
            # 0 == illegal; 1 == success; >1 == error
            if status_code == 1:
                # Ack on message and copy original payload data
                self.can_ack_list[can_channel.payload_length][0] = can_channel.data[0] + 0x80
                self.can_ack_list[can_channel.payload_length][1] = status_code
                for i in range(1, can_channel.payload_length):
                    # Copy original data in the ack message as long as it fits
                    self.can_ack_list[can_channel.payload_length][i + 1] = can_channel.data[i]
                    if i == 6:
                        # Maximum data payload size is 8 bytes
                        break
                try:
                    # Send CAN ack message
                    self.hybrid_leg.can_send.cpy_mess_into_can_send(self.can_ack_list[can_channel.payload_length], self.can_ack_id)
                    # Reset boolean to False if function ran successfully
                    self.hybrid_leg.error_flag.clear('can_ack_failed')
                except:
                    self.hybrid_leg.error_flag.activate('can_ack_failed')

            else:
                # Ack on message without copying original data
                self.can_ack_list[1][0] = can_channel.data[0] + 0x80
                self.can_ack_list[1][1] = status_code
                try:
                    # Send CAN ack message
                    self.hybrid_leg.can_send.cpy_mess_into_can_send(self.can_ack_list[1], self.can_ack_id)
                    # Reset boolean to False if function ran successfully
                    self.hybrid_leg.error_flag.clear('can_ack_failed')
                except:
                    self.hybrid_leg.error_flag.activate('can_ack_failed')

            can_channel.can_ack_flag = False
        else:
            self.hybrid_leg.error_flag.clear('can_ack_failed')

    def can_ack_all(self, status_code):
        # Ack on all pending messages
        self.can_ack(self.hybrid_leg.CAPS1, status_code)
        self.can_ack(self.hybrid_leg.CAPS5, status_code)
        self.can_ack(self.hybrid_leg.CAPS7, status_code)
        if hasattr(self.hybrid_leg, 'FTS_CONTROL'):
            self.can_ack(self.hybrid_leg.FTS_CONTROL, status_code)
        # self.can_ack(self.hybrid_leg.CAPS9, status_code)
        # self.can_ack(self.hybrid_leg.CAPS10, status_code)
        # self.can_ack(self.hybrid_leg.CAPS13, status_code)

# method = None
# for key in self.can_function_dict:
#     if key == can_id:
#         method = self.can_function_dict[key]
# if not key:
#     method = self.process_DEFAULT_ID
