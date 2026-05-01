##############################################################################################
# @file       CAN_PDCP_Streaming_Message.py
#
# @details    CAN streaming code
#
# @author     Levi Hargrove, modified by Chrestien Ilaya
##############################################################################################

import array

from . import upy_float
from . import assembler_functions

print('IMPORTING CAN PDCP STREAMING MESSAGE')

class CanPdcpNidHandler:
    """
    Handles creation and interaction with Control and Streaming NIDs for CAPS.
    Provides methods for message formatting, sending, and channel configuration.

    Parameters:
    1. can_nid: 0xYZZ, where 0xY00 sets the priority level and 0x0ZZ specifies
                the NID associated with the PDCP
    2. adc_resolution: ADC resolution (12 or 16 bits)
    3. zero_and_gain_array: Array containing zero and gain values
        -   CAN input gain should be the full scale
            bit resolution (2^16)/2 = 32767 (16 bit in this case)
            divided by the max value assigned in the ODV file.
            The zero_scale should be the "zero" location of the
            full scale bit resolution, normally (2^16)/2.
    4. outmap: Output channel mapping
    5. can_sender: CAN sender instance
    6. is_control_nid: Boolean flag indicating if this is a control NID
    """

    def __init__(
        self,
        can_nid,
        adc_resolution,
        zero_and_gain_array,
        outmap,
        can_sender,
        is_control_nid=False
    ):
        self.can_nid = can_nid
        self.adc_resolution = adc_resolution
        self.can_sender = can_sender

        # Creating a Control NID flag to differentiate from Streaming NIDs
        # Control NIDs are read-only and do not contain channel streaming data
        self.is_control_nid = is_control_nid

        self.data = bytearray(8)
        self.empty_buffer = bytearray(8)
        self.empty_buffer[0] = 0
        self.empty_buffer[1] = 0
        self.empty_buffer[2] = 0
        self.empty_buffer[3] = 0
        self.empty_buffer[4] = 0
        self.empty_buffer[5] = 0
        self.empty_buffer[6] = 0
        self.empty_buffer[7] = 0
        self.seq_num_restart_flag = False

        if self.is_control_nid:
            # Set up additional control NID parameters
            self.payload_length = 0
            self.received_id = 0
            self.can_ack_flag = False
            # Disabling send function for Control NIDs
            self.send = None
        else:
            # Set up Streaming / DATA NIDs
            self.send = self._send_message
            if self.adc_resolution == 12:
                self.max_chans = 4
                self.gain_map = array.array('f', [0, 0, 0, 0])
                full_scale = (2 ** self.adc_resolution - 1)
                for i in range(self.max_chans):
                    zero_and_gain_array[i * 2] = int(
                        zero_and_gain_array[i * 2] * full_scale
                    )
                    self.gain_map[i] = zero_and_gain_array[i * 2 + 1]
                # Allow for up to 4 channels (a zero and gain for each channel)
                self.zero_and_gain_array = zero_and_gain_array
                self.fast_read_message = (
                    assembler_functions.fast_caps_read_12bit_4chans
                )
                self.fast_write_message = (
                    assembler_functions.fw_CAPS_u12bit
                )
                self.chan0 = upy_float.upyFloat(0.0)
                self.chan1 = upy_float.upyFloat(0.0)
                self.chan2 = upy_float.upyFloat(0.0)
                self.chan3 = upy_float.upyFloat(0.0)
                self.outmap = outmap
            elif self.adc_resolution == 16:
                self.max_chans = 3
                self.gain_map = array.array('f', [0, 0, 0])
                full_scale = (2 ** self.adc_resolution - 1)
                for i in range(self.max_chans):
                    zero_and_gain_array[i * 2] = int(
                        zero_and_gain_array[i * 2] * full_scale
                    )
                    self.gain_map[i] = zero_and_gain_array[i * 2 + 1]
                # Allow for up to 3 channels (a zero and gain for each channel)
                self.zero_and_gain_array = zero_and_gain_array
                self.fast_read_message = (
                    assembler_functions.fast_caps_read_16bit
                )
                self.fast_write_message = (
                    assembler_functions.fw_CAPS_u16bit
                )
                self.chan0 = upy_float.upyFloat(0.0)
                self.chan1 = upy_float.upyFloat(0.0)
                self.chan2 = upy_float.upyFloat(0.0)
                self.outmap = outmap
            else:
                raise ValueError(
                    "Only 12 or 16 bit streaming CAPS channels are supported."
                )

    def _send_message(self):
        self.fast_write_message(
            self.data,
            self.outmap,
            self.gain_map
        )
        # Sequence number is the first byte of the message
        if self.seq_num_restart_flag:
            # Introduce a sequence break to indicate a restart has occurred
            self.data[0] = 0
            self.seq_num_restart_flag = False
        # Increment the sequence number. This starts at 0 and wraps at 255
        self.data[0] = self.data[0] + 1
        self.can_sender.cpy_mess_into_can_send(
            self.data,
            self.can_nid
        )

    def set_chan_zero_and_gain(self, chan_num, zero_scale, gain):
        # Channels are zero indexed
        self.zero_and_gain_array[chan_num * 2] = zero_scale
        self.zero_and_gain_array[chan_num * 2 + 1] = gain

    def set_all_zero_scale_to_value(self, zero_scale):
        # Channels are zero indexed
        for a in range(self.max_chans):
            self.set_chan_zero_and_gain[a * 2] = zero_scale

    def set_all_gains_to_value(self, gains):
        # Channels are zero indexed
        for a in range(self.max_chans):
            self.set_chan_zero_and_gain[a * 2 + 1] = gains

    def process_can_message(self, fbuf):
        self.fast_read_message(
            self.data,
            self.zero_and_gain_array,
            self.outmap,
            fbuf
        )

    def zero_and_clear(self):
        self.fast_read_message(
            self.data,
            self.zero_and_gain_array,
            self.outmap,
            self.empty_buffer
        )
        
    def zero(self):
        """
        Zeros caps values
        """
        self.data[0] = 0
        self.data[1] = 0
        self.data[2] = 0
        self.data[3] = 0
        self.data[4] = 0
        self.data[5] = 0
        self.data[6] = 0
        self.data[7] = 0