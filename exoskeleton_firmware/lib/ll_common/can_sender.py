#############################################################################
# @file       can_sender.py
#
# @details    Code is used for storing CAN messages in a buffer and sending them
#             without blocking interrupts.
#
# @author     Levi Hargrove
#############################################################################
##

#
#   IMPORTS
#
import time

from uctypes import addressof       # type: ignore
import stm                          # type: ignore
import pyb                          # type: ignore
from micropython import const       # type: ignore

from . import assembler_functions
from . import error_flag
from .param_dict import PARAM_DICT

from config import PROCESSOR

print('IMPORTING CAN SENDER')

_CAN_SEND_BUFFER_LEN = const(20)    # Number of entries in the buffer


class canSender():

    def __init__(self, can):
        self.can = can
        self.can_send_buffer = []
        self.can_send_buffer_mv = []
        self.can_send_ids = []
        self.can_send_len_idx = []
        self.current_idx = 0
        self.send_counter = 0
        self.buffer_counter = 0
        self.sent_idx = 0
        self.stop_CAN_send_flag = True  # Initialize to True so that CAN sender does not automatically send msgs at boot up
        self.PARAM_DICT = PARAM_DICT

        # Assign send_message() function based on processor
        if PROCESSOR == 'STM32H747':
            self.send_message = self.send_message_H7
            self.FDCAN1_TXFQS = stm.FDCAN1 + stm.FDCAN_TXFQS
        elif PROCESSOR == 'STM32F767':
            self.send_message = self.send_message_F7
            self.CAN1_TSR_3 = stm.CAN1 + 0x08 + 3       # Address of CAN1_TSR[3]

        # Create a circular buffer of CAN messages that you want to send
        for i in range(_CAN_SEND_BUFFER_LEN):
            self.can_send_buffer.append(bytearray(8))
            self.can_send_ids.append(0)
            self.can_send_len_idx.append(0)
        # Create a list of memoryviews for different length slices of self.can_send_buffer
        for buffer in self.can_send_buffer:
            buffer_slices = [memoryview(buffer)[:i] for i in range(1, 9)]
            self.can_send_buffer_mv.append(buffer_slices)

    # Copy a message into the sending buffer and increment the current_idx in
    # the circular buffer. This function takes about 45 us on SF6 pyboard
    def cpy_mess_into_can_send(self, data, did):
        assembler_functions.cpy_can(0, self.can_send_buffer[self.current_idx], data)
        self.can_send_ids[self.current_idx] = did
        self.can_send_len_idx[self.current_idx] = len(data) - 1
        self.buffer_counter += 1
        self.current_idx = (self.current_idx + 1) % _CAN_SEND_BUFFER_LEN
        if self.buffer_counter - self.send_counter > _CAN_SEND_BUFFER_LEN:
            self.reset_buffer()
        else:
            pass

    def reset_buffer(self):
        self.current_idx = 0
        self.send_counter = 0
        self.buffer_counter = 0
        self.sent_idx = 0

    # -------------------------------------------------------------------------
    #
    #   send_message_H7()
    #
    # -------------------------------------------------------------------------
    def send_message_H7(self, timer):
        if not self.stop_CAN_send_flag:
            tim_0 = time.ticks_us()
            num_mess = (self.current_idx - self.sent_idx) % _CAN_SEND_BUFFER_LEN
            counter = 0
            if num_mess == 0:
                return 

            # print(num_mess)
            try:

                # for _ in range(num_mess):
                # while self.current_idx != self.sent_idx:
                free_size = stm.mem32[self.FDCAN1_TXFQS] & 0x3F
                if free_size < num_mess:
                    print('TX QUEUE FULL. Will try next time.')
                    return
                irq_state = pyb.disable_irq()
                while counter < num_mess:
                    self.can.send(
                        self.can_send_buffer_mv[self.sent_idx][
                            self.can_send_len_idx[self.sent_idx]
                            ],                                      # data
                        self.can_send_ids[self.sent_idx],           # id
                        timeout=0,                                  # if timeout is 0 (default), then the buffers should be used
                        fdf=False,                                  # FD frame format
                        brs=False,                                  # Use bit rate switching (FD only)
                        extframe=False                              # Should frame use extended format
                    )
                    self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                    self.send_counter += 1
                    counter += 1
                pyb.enable_irq(irq_state)
            except:
                print('!')

            tim_1 = time.ticks_us()
            self.PARAM_DICT['SEND_MESSAGE_TIMER'] = time.ticks_diff(tim_1, tim_0)

            return

    # -------------------------------------------------------------------------
    #
    #   send_message_F7()
    #
    # -------------------------------------------------------------------------
    def send_message_F7(self, timer):
        # Send a message if thre is one available to send. This runs on a
        # timer. After the message gets the sent message indicator in the
        # buffer gets incremented. This function takes about 50-90 us depending
        # on how many messages are sent.
        if not self.stop_CAN_send_flag:
            tim_0 = time.ticks_us()
            num_mess = (self.current_idx - self.sent_idx) % _CAN_SEND_BUFFER_LEN

            if num_mess == 0:
                return                                                                                          # No messages to send so return

            tx_status = stm.mem8[self.CAN1_TSR_3] & 0x1C                                                        # Check the transmit mailboxes on the STM32

            if tx_status == 0x00:                                                                               # Can send up to 3 messages!
                return                                                                                          # No space in the transmit mailbox yet so return
            elif tx_status == 0x1C and num_mess >= 3:
                irq_state = pyb.disable_irq()                                                                   # Can send up to 3 messages and there are at least to send!
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1

            elif tx_status == 0x1C and num_mess == 2:
                irq_state = pyb.disable_irq()                                                                   # Can send up to 3 messages, but only 1 available to send!
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1

            elif tx_status == 0x1C and num_mess == 1:                                                           # Can send up to 3 messages, but only 1 available to send!
                irq_state = pyb.disable_irq()
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1

            elif (tx_status == 0x18 or tx_status == 0x0C or tx_status == 0x14) and num_mess >= 2:               # Can send 2 messages only, and there at least 2 to send
                irq_state = pyb.disable_irq()
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1

            elif (tx_status == 0x18 or tx_status == 0x0C or tx_status == 0x14) and num_mess == 1:               # Can send 2 messages only, and thre is only 1 message to send
                irq_state = pyb.disable_irq()
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1
            else:
                irq_state = pyb.disable_irq()                                                                   # Can only send 1 message, and there is at least 1 message to send
                self.can.send(self.can_send_buffer_mv[self.sent_idx][self.can_send_len_idx[self.sent_idx]], self.can_send_ids[self.sent_idx])
                pyb.enable_irq(irq_state)
                self.sent_idx = (self.sent_idx + 1) % _CAN_SEND_BUFFER_LEN
                self.send_counter += 1

            tim_1 = time.ticks_us()
            self.PARAM_DICT['SEND_MESSAGE_TIMER'] = time.ticks_diff(tim_1, tim_0)
