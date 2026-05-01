import errno

import micropython                                  # type: ignore
import pyb
# type: ignore
import json
import time, socket
from lib import config, tunable_settings
from lib.CtrlFactory.CtrlFactory import CtrlFactory
from lib.ll_common import mode_switch
from lib.IO.ReporterInterface import Reporter

import struct

from lib.ll_common.error_flag import print_error


class ReporterWifiUDP(Reporter):
    def __init__(self,  ctrlFact:CtrlFactory,
                 params_file:str, reporter_ip:str):
        super().__init__(ctrlFact, params_file)

        self.receiver_ip = reporter_ip #self.params["reporter"]["receiver_ip"]
        robot_name = ctrlFact.robot.robot_name
        self.port = self.params["reporter"]["port"][robot_name]

        self.connect_socket()

        self.ctrl_freq = self.device_parameters.TIMER_CONTROLLER['FREQ']
        self.start_timer()
        self.t0= time.ticks_ms()

    def connect_socket(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connection_failed = False
        # try:
        #     self.s.connect((self.receiver_ip, self.port))
        #     self.connection_failed = False
        # except OSError as e:
        #     print("Socket error:", e, ", Unable to use wifi --> stdout")
        #     self.connection_failed = True


    def reporter(self, _):
        if self.sending:
            return
        self.sending = True

        # sending out
        try:
            start = time.ticks_us()
            self.create_report(self.msg_floats, 0)
            end1 = time.ticks_us()
            if time.ticks_diff(end1, start) > 1000:
                print("reporter create report took:", time.ticks_diff(end1, start), "µs")

            start = time.ticks_us()
            self.s.sendto(self.msg_buffer, (self.receiver_ip, self.port))
            end1 = time.ticks_us()
            if time.ticks_diff(end1, start) > 1000:
                print("reporter send took:", time.ticks_diff(end1, start), "µs")
        except OSError as e:
            print("Send error:", e)
            # self.connection_failed = True
        finally:
            self.sending = False


    def report_callback(self, timer_object):
        if not(self.connection_failed):
            super().report_callback(timer_object)




