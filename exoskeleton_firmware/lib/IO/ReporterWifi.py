import errno

import micropython                                  # type: ignore
import pyb
# type: ignore

import time, socket
from lib import config, tunable_settings
from lib.ll_common import mode_switch
from lib.IO.ReporterInterface import Reporter
from lib.CtrlFactory.CtrlFactory import CtrlFactory

class ReporterWifi(Reporter):
    def __init__(self, ctrlFact:CtrlFactory,
                 params_file:str):
        super().__init__(ctrlFact, params_file)

        self.receiver_ip = self.params["reporter"]["receiver_ip"]
        # TODO: check if make sense to have two ports or a single one and different headers
        self.port = self.params["reporter"]["port"][tunable_settings.DEVICE_NAME]

        self.connect_socket()

        self.ctrl_freq = self.device_parameters.TIMER_CONTROLLER['FREQ']
        self.start_timer()

    def connect_socket(self):
        self.s = socket.socket()
        self.connection_failed = True
        try:
            self.s.connect((self.receiver_ip, self.port))
            self.connection_failed = False
        except OSError as e:
            print("Socket error:", e, ", Unable to use wifi --> stdout")
            self.connection_failed = True


    def reporter(self, _):
        if self.sending:
            return
        self.sending = True

        # sending out
        try:
            rp = self.create_report()
            # print(rp)
            msg = ",".join(str(x) for x in rp) + "\n"
            self.s.send(msg.encode())
        except OSError as e:
            print("Send error:", e)
            self.connection_failed = True # TODO: attempt to restablish the connection --> this I think sometimes crash
        finally:
            self.sending = False


    def report_callback(self, timer_object):
        if not(self.connection_failed):
            super().report_callback(timer_object)




