
import micropython                                  # type: ignore
import pyb                                          # type: ignore
from lib.CtrlFactory.CtrlFactory import CtrlFactory
import time
from array import array


import json


class Reporter:
    def __init__(self, ctrlFact : CtrlFactory, params_file:str):

        with open(params_file, "r") as f:
            self.params = json.load(f)

        self.ctrlFact = ctrlFact
        self.reporter_ref = self.reporter
        self.device_parameters = self.ctrlFact.device_parameters
        self.count =0
        self.reporter_frq= self.device_parameters.TIMER_REPORTER['FREQ']
        self.sending = False

        # reporter dimension of the robot msg and ctrl msg
        self.rep_robot_msg_dim = self.params["reporter"]['dim_robot_msg']
        self.rep_ctrl_msg_dim = self.params["reporter"]['dim_ctrl_msg']
        self.rep_dec_msg_dim = self.params["reporter"]['dim_decorator_msg']
        self.rep_ctrlFact_msg_dim = self.params["reporter"]['dim_ctrl_fact_msg']

        self.dim_msg = (self.rep_robot_msg_dim +
                                self.rep_ctrl_msg_dim +
                                self.rep_dec_msg_dim +
                                self.rep_ctrlFact_msg_dim)
        self.ctrlFact_msg = [0.0]*self.dim_msg

        self.rp_msg = [0.0]*(self.dim_msg+1)
        self.msg_floats = array('f', [0.0] * (self.dim_msg + 1))

        # Zero-copy byte view of the SAME array
        self.msg_buffer = memoryview(self.msg_floats)

    def start_timer(self):
        self.report = False
        self.timer_reporter = pyb.Timer(
            self.device_parameters.TIMER_REPORTER['ID'],
            freq= self.reporter_frq,
            callback=self.report_callback
        )

    def reporter(self, _):
        pass

    def create_report(self, out, index) ->int:
        out[index] = self.count / self.reporter_frq
        index += 1
        return self.ctrlFact.create_report(out, index)


    def enable_reporter(self):
        self.report = True

    def disable_reporter(self):
        self.report = False


    def report_callback(self, timer_object):
        if self.report and not getattr(self, "sending", False):
            try:
                micropython.schedule(self.reporter_ref, 0)
                self.count+=1
            except RuntimeError as e:
                pass

