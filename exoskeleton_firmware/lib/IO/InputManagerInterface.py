
import micropython                                  # type: ignore
import pyb                                          # type: ignore
import array

from lib.CtrlFactory.CtrlFactory import CtrlFactory

"""
Jan 27/26 switching to single port architecture with header
[TCP/UDP Socket (with ctrl header)] --> [Input Manager] --> [CtrlFact: decoding and routing]
The CtrlFact manages also the ctrl decorators
"""

class InputManagerInterface:
    def __init__(self, ctrlFact: CtrlFactory,
                 receiver_ip='192.168.0.68'): #, robot:Robot, controller:Controller):

        self.receiver_ip = receiver_ip

        self.ctrlFact = ctrlFact
        self.single_port = 12345
        self.socket = None
        self.connection = None

        # port that receives the commands
        # this is the default one of the ctrlFact at the beginning but changes dynamically depending by which ctrl is activated
        # self.ports2ctrl = {ctrlFact.ctrl.ctrl_port: ctrlFact}

        #TODO: add decorators
        # each ctrl can come with additional decorators
        # each decorator has a port to communicate externally
        # for d in ctrl.decorators:
        #     self.ports2ctrl[d.ctrl_port] = d

        # each ctrl will also be associated with a socket and a connection in case the connection is open
        # self.port2sockets ={k: None for k in self.ports2ctrl.keys()}
        # self.port2connections = {k: None for k in self.ports2ctrl.keys()}


        self.device_parameters = self.ctrlFact.device_parameters
        self.running = False
        # 25 float32 = 100 bytes

        self.dim_msg = self.ctrlFact.dim_msg
        self.fmt_msg = '<{}f'.format(self.dim_msg)
        self.rx_buffer = bytearray(self.dim_msg * 4)
        self.rx_floats = array.array('f', self.rx_buffer)

    def connect_socket(self):
        pass

    def initialize_timer(self):

        self.timer_receiver = pyb.Timer(
            self.device_parameters.TIMER_INPUT_MANAGER['ID'],
            freq=self.device_parameters.TIMER_INPUT_MANAGER['FREQ'],
            callback=self.receiver_callback
        )
        print("Receiver connected")

    def process_input(self,data):
        pass

    def receiver(self, _):
        pass


    def receiver_callback(self, timer_object):
        # if self.report and not getattr(self, "sending", False):
        if not self.running:
            try:
                micropython.schedule(self.receiver_ref, 0)
            except RuntimeError as e:
                # print(e)
                # print("The queue filled for Input Manager")
                pass

