
import micropython                                  # type: ignore
import pyb                                          # type: ignore
import time, socket
from lib.IO.InputManagerInterface import InputManagerInterface
from lib.CtrlFactory.CtrlFactory import CtrlFactory
import struct


class IMUDP(InputManagerInterface):
    def __init__(self, ctrlFact: CtrlFactory, receiver_ip='192.168.0.68'): #, robot:Robot, controller:Controller):
        super().__init__(ctrlFact, receiver_ip)
        self.receiver_ref = self.receiver
        self.connect_socket()
        self.initialize_timer()
        print("Initialization complete")

    def connect_socket(self):
        # for port in self.port2sockets:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.receiver_ip, self.single_port))
        self.socket.settimeout(0.001)

    def process_input(self, data):
        # uncomment this one for RTT
        # self.socket.sendto(data, addr)
        values = struct.unpack(self.fmt_msg, data)
        self.ctrlFact.processInputs([float(value) for value in values])

    def receiver(self, _):
        if self.running:
            return
        self.running = True
        # reception

        if self.socket:
            try:
                data, addr = self.socket.recvfrom(256)
                # print(data)
                if data:
                    self.process_input(data)
            except OSError as e:
                if e.args[0] == 110  or e.args[0]==11:# ERIMEDOUT
                    pass
                else:
                    print("Socket error in reception:", e)
            finally:
                self.running = False

        self.running = False
