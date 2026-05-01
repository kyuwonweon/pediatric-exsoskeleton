
import micropython                                  # type: ignore
import pyb                                          # type: ignore
import time, socket
from lib.IO.InputManagerInterface import InputManagerInterface
from lib.Controllers.ControllerInterface import Controller
from lib.CtrlFactory.CtrlFactory import CtrlFactory
import struct


class IMTCP(InputManagerInterface):
    def __init__(self, ctrlFact: CtrlFactory,
                 receiver_ip='192.168.0.68'): #, robot:Robot, controller:Controller):
        super().__init__(ctrlFact, receiver_ip)
        self.receiver_ref = self.receiver
        self.connect_socket()
        self.initialize_timer()


        print("Initialization complete")

    def connect_socket(self):
        # for port in self.port2sockets:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.receiver_ip, self.single_port))
        self.socket.listen(1)
        self.socket.settimeout(0.001)

    def check_connected_client(self, port):
        """
        Identify if client is connected. This will fille the port2conn dictionary (if clients connect).
        Returns:

        """
        try:
            # print("waiting connection!")
            conn, addr = self.socket.accept()  # new socket is conn
            conn.settimeout(0.01)
            print(f"Connected by {addr}")
            # all not none elements in port2conn are established connections
            self.connection = conn
        except OSError as e:
            if e.args[0] == 110 or e.args[0] == 11 or e.args[0] == 113:  # ERIMEDOUT or EAGAIN
                pass
            else:
                print(e)
        finally:
            self.running = False

    def process_input(self, data):
        if len(data) != len(self.rx_buffer):
            raise RuntimeError("Wrong data length")

        # Copy bytes (no allocation)
        self.rx_buffer[:] = data
        # Manually unpack each float into preallocated array
        for i in range(len(self.rx_floats)):
            start = i * 4
            end = start + 4
            self.rx_floats[i] = struct.unpack('<f', self.rx_buffer[start:end])[0]

        # Now rx_floats contains the decoded floats
        print("Received msg: ", self.rx_floats)

        if self.ctrlFact.processInputs(self.rx_floats):
            # send confirmation of success
            self.connection.send(data)
        else:
            print("bad processed")
            # else send different message
            self.connection.send(b'')


    def receiver(self, _):
        if self.running:
            return
        self.running = True

        # connection already established (the connection is not none)
        if self.connection:
            try:
                data = self.connection.recv(256) # receives bytes
                if data:
                    self.process_input(data)
                else:
                    # When connection lost put controller to idle and all values to zero
                    self.ctrlFact.processInputs([0.0] * self.ctrlFact.dim_msg)
                    print("Connection closed")
                    self.connection.close()
                    self.connection = None
            except OSError as e:
                if e.args[0] ==110 :  # ERIMEDOUT
                    pass
                else:
                    print("Socket error in reception:", e)
                    self.ctrlFact.processInputs([0.0] * self.ctrlFact.dim_msg)
                    print("Connection closed")
                    self.connection.close()
                    self.connection = None
            finally:
                self.running = False
        else:
            self.check_connected_client(self.single_port)

