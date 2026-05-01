#!/usr/bin/env python3

import socket
import json
import time
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray
import numpy as np

from rcl_interfaces.msg import SetParametersResult

from std_srvs.srv import Trigger

from micropython_logger.param import load_controller_params

"""
This class is design for fast comunication 
in particular this class manages comunication of the kind
rostopic --> udp
the only exeption (at the moment) is to add the ros service for enabling and change K and B for the teloep
"""


class ROS2udp(Node):  
    def __init__(self):
        super().__init__('ros2_udp')
        self.declare_parameter('robot', 'Tails')
        self.declare_parameter('ctrl', 'Oscillator')

        robot = self.get_parameter('robot').get_parameter_value().string_value
        self.ctrl = self.get_parameter('ctrl').get_parameter_value().string_value

        self.get_logger().info(f"Robot: {robot}")
        self.get_logger().info(f"Ctrl: {self.ctrl}")

        self.port = 12345

        if robot == "Tails":
            # self.receiver_ip= '192.168.0.68'
            self.receiver_ip = '192.168.0.202' # in the case in which tails is the access point
        elif robot == "Shadow":
            self.receiver_ip = '192.168.0.254'
        else:
            raise Exception("Wrong Robot assignement")
        self.get_logger().info(f"Connecting to IP: {self.receiver_ip}, Port: {self.port}")

        if self.ctrl == "Teleop":
            header = 3
        if self.ctrl == "SMWalking":
            header = 4
        elif self.ctrl == "OscillatorPos":
            header = 6
        elif self.ctrl == "OscillatorForce":
            header = 5
        elif self.ctrl == "FF":
            header = 1
        elif self.ctrl == "FB":
            header = 2
        else:
            header = 0

        #Dynamically Load Tuning Parameters
        self.param_idx_map = load_controller_params(self, self.ctrl)

        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # wait a second the socket to start
        time.sleep(1)

        self.header_msg = np.array([header])
        self.ctrl_msg = np.zeros(20)
        # robot msg:
        # list[int], by default all zero
        # if index 0 ==1 --> imu calibration
        # elif index 2 ==1 --> load-cell calibration[int]
        self.robot_msg = np.zeros(4)

        if self.ctrl == "Teleop":
            self.en, self.K, self.B = None, None, None
            # subscriber to the ros node containing the message to convert to udp
            self.sub_teleop = self.create_subscription(Float64MultiArray,
                                                       f'ctrl_command_{robot}',
                                                       self.callback_telop_subs, 10)
            self.get_logger().info("Teleop mode active.")

        self.service_IMU_Cal = self.create_service(Trigger, f'{robot}/IMU_calibration', self.imu_cal_callback_)
        self.service_Load_Cel_cal = self.create_service(Trigger, f'{robot}/LoadCell_calibration', self.LoadCell_cal_callback_)
        self.add_on_set_parameters_callback(self.param_update_callback)

    def callback_telop_subs(self, msg):
        """Update with teleop changes"""
        try:
            rp = list(msg.data)

            enable_idx = self.param_idx_map.get("enable_controller", 0)
            k_idx = self.param_idx_map.get("K_stiffness", 3)
            b_idx = self.param_idx_map.get("B_damping", 4)

            rp[0] = self.ctrl_msg[enable_idx]
            rp[3] = self.ctrl_msg[k_idx]
            rp[4] = self.ctrl_msg[b_idx]

            out_str = ",".join(str(x) for x in rp) + "\n"
            # print(msg)
            self.s.sendto(out_str.encode(), (self.receiver_ip, self.port))

        except AttributeError as e:
            print(e)
            print("Client not yet connected")
        except socket.timeout as e:
            print(e)
            print("Client not yet connected")
        except BrokenPipeError as e:
            print(e)
            print("Client not yet connected")

    def imu_cal_callback_(self, req, res):
        self.get_logger().info("IMU Cal called!")
        self.robot_msg[0] = 1.0
        success = self.send_and_Ack_msg()
        if success:
            res.message = "IMU cal completed successfully."
        else:
            res.message = "IMU cal Not completed."
        # restore default value
        self.robot_msg[0] = 0.0
        return res

    def LoadCell_cal_callback_(self, req, res):
        self.get_logger().info("LOAD cell cal called!")
        self.robot_msg[1] = 1.0
        success = self.send_and_Ack_msg()
        if success:
            res.message = "Load cell cal completed successfully."
        else:
            res.message = "Load cell cal Not completed."
        self.robot_msg[1] = 0.0
        return res

    def send_and_Ack_msg(self):
        success = False

        try:
            # build payload
            rp = np.zeros(25, dtype=np.float32)
            rp[0] = self.header_msg[0]
            rp[1:21] = self.ctrl_msg
            rp[21:25] = self.robot_msg

            msg = rp.tobytes() + b'\n' # no newline needed for UDP

            # send
            start = time.perf_counter()
            self.s.sendto(msg, (self.receiver_ip, self.port))

            # wait for ACK
            self.s.settimeout(0.1)  # 100 ms
            data, addr = self.s.recvfrom(128)

            if data == msg:
                elapsed_ms = (time.perf_counter() - start) * 1000
                print(f"UDP RTT: {elapsed_ms:.2f} ms")
                success = True

        except socket.timeout:
            self.get_logger().warn("UDP ACK timeout")
        except Exception as e:
            self.get_logger().error(f"UDP error:{e}")

        return success

    def param_update_callback(self, params):
        for param in params:
            if param.name in self.param_idx_map:
                idx = self.param_idx_map[param.name]

                if param.type_ == param.Type.DOUBLE:
                    val = param.value
                elif param.type_ == param.Type.INTEGER:
                    val = float(param.value)
                elif param.type_ == param.Type.BOOL:
                    val = 1.0 if param.value else 0.0
                else:
                    continue 
                self.ctrl_msg[idx] = val
                self.get_logger().info(f"Updated {param.name} to {val} at index {idx}")
        if self.ctrl != "Teleop":
            self.send_and_Ack_msg()
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    r2u = ROS2udp()
    rclpy.spin(r2u)
    r2u.s.close()
    r2u.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()