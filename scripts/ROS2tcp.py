#!/usr/bin/env python3

import socket
import json
import time
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray

from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

from micropython_logger.param import load_controller_params
"""
This class is design for slower but more reliable comunication 
in particular this class manages comunication of the kind
dynamic config --> tcp
"""


class ROS2tcp(Node):
    def __init__(self):
        # define ros interfaces
        super().__init__('ros2_tcp')
        self.declare_parameter('robot', 'CubeMars')
        self.declare_parameter('ctrl', 'Oscillator')

        robot = self.get_parameter('robot').get_parameter_value().string_value
        ctrl = self.get_parameter('ctrl').get_parameter_value().string_value 

        self.get_logger().info(f"Robot: {robot}")
        self.get_logger().info(f"Ctrl: {ctrl}")

        port = 12345

        if robot == "Tails":
            receiver_ip = '192.168.0.68'
            # receiver_ip= '192.168.4.1' # in the case in which tails is the access point
        elif robot == "Shadow":
            receiver_ip = '192.168.0.254'
        elif robot == "CubeMars":
            receiver_ip = '192.168.0.202'
        else:
            raise Exception("Wrong Robot assignement")
        self.get_logger().info(f"Connecting to IP: {receiver_ip}, Port: {port}")

        if ctrl == "Teleop":
            header = 3
        if ctrl == "SMWalking":
            header = 4
        elif ctrl == "OscillatorPos":
            header = 6
        elif ctrl == "OscillatorForce":
            header = 5
        elif ctrl == "FF":
            header = 1
        elif ctrl == "FB":
            header = 2
        else:
            header = 0  # Fallback

        # Dynamically Load Tuning Parameters
        self.param_idx_map = load_controller_params(self, ctrl)

        # define socket interfaces
        self.s = socket.socket()
        try:
            self.s.connect((receiver_ip, port))
            self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.s.settimeout(2.0)
        except OSError as e:
            self.get_logger().error(f"Socket error: {e} Unable to use wifi --> stdout")

        # wait a second the socket to start
        time.sleep(1)

        self.header_msg = np.array([header])
        self.ctrl_msg = np.zeros(20)
        # robot msg:
        # list[int], by default all zero
        # if index 0 ==1 --> imu calibration
        # elif index 2 ==1 --> load-cell calibration[int]
        self.robot_msg = np.zeros(4)

        self.service_IMU_Cal = self.create_service(Trigger, 'IMU_calibration',
                                                   self.imu_cal_callback_)

        self.service_Load_Cel_cal = self.create_service(
                                                Trigger,
                                                'LoadCell_calibration',
                                                self.LoadCell_cal_callback_)
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        
    def send_and_Ack_msg(self):
        success = False
        try:
            rp = np.zeros(25, dtype=np.float32)
            rp[0:1] = self.header_msg
            rp[1:21] = self.ctrl_msg
            rp[21:25] = self.robot_msg

            msg = rp.astype(np.float32).tobytes()
            # compute time to go and come bacl
            start = time.perf_counter()
            self.s.send(msg)
            # self.get_logger().info("--sent message")
            # self.s.settimeout(2.0)
            data = self.s.recv(len(msg))
            elapsed_ms = (time.perf_counter() - start) * 1000

            if data == msg:
                success = True
                self.get_logger().info(f"----Received confirmation, Time to go and comeback: {elapsed_ms:.2f} ms")
            else:
                self.get_logger().info("Wrong message back")
                self.get_logger().info(f"Rcv: {data}")
                self.get_logger().info(f"Sent: {msg.decode()}")
                self.get_logger().info("PROBLEM!!!!!")
                
            self.get_logger().info("-------------------------------")
        except AttributeError as e:
            self.get_logger().info(e)
            self.get_logger().info("Client not yet connected")
        except socket.timeout as e:
            self.get_logger().info(e)
            self.get_logger().info("Client not yet connected")
        except BrokenPipeError as e:
            self.get_logger().info(e)
            self.get_logger().info("Client not yet connected")
        return success

    def parameter_update_callback(self, params):
        # You can perform actions based on the new parameter values here
        # call back of dyn reconfig for the controller
        for param in params:
            if param.name in self.param_idx_map:
                idx = self.param_idx_map[param.name]

                if param.type_ == param.Type.DOUBLE:
                    val = param.double_value 
                elif param.type_ == param.Type.INTEGER:
                    val = float(param.integer_value)
                elif param.type_ == param.Type.BOOL:
                    val = 1.0 if param.bool_value else 0.0
                else:
                    continue 
                self.ctrl_msg[idx] = val
                self.get_logger().info(f"Updated {param.name} to {val} at index {idx}")
        self.send_and_Ack_msg()

        return SetParametersResult(successful=True)

    def imu_cal_callback_(self, req, res):
        self.get_logger().info("IMU Cal called!")
        self.robot_msg[0] = 1.0
        success = self.send_and_Ack_msg()
        res.success = success
        if success:
            res.message = "IMU cal completed successfully."
        else:
            res.message = "IMU cal Not completed."
        # restore default value
        self.robot_msg[0] = 0.0
        return res

    def LoadCell_cal_callback_(self, req, res):
        self.get_logger().info('LOAD cell cal called!')
        self.robot_msg[1] = 1.0
        success = self.send_and_Ack_msg()

        res.success = success
        if success:
            res.message = "Load cell cal completed successfully."
        else:
            res.message = "Load cell cal Not completed."
        self.robot_msg[1] = 0.0
        return res


def main(args=None):
    rclpy.init(args=args)
    r2t = ROS2tcp()
    rclpy.spin(r2t)
    r2t.s.close()
    r2t.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
