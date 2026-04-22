#!/usr/bin/env python3

import socket
import json
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

from dynamic_reconfigure.server import Server
from micropython_logger.cfg import GuiOscillatorPosConfig, GuiOscillatorForceConfig, GuiFFCalConfig, GuiFBTranspConfig, GuiTeleopConfig, GuiSMWalkingConfig

from std_srvs.srv import Trigger, TriggerResponse

"""
This class is design for slower but more reliable comunication 
in particular this class manages comunication of the kind
dynamic config --> tcp
"""
class ROS2tcp:
    def __init__(self):
        # define ros interfaces
        rospy.init_node('ROS2tcp', anonymous=True) # TODO: this can be cleaned 
        port =12345
        robot = rospy.get_param('~robot', "CubeMars") 
        print("Robot ", robot)
        if robot == "Tails":
            receiver_ip= '192.168.0.68'
            # receiver_ip= '192.168.4.1' # in the case in which tails is the access point
        elif robot == "Shadow":
            receiver_ip= '192.168.0.254'
        elif robot == "CubeMars":
            receiver_ip= '192.168.0.202'
        else:
            raise Exception("Wrong Robot assignement")
        print(receiver_ip, port)


        ctrl = rospy.get_param('~ctrl', "Oscillator") 
        print("Ctrl ", ctrl )
        if ctrl == "Teleop":
            header = 3
            config_params = GuiTeleopConfig
        if ctrl == "SMWalking":
            header = 4
            config_params = GuiSMWalkingConfig
        elif ctrl== "OscillatorPos":
            header = 6
            config_params = GuiOscillatorPosConfig
        elif ctrl== "OscillatorForce":
            header = 5
            config_params = GuiOscillatorForceConfig
        elif ctrl== "FF":
            header = 1
            config_params = GuiFFCalConfig
        elif ctrl == "FB":
            header = 2
            config_params = GuiFBTranspConfig

        # define socket interfaces
        self.s = socket.socket()
        try:
            self.s.connect((receiver_ip, port))
            self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.s.settimeout(2.0)
        except OSError as e:
            print("Socket error:", e, ", Unable to use wifi --> stdout")

        # wait a second the socket to start
        time.sleep(1)

        self.header_msg = np.array([header])
        self.ctrl_msg = np.zeros(20)
        # robot msg:
        # list[int], by default all zero
        # if index 0 ==1 --> imu calibration
        # elif index 2 ==1 --> load-cell calibration[int]
        self.robot_msg = np.zeros(4) 


        # Create a Dynamic Reconfigure Server
        srv = Server(config_params, self.callback)
        service_IMU_Cal = rospy.Service('IMU_calibration', Trigger, self.imu_cal_callback_)
        service_Load_Cel_cal = rospy.Service('LoadCell_calibration', Trigger, self.LoadCell_cal_callback_)

        
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
            # print("--sent message")
            # self.s.settimeout(2.0)
            data = self.s.recv(len(msg))
            elapsed_ms = (time.perf_counter() - start) * 1000

            if data== msg:
                success = True
                print("----Received confirmation, Time to go and comeback: ", elapsed_ms)
            else:
                raise ValueError("Wrong message back")
                print("Rcv: ", data.decode())
                print("Sent: ", msg.decode())
                print("PROBLEM!!!!!")
                
            print("-------------------------------")
        except AttributeError as e:
            print(e)
            print("Client not yet connected")
        except socket.timeout as e:
            print(e)
            print("Client not yet connected")
        except BrokenPipeError as e:
            print(e)
            print("Client not yet connected")
        return success

    def callback(self, config, level):
        # rospy.loginfo("Reconfigure Request: {freq_value}, {enable_controller}, {use_FF}, {use_Energy_var_freq}, {K_stiffness}, {B_damping}".format(**config))
        # You can perform actions based on the new parameter values here
        # call back of dyn reconfig for the controller
        print("---------------------")
        for k_i, k in enumerate(config.keys()):
            if k!= "groups":
                print(k, config[k])
                self.ctrl_msg[k_i] = (float(config[k]))
        print("---------------------")
        
        self.send_and_Ack_msg()

        return config
    
    def imu_cal_callback_(self, req):
        rospy.loginfo("IMU Cal called!")
        self.robot_msg[0] = 1
        success = self.send_and_Ack_msg()
        if success:
            msg = "IMU cal completed successfully."
        else:
            msg = "IMU cal Not completed."
        # restore default value
        self.robot_msg[0] =0
        
        return TriggerResponse(success=success, message=msg)

    def LoadCell_cal_callback_(self, req):
        rospy.loginfo("LOAD cell cal called!")
        self.robot_msg[1] = 1
        success = self.send_and_Ack_msg()
        if success:
            msg = "Load cell cal completed successfully."
        else:
            msg = "Load cell cal Not completed."
        self.robot_msg[1] = 0
        return TriggerResponse(success=success, message=msg)

    
    def main_loop(self):
        while not rospy.is_shutdown():
            pass

  
if __name__ == "__main__":
    R2t = ROS2tcp()
    try:
        R2t.main_loop()
    except KeyboardInterrupt:
        pass
