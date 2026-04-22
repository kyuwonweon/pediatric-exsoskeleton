#!/usr/bin/env python3

import socket
import json
import time
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

from dynamic_reconfigure.server import Server
from micropython_logger.cfg import GuiOscillatorPosConfig, GuiOscillatorForceConfig, GuiFFCalConfig, GuiFBTranspConfig, GuiTeleopConfig, GuiSMWalkingConfig

from std_srvs.srv import Trigger, TriggerResponse

"""
This class is design for fast comunication 
in particular this class manages comunication of the kind
rostopic --> udp
the only exeption (at the moment) is to add the ros service for enabling and change K and B for the teloep
"""
class ROS2udp:  
    def __init__(self):
        # define ros interfaces
        rospy.init_node('ROS2udp', anonymous=True) # TODO: this can be cleaned 
        self.port =12345
        robot = rospy.get_param('~robot', "Tails") 
        print("Robot ", robot)
        if robot == "Tails":
            # self.receiver_ip= '192.168.0.68'
            self.receiver_ip= '192.168.4.1' # in the case in which tails is the access point
        elif robot == "Shadow":
            self.receiver_ip= '192.168.0.254'
        else:
            raise Exception("Wrong Robot assignement")
        print(self.receiver_ip, self.port)


        self.ctrl = rospy.get_param('~ctrl', "Oscillator") 
        print("Ctrl ", self.ctrl )
        if self.ctrl == "Teleop":
            header = 3
            config_params = GuiTeleopConfig
        if self.ctrl == "SMWalking":
            header = 4
            config_params = GuiSMWalkingConfig
        elif self.ctrl== "OscillatorPos":
            header = 6
            config_params = GuiOscillatorPosConfig
        elif self.ctrl== "OscillatorForce":
            header = 5
            config_params = GuiOscillatorForceConfig
        elif self.ctrl== "FF":
            header = 1
            config_params = GuiFFCalConfig
        elif self.ctrl == "FB":
            header = 2
            config_params = GuiFBTranspConfig

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
            rospy.Subscriber('ctrl_command_'+robot, Float64MultiArray, self.callback_telop_subs, queue_size=1) # check if queuing can create any issue...  
            srv = Server(config_params, self.callback_teleop_srv)
        else:
            # Create a Dynamic Reconfigure Server
            srv = Server(config_params, self.callback)
        service_IMU_Cal = rospy.Service(f'{robot}/IMU_calibration', Trigger, self.imu_cal_callback_)
        service_Load_Cel_cal = rospy.Service(f'{robot}/LoadCell_calibration', Trigger, self.LoadCell_cal_callback_)

    
    def callback_telop_subs(self, data):

        todo
        success = False
        try:
            # rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data.data)
            rp = [x for x in data.data]

            if self.ctrl == "Teleop":
                rp[0]= self.en
                rp[3] = self.K
                rp[4] = self.B

            # TODO check converting float directly to bytes is better
            msg = ",".join(str(x) for x in rp) + "\n"
            # print(msg)
            self.s.sendto(msg.encode(), (self.receiver_ip, self.port))

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

    def callback_teleop_srv(self, config, level):
        todo
        self.en = float(config['enable_controller'])
        self.K = config['K_stiffness']
        self.B = config['B_damping']
        print(f"Controller status: {self.en}, {self.K}, {self.B}")

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


    def send_msg(self):
        success = False
        try:
            rp = np.zeros(25, dtype=np.float32)
            rp[0:1] = self.header_msg 
            rp[1:21] = self.ctrl_msg
            rp[21:25] = self.robot_msg
            msg = rp.astype(np.float32).tobytes() + b'\n'
            # print(msg)
            self.s.sendto(msg, (self.receiver_ip, self.port))

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
    
    def send_and_Ack_msg(self):
        success = False

        try:
            # build payload
            rp = np.zeros(25, dtype=np.float32)
            rp[0] = self.header_msg 
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
            print("UDP ACK timeout")
        except Exception as e:
            print("UDP error:", e)

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


    def main_loop(self):
        while not rospy.is_shutdown():
            pass
  
if __name__ == "__main__":
    R2t = ROS2udp()
    try:
        R2t.main_loop()
    except KeyboardInterrupt:
        pass
