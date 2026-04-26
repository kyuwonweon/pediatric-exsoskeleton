#!/usr/bin/env python3

import socket
import json
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

# host address is the computer address
# port is dependent by which robot is transmitting
class udp2ROS:
    def __init__(self, host_adress= '192.168.0.252'):
        
        # define ros interfaces
        rospy.init_node('udp2ROS', anonymous=True)
        print("Ros Init success")
        robot = rospy.get_param('~robot', "CubeMars")
        self.pub_robot = rospy.Publisher('robot_state_'+robot, Float64MultiArray, queue_size=10)   
        self.pub_ctrl = rospy.Publisher('ctrl_state_'+robot, Float64MultiArray, queue_size=10)     
        print("Ros pubs")

        if robot =="Tails": port= 12345
        elif robot == "Shadow": port = 12344
        elif robot == "CubeMars": port = 12346

        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(host_adress, port)
        self.s.bind((host_adress, port))
        self.s.settimeout(0.001)
        print(f"Listening on {host_adress}:{port}")

        # quantify stats
        self.last_time = None
        self.packet_count = 0
        self.total_time = 0.0
        self.last_index = None
        self.lost_packets = 0
        self.last_pub_time = time.time()
        self.pub_count = 0
        self.receiving_addr = ""
        self.new_msg_flag = False

        # dimensions of the messages (these are defined in the json inside the board) 
        # TODO: potentially at one point copy json
        self.dim_robot_msg = 10
        self.dim_ctrl_msg = 10
        self.dim_dec_msg = 5
        self.dim_ctrl_fact_msg =1
        self.dim_reporter_msg = 1
        self.len_msg = (self.dim_robot_msg + self.dim_ctrl_msg + 
                        self.dim_dec_msg + self.dim_ctrl_fact_msg + self.dim_reporter_msg)

    def _log_stats(self):
        if self.packet_count > 0:
            avg_freq = self.packet_count / self.total_time if self.total_time > 0 else 0
            avg_pub_freq = self.pub_count / (time.time() - self.last_pub_time)
            print(f"[Stats] Rec from addr: {self.receiving_addr} | Rx freq: {avg_freq:.1f} Hz | Lost: {self.lost_packets} | ROS pub freq: {avg_pub_freq:.1f} Hz")
            # reset
            self.packet_count = 0
            self.total_time = 0
            self.pub_count = 0
            self.last_pub_time = time.time()

    def main_loop(self):
        while not rospy.is_shutdown():
            
            data = b''
            try:
                while not rospy.is_shutdown():
                    try:
                        data, self.receiving_addr = self.s.recvfrom(1024) # check this
                        # self.s.sendto(data, self.receiving_addr)
                        if not data:
                            print("Connection closed by client --> waiting new client")
                            break
                        else: 
                            now = time.time()
                            if self.last_time:
                                self.total_time += (now - self.last_time)
                            self.last_time = now
                            self.packet_count += 1
                            # print(buffer)

                            expected_len = 4 * self.len_msg
                            if len(data) != expected_len:
                                rospy.logwarn(f"Invalid data size: {len(data)} bytes")
                                continue
                            self.new_msg_flag = True
                    except socket.timeout:
                        # no client connected within timeout — loop again
                        pass
                    except KeyboardInterrupt:
                        break
                    
                    
                    if self.new_msg_flag:
                        try:
                            # TODO: maybe add a quality check indicator (for instance one of the values should be always constant)
                            msg_splitted = np.frombuffer(data, dtype=np.float32)
                            # msg_splitted = msg.split(",")
                            start_index = 0
                            msg_header = [float(v) for v in msg_splitted[start_index: start_index + self.dim_reporter_msg]] # reporter header
                            start_index += self.dim_reporter_msg
                            # ctrl Fact
                            msg_ctrl_fact = [float(v) for v in msg_splitted[start_index: start_index + self.dim_ctrl_fact_msg]]
                            start_index += self.dim_ctrl_fact_msg
                            # CTRL  
                            msg_ctrl = [float(v) for v in msg_splitted[start_index: start_index + self.dim_ctrl_msg]]
                            start_index += self.dim_ctrl_msg
                            # decorator
                            msg_dec = [float(v) for v in msg_splitted[start_index: start_index + self.dim_dec_msg]]
                            start_index += self.dim_dec_msg

                            msg_robot =  [float(v) for v in msg_splitted[start_index: (start_index+ self.dim_robot_msg)]]
                            
                            # add header to both messages
                            msg_robot = msg_header+ msg_robot
                            msg_ctrl = msg_header + msg_ctrl

                            msg_ros_robot = Float64MultiArray(data=msg_robot)
                            msg_ros_ctrl = Float64MultiArray(data=msg_ctrl)
                            self.pub_robot.publish(msg_ros_robot)
                            self.pub_ctrl.publish(msg_ros_ctrl)
                            self.new_msg_flag =  False

                            # --- Metrics ---
                            self.pub_count += 1

                            # Check for packet loss (assuming first value is index)
                            if self.last_index is not None and len(msg_ctrl) > 0:
                                idx = int(msg_header[0])
                                if idx != self.last_index + 1:
                                    self.lost_packets += (idx - self.last_index - 1)
                                self.last_index = idx
                            else:
                                if len(msg_ctrl) > 0:
                                    self.last_index = int(msg_header[0])

                            # print stats every 5s
                            if time.time() - self.last_pub_time > 5:
                                self._log_stats()
                        
                        except KeyboardInterrupt:
                            break
                        except ValueError as e:
                            print(e)
                            # rospy.logwarn(f"Invalid data: {msg_splitted}")
                            # skip, don't close the socket
                            continue
                            
            except OSError as e:
                rospy.logwarn(f"Socket error: {e}")
            finally:
                self.s.close()
                print("Connection ended, waiting for new client...")


  
if __name__ == "__main__":

    ## in the case in which it is the leg the host
    access_point_IP = '192.168.4.1'
    computer_IP= '192.168.4.16'

    # u2R = udp2ROS(host_adress=computer_IP)
    u2R = udp2ROS()
    try:
        u2R.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        u2R.s.close()
        print("Connection ended, waiting for new client...")
