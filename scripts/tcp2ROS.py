#!/usr/bin/env python3

import socket
import json
import time
import rospy
from std_msgs.msg import Float64MultiArray

# TODO: do the board the server and this one client?

class tcp2ROS:
    def __init__(self, host_adress= '192.168.0.251', port= 12345):
        # define ros interfaces
        rospy.init_node('tcp2ROS')
        self.pub = rospy.Publisher('my_multiarray_topic', Float64MultiArray, queue_size=10)          
    
        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host_adress, port))
        self.s.listen(1) # TODO: check these
        self.s.settimeout(0.01)
        print(f"Listening on {host_adress}:{port}")

        # quantify stats
        self.last_time = None
        self.packet_count = 0
        self.total_time = 0.0
        self.last_index = None
        self.lost_packets = 0
        self.last_pub_time = time.time()
        self.pub_count = 0

    def _log_stats(self):
        if self.packet_count > 0:
            avg_freq = self.packet_count / self.total_time if self.total_time > 0 else 0
            avg_pub_freq = self.pub_count / (time.time() - self.last_pub_time)
            print(f"[Stats] Rx freq: {avg_freq:.1f} Hz | Lost: {self.lost_packets} | ROS pub freq: {avg_pub_freq:.1f} Hz")
            # reset
            self.packet_count = 0
            self.total_time = 0
            self.pub_count = 0
            self.last_pub_time = time.time()

    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                # print("waiting connection!")
                self.conn, addr = self.s.accept() # new socket is conn
                self.conn.settimeout(0.01)
                print(f"Connected by {addr}")
            except socket.timeout:
                # no client connected within timeout — loop again
                continue
            except KeyboardInterrupt:
                break

            buffer = ""
            try:
                while not rospy.is_shutdown():
                    try:
                        data = self.conn.recv(1024) # check this
                        if not data:
                            print("Connection closed by client --> waiting new client")
                            break
                        else: 
                            buffer += data.decode()
                            now = time.time()
                            if self.last_time:
                                self.total_time += (now - self.last_time)
                            self.last_time = now
                            self.packet_count += 1
                            # print(buffer)
                    except socket.timeout:
                        # no client connected within timeout — loop again
                        pass
                    except KeyboardInterrupt:
                        break
                    
                    # process buffer if any
                    while "\n" in buffer:
                        msg, buffer = buffer.split("\n", 1)
                        # print(buffer)

                        if not msg.strip():
                            continue
                        try:
                            msg_float = [float(v) for v in msg.split(",")]
                            msg_ros = Float64MultiArray(data=msg_float)
                            self.pub.publish(msg_ros)

                            # --- Metrics ---
                            self.pub_count += 1

                            # Check for packet loss (assuming first value is index)
                            if self.last_index is not None and len(msg_float) > 0:
                                idx = int(msg_float[0])
                                if idx != self.last_index + 1:
                                    self.lost_packets += (idx - self.last_index - 1)
                                self.last_index = idx
                            else:
                                if len(msg_float) > 0:
                                    self.last_index = int(msg_float[0])

                            # print stats every 5s
                            if time.time() - self.last_pub_time > 5:
                                self._log_stats()
                        
                        except KeyboardInterrupt:
                            break
                        except ValueError:
                            rospy.logwarn(f"Invalid data: {msg}")
                            # skip, don't close the socket
                            continue
                        
            except OSError as e:
                rospy.logwarn(f"Socket error: {e}")
            finally:
                self.conn.close()
                print("Connection ended, waiting for new client...")


  
if __name__ == "__main__":
    t2R = tcp2ROS()
    try:
        t2R.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        t2R.conn.close()
        print("Connection ended, waiting for new client...")
