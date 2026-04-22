#!/usr/bin/env python3

import socket
import json
import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


# TODO: do the board the server and this one client?

class MultiRobot:
    def __init__(self):
        # define ros interfaces
        rospy.init_node('multi_robot')

        self.tails_pos = 0.0
        self.tails_vel = 0.0

        self.shadow_pos = 0.0
        self.shadow_vel = 0.0

        # Subscribers
        rospy.Subscriber('robot_state_Tails', Float64MultiArray, self.callbackTails) 

        # Publishers
        self.pub_command_shadow = rospy.Publisher('ctrl_command_Shadow', Float64MultiArray, queue_size=10) 
        self.t0= time.time()      
    
    def callbackTails(self, data):
        self.tails_pos = data.data[1]
        self.tails_vel = data.data[2]

        msg_shadow = [0, self.tails_pos, self.tails_vel, 0.0, 0.0, 0.0]
            
        msg_ros_shadow = Float64MultiArray(data=msg_shadow)
        self.pub_command_shadow.publish(msg_ros_shadow)

    def callbackShadow(self, data):
        self.shadow_pos = data.data[1]
        self.shadow_vel = data.data[2]
        
    def main_loop(self):
        while not rospy.is_shutdown():
            
            t = time.time() - self.t0
           
            rospy.sleep(1) # Sleep 

    

  
if __name__ == "__main__":
    mr = MultiRobot()
    mr.main_loop()
    