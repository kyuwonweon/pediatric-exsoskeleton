#!/usr/bin/env python3

import socket
import json
import time
import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# TODO: do the board the server and this one client?

class MultiRobot(Node):
    def __init__(self):
        # define ros interfaces
        super().__init__('multi_robot')

        self.tails_pos = 0.0
        self.tails_vel = 0.0

        self.shadow_pos = 0.0
        self.shadow_vel = 0.0

        # Subscribers=
        self.sub_tails = self.create_subscription(Float64MultiArray,
                                                  'robot_state_Tails',
                                                  self.callbackTails, 10)
        self.sub_shadow = self.create_subscription(Float64MultiArray,
                                                   'robot_state_Shadow',
                                                   self.callbackShadow, 10)
        # Publishers
        self.pub_command_shadow = self.create_publisher(Float64MultiArray,
                                                        'ctrl_command_Shadow',
                                                        10)
        self.t0 = time.time()

    def callbackTails(self, data):
        self.tails_pos = data.data[1]
        self.tails_vel = data.data[2]

        msg_shadow = [0.0, self.tails_pos, self.tails_vel, 0.0, 0.0, 0.0]

        msg_ros_shadow = Float64MultiArray(data=msg_shadow)
        self.pub_command_shadow.publish(msg_ros_shadow)

    def callbackShadow(self, data):
        self.shadow_pos = data.data[1]
        self.shadow_vel = data.data[2]


def main(args=None):
    rclpy.init(args=args)
    mr = MultiRobot()
    rclpy.spin(mr)
    mr.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
