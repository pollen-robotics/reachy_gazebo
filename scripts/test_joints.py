#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: test_joints.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: Monday, October 18 2021
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event
from std_msgs.msg import Float64MultiArray
from copy import copy
#import tf_transformations
from geometry_msgs.msg import Quaternion, Point
import numpy as np
import time
import random

JOINTS_NAMES = ['l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw', 'l_elbow_pitch', 'l_forearm_yaw', 'l_wrist_pitch', 'l_wrist_roll', 'r_shoulder_pitch', 'r_shoulder_roll',
                'r_arm_yaw', 'r_elbow_pitch', 'r_forearm_yaw', 'r_wrist_pitch', 'r_wrist_roll', 'orbita_roll', 'orbita_pitch', 'orbita_yaw', 'l_antenna', 'r_antenna', 'l_gripper', 'r_gripper']


JOINTS_DICT = {'l_shoulder_pitch': 0.0, 'l_shoulder_roll': 0.0, 'l_arm_yaw': 0.0, 'l_elbow_pitch': np.radians(-90.0), 'l_forearm_yaw': 0.0, 'l_wrist_pitch': 0.0, 'l_wrist_roll': 0.0, 'r_shoulder_pitch': 0.0, 'r_shoulder_roll': 0.0,
               'r_arm_yaw': 0.0, 'r_elbow_pitch': np.radians(-90.0), 'r_forearm_yaw': 0.0, 'r_wrist_pitch': 0.0, 'r_wrist_roll': 0.0, 'orbita_roll': 0.0, 'orbita_pitch': 0.0, 'orbita_yaw': 0.0, 'l_antenna': np.radians(70.0), 'r_antenna': np.radians(-70.0), 'l_gripper': 0.0, 'r_gripper': 0.0}


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.logger = self.get_logger()
        self.t = 0.0

    def timer_callback(self):
        msg = Float64MultiArray()



        s = np.radians(15.0)*np.sin(2.0*np.pi*0.5*self.t)
        # # s = 0.0
        self.t += self.timer_period
        # for j in JOINTS_NAMES:
        #     msg.joint_names.append(j)
        #     p.positions.append(s)

        # for j, v in JOINTS_DICT.items():
        #     msg.joint_names.append(j)
        #     p.positions.append(v)

        # msg.points.append(p)

        for j, v in JOINTS_DICT.items():
            msg.data.append(s)

        self.logger.warn('msg: {}'.format(msg))

        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
