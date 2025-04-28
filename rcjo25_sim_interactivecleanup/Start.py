#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
# import smach, smach_ros
from .element.ros_node_control3 import RosNodeController
from .element.run_control import RunControl

class Start(Node):
    def __init__(self):
        super().__init__('start')
        self.ros_node_control3_node = RosNodeController()
        self.run_ctrl_node = RunControl()

    def execute(self):
        self.ros_node_control3_node.processing("kill")
        self.run_ctrl_node.process_run_ctr("stop")

        self.ros_node_control3_node.processing("start")
        self.run_ctrl_node.process_run_ctr("start")


def main():
    rclpy.init()
    node = Start()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()