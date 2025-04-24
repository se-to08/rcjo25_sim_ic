#!/usr/bin/env python3
#coding:utf-8

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import time  # For rospy.sleep() equivalent

from interactive_cleanup.msg import *
from .states.move import Move

class TaskCommon(Node):
     def __init__(self):
        super().__init__('task_common_node')
        self.pub_icu_msg = self.create_publisher(
            InteractiveCleanupMsg,
            '/interactive_cleanup/message/to_moderator',
            10
        )

    def pub_task_msg(message):
        self.send_msg = InteractiveCleanupMsg()
        self.send_msg.message = message
        self.pub_icu_msg.publish(self.send_msg)

    def wait_ready_and_send_ready_command():
        self.get_logger().info("Waiting Are_you_ready message start.")
        while rclpy.ok():
            if get_are_you_ready_state() == True:
                move.set_session_start_time(self.get_clock().now()) #move.pyを作る
                self.pub_task_msg("I_am_ready") 
                break
            else:
                self.get_logger().info("waiting for Are_you_ready.")
                time.sleep(1)

def main():
    rclpy.init()
    task_common = TaskCommon()

    # # Example usage
    # svc_name = '/example_service'
    # is_running = True

    # toggle_service.run(svc_name, is_running)

    rclpy.shutdown()

if __name__ == '__main__':
    main()