#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from interactive_cleanup.msg import InteractiveCleanupMsg
from rclpy.duration import Duration
import time 
# 同じディレクトリにある場合
# from .task_common import TaskCommon

class TaskCommon(Node):
    def __init__(self):
        super().__init__('task_common_node')
        self.publisher = self.create_publisher(InteractiveCleanupMsg, "/interactive_cleanup/message/to_moderator", 10)
        self.sub_msg = None
        self.subscriber = self.create_subscription(
            InteractiveCleanupMsg,
            "/interactive_cleanup/message/to_robot",
            self.icu_msg_cb,
            10
        )
        self.get_logger().info("TaskCommon2 node started.")
        self.Get_Pick_It_Up_Flag = False
        self.Get_Clean_Up_Flag = False

    def icu_msg_cb(self, msg):#アバターからのメッセージ
        self.get_logger().info(f"Received message: '{msg.message}'")
        if str(msg.message) == "Pick_it_up!":		self.Get_Pick_It_Up_Flag = True
        elif str(msg.message) == "Clean_up!":		self.Get_Clean_Up_Flag = True
        self.sub_msg = msg

    def send_msg(self,message,detail):
        p_msg = InteractiveCleanupMsg()
        p_msg.message = message
        p_msg.detail = detail
        self.publisher.publish(p_msg)
        self.get_logger().info(f"Published: message='{message}', detail='{detail}'")

    def get_msg(self):
        self.get_logger().info('MESSAGE_GETTED\n') 
        return self.sub_msg

    def wait_pick_it_up_command(self):
        self.get_logger().info('Waiting Pick_It_Up message.\n')
        while rclpy.ok():
            if self.get_pick_it_up_state() == True:
                break
            else:
                self.get_logger().info("waiting for Pick_It_Up.")
                rclpy.spin_once(self)
                time.sleep(1)

    def get_pick_it_up_state(self):
        return self.Get_Pick_It_Up_Flag

    def wait_clean_up_command(self):
        self.get_logger().info("Waiting Clean_Up message.")
        while rclpy.ok():
            if self.get_clean_up_state() == True:
                break
            else:
                self.get_logger().info("waiting for Clean_Up.")
                rclpy.spin_once(self)
                time.sleep(1)

    def get_clean_up_state(self):
        return self.Get_Clean_Up_Flag

def main(args=None):
    rclpy.init(args=args)
    task_common = TaskCommon()
    try:
        rclpy.spin(task_common)
    except KeyboardInterrupt:
        pass
    finally:
        task_common.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()