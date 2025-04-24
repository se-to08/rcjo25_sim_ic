#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from interactive_cleanup.msg import InteractiveCleanupMsg
from rclpy.duration import Duration
# 同じディレクトリにある場合
# from .task_common_send_ready import TaskCommonSendReady

class TaskCommonSendReady(Node):
    def __init__(self):
        super().__init__('task_common_send_ready_node2')
        self.publisher = self.create_publisher(InteractiveCleanupMsg, "/interactive_cleanup/message/to_moderator", 10)
        self.sub_msg = None
        self.subscriber = self.create_subscription(
            InteractiveCleanupMsg,
            "/interactive_cleanup/message/to_robot",
            self.icu_msg_cb,
            10
        )
        self.get_logger().info("TaskCommonSendReady2 node started.")

    def icu_msg_cb(self, msg):#アバターからのメッセージ
        self.get_logger().info(f"Received message: '{msg.message}'")
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

def main(args=None):
    rclpy.init(args=args)
    task_common_send_ready = TaskCommonSendReady()
    try:
        rclpy.spin(task_common_send_ready)
    except KeyboardInterrupt:
        pass
    finally:
        task_common_send_ready.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()