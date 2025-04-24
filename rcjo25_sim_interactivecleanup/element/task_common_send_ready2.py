#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from interactive_cleanup.msg import InteractiveCleanupMsg
from rclpy.duration import Duration
# 同じディレクトリにある場合
# from .task_common_send_ready import TaskCommonSendReady

class TaskCommonSendReady2(Node):
    def __init__(self):
        super().__init__('task_common_send_ready_node2')
        self.publisher = self.create_publisher(InteractiveCleanupMsg, "/interactive_cleanup/message/to_moderator", 10)
        self.received_ready = False
        self.subscriber = self.create_subscription(
            InteractiveCleanupMsg,
            "/interactive_cleanup/message/to_robot",
            self.icu_msg_cb,
            10
        )
        self.get_logger().info("TaskCommonSendReady2 node started.")

    def icu_msg_cb(self, msg):#アバターからのメッセージ
        self.get_logger().info(f"Received message: '{msg.message}'")
        if not self.received_ready and str(msg.message).strip() == 'Are_you_ready?':
            self.received_ready = True
            self.pub_task_msg("I_am_ready", "")

    def pub_task_msg(self, message, detail=""):
        send_msg = InteractiveCleanupMsg()
        send_msg.message = message
        send_msg.detail = detail
        self.publisher.publish(send_msg)
        self.get_logger().info(f"Published: message='{message}', detail='{detail}'")

def main(args=None):
    rclpy.init(args=args)
    task_common_send_ready2 = TaskCommonSendReady2()
    try:
        rclpy.spin(task_common_send_ready2)
    except KeyboardInterrupt:
        pass
    finally:
        task_common_send_ready2.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()