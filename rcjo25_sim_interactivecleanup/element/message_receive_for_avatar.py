#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from interactive_cleanup.msg import InteractiveCleanupMsg
from rclpy.duration import Duration

class MessageReceiveForAvatar(Node):
    def __init__(self):
        super().__init__('message_receive_for_avatar_node')
        self.sub_icu_msg = self.create_subscription(
            InteractiveCleanupMsg,
            "/interactive_cleanup/message/to_robot",
            self.icu_msg_cb,
            10
        )
        self.waiting_for_ready = True
        self.get_logger().info("Waiting for 'Are_you_ready?' message...")

    def icu_msg_cb(self, msg):#アバターからのメッセージ
        self.get_logger().info(f"Received message from avatar: '{msg.message}'")
        print(f"アバターからのメッセージ: '{msg.message}'") # メッセージをプリント

        if self.waiting_for_ready:
            if str(msg.message).strip() == 'Are_you_ready?':
                self.get_logger().info("Received 'Are_you_ready?'.")
                self.waiting_for_ready = False
                print("待機を終了します。")

def main(args=None):
    rclpy.init(args=args)
    message_receive_for_avatar = MessageReceiveForAvatar()
    try:
        rclpy.spin(message_receive_for_avatar)
    except KeyboardInterrupt:
        pass
    finally:
        message_receive_for_avatar.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()