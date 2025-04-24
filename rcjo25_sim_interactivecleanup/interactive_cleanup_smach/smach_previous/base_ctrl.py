#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from hsr_ros.srv import OdomBase  # サービス定義はROS 1のものをそのまま利用（必要に応じてROS 2用に変換）

class BaseControlClient(Node):
    def __init__(self):
        super().__init__('base_control_client')
        self.client = self.create_client(OdomBase, '/robot_ctrl/base_ctrl')

    def send_command(self, send_msg):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスが利用可能になるまで待機中...')
        request = OdomBase.Request()
        request.send_str = send_msg
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().res_str
        else:
            self.get_logger().error('サービス呼び出しに失敗しました: %r' % future.exception())
            return None

def processing_ros2(client_node, send_msg):
    """ 引数を'/robot_ctrl/base_ctrl'に送る """
    response = client_node.send_command(send_msg)
    return response

def main(args=None):
    rclpy.init(args=args)
    client_node = BaseControlClient()
    print("do processing_ros2() @robot_motion_ros2.py")
    # result = processing_ros2(client_node, "")
    # print(f"Result: {result}")
    # result = processing_ros2(client_node, "X:50")
    # print(f"Result: {result}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()