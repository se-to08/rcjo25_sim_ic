#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from hsr_interfaces.srv import GetFloat  # ROS 2 のサービス定義に合わせてください
from std_msgs.msg import String
import asyncio

# ROS 2ではsend_robot_motionに直接対応する機能がない場合があります。
# 必要に応じてactionlibなどを使用して実装する必要があります。
# ここでは簡単なString Publisherで代替します。
ROBOT_MOTION_TOPIC = '/robot_motion_command'

class ObjectDepthNode(Node):
    def __init__(self):
        super().__init__('object_depth_node')
        self.depth_client = self.create_client(GetFloat, '/robot_ctrl/get_object_depth')
        self.motion_publisher = self.create_publisher(String, ROBOT_MOTION_TOPIC, 10)

    async def get_obj_depth(self):
        self.get_logger().info("Get depth of grasping object")
        motion_msg = String()
        motion_msg.data = 'MEASUREMENT_POSE'  # ROS 2のモーションコマンドに合わせる
        self.motion_publisher.publish(motion_msg)
        self.get_logger().info(f'Published motion command: {motion_msg.data}')
        await asyncio.sleep(4.0)

        while not self.depth_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/get_object_depth service...')

        request = GetFloat.Request()
        future = self.depth_client.call_async(request)
        await future
        if future.result() is not None:
            depth = future.result().data
            self.get_logger().info(f'Object depth: {depth}')
            return depth + 0.05
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/get_object_depth')
            return 0.0  # エラーの場合のデフォルト値

async def main(args=None):
    rclpy.init(args=args)
    object_depth_node = ObjectDepthNode()
    try:
        depth = await object_depth_node.get_obj_depth()
        object_depth_node.get_logger().info(f"Depth from main: {depth}")
    finally:
        object_depth_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())