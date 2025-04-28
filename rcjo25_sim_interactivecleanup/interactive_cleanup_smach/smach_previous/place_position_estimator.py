#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener
from placeable_position_estimator_interfaces.srv import ExecuteCtrl  # ROS 2 のサービス定義に合わせてください
import asyncio

class PlaceablePositionEstimatorNode(Node):
    def __init__(self):
        super().__init__('placeable_position_estimator_wrapper')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.estimator_ctrl_client = self.create_client(ExecuteCtrl, "/placeable_position_estimator/execute_ctrl")

    async def wait_for_frame_exists(self, frame_name, timeout_sec):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < rclpy.time.Duration(seconds=timeout_sec):
            if self.tf_buffer.can_transform(self.get_namespace() + "map", frame_name, rclpy.time.Time(), rclpy.time.Duration(seconds=0.1)):
                return True
            else:
                await asyncio.sleep(1.0)
        return False

    async def placeable_position_estimator_ctrl(self, mode):
        while not self.estimator_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /placeable_position_estimator/execute_ctrl service...')
        request = ExecuteCtrl.Request()
        request.mode = mode
        future = self.estimator_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error('Service call failed for /placeable_position_estimator/execute_ctrl')
            return False

async def main(args=None):
    rclpy.init(args=args)
    estimator_node = PlaceablePositionEstimatorNode()
    try:
        frame_exists = await estimator_node.wait_for_frame_exists("some_frame", 5.0)
        estimator_node.get_logger().info(f"Frame exists: {frame_exists}")

        response = await estimator_node.placeable_position_estimator_ctrl(1) # 例: mode=1 で実行
        estimator_node.get_logger().info(f"Estimator control response: {response}")
    finally:
        estimator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())