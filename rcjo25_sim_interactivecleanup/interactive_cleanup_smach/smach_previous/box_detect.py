#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from box_entry_gate_detection.srv import ExecuteCtrl  # サービス定義はROS 1のものをそのまま利用（必要に応じてROS 2用に変換）
from std_srvs.srv import SetBool

class BoxDetectionNode(Node):
    def __init__(self):
        super().__init__('box_detection_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.box_entry_gate_detection_client = self.create_client(
            ExecuteCtrl, '/box_entry_gate_detection/execute_ctrl')

    def wait_for_frame_exists(self, frame_name, timeout_sec):
        start_time = self.get_clock().now()
        while self.get_clock().now() < start_time + rclpy.time.Duration(seconds=timeout_sec):
            try:
                self.tf_buffer.lookup_transform(
                    self.get_namespace(),  # target frame (e.g., "map", "base_footprint")
                    frame_name,             # source frame
                    rclpy.time.Time(),
                    rclpy.time.Duration(seconds=1.0)
                )
                return True
            except tf2_ros.TransformException as ex:
                self.get_logger().info(f'Waiting for frame {frame_name}... ({ex})')
                rclpy.spin_once(self, timeout_sec=1.0)
        return False

    async def box_entry_gate_detection_ctrl(self, mode):
        while not self.box_entry_gate_detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /box_entry_gate_detection/execute_ctrl service...')
        request = ExecuteCtrl.Request()
        request.mode = mode
        future = self.box_entry_gate_detection_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False

async def wait_for_frame_exists_ros2(node, frame_name, timeout_sec):
    start_time = node.get_clock().now()
    while node.get_clock().now() < start_time + rclpy.time.Duration(seconds=timeout_sec):
        try:
            node.tf_buffer.lookup_transform(
                node.get_namespace(),  # target frame
                frame_name,             # source frame
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=1.0)
            )
            return True
        except tf2_ros.TransformException as ex:
            node.get_logger().info(f'Waiting for frame {frame_name}... ({ex})')
            rclpy.spin_once(node, timeout_sec=1.0)
    return False

async def box_entry_gate_detection_ctrl_ros2(node, mode):
    client = node.create_client(ExecuteCtrl, '/box_entry_gate_detection/execute_ctrl')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /box_entry_gate_detection/execute_ctrl service...')
    request = ExecuteCtrl.Request()
    request.mode = mode
    future = client.call_async(request)
    await future
    if future.result() is not None:
        return future.result().response
    else:
        node.get_logger().error(f'Service call failed: {future.exception()}')
        return False

async def main(args=None):
    rclpy.init(args=args)
    box_detect_node = BoxDetectionNode()
    try:
        box_detect_node.get_logger().info("do processing() @box_detect_ros2.py")
        result_ctrl = await box_detect_node.box_entry_gate_detection_ctrl(True)
        box_detect_node.get_logger().info(f"box_entry_gate_detection_ctrl(True) result: {result_ctrl}")
        res_place_pos = box_detect_node.wait_for_frame_exists("placeable_point", 30)
        box_detect_node.get_logger().info(f"res_place_pos: {res_place_pos}")
        await box_detect_node.box_entry_gate_detection_ctrl(False)
    finally:
        box_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())