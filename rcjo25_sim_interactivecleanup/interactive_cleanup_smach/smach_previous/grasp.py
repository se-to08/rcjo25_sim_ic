#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from hsr_interfaces.srv import GripperMove, IsGrasped, GripperControl, ObjectReachControl  # ROS 2のサービス定義に合わせてください
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time

# ROS 2ではsend_robot_motionに直接対応する機能がない場合があります。
# 必要に応じてactionlibなどを使用して実装する必要があります。
# ここでは簡単なString Publisherで代替します。
ROBOT_MOTION_TOPIC = '/robot_motion_command'

class GraspingNode(Node):
    def __init__(self):
        super().__init__('grasping_node')
        self.motion_publisher = self.create_publisher(String, ROBOT_MOTION_TOPIC, 10)
        self.gripper_move_client = self.create_client(GripperMove, '/robot_ctrl/gripper_move_to_target')
        self.is_grasped_client = self.create_client(IsGrasped, '/robot_ctrl/is_grasped')
        self.gripper_ctrl_client = self.create_client(GripperControl, '/robot_ctrl/gripper_open_and_close')
        self.object_reach_ctrl_client = self.create_client(ObjectReachControl, '/robot_ctrl/object_reach_ctrl')

    def adjustment_hand(self):
        motion_msg = String()
        motion_msg.data = 'LIFT_UP_HAND'  # ROS 2のモーションコマンドに合わせてください
        self.motion_publisher.publish(motion_msg)
        self.get_logger().info(f'Published motion command: {motion_msg.data}')

    async def gripper_move_to_target(self, target_frame_name, point_distance):
        while not self.gripper_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/gripper_move_to_target service...')
        request = GripperMove.Request()
        request.target_frame_name = target_frame_name
        request.point_distance = float(point_distance)  # ROS 2の型に合わせてfloatに変換
        future = self.gripper_move_client.call_async(request)
        await future
        if future.result() is not None:
            if future.result().is_moved:
                return 'SUCCESS'
            else:
                return 'FAILURE'
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/gripper_move_to_target')
            return 'FAILURE'

    async def judge(self):
        while not self.is_grasped_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/is_grasped service...')
        request = IsGrasped.Request()
        future = self.is_grasped_client.call_async(request)
        await future
        if future.result() is not None:
            if future.result().is_grasped:
                self.get_logger().info('object grasped !!!')
                return 'SUCCESS'
            else:
                self.get_logger().info('not grasped ...')
                return 'FAILURE'
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/is_grasped')
            return 'FAILURE'

    async def gripper_close_hand(self):
        while not self.gripper_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/gripper_open_and_close service...')
        request = GripperControl.Request()
        request.position = 0.00
        future = self.gripper_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            if future.result().is_moved:
                return 'SUCCESS'
            else:
                return 'FAILURE'
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/gripper_open_and_close (close)')
            return 'FAILURE'

    async def gripper_open_hand(self):
        while not self.gripper_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/gripper_open_and_close service...')
        request = GripperControl.Request()
        request.position = 1.57
        future = self.gripper_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            if future.result().is_moved:
                return 'SUCCESS'
            else:
                return 'FAILURE'
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/gripper_open_and_close (open)')
            return 'FAILURE'

    async def processing_object_reach(self, target_obj):
        while not self.object_reach_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /robot_ctrl/object_reach_ctrl service...')
        request = ObjectReachControl.Request()
        request.target_object = target_obj
        self.get_logger().info(f'Target object for reach: {target_obj}')
        future = self.object_reach_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            if future.result().res_bool:
                self.get_logger().info(f'target : {target_obj} object_reach ok')
                return 'SUCCESS'
            else:
                self.get_logger().info(f'target : {target_obj} NOT object_reach')
                return 'FAILURE'
        else:
            self.get_logger().error('Service call failed for /robot_ctrl/object_reach_ctrl')
            return 'FAILURE'

async def main(args=None):
    rclpy.init(args=args)
    grasping_node = GraspingNode()
    try:
        grasping_node.get_logger().info("do processing() @grasping_node")
        judge_result = await grasping_node.judge()
        grasping_node.get_logger().info(f'Judge result: {judge_result}')
        open_result = await grasping_node.gripper_open_hand()
        grasping_node.get_logger().info(f'Gripper open result: {open_result}')
        close_result = await grasping_node.gripper_close_hand()
        grasping_node.get_logger().info(f'Gripper close result: {close_result}')
        # await grasping_node.processing() # 元のコードにはprocessing()という関数はありません
    finally:
        grasping_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())