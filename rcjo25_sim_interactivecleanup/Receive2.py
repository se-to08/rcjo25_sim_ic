#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
import asyncio
from rclpy.node import Node
from geometry_msgs.msg import Point
from rcjo25_sim_interactivecleanup.element.tf_listener2_for_debug import TfListener2NodeForDebug
from rcjo25_sim_interactivecleanup.element.task_common import TaskCommon
from rcjo25_sim_interactivecleanup.element.person_region_estimator import PersonRegionEstimator
from rcjo25_sim_interactivecleanup.element.pointing_direction_estimator5 import PointingDirectionEstimator
from rcjo25_sim_interactivecleanup.element.change_pose import MoveToPoseClient


class Receive(Node):
    def __init__(self):
        super().__init__('receive')
        self.Initial_Position = Point()
        self.Object_Area = ""
        self.Pointing_Position1 = Point()
        self.Pointing_Position2 = Point()
        self.Pointing_Direction1 = int()
        self.Pointing_Direction2 = int()
        self.task_common_node = TaskCommon()
        self.Receive_Challenge_Times = 2
        self.tf_listener2_for_debug_node = TfListener2NodeForDebug()
        self.person_region_estimator_node = PersonRegionEstimator()
        self.pointing_direction_node = PointingDirectionEstimator()
        self.change_pose_node = MoveToPoseClient()

    async def execute(self):
        self.get_logger().info("11111111111111111111111111111111111111111111111111111111")
        self.get_logger().info("person フレーム")
        self.Initial_Position = self.tf_listener2_for_debug_node.get_tf("person", "map")
        if self.Initial_Position == Point():
            self.Initial_Position.x = 2.25
            self.Initial_Position.y = 0.0
        print("Person_Initial_position")
        print(self.Initial_Position)        
        
        self.get_logger().info("22222222222222222222222222222222222222222222222222222222")
        self.get_logger().info("ready送信")
        while True:
            msg = self.task_common_node.get_msg()
            if msg is not None:
                self.get_logger().info(f"Received message: '{msg.message}'")
                if msg.message == "Are_you_ready?":
                    self.get_logger().info("Are_you_ready? received, sending I_am_ready.")
                    self.task_common_node.send_msg("I_am_ready", "")
                    break
            else:
                self.get_logger().info("Waiting for message...")

            rclpy.spin_once(self.task_common_node)
            await asyncio.sleep(0.5)  # メッセージのチェック間隔を調整

        self.get_logger().info("33333333333333333333333333333333333333333333333333333333")
        self.get_logger().info("指差し認識開始１")
        for receive_challenge in range(self.Receive_Challenge_Times):
            self.change_pose_node.change_pose('initial_pose')  # poseを変更

            self.task_common_node.wait_pick_it_up_command()  # 命令待機
            rclpy.spin_once(self.task_common_node)  # コールバックを処理

            self.Pointing_Position1 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(self.Pointing_Position1)

            await asyncio.sleep(5.0)

            if self.Pointing_Position1 == Point():
                await asyncio.sleep(1.0)
                self.Pointing_Position1 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION1")
            print(self.Pointing_Position1)
            self.Object_Area = self.person_region_estimator_node.search_person_region(self.Pointing_Position1, self.Initial_Position)
            print("Object_Area:", self.Object_Area)

            await asyncio.sleep(1.0)

            # 指差し開始
            print("指差し開始")

            self.Pointing_Direction1 = self.pointing_direction_node.pointing_direction()
            self.task_common_node.wait_clean_up_command()

            self.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(self.Pointing_Position2)
            await asyncio.sleep(2.0)

            self.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(self.Pointing_Position2)
            await asyncio.sleep(5.0)

            if self.Pointing_Position2 == Point():
                await asyncio.sleep(1.5)
                self.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION2")
            print(self.Pointing_Position2)
            await asyncio.sleep(2.0)

            if self.Pointing_Position2 == Point():
                await asyncio.sleep(1.5)
                self.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION2")
            print(self.Pointing_Position2)
            await asyncio.sleep(2.0)

            self.Pointing_Direction2 = self.pointing_direction_node.pointing_direction()
            
            # 次のステップへ
            # Next_State = 'search'
            # return 'finish'

def main():
    rclpy.init()
    node = Receive()

    # 非同期実行
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.execute())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
