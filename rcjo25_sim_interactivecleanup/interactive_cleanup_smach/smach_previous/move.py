#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
# from sobit_navigation_module import SOBITNavigationLibraryPython # ROS 2には対応していない可能性
from actionlib_msgs.msg import GoalID
# from smach_files import task_commom # smach_files内のROS 1の機能は直接使えない
# from smach_files import send_robot_motion # smach_files内のROS 1の機能は直接使えない
import time
import math

# SOBITNavigationLibraryPython の代替を検討する必要があります。
# move_base (Navigation2) の ActionClient を使用するなどの方法が考えられます。
# ここでは代替として、簡単なナビゲーション関連のPublisherとService Clientのプレースホルダーを定義します。
NAV_GOAL_TOPIC = '/move_base_simple/goal' # 例: move_base のゴールPublisher
NAV_CANCEL_TOPIC = '/move_base/cancel' # 例: move_base のキャンセルPublisher
# task_commom の機能 (get_task_failed_state) は、ROS 2のパラメータやTopicで状態を共有するなどの方法で代替する必要があります。
# send_robot_motion の機能は、ROS 2のActionClientやTopic Publisherで代替する必要があります。
BASE_CTRL_TOPIC = '/base_ctrl_command' # 例: ベース制御コマンドを送信するTopic

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        self.nav_goal_publisher = self.create_publisher(Pose, NAV_GOAL_TOPIC, 10)
        self.nav_cancel_publisher = self.create_publisher(GoalID, NAV_CANCEL_TOPIC, 10)
        self.base_ctrl_publisher = self.create_publisher(String, BASE_CTRL_TOPIC, 10)
        self.session_start_time = self.get_clock().now() # rclpy.Time() で初期化
        self.nav_goal_active = False # ゴールがアクティブかどうか
        self.nav_goal_id = GoalID() # ゴールIDのプレースホルダー

        # task_commom の代替として、パラメータを取得する例
        self.declare_parameter('task_failed', False)

    def get_task_failed_state(self):
        return self.get_parameter('task_failed').get_parameter_value().bool_value

    def set_session_start_time(self, start_time):
        self.session_start_time = start_time
        self.get_logger().info(f"SESSION START TIME: {self.session_start_time.to_sec()}")

    def feedback_cb(self, feedback):
        if self.get_clock().now() > (self.session_start_time + rclpy.time.Duration(seconds=350)) or self.get_task_failed_state():
            self.cancel_moving()
            self.get_logger().warn("#### send move cancel! (timeout)")

    def cancel_moving(self):
        cancel_msg = GoalID()
        cancel_msg.stamp = self.get_clock().now().to_msg()
        cancel_msg.id = self.nav_goal_id.id # 現在のゴールIDをセット
        self.nav_cancel_publisher.publish(cancel_msg)
        self.nav_goal_active = False
        self.get_logger().info("Move cancelled.")

    async def processing(self, msg):
        try:
            for i in range(2):
                if self.get_task_failed_state():
                    return 'FAILURE'

                goal_pose = Pose()
                goal_pose.position.x = float(self.posi_x)
                goal_pose.position.y = float(self.posi_y)
                goal_pose.position.z = float(self.posi_z)
                goal_pose.orientation.x = float(self.rot_x)
                goal_pose.orientation.y = float(self.rot_y)
                goal_pose.orientation.z = float(self.rot_z)
                goal_pose.orientation.w = float(self.rot_w)

                self.nav_goal_active = True
                self.nav_goal_publisher.publish(goal_pose)
                self.get_logger().info(f"Moving to: {goal_pose}")
                move_start_time = self.get_clock().now()

                while self.nav_goal_active:
                    if self.get_task_failed_state() or (self.get_clock().now() - move_start_time) > rclpy.time.Duration(seconds=60):
                        self.cancel_moving()
                        self.get_logger().warn("#### send move cancel! (timeout or task failed)")
                        return 'FAILURE'
                    # ここでナビゲーションの終了判定を行う必要があります。
                    # 通常はActionClientのフィードバックや結果を利用します。
                    # 簡単な例として、一定時間後に成功とみなす処理を追加します。
                    elapsed_time = self.get_clock().now() - move_start_time
                    if elapsed_time > rclpy.time.Duration(seconds=10): # 仮の成功判定
                        self.nav_goal_active = False
                        self.get_logger().info("####GOAL REACHED (simulated)####")
                        return 'SUCCESS'

                    self.get_logger().info("move2Location")
                    await asyncio.sleep(0.1) # rclpy.Rate(10) の代替

                if self.get_task_failed_state():
                    return 'FAILURE'

        except Exception as e:
            self.get_logger().error(f'Could not Start Navigation: {e}')
            return 'FAILURE'

        return 'FAILURE'

    async def processing_pose(self, msg):
        # send_robot_motion.processing('DETECTING_BOX_POSE') # 代替処理が必要
        self.get_logger().info("Simulating: DETECTING_BOX_POSE (sleep 2 seconds)")
        await asyncio.sleep(2)

        if msg is None:
            return 'FAILURE'

        posi_x = msg.position.x
        posi_y = msg.position.y
        posi_z = msg.position.z

        rot_x = msg.orientation.x
        rot_y = msg.orientation.y
        rot_z = msg.orientation.z
        rot_w = msg.orientation.w

        try:
            if self.get_task_failed_state():
                return 'FAILURE'

            goal_pose = Pose()
            goal_pose.position.x = float(posi_x)
            goal_pose.position.y = float(posi_y)
            goal_pose.position.z = float(posi_z)
            goal_pose.orientation.x = float(rot_x)
            goal_pose.orientation.y = float(rot_y)
            goal_pose.orientation.z = float(rot_z)
            goal_pose.orientation.w = float(rot_w)

            self.nav_goal_active = True
            self.nav_goal_publisher.publish(goal_pose)
            self.get_logger().info(f"Moving to pose: {goal_pose}")
            move_start_time = self.get_clock().now()

            while self.nav_goal_active:
                elapsed_time = self.get_clock().now() - move_start_time
                if self.get_task_failed_state() or elapsed_time > rclpy.time.Duration(seconds=60):
                    self.cancel_moving()
                    self.get_logger().warn("#### MOVE CANCEL (timeout or task failed) ####")
                    return 'FAILURE'

                # ここでナビゲーションの終了判定を行う必要があります。
                # 通常はActionClientのフィードバックや結果を利用します。
                # 簡単な例として、一定時間後に成功とみなす処理を追加します。
                if elapsed_time > rclpy.time.Duration(seconds=10): # 仮の成功判定
                    self.nav_goal_active = False
                    self.get_logger().info("#### MOVE SUCCESS (simulated) ####")
                    return 'SUCCESS'

                self.get_logger().info("move2Location")
                await asyncio.sleep(0.1)

            return 'SUCCESS' # 仮の成功

        except Exception as e:
            self.get_logger().error(f'Could not Start Navigation: {e}')
            return 'FAILURE'

    def escape_cost_position(self, obj_area, search_pose):
        search_pose_modified = Pose()
        search_pose_modified.position.x = search_pose.position.x
        search_pose_modified.position.y = search_pose.position.y
        search_pose_modified.position.z = search_pose.position.z
        search_pose_modified.orientation = search_pose.orientation

        if obj_area == "center":
            search_pose_modified.position.x -= 0.40
        elif obj_area == "left":
            search_pose_modified.position.y += 0.50
        elif obj_area == "right":
            search_pose_modified.position.y -= 0.50
        elif obj_area == "top":
            search_pose_modified.position.x += 0.50
        elif obj_area == "bottom":
            search_pose_modified.position.x -= 0.50
        elif obj_area == "top_left":
            search_pose_modified.position.x += 0.50
            search_pose_modified.position.y += 0.50
        elif obj_area == "top_right":
            search_pose_modified.position.x += 0.50
            search_pose_modified.position.y -= 0.50
        elif obj_area == "bottom_left":
            search_pose_modified.position.x -= 0.50
            search_pose_modified.position.y += 0.50
        elif obj_area == "bottom_right":
            search_pose_modified.position.x -= 0.50
            search_pose_modified.position.y -= 0.50
        else:
            pass

        return search_pose_modified

    def rotate_to_object_area(self, obj_area):
        command = String()
        if obj_area == "left":
            command.data = "T:90"
        elif obj_area == "right":
            command.data = "T:-90"
        elif obj_area == "bottom":
            command.data = "T:180"
        elif obj_area == "bottom_left":
            command.data = "T:135"
        elif obj_area == "bottom_right":
            command.data = "T:-135"
        elif obj_area == "top_left":
            command.data = "T:45"
        elif obj_area == "top_right":
            command.data = "T:-45"
        else:
            pass
        self.base_ctrl_publisher.publish(command)
        self.get_logger().info(f"Published base control command: {command.data}")

    def search_object_area(self, area):
        search_position = Pose()
        if area == "center":
            search_position.position.x = 0.5
            search_position.position.y = 0.0
            search_position.orientation.w = 1.0
        elif area == "right":
            search_position.position.x = 2.23
            search_position.position.y = -1.1
            search_position.orientation.z = -1.0
            search_position.orientation.w = 1.0
        elif area == "left":
            search_position.position.x = 2.23
            search_position.position.y = 1.1
            search_position.orientation.z = 1.0
            search_position.orientation.w = 1.0
        elif area == "top":
            search_position.position.x = 3.2
            search_position.position.y = 0.0
            search_position.orientation.z = 0.0
            search_position.orientation.w = 1.0
        elif area == "bottom":
            search_position.position.x = 0.5
            search_position.position.y = 0.0
            search_position.orientation.w = 1.0
        elif area == "top_left":
            search_position.position.x = 2.6
            search_position.position.y = 0.6
            search_position.orientation.z = 0.5
            search_position.orientation.w = 1.0
        elif area == "top_right":
            search_position.position.x = 2.6
            search_position.position.y = -0.6
            search_position.orientation.z = -0.5
            search_position.orientation.w = 1.0
        elif area == "bottom_left":
            search_position.position.x = 0.0
            search_position.position.y = 0.5
            search_position.orientation.z = 1.0
            search_position.orientation.w = 1.0
        elif area == "bottom_right":
            search_position.position.x = 0.0
            search_position.position.y = -0.5
            search_position.orientation.z = -1.0
            search_position.orientation.w = 1.0
        elif area == "most_bottom_left":
            search_position.position.x = -0.5
            search_position.position.y = 0.2
            search_position.orientation.z = 1.0
            search_position.orientation.w = 1.0
        elif area == "most_bottom_right":
            search_position.position.x = -0.5
            search_position.position.y = -0.2
            search_position.orientation.z = -1.0
            search_position.orientation.w = 1.0
        else:
            return Pose()

        return search_position

async def main(args=None):
    rclpy.init(args=args)
    move_node = MoveNode()
    try:
        move_node.get_logger().info("do processing() @move_node")
        move_node.set_session_start_time(move_node.get_clock().now())
        move_node.posi_x = 1.0 # テスト用の座標
        move_node.posi_y = 0.0
        move_node.posi_z = 0.0
        move_node.rot_x = 0.0
        move_node.rot_y = 0.0
        move_node.rot_z = 0.0
        move_node.rot_w = 1.0

        await asyncio.sleep(3)

        result = await move_node.processing("isan")
        move_node.get_logger().info(f"Processing result: {result}")

        search_position = Pose()
        search_position.position.x = 0.5
        search_position.position.y = 0.0
        search_position.orientation.z = 0.0
        search_position.orientation.w = 1.0

        result_pose = await move_node.processing_pose(search_position)
        move_node.get_logger().info(f"Processing pose result: {result_pose}")

    finally:
        move_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())