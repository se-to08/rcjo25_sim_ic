#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
import math
import copy
import time

import tf2_ros
import tf2_py
import tf_transformations

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from interactive_cleanup.msg import InteractiveCleanupMsg

from smach_files import move

from rclpy.qos import QoSProfile, qos_profile_sensor_data

class InteractiveCleanupNode(Node):

    def __init__(self):
        super().__init__('interactive_cleanup_node')

        # Publisherの定義
        self.pub_icu_msg = self.create_publisher(
            InteractiveCleanupMsg,
            '/interactive_cleanup/message/to_moderator',
            10
        )

        # 最後に受信したメッセージ
        self.Last_Receive_ICU_Msg = InteractiveCleanupMsg()

        # フラグ
        self.Get_Are_You_Ready_Flag = False
        self.Get_Pick_It_Up_Flag = False
        self.Get_Clean_Up_Flag = False
        self.Get_Yes_Flag = False
        self.Get_No_Flag = False
        self.Get_Task_Failed_Flag = False

        # 最新データ保存
        self.Latest_Image = Image()
        self.Latest_Point = PointCloud2()

        # TFリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # サブスクライバ
        self.create_subscription(
            InteractiveCleanupMsg,
            '/interactive_cleanup/message/to_robot',
            self.icu_msg_cb,
            10
        )

        self.create_subscription(
            Image,
            '/hsrb/head_rgbd_sensor/rgb/image_raw',
            self.rgb_image_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PointCloud2,
            '/hsrb/head_rgbd_sensor/depth/points',
            self.point_cloud_cb,
            qos_profile_sensor_data
        )

    # メッセージ送信
    def pub_task_msg(self, message):
        send_msg = InteractiveCleanupMsg()
        send_msg.message = message
        self.pub_icu_msg.publish(send_msg)

    def repointing(self):
        self.Get_Pick_It_Up_Flag = False
        self.Get_Clean_Up_Flag = False

    def reset_task_flag(self):
        self.Get_Are_You_Ready_Flag = False
        self.Get_Pick_It_Up_Flag = False
        self.Get_Clean_Up_Flag = False
        self.Get_Yes_Flag = False
        self.Get_Task_Failed_Flag = False

    # コールバック
    def icu_msg_cb(self, msg):
        self.Last_Receive_ICU_Msg = copy.copy(msg)
        if msg.message == "Are_you_ready?":
            self.Get_Are_You_Ready_Flag = True
        elif msg.message == "Pick_it_up!":
            self.Get_Pick_It_Up_Flag = True
        elif msg.message == "Clean_up!":
            self.Get_Clean_Up_Flag = True
        elif msg.message == "Yes":
            self.Get_Yes_Flag = True
        elif msg.message == "No":
            self.Get_No_Flag = True
        elif msg.message == "Task_succeeded" or msg.message == "Mission_complete":
            self.Get_Task_Failed_Flag = False
        else:
            self.Get_Task_Failed_Flag = True

    def rgb_image_cb(self, msg):
        self.Latest_Image = msg

    def point_cloud_cb(self, msg):
        self.Latest_Point = msg

    # 各種状態取得
    def get_are_you_ready_state(self):
        return self.Get_Are_You_Ready_Flag

    def get_pick_it_up_state(self):
        return self.Get_Pick_It_Up_Flag

    def get_clean_up_state(self):
        return self.Get_Clean_Up_Flag

    def get_yes_answer_state(self):
        return self.Get_Yes_Flag

    def get_no_answer_state(self):
        return self.Get_No_Flag

    def get_task_failed_state(self):
        return self.Get_Task_Failed_Flag

    def wait_ready_and_send_ready_command(self):
        self.get_logger().info("Waiting Are_you_ready message start.")
        while rclpy.ok():
            if self.get_are_you_ready_state():
                move.set_session_start_time(self.get_clock().now())
                self.pub_task_msg("I_am_ready")
                break
            else:
                self.get_logger().info("waiting for Are_you_ready.")
                time.sleep(1.0)

    def wait_pick_it_up_command(self):
        self.get_logger().info("Waiting Pick_It_Up message.")
        while rclpy.ok():
            if self.get_pick_it_up_state():
                break
            else:
                self.get_logger().info("waiting for Pick_It_Up.")
                time.sleep(1.0)

    def wait_clean_up_command(self):
        self.get_logger().info("Waiting Clean_Up message.")
        while rclpy.ok():
            if self.get_clean_up_state():
                break
            else:
                self.get_logger().info("waiting for Clean_Up.")
                time.sleep(1.0)

    def wait_correct_answer_command(self):
        self.get_logger().info("Waiting Is_this_correct message.")
        result = False
        while rclpy.ok():
            if self.get_yes_answer_state():
                result = True
                break
            elif self.get_no_answer_state():
                result = False
                break
            else:
                self.get_logger().info("waiting for correct answer...")
                time.sleep(1.0)
        return result

    def search_frame_for_subgoal(self, source_frame_name, target_frame_name):
        try:
            trans = self.tf_buffer.lookup_transform(
                source_frame_name,
                target_frame_name,
                rclpy.time.Time()
            )
            point = Point()
            point.x = trans.transform.translation.x
            point.y = trans.transform.translation.y
            point.z = trans.transform.translation.z
            return point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('座標変換失敗')
            return Point()

    def search_close_frames(self, source_frame_name, target_frame_names):
        if len(target_frame_names) == 0:
            self.get_logger().error('発見したオブジェクトリストが0')
            return 'FAILURE'

        saved_frame_name = []
        saved_distance = []

        for target in target_frame_names:
            try:
                trans = self.tf_buffer.lookup_transform(source_frame_name, target, rclpy.time.Time())
                dist = math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2
                )
                saved_frame_name.append(target)
                saved_distance.append(dist)
            except Exception:
                self.get_logger().error('座標変換失敗')
                return 'FAILURE'

        sorted_frames = [frame for _, frame in sorted(zip(saved_distance, saved_frame_name))]
        return sorted_frames

    def search_most_closest_frame(self, source_frame_name, target_frame_names):
        if len(target_frame_names) == 0:
            self.get_logger().error('発見したオブジェクトリストが0')
            return 'FAILURE'

        saved_distance = []

        for target in target_frame_names:
            try:
                trans = self.tf_buffer.lookup_transform(source_frame_name, target, rclpy.time.Time())
                dist = math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2
                )
                saved_distance.append(dist)
            except Exception:
                self.get_logger().error('座標変換失敗')
                return 'FAILURE'

        target_index = saved_distance.index(min(saved_distance))
        return target_frame_names[target_index]

    def search_most_closest_location(self, source_frame_name):
        furniture_list = [f'DestinationCandidatesPosition{str(i).zfill(2)}' for i in range(1, 11)]

        if len(furniture_list) == 0:
            return 'FAILURE'

        saved_distance = []
        for furniture in furniture_list:
            try:
                trans = self.tf_buffer.lookup_transform(source_frame_name, furniture, rclpy.time.Time())
                dist = math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2
                )
                saved_distance.append(dist)
            except Exception:
                return 'FAILURE'

        min_index = saved_distance.index(min(saved_distance))
        return furniture_list[min_index]

    def search_most_closest_point(self, point):
        furniture_list = [f'DestinationCandidatesPosition{str(i).zfill(2)}' for i in range(1, 11)]

        if len(furniture_list) == 0:
            self.get_logger().error('cannot detect destination')
            return 'FAILURE'

        saved_distance = []
        for furniture in furniture_list:
            try:
                trans = self.tf_buffer.lookup_transform('map', furniture, rclpy.time.Time())
                dist = math.sqrt(
                    (trans.transform.translation.x - point.x) ** 2 +
                    (trans.transform.translation.y - point.y) ** 2
                )
                saved_distance.append(dist)
            except Exception:
                return 'FAILURE'

        min_index = saved_distance.index(min(saved_distance))
        return furniture_list[min_index]

    def wait_for_frame_exists(self, frame_name, timeout_sec):
        start_time = self.get_clock().now()
        while not self.tf_buffer.can_transform('map', frame_name, rclpy.time.Time()):
            time.sleep(1.0)
            self.get_logger().info(f'waiting {frame_name} frame...')
            if (self.get_clock().now() - start_time).nanoseconds * 1e-9 > timeout_sec:
                self.get_logger().error(f'waiting {frame_name} frame timeout.')
                return 'FAILURE'
        self.get_logger().info('find putable point')
        return 'SUCCESS'

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCleanupNode()
    
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
