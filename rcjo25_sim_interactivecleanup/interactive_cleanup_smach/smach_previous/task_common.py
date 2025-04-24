#!/usr/bin/env python3
# coding:utf-8
import sys
import rclpy
from rclpy.node import Node
import math
import tf2_ros
import copy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, Point
from hsr_ros.srv import RobotMotion, DetectCtrl, OdomBase, PutCtrl, WatchMotion
from visualization_msgs.msg import Marker
from interactive_cleanup.msg import InteractiveCleanupMsg

# Publisherの定義
class InteractiveCleanupPublisher(Node):
    def __init__(self):
        super().__init__('interactive_cleanup_publisher')
        self.pub_icu_msg = self.create_publisher(InteractiveCleanupMsg, "/interactive_cleanup/message/to_moderator", 10)
        self.listener = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.listener, self)
        self.latest_image = Image()
        self.latest_point = PointCloud2()
        self.last_receive_icu_msg = InteractiveCleanupMsg()
        self.get_are_you_ready_flag = False
        self.get_pick_it_up_flag = False
        self.get_clean_up_flag = False
        self.get_yes_flag = False
        self.get_no_flag = False
        self.get_task_failed_flag = False
        self.get_good_pose = False
        self.session_start_time = self.get_clock().now()

        # Subscriberの定義
        self.sub_icu_msg = self.create_subscription(InteractiveCleanupMsg, "/interactive_cleanup/message/to_robot", self.icu_msg_cb, 10)
        self.sub_rgb_image = self.create_subscription(Image, "/hsrb/head_rgbd_sensor/rgb/image_raw", self.rgb_image_cb, 10)
        self.sub_point_cloud = self.create_subscription(PointCloud2, "/hsrb/head_rgbd_sensor/depth/points", self.point_cloud_cb, 10)

    def pub_task_msg(self, message):
        send_msg = InteractiveCleanupMsg()
        send_msg.message = message
        self.pub_icu_msg.publish(send_msg)
        self.get_logger().info(f'Published message: "{message}"')

    def repointing(self):
        self.get_pick_it_up_flag = False
        self.get_clean_up_flag = False

    def reset_task_flag(self):#タスク管理のフラグをリセットする関数
        self.last_receive_icu_msg = InteractiveCleanupMsg()
        self.get_are_you_ready_flag = False
        self.get_pick_it_up_flag = False
        self.get_clean_up_flag = False
        self.get_yes_flag = False
        self.get_no_flag = False
        self.get_task_failed_flag = False

    def icu_msg_cb(self, msg):#アバターからのメッセージ
        self.last_receive_icu_msg = copy.copy(msg)
        if str(msg.message) == "Are_you_ready?":
            self.get_are_you_ready_flag = True
        elif str(msg.message) == "Pick_it_up!":
            self.get_pick_it_up_flag = True
        elif str(msg.message) == "Clean_up!":
            self.get_clean_up_flag = True
        elif str(msg.message) == "Yes":
            self.get_yes_flag = True
        elif str(msg.message) == "No":
            self.get_no_flag = True
        elif str(msg.message) == "Task_succeeded":
            self.get_task_failed_flag = False
        elif str(msg.message) == "Mission_complete":
            self.get_task_failed_flag = False
        else:
            self.get_task_failed_flag = True
        self.get_logger().info(f'Received message from avatar: "{msg.message}"')

    def rgb_image_cb(self, msg):#最新の画像をローカルに保存
        self.latest_image = msg

    def point_cloud_cb(self, msg):#最新の点群をローカルに保存
        self.latest_point = msg

    """ 各種状態の取得関数 """
    def get_are_you_ready_state(self):
        return self.get_are_you_ready_flag

    def get_pick_it_up_state(self):
        return self.get_pick_it_up_flag

    def get_clean_up_state(self):
        return self.get_clean_up_flag

    def get_yes_answer_state(self):
        return self.get_yes_flag

    def get_no_answer_state(self):
        return self.get_no_flag

    def get_task_failed_state(self):
        return self.get_task_failed_flag

    def wait_ready_and_send_ready_command(self):
        self.get_logger().info("Waiting Are_you_ready message start.")
        while rclpy.ok():
            if self.get_are_you_ready_state():
                # move.set_session_start_time(self.get_clock().now()) # 'move' モジュールは提供されていません
                self.session_start_time = self.get_clock().now()
                self.pub_task_msg("I_am_ready")
                break
            else:
                self.get_logger().info("waiting for Are_you_ready.")
                rclpy.spin_once(self, timeout_sec=1.0)

    def wait_pick_it_up_command(self):
        self.get_logger().info("Waiting Pick_It_Up message.")
        while rclpy.ok():
            if self.get_pick_it_up_state():
                break
            else:
                self.get_logger().info("waiting for Pick_It_Up.")
                rclpy.spin_once(self, timeout_sec=1.0)

    def wait_clean_up_command(self):
        self.get_logger().info("Waiting Clean_Up message.")
        while rclpy.ok():
            if self.get_clean_up_state():
                break
            else:
                self.get_logger().info("waiting for Clean_Up.")
                rclpy.spin_once(self, timeout_sec=1.0)

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
                self.get_logger().info("waiting for correct ansser...")
                rclpy.spin_once(self, timeout_sec=1.0)
        return result

    def search_frame_for_subgoal(self, source_frame_name, target_frame_name):
        point = Point()
        try:
            transform = self.listener.lookup_transform(source_frame_name, target_frame_name, rclpy.time.Time())
            point.x = transform.transform.translation.x
            point.y = transform.transform.translation.y
            point.z = transform.transform.translation.z
            return point
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Coordinate transformation failed: {e}")
            return point
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"TF connectivity error: {e}")
            return point
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation error: {e}")
            return point

    def search_close_frames(self, source_frame_name, target_frame_names):#オブジェクトのフレームを近い順にソートする関数
        self.get_logger().info(f"source_frame_name: {source_frame_name}")
        self.get_logger().info(f"target_frame_names: {target_frame_names}")

        if not target_frame_names:#発見したオブジェクトリストが0の時
            self.get_logger().info("発見したオブジェクトリストが0の時")
            return 'FAILURE'

        saved_frame_name = []#名前の保存配列
        saved_distance = []#距離の保存配列
        sorted_distance = []
        target_object = []
        for target_frame in target_frame_names:
            try:
                transform = self.listener.lookup_transform(source_frame_name, target_frame, rclpy.time.Time())
                trans = transform.transform.translation
                distance = math.sqrt(trans.x**2 + trans.y**2)
                saved_frame_name.append(target_frame)
                saved_distance.append(distance)
                self.get_logger().info(f"frame_name [{target_frame}] : {distance}")
                self.get_logger().info(f'distance [{source_frame_name} - {target_frame}] = {distance}')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Coordinate transformation failed for {target_frame}: {e}")
                return 'FAILURE'

        self.get_logger().info(f"saved_distance : {saved_distance}")
        sorted_distance = sorted(saved_distance)
        self.get_logger().info(f"sorted_distance : {sorted_distance}")

        for dist in sorted_distance:
            target_object.append(saved_frame_name[saved_distance.index(dist)])

        self.get_logger().info(f"Sorted target objects: {target_object}")
        return target_object

    def search_most_closest_frame(self, source_frame_name, target_frame_names):#指定したフレームと最も近いフレームを探す関数
        self.get_logger().info(f"source_frame_name: {source_frame_name}")
        self.get_logger().info(f"target_frame_names: {target_frame_names}")

        if not target_frame_names:#発見したオブジェクトリストが0の時
            self.get_logger().info("発見したオブジェクトリストが0の時")
            return 'FAILURE'

        saved_distance = []#距離の保存配列
        for target_frame in target_frame_names:
            try:
                transform = self.listener.lookup_transform(source_frame_name, target_frame, rclpy.time.Time())
                trans = transform.transform.translation
                distance = math.sqrt(trans.x**2 + trans.y**2)
                saved_distance.append(distance)
                self.get_logger().info(f'distance [{source_frame_name} - {target_frame}] = {distance}')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Coordinate transformation failed for {target_frame}: {e}")
                return 'FAILURE'

        #最も距離が近いオブジェクト
        min_distance = min(saved_distance)
        target_index = saved_distance.index(min_distance)
        target_object = target_frame_names[target_index]
        self.get_logger().info(f'Most Closest Object = [{target_object}], Distance = [{min_distance}]')
        return target_object

    def search_most_closest_location(self, source_frame_name):#指定したフレームと最も近い家具名を返す
        furniture_list = ['DestinationCandidatesPosition01','DestinationCandidatesPosition02','DestinationCandidatesPosition03','DestinationCandidatesPosition04','DestinationCandidatesPosition05','DestinationCandidatesPosition06','DestinationCandidatesPosition07','DestinationCandidatesPosition08','DestinationCandidatesPosition09','DestinationCandidatesPosition10']#置くべき家具リストを読み込み
        if not furniture_list:#場所のリストが取得できなかった場合
            self.get_logger().warn("Cannot detect destination list.")
            return 'FAILURE'

        saved_distance = []
        for furniture in furniture_list:
            try:
                transform = self.listener.lookup_transform(source_frame_name, furniture, rclpy.time.Time())
                trans = transform.transform.translation
                distance = math.sqrt(trans.x**2 + trans.y**2)
                saved_distance.append(distance)
                self.get_logger().info(f'distance [{source_frame_name} - {furniture}] = {distance}')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Coordinate transformation failed for {furniture}: {e}")
                return 'FAILURE'

        min_distance = min(saved_distance)
        min_index = saved_distance.index(min_distance)
        target_frame_name = furniture_list[min_index] #指差し地点から最も近い家具名
        self.get_logger().info(f'Most Closest Location = [{target_frame_name}], Distance = [{min_distance}]')
        return target_frame_name

    def search_most_closest_point(self, point):#指定した座標と最も近い家具名を返す
        furniture_list = ['DestinationCandidatesPosition01','DestinationCandidatesPosition02','DestinationCandidatesPosition03','DestinationCandidatesPosition04','DestinationCandidatesPosition05','DestinationCandidatesPosition06','DestinationCandidatesPosition07','DestinationCandidatesPosition08','DestinationCandidatesPosition09','DestinationCandidatesPosition10']#置くべき家具リストを読み込み
        if not furniture_list:#場所のリストが取得できなかった場合
            self.get_logger().warn("Cannot detect destination list.")
            return 'FAILURE'

        saved_distance = []
        for furniture in furniture_list:
            try:
                transform = self.listener.lookup_transform("map", furniture, rclpy.time.Time())
                trans = transform.transform.translation
                distance = math.sqrt((trans.x - point.x)**2 + (trans.y - point.y)**2)
                saved_distance.append(distance)
                #print 'distance [', source_frame_name, '-', furniture_list[i], '] = ', saved_distance[i]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Coordinate transformation failed for {furniture}: {e}")
                return 'FAILURE'

        min_distance = min(saved_distance)
        min_index = saved_distance.index(min_distance)
        target_frame_name = furniture_list[min_index] #指差し地点から最も近い家具名
        self.get_logger().info(f'Most Closest Location = [{target_frame_name}], Distance = [{min_distance}]')
        return target_frame_name

    def wait_for_frame_exists(self, frame_name, timeout_sec):
        start_time = self.get_clock().now()
        timeout_flag = False
        while rclpy.ok() and not self.listener.can_transform("map", frame_name, rclpy.time.Time()): # 'map' を適切な固定フレームに置き換えてください
            self.get_logger().info(f"waiting {frame_name} frame")
            rclpy.sleep(1.0)
            if self.get_clock().now() > (start_time + rclpy.duration.Duration(seconds=timeout_sec)):
                timeout_flag = True
                break

        if timeout_flag:
            self.get_logger().error(f"waiting {frame_name} frame timeout.")
            return 'FAILURE'
        else:
            self.get_logger().info("find putable point")
            return 'SUCCESS'

def main(args=None):
    rclpy.init(args=args)
    interactive_cleanup_node = InteractiveCleanupPublisher()
    try:
        # ここで、書き換えたROS2のノードが起動した後に実行したい処理を記述します。
        # 例えば、以下のように、ready信号を待って送信する処理を呼び出すことができます。
        interactive_cleanup_node.wait_ready_and_send_ready_command()

        # 必要に応じて、他の状態を待つ処理や、オブジェクトの探索処理などを呼び出すことができます。
        # 例:
        # interactive_cleanup_node.wait_pick_it_up_command()
        # closest_object = interactive_cleanup_node.search_most_closest_frame("hsrb/base_link", ["object_frame_1", "object_frame_2"])
        # if closest_object != '