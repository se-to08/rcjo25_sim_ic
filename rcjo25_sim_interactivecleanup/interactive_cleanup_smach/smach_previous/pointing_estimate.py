#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener
import numpy as np
from geometry_msgs.msg import Point, PointStamped, Pose
from visualization_msgs.msg import Marker
from sympy.geometry import Polygon
import math
import pandas as pd
import asyncio
from lightweight_human_pose_estimation.msg import PoseWithMultiArrays, BodyPart

class PersonPoseAnalyzer(Node):
    def __init__(self):
        super().__init__('person_pose_analyzer')
        # TF Listenerの初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # マーカーPublisherの初期化
        self.pub_pointing_direction = self.create_publisher(Marker, "pointing_direction", 10)
        self.pub_search_region = self.create_publisher(Marker, "search_region", 10)
        self.pub_target_candidate = self.create_publisher(Marker, "target_candidate", 10)

    def judge_person_direction_pro(self, poses):
        # (省略 - 前回の回答と同じ)
        left_sh_x = []
        right_sh_x = []
        left_sh_y = []
        right_sh_y = []

        left_sh_ave = Point()
        right_sh_ave = Point()

        for pose_data in poses:
            if pose_data.persons:
                person = pose_data.persons[0]
                for body_part in person.body_part:
                    if body_part.part_id == 7:
                        left_sh_x.append(body_part.x)
                        left_sh_y.append(body_part.y)
                    elif body_part.part_id == 8:
                        right_sh_x.append(body_part.x)
                        right_sh_y.append(body_part.y)

        angle = 360
        if not left_sh_x or not right_sh_x:
            return angle

        left_sh_x_series = pd.Series(left_sh_x)
        left_sh_y_series = pd.Series(left_sh_y)
        right_sh_x_series = pd.Series(right_sh_x)
        right_sh_y_series = pd.Series(right_sh_y)

        def filter_outliers(series):
            Q1 = series.quantile(0.25)
            Q3 = series.quantile(0.75)
            IQR = Q3 - Q1
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR
            return series[(series >= lower_bound) & (series <= upper_bound)].dropna()

        left_sh_x_filtered = filter_outliers(left_sh_x_series)
        left_sh_y_filtered = filter_outliers(left_sh_y_series)
        right_sh_x_filtered = filter_outliers(right_sh_x_series)
        right_sh_y_filtered = filter_outliers(right_sh_y_series)

        if not left_sh_x_filtered.empty and not left_sh_y_filtered.empty and not right_sh_x_filtered.empty and not right_sh_y_filtered.empty:
            left_sh_ave.x = left_sh_x_filtered.mean()
            left_sh_ave.y = left_sh_y_filtered.mean()
            left_sh_ave.z = 0.0
            right_sh_ave.x = right_sh_x_filtered.mean()
            right_sh_ave.y = right_sh_y_filtered.mean()
            right_sh_ave.z = 0.0

            self.get_logger().info(f"GET_PERSON_POSE_AVERAGE (filtered): left={left_sh_ave}, right={right_sh_ave}")

            left_sh_map = self.transform_map(left_sh_ave)
            right_sh_map = self.transform_map(right_sh_ave)

            delta_y = right_sh_map.y - left_sh_map.y
            delta_x = right_sh_map.x - left_sh_map.x

            if delta_y == 0:
                if delta_x > 0:
                    angle = 90
                elif delta_x < 0:
                    angle = -90
            elif delta_y < 0:
                angle = int(math.degrees(math.atan(delta_x / delta_y)))
            elif delta_y > 0:
                if delta_x > 0:
                    angle = int(math.degrees(math.atan(delta_x / delta_y))) - 180
                elif delta_x < 0:
                    angle = int(math.degrees(math.atan(delta_x / delta_y))) + 180
            return angle
        else:
            return 360

    def judge_hand(self, person_pose: PoseWithMultiArrays, person_map: Point):
        # (省略 - 前回の回答と同じ)
        left_elbow = None
        right_elbow = None
        left_wrist = None
        right_wrist = None

        if person_pose.persons:
            person = person_pose.persons[0]
            for body_part in person.body_part:
                if body_part.part_id == 9:
                    left_elbow = Point(x=body_part.x, y=body_part.y, z=body_part.z)
                elif body_part.part_id == 10:
                    right_elbow = Point(x=body_part.x, y=body_part.y, z=body_part.z)
                elif body_part.part_id == 11:
                    left_wrist = Point(x=body_part.x, y=body_part.y, z=body_part.z)
                elif body_part.part_id == 12:
                    right_wrist = Point(x=body_part.x, y=body_part.y, z=body_part.z)

        if left_elbow and right_elbow and left_wrist and right_wrist:
            left_elbow_map = self.transform_map(left_elbow)
            right_elbow_map = self.transform_map(right_elbow)
            left_wrist_map = self.transform_map(left_wrist)
            right_wrist_map = self.transform_map(right_wrist)

            def is_pointing(elbow_map, wrist_map, person_map, tolerance=1.5):
                dist_person_wrist = self.get_distance(person_map.x, person_map.y, wrist_map.x, wrist_map.y)
                dist_person_elbow = self.get_distance(person_map.x, person_map.y, elbow_map.x, elbow_map.y)
                if dist_person_wrist < tolerance and dist_person_elbow < tolerance:
                    arm_vector = np.array([wrist_map.x - elbow_map.x, wrist_map.y - elbow_map.y])
                    person_vector = np.array([wrist_map.x - person_map.x, wrist_map.y - person_map.y])
                    dot_product = np.dot(arm_vector, person_vector)
                    arm_norm = np.linalg.norm(arm_vector)
                    person_norm = np.linalg.norm(person_vector)
                    if arm_norm > 0 and person_norm > 0:
                        cos_angle = dot_product / (arm_norm * person_norm)
                        angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))
                        angle_deg = np.degrees(angle_rad)
                        if angle_deg < 60:
                            return True, elbow_map, wrist_map
                return False, Point(), Point()

            is_left_pointing, left_start, left_end = is_pointing(left_elbow_map, left_wrist_map, person_map)
            is_right_pointing, right_start, right_end = is_pointing(right_elbow_map, right_wrist_map, person_map)

            if is_right_pointing and is_left_pointing:
                if left_wrist_map.z < right_wrist_map.z:
                    self.pub_person_hand_marker(left_start, left_end)
                    return left_start, left_end
                else:
                    self.pub_person_hand_marker(right_start, right_end)
                    return right_start, right_end
            elif is_right_pointing:
                self.pub_person_hand_marker(right_start, right_end)
                return right_start, right_end
            elif is_left_pointing:
                self.pub_person_hand_marker(left_start, left_end)
                return left_start, left_end

        return Point(), Point()

    def get_distance(self, x1, y1, x2, y2):
        """2点間の距離を計算します。"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def rotation(self, x, t, deg=False):
        """2Dベクトルを指定した角度で回転させます。"""
        if deg:
            t = np.deg2rad(t)
        a = np.array([[np.cos(t), np.sin(t)],
                      [-np.sin(t), np.cos(t)]])
        return np.dot(a, x)

    async def estimate_position(self, command, start_point, end_point):
        # (省略 - 前回の回答と同じ)
        posi_list = []
        if command == "object":
            for i in range(19):
                point_frame = "GraspingCandidatesPosition" + str(i + 1).zfill(2)
                tf_point = await self.get_tf(point_frame)
                if tf_point:
                    posi_list.append(tf_point)
        elif command == "put":
            for i in range(10):
                point_frame = "DestinationCandidatesPosition" + str(i + 1).zfill(2)
                tf_point = await self.get_tf(point_frame)
                if tf_point:
                    posi_list.append(tf_point)

        search_dist = 20.0
        search_deg = 30.0
        v_start = np.array([start_point.x, start_point.y])
        v_end = np.array([end_point.x, end_point.y])
        v_dummy = v_end - v_start
        hand_length = np.linalg.norm(v_dummy)
        if hand_length == 0.0:
            return ""

        search = (0.3 / max(0.01, hand_length)) * search_dist
        a = search * np.eye(2)
        v_aug = np.dot(a, v_dummy)
        v2 = self.rotation(v_aug, search_deg, deg=True) + v_start
        v3 = self.rotation(v_aug, -1.0 * search_deg, deg=True) + v_start

        self.pub_search_region_marker(Point(v_start[0], v_start[1], 0.0), Point(v2[0], v2[1], 0.0), Point(v3[0], v3[1], 0.0))
        poly = Polygon((v_start[0], v_start[1]), (v2[0], v2[1]), (v3[0], v3[1]))

        target_frame = ""
        min_length = float('inf')
        candidate_point = Point()
        found_candidate = False
        for i in range(len(posi_list)):
            if posi_list[i]:
                search_result = poly.encloses_point((posi_list[i].x, posi_list[i].y))
                self.get_logger().info(f"Candidate {i+1}: ({posi_list[i].x:.2f}, {posi_list[i].y:.2f}), Enclosed: {search_result}")
                if search_result:
                    length = self.get_distance(start_point.x, start_point.y, posi_list[i].x, posi_list[i].y)
                    if length < min_length:
                        min_length = length
                        target_frame = (f"GraspingCandidatesPosition{str(i + 1).zfill(2)}"
                                        if command == "object"
                                        else f"DestinationCandidatesPosition{str(i + 1).zfill(2)}")
                        candidate_point = posi_list[i]
                        found_candidate = True
                        self.get_logger().info(f"Found candidate: {target_frame}, distance: {length:.2f}")

        if found_candidate:
            self.pub_target_candidate_marker(candidate_point)
        return target_frame

    async def get_tf(self, point_frame):
        """指定されたTFフレームのPoint（x, y, z）を取得します。"""
        map_frame = "map"
        try:
            transform = self.tf_buffer.lookup_transform(map_frame, point_frame, rclpy.time.Time(), rclpy.time.Duration(seconds=3.0))
            point = Point()
            point.x = transform.transform.translation.x
            point.y = transform.transform.translation.y
            point.z = transform.transform.translation.z
            return point
        except tf2_ros.LookupException as ex:
            self.get_logger().error(f"TF Lookup Error: {ex}")
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().error(f"TF Connectivity Error: {ex}")
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f"TF Extrapolation Error: {ex}")
        return None

    async def get_tf_hsr(self, point_frame):
        """HSRのTFフレームのPose（positionとorientation）を取得します（複数回試行）。"""
        map_frame = "map"
        for count in range(5):
            try:
                transform = self.tf_buffer.lookup_transform(map_frame, point_frame, rclpy.time.Time(), rclpy.time.Duration(seconds=0.5))
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation.x = transform.transform.rotation.x
                pose.orientation.y = transform.transform.rotation.y
                pose.orientation.z = transform.transform.rotation.z
                pose.orientation.w = transform.transform.rotation.w
                return pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"TF Error (Attempt {count+1}): {e}")
                await asyncio.sleep(0.5)
        return Pose()

    def search_person_region(self, person, initial_person):
        """
        初期位置からの人物の移動に基づいて、人物がいる可能性のある領域を推定します。
        領域は、中心、上、下、左、右、およびそれらの組み合わせで定義されます。

        Args:
            person (geometry_msgs.msg.Point): 現在の人物の位置（map座標系）。
            initial_person (geometry_msgs.msg.Point): 初期状態の人物の位置（map座標系）。

        Returns:
            str: 推定された人物がいる領域の名前。
        """
        difference_posi = Point()
        target_region = ""

        difference_posi.x = initial_person.x - person.x
        difference_posi.y = initial_person.y - person.y

        threshold_linear = 0.3
        threshold_angular = 0.4

        if initial_person == Point():
            if 1.5 <= person.x <= 2.8 and -0.5 <= person.y <= 0.5:
                target_region = "center"
            elif 2.8 < person.x and -0.5 <= person.y <= 0.5:
                target_region = "top"
            elif person.x < 1.5 and -0.5 <= person.y <= 0.5:
                target_region = "bottom"
            elif 2.0 <= person.x <= 2.6 and 0.5 < person.y:
                target_region = "left"
            elif 2.0 <= person.x <= 2.6 and person.y < -0.5:
                target_region = "right"
            elif 2.6 < person.x and person.y < -0.5:
                target_region = "top_right"
            elif 2.6 < person.x and 0.5 < person.y:
                target_region = "top_left"
            elif person.x < 2.0 and person.y < -0.5:
                target_region = "bottom_right"
            elif person.x < 2.0 and 0.5 < person.y:
                target_region = "bottom_left"
            else:
                target_region = ""
            self.get_logger().info(f"### Person Position : {target_region} ###")
            return target_region

        else:
            if math.fabs(difference_posi.x) <= threshold_linear and math.fabs(difference_posi.y) <= threshold_angular:
                target_region = "center"
            elif difference_posi.x < -threshold_linear and math.fabs(difference_posi.y) <= threshold_angular:
                target_region = "top"
            elif difference_posi.x > threshold_linear and math.fabs(difference_posi.y) <= threshold_angular:
                target_region = "bottom"
            elif math.fabs(difference_posi.x) <= threshold_linear and difference_posi.y < -threshold_angular:
                target_region = "left"
            elif math.fabs(difference_posi.x) <= threshold_linear and difference_posi.y > threshold_angular:
                target_region = "right"
            elif difference_posi.x < -threshold_linear and difference_posi.y < -threshold_angular:
                target_region = "top_left"
            elif difference_posi.x < -threshold_linear and difference_posi.y > threshold_angular:
                target_region = "top_right"
            elif difference_posi.x > threshold_linear and difference_posi.y < -threshold_angular:
                target_region = "bottom_left"
            elif difference_posi.x > threshold_linear and difference_posi.y > threshold_angular:
                target_region = "bottom_right"
            else:
                target_region = ""
            self.get_logger().info(f"### Person Position : {target_region} ###")
            return target_region

    def transform_map(self, point):
        """camera_frameからmap座標系に変換します。"""
        from geometry_msgs.msg import PointStamped
        map_frame = "map"
        camera_frame = "head_rgbd_sensor_depth_frame"
        point_stamped = PointStamped()
        point_stamped.header.frame_id = camera_frame
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = point
        try:
            transformed_point = self.tf_buffer.transform(point_stamped, map_frame, timeout=rclpy.duration.Duration(seconds=4.0))
            return transformed_point.point
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform {camera_frame} to {map_frame}: {ex}")
            return Point()

    def pub_person_hand_marker(self, start, end):
        """指差しの方向をRVizに表示するマーカーをPublishします。"""
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = self.get_clock().now().to_msg()
        marker_data.ns = "pointing_direction"
        marker_data.id = 0
        marker_data.type = Marker.ARROW
        marker_data.action = Marker.ADD
        marker_data.pose.position = start
        # End pointを基準に方向を計算
        orientation = self.calculate_orientation(start, end)
        marker_data.pose.orientation = orientation
        marker_data.scale.x = self.get_distance(start.x, start.y, start.z, end.x, end.y, end.z)
        marker_data.scale.y = 0.05
        marker_data.scale.z = 0.05
        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.lifetime = rclpy.duration.Duration(seconds=30.0)
        self.pub_pointing_direction.publish(marker_data)

    def calculate_orientation(self, start, end):
        """始点と終点からQuaternionのOrientationを計算します。"""
        from tf_transformations import quaternion_from_euler
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        angle = math.atan2(delta_y, delta_x)
        # ロールとピッチは0と仮定
        return self.euler_to_quaternion(0.0, 0.0, angle)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Euler角(rad)をQuaternionに変換します。"""
        from tf_transformations import quaternion_from_euler
        return quaternion_from_euler(roll, pitch, yaw)

    def pub_search_region_marker(self, p1, p2, p3):
        """指差しに基づいて探索する領域をRVizに表示するためのマーカーをPublishします。"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.points = [p1, p2, p3, p1]  # 始点に戻る
        marker.lifetime = rclpy.duration.Duration(seconds=30.0)
        self.pub_search_region.publish(marker)

    def pub_target_candidate_marker(self, point):
        """推定された把持または設置の候補位置をRVizに表示するためのマーカーをPublishします。"""
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = self.get_clock().now().to_msg()
        marker_data.ns = "target_candidate"
        marker_data.id = 3
        marker_data.type = Marker.SPHERE
        marker_data.action = Marker.ADD
        marker_data.pose.position = point
        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 0.0
        marker_data.pose.orientation.w = 1.0
        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1
        marker_data.lifetime = rclpy.duration.Duration(seconds=30.0)
        self.pub_target_candidate.publish(marker_data)

async def main(args=None):
    rclpy.init(args=args)
    node = PersonPoseAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())