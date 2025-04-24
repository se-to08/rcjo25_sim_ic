#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int64, Float32, Float32MultiArray
from geometry_msgs.msg import Vector3
# from rc2021_sim_sigverse_ic.srv import FollowCtrl # ROS 2 のサービス定義に合わせてください
# from face_direction_estimater.srv import Face_direction_ctrl # ROS 2 のサービス定義に合わせてください
# from lightweight_human_pose_estimation.msg import KeyPoints_3d # ROS 2 のメッセージ定義に合わせてください
import numpy as np
import asyncio

class PersonDirectionNode(Node):
    def __init__(self):
        super().__init__('person_direction_node')
        self.face_direction_list = []
        self.face_direction_count = []
        self.key_points_dict = None
        self.head_direction_p = []

        # サービスのクライアント
        self.face_direction_ctrl_client = self.create_client(self.get_service_type('follow_ctrl'), '/person_direction_detect/face_direction/estimate_ctrl')
        self.head_direction_ctrl_client = self.create_client(self.get_service_type('face_direction_ctrl'), 'face_direction_estimater/detect_ctrl')
        self.person_follower_ctrl_client = self.create_client(self.get_service_type('follow_ctrl'), '/follow_ctrl')

        # サブスクライバー
        self.sub_face_direction = self.create_subscription(Int64, "/person_direction_detect/face_direction", self.callback_face, 10)
        self.sub_head_direction = self.create_subscription(Float32MultiArray, "face_direction_estimater/face_angle", self.callback_head, 10)
        self.sub_pointing_direction = self.create_subscription(self.get_message_type('key_points_3d'), "/lightweight_human_pose_estimation/human_pose_estimation/pose_3d", self.callback_point, 10)

    def get_service_type(self, service_name):
        if service_name == 'follow_ctrl':
            from your_package.srv import FollowCtrl # 実際のROS 2サービス定義に合わせてください
            return FollowCtrl
        elif service_name == 'face_direction_ctrl':
            from your_package.srv import FaceDirectionCtrl # 実際のROS 2サービス定義に合わせてください
            return FaceDirectionCtrl
        else:
            return None

    def get_message_type(self, message_name):
        if message_name == 'key_points_3d':
            from your_package.msg import KeyPoints3d # 実際のROS 2メッセージ定義に合わせてください
            return KeyPoints3d
        else:
            return None

    async def face_direction_ctrl_service(self, ctrl):
        self.face_direction_count = []
        self.face_direction_list = []
        while not self.face_direction_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /person_direction_detect/face_direction/estimate_ctrl service...')
        request = self.get_service_type('follow_ctrl').Request()
        request.request = ctrl
        future = self.face_direction_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error('Service call failed for /person_direction_detect/face_direction/estimate_ctrl')
            return False

    async def head_direction_ctrl_service(self, ctrl):
        self.head_direction_p = []
        self.get_logger().info(f"Head direction ctrl: {ctrl}")
        self.get_logger().info(f"Current head_direction_p: {self.head_direction_p}")
        while not self.head_direction_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for face_direction_estimater/detect_ctrl service...')
        request = self.get_service_type('face_direction_ctrl').Request()
        request.request = ctrl
        future = self.head_direction_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error('Service call failed for face_direction_estimater/detect_ctrl')
            return False

    async def person_follower_ctrl_service(self, ctrl):
        while not self.person_follower_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /follow_ctrl service...')
        request = self.get_service_type('follow_ctrl').Request()
        request.request = ctrl
        future = self.person_follower_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error('Service call failed for /follow_ctrl')
            return False

    def person_follow_initialize(self):
        self.face_direction_list = []
        self.face_direction_count = []
        self.head_direction_p = []

    def callback_face(self, msg):
        face_direction_qua = msg.data
        self.face_direction_list.append(face_direction_qua)

    def callback_head(self, msg):
        if len(msg.data) > 0:
            head_diection_tmp = msg.data[0]
            self.get_logger().info(f"Received head angle: {head_diection_tmp}")
            self.head_direction_p.append(head_diection_tmp)

    def callback_point(self, msg):
        if self.key_points_dict is None:
            self.key_points_dict = dict(nose=msg.key_point_3d[0].nose,
                                        neck=msg.key_point_3d[0].neck,
                                        r_sho=msg.key_point_3d[0].r_sho,
                                        l_sho=msg.key_point_3d[0].l_sho,
                                        r_wri=msg.key_point_3d[0].r_wri,
                                        l_wri=msg.key_point_3d[0].l_wri)
        elif msg.key_point_3d[0].r_wri.z > self.key_points_dict["r_wri"].z:
            self.key_points_dict = dict(nose=msg.key_point_3d[0].nose,
                                        neck=msg.key_point_3d[0].neck,
                                        r_sho=msg.key_point_3d[0].r_sho,
                                        l_sho=msg.key_point_3d[0].l_sho,
                                        r_wri=msg.key_point_3d[0].r_wri,
                                        l_wri=msg.key_point_3d[0].l_wri)
            self.get_logger().info("========================")
            self.get_logger().info(f"r_wri z: {np.array(self.key_points_dict['r_wri'].z)}")
            self.get_logger().info("========================")
            self.pointing_direction()

    def pointing_direction(self):
        if self.key_points_dict is not None:
            base_vector = np.array([self.key_points_dict["nose"].x, self.key_points_dict["nose"].y]) - np.array([self.key_points_dict["neck"].x, self.key_points_dict["neck"].y])
            r_point_vector = np.array([self.key_points_dict["r_sho"].x, self.key_points_dict["r_sho"].y]) - np.array([self.key_points_dict["r_wri"].x, self.key_points_dict["r_wri"].y])
            l_point_vector = np.array([self.key_points_dict["l_sho"].x, self.key_points_dict["l_sho"].y]) - np.array([self.key_points_dict["l_wri"].x, self.key_points_dict["l_wri"].y])

            r_dot = np.dot(base_vector, r_point_vector)
            l_dot = np.dot(base_vector, l_point_vector)
            r_norm = np.linalg.norm(r_point_vector)
            l_norm = np.linalg.norm(l_point_vector)
            base_norm = np.linalg.norm(base_vector)

            if r_norm > 1e-6 and l_norm > 1e-6 and base_norm > 1e-6:
                r_cos = r_dot / (r_norm * base_norm)
                l_cos = l_dot / (l_norm * base_norm)
                self.get_logger().info(f"r_cos: {r_cos}, l_cos: {l_cos}")
                r_angle = np.degrees(np.arccos(np.clip(r_cos, -1.0, 1.0)))
                l_angle = np.degrees(np.arccos(np.clip(l_cos, -1.0, 1.0)))
                self.get_logger().info(f"r_angle: {r_angle}, l_angle: {l_angle}")
                if r_angle > l_angle:
                    self.get_logger().info(f"Pointing angle (right arm): {r_angle}")
                    return r_angle
                else:
                    self.get_logger().info(f"Pointing angle (left arm): {l_angle}")
                    return l_angle
            else:
                self.get_logger().warn("Norm is too small, cannot calculate pointing direction.")
                return None
        else:
            return None

    def face_direction(self):
        self.get_logger().info("FACE ANGLE_check")
        self.face_direction_count = [self.face_direction_list.count(i * 45) for i in range(8)]
        self.get_logger().info(f"Face direction counts: {self.face_direction_count}")
        max_count_index = np.argmax(self.face_direction_count)
        angle = 45 * max_count_index
        self.get_logger().info(f"FACE ANGLE: {angle}")
        return angle

    def head_direction(self):
        if len(self.head_direction_p) > 2:
            self.get_logger().info(f"Head direction history: {self.head_direction_p}")
            ave = sum(self.head_direction_p) / len(self.head_direction_p)
            angle = 180 - ave
            self.get_logger().info(f"HEAD ANGLE: {angle}")
            return angle
        else:
            return 'FAILURE'

async def main(args=None):
    rclpy.init(args=args)
    person_direction_node = PersonDirectionNode()
    try:
        # Example of calling the services (you would integrate this into your state machine)
        # response_face = await person_direction_node.face_direction_ctrl_service(True)
        # person_direction_node.get_logger().info(f"Face direction control response: {response_face}")
        # response_head = await person_direction_node.head_direction_ctrl_service(True)
        # person_direction_node.get_logger().info(f"Head direction control response: {response_head}")
        # response_follow = await person_direction_node.person_follower_ctrl_service(True)
        # person_direction_node.get_logger().info(f"Person follower control response: {response_follow}")

        await asyncio.spin(person_direction_node)
    except KeyboardInterrupt:
        pass
    finally:
        person_direction_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())