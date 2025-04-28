import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from sobits_interfaces.msg import KeyPointArray, KeyPoint
import math
import numpy as np

def normalize_angle(angle_degrees):
    while angle_degrees > 180.0:
        angle_degrees -= 360.0
    while angle_degrees <= -180.0:
        angle_degrees += 360.0
    return angle_degrees

class PointingDirectionEstimator(Node):
    def __init__(self):
        super().__init__('pointing_direction_estimator')
        self.subscription = self.create_subscription(
            KeyPointArray,
            '/human_pose/keypoint_3d_array',  # Keypoint3DArray() がpublishされるトピック名に合わせて変更
            self.pointing_direction_callback,
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pointing_direction_callback(self, msg):
        person_frame = "person"  # 親フレーム名
        right_shoulder_frame = "right_shoulder"  # 右肩のキーポイント名
        right_wrist_frame = "right_wrist"  # 右手首のキーポイント名
        base_footprint_frame = "base_footprint"

        try:
            # KeyPointArray メッセージから対応する key_names と key_points を取得
            key_names = msg.key_points_array[0].key_names
            key_points = msg.key_points_array[0].key_points

            # 'right_shoulder' の位置を取得
            if "right_shoulder" in key_names:
                idx = key_names.index("right_shoulder")
                right_shoulder = key_points[idx]
                right_shoulder_pos = [right_shoulder.x, right_shoulder.y, right_shoulder.z]
            else:
                self.get_logger().warn("Right shoulder keypoint not found")
                return

            # 'right_wrist' の位置を取得
            if "right_wrist" in key_names:
                idx = key_names.index("right_wrist")
                right_wrist = key_points[idx]
                right_wrist_pos = [right_wrist.x, right_wrist.y, right_wrist.z]
            else:
                self.get_logger().warn("Right wrist keypoint not found")
                return

            # 相対ベクトルを計算
            trans_r_relative = [right_shoulder_pos[0] - right_wrist_pos[0],
                                right_shoulder_pos[1] - right_wrist_pos[1],
                                right_shoulder_pos[2] - right_wrist_pos[2]]

            # 水平方向の角度を計算
            r_angle_rad = math.atan2(trans_r_relative[1], trans_r_relative[0])
            r_angle_deg = normalize_angle(math.degrees(r_angle_rad))

            self.get_logger().info(f"Angle (right shoulder to wrist): {r_angle_deg:.2f} degrees")
            return r_angle_deg

        except TransformException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            try:
                # /base_footprint から /person への変換を取得
                transform_person_base = self.tf_buffer.lookup_transform(
                    base_footprint_frame,
                    person_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.3)
                )
                trans_person_base = [transform_person_base.transform.translation.x,
                                     transform_person_base.transform.translation.y,
                                     transform_person_base.transform.translation.z]

                # 水平方向の角度を計算
                angle_rad = math.atan2(trans_person_base[1], trans_person_base[0])
                angle_deg = normalize_angle(math.degrees(angle_rad))

                self.get_logger().warn(f"Falling back to angle (base to person): {angle_deg:.2f} degrees")
                return angle_deg

            except TransformException as e_base:
                self.get_logger().error(f"Transform lookup between {base_footprint_frame} and {person_frame} failed: {e_base}")
                return None

def main(args=None):
    rclpy.init(args=args)
    pointing_direction_estimator = PointingDirectionEstimator()
    rclpy.spin(pointing_direction_estimator)
    pointing_direction_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
