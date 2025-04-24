import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Keypoint3DArray, Keypoint3D
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
            Keypoint3DArray,
            '/human_pose/keypoint_3d_array',  # Keypoint3DArray() がpublishされるトピック名に合わせて変更
            self.pointing_direction_callback,
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pointing_direction_callback(self, msg):
        person_frame = "person"  # 親フレーム名
        right_shoulder_frame = "person_0_r_sho"  # 右肩のキーポイント名
        right_wrist_frame = "person_0_r_wri"  # 右手首のキーポイント名
        base_footprint_frame = "base_footprint"

        try:
            # /person から /person_0_r_sho への変換を取得
            transform_r_sho = self.tf_buffer.lookup_transform(
                person_frame,
                right_shoulder_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3)  # 短めのタイムアウト
            )
            trans_r_sho = [transform_r_sho.transform.translation.x,
                             transform_r_sho.transform.translation.y,
                             transform_r_sho.transform.translation.z]

            # /person から /person_0_r_wri への変換を取得
            transform_r_wri = self.tf_buffer.lookup_transform(
                person_frame,
                right_wrist_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3)  # 短めのタイムアウト
            )
            trans_r_wri = [transform_r_wri.transform.translation.x,
                             transform_r_wri.transform.translation.y,
                             transform_r_wri.transform.translation.z]

            # 相対ベクトルを計算
            trans_r_relative = [trans_r_sho[0] - trans_r_wri[0],
                                trans_r_sho[1] - trans_r_wri[1],
                                trans_r_sho[2] - trans_r_wri[2]]

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