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
        # Subscribing to '/human_pose/keypoint_3d_array' topic
        self.sub_pointing_direction = self.create_subscription(
            KeyPointArray,
            '/human_pose/keypoint_3d_array',
            self.callback_point,
            10
        )
        
        self.key_points_dict = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback_point(self, msg):
        # key_points_array が空だったら何もしない
        if not msg.key_points_array:
            self.get_logger().warn("Received empty key_points_array. Skipping this message.")
            return

        self.key_points_dict.clear()

        for idx, key_point in enumerate(msg.key_points_array[0].key_points):
            self.key_points_dict[msg.key_points_array[0].key_names[idx]] = {
                'x': key_point.x,
                'y': key_point.y,
                'z': key_point.z
            }
        
        self.pointing_direction() #mainから呼び出すときはコメントアウト


    def pointing_direction(self):
        person_frame = "person"
        base_footprint_frame = "base_footprint"

        # 右手
        right_shoulder_frame = "right_shoulder"
        right_wrist_frame = "right_wrist"
        # 左手
        left_shoulder_frame = "left_shoulder"
        left_wrist_frame = "left_wrist"

        try:
            if right_shoulder_frame in self.key_points_dict and right_wrist_frame in self.key_points_dict:
                shoulder_pos = [self.key_points_dict[right_shoulder_frame]['x'],
                                self.key_points_dict[right_shoulder_frame]['y'],
                                self.key_points_dict[right_shoulder_frame]['z']]
                wrist_pos = [self.key_points_dict[right_wrist_frame]['x'],
                            self.key_points_dict[right_wrist_frame]['y'],
                            self.key_points_dict[right_wrist_frame]['z']]
                hand_used = "right"
            elif left_shoulder_frame in self.key_points_dict and left_wrist_frame in self.key_points_dict:
                shoulder_pos = [self.key_points_dict[left_shoulder_frame]['x'],
                                self.key_points_dict[left_shoulder_frame]['y'],
                                self.key_points_dict[left_shoulder_frame]['z']]
                wrist_pos = [self.key_points_dict[left_wrist_frame]['x'],
                            self.key_points_dict[left_wrist_frame]['y'],
                            self.key_points_dict[left_wrist_frame]['z']]
                hand_used = "left"
            else:
                self.get_logger().warn("Neither right nor left hand keypoints found")
                return

            # 相対ベクトルを計算（手首の位置 - 肩の位置）
            trans_relative = [wrist_pos[0] - shoulder_pos[0],
                            wrist_pos[1] - shoulder_pos[1],
                            wrist_pos[2] - shoulder_pos[2]]

            # 水平方向の角度を計算（ロボット基準）
            angle_rad = math.atan2(trans_relative[1], trans_relative[0])
            angle_deg = normalize_angle(math.degrees(angle_rad))

            self.get_logger().info(f"Angle ({hand_used} wrist from robot perspective): {angle_deg:.2f} degrees")
            return angle_deg

        except TransformException as e:
            self.get_logger().warn(f"Transform from base_footprint to person failed: {e}")
            try:
                now = self.get_clock().now().to_msg()
                transform_person_base = self.tf_buffer.lookup_transform(
                    base_footprint_frame,
                    person_frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.3)
                )
                trans_person_base = [transform_person_base.transform.translation.x,
                                    transform_person_base.transform.translation.y,
                                    transform_person_base.transform.translation.z]

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

    # メッセージを直接渡して `pointing_direction` を呼び出す
    # message = KeyPointArray()  # ここで適切なメッセージをセットする
    # pointing_direction_estimator.pointing_direction(message)
    
    rclpy.spin(pointing_direction_estimator)
    pointing_direction_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
