import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # ←これ大事！位置をtransformするために必要

class Object3DListener(Node):
    def __init__(self):
        super().__init__('object_3d_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # /ssd_ros/object_3d_poses を購読
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/ssd_ros/object_3d_poses',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if len(msg.detections) == 0:
            self.get_logger().info('No detections.')
            return

        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == "person":
                    # 位置を取得
                    position = result.pose.pose.position
                    self.get_logger().info(f"検出位置 (base_footprint基準): x={position.x}, y={position.y}, z={position.z}")

                    # PointStamped にしてtransformする
                    point_stamped = PointStamped()
                    point_stamped.header.frame_id = detection.header.frame_id  # "base_footprint"
                    point_stamped.header.stamp = detection.header.stamp  # 正しいタイムスタンプを設定
                    point_stamped.point = position

                    # 変換できるか事前に確認
                    try:
                        if self.tf_buffer.can_transform(
                                'map',  # 変換先フレーム
                                point_stamped.header.frame_id,
                                point_stamped.header.stamp,
                                timeout=rclpy.duration.Duration(seconds=3.0)):

                            transformed_point = self.tf_buffer.transform(
                                point_stamped,
                                'map',  # 変換先フレーム
                                timeout=rclpy.duration.Duration(seconds=3.0)
                            )
                            self.get_logger().info(f"map基準の位置: x={transformed_point.point.x}, y={transformed_point.point.y}, z={transformed_point.point.z}")
                        else:
                            self.get_logger().warn(f"変換できない: {point_stamped.header.frame_id} -> map")
                    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                        self.get_logger().error(f"変換エラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Object3DListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
