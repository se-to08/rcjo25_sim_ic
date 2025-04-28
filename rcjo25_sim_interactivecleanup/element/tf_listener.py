import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point
import time  # rospy.sleep の代わりに time.sleep
from rclpy.duration import Duration

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tf(self, point_frame):
        map_frame = "map"  # target_frame
        self.get_logger().info(f"I-----------------------I")
        self.get_logger().info(f"{point_frame}")
        self.get_logger().info(f"I-----------------------I")

        if point_frame == "person_first":
            point_frame = "person"
            self.get_logger().info("person_first ----->>>>>>>> person")
            time.sleep(2.0)

        try:
            current_time = self.get_clock().now()
            timeout_duration = Duration(seconds=4.0)  # タイムアウト時間を Duration オブジェクトとして定義

            # 変換できるかチェック
            if not self.tf_buffer.can_transform(
                map_frame,
                point_frame,
                current_time,
                timeout=timeout_duration
            ):
                self.get_logger().error(f"変換できません: {map_frame} -> {point_frame}")
                return Point()

            transform = self.tf_buffer.lookup_transform(
                map_frame,
                point_frame,
                current_time,
                timeout=timeout_duration
            )
            point = Point()
            point.x = transform.transform.translation.x
            point.y = transform.transform.translation.y
            point.z = transform.transform.translation.z
            self.get_logger().info(f"point person: {point}")
            return point
        except tf2_ros.LookupException as ex:
            self.get_logger().error(f"TF ERROR frame: {point_frame}")
            self.get_logger().error(str(ex))
            self.get_logger().info("エラー1")
            return Point()
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().error(f"TF ERROR frame: {point_frame}")
            self.get_logger().error(str(ex))
            self.get_logger().info("エラー2")
            return Point()
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f"TF ERROR frame: {point_frame}")
            self.get_logger().error(str(ex))
            self.get_logger().info("エラー3")
            return Point()
        except Exception as ex:
            self.get_logger().error(f"TF UNEXPECTED ERROR frame: {point_frame}")
            self.get_logger().error(str(ex))
            return Point()

def main(args=None):
    rclpy.init(args=args)
    tf_listener_node = TfListenerNode()

    try:
        timeout_sec = 10.0  # 最大待ち時間（秒）
        start_time = time.time()
        timeout_duration = Duration(seconds=5.0) # can_transform のタイムアウト

        # "map -> person" が取れるまで待つ
        success = False
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(tf_listener_node, timeout_sec=0.1)
            if tf_listener_node.tf_buffer.can_transform(
                "map", "person", tf_listener_node.get_clock().now(), timeout=timeout_duration
            ):
                success = True
                break

        if not success:
            tf_listener_node.get_logger().error("Timeout: TF map -> person 取得できませんでした")
        else:
            point = tf_listener_node.get_tf("person")
            print(f"取得したポイント: {point}")

    finally:
        tf_listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()