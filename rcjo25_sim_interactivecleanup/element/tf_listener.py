import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point
import time  # rospy.sleep の代わりに time.sleep

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tf(self, point_frame):
        map_frame = "map"  # ROS 2 では先頭の '/' は慣例的に省略されます
        self.get_logger().info(f"I-----------------------I")
        self.get_logger().info(f"{point_frame}")
        self.get_logger().info(f"I-----------------------I")

        if point_frame == "person_first":
            point_frame = "person"
            self.get_logger().info("person_first ----->>>>>>>> person")
            time.sleep(2.0)  # rospy.sleep の代わりに time.sleep

        try:
            # ROS 2 ではフレームの存在確認は通常不要
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                point_frame,
                rclpy.time.Time(),  # 最新の変換
                timeout=rclpy.duration.Duration(seconds=4.0)
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
        point = tf_listener_node.get_tf("person") # 取得したいフレーム名をここで指定する
        print(f"取得したポイント: {point}")
    finally:
        tf_listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()