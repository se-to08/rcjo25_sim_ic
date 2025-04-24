import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point
import time  # rospy.sleep の代わりに time.sleep

class TfListenerNode2(Node):
    def __init__(self):
        super().__init__('tf_listener_node2')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person_found = False  # person フレームが見つかったかどうかを追跡するフラグ

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
            self.get_logger().info(f"point {point_frame}: {point}")
            if point_frame == "person" and not self.person_found:
                self.get_logger().info("person フレームを見つけました！")
                print("person フレームを見つけました！")
                self.person_found = True
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
    tf_listener_node2 = TfListenerNode2()
    try:
        while rclpy.ok():
            point = tf_listener_node2.get_tf("person") # 取得したいフレーム名をここで指定する
            # person フレームが見つかったかどうかは get_tf 内で処理
            time.sleep(1.0) # 必要に応じてポーリングレートを調整
    except KeyboardInterrupt:
        pass
    finally:
        tf_listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()