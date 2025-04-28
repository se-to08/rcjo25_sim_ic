import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point
import time  # rospy.sleep の代わりに time.sleep

class TfListener2NodeForDebug(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person_found_map = False
        self.person_found_base_footprint = False

    def get_tf(self, target_frame, source_frame):
        self.get_logger().info(f"I-----------------------I")
        self.get_logger().info(f"Looking for {target_frame} from {source_frame}")
        self.get_logger().info(f"I-----------------------I")

        if target_frame == "person_first":
            target_frame = "person"
            self.get_logger().info("person_first ----->>>>>>>> person")
            time.sleep(2.0)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # 最新の変換
                timeout=rclpy.duration.Duration(seconds=10.0)
            )
            point = Point()
            point.x = transform.transform.translation.x
            point.y = transform.transform.translation.y
            point.z = transform.transform.translation.z
            self.get_logger().info(f"Point {target_frame} in {source_frame}: {point}")

            if target_frame == "person":
                if source_frame == "map" and not self.person_found_map:
                    self.get_logger().info("person フレームを map から見つけました！ (デバッグ)")
                    print("person フレームを map から見つけました！ (デバッグ)")
                    self.person_found_map = True
                elif source_frame == "base_footprint" and not self.person_found_base_footprint:
                    self.get_logger().info("person フレームを base_footprint から見つけました！ (デバッグ)")
                    print("person フレームを base_footprint から見つけました！ (デバッグ)")
                    self.person_found_base_footprint = True
            return point

        except tf2_ros.LookupException as ex:
            self.get_logger().error(f"TF ERROR frame: {target_frame} from {source_frame}")
            self.get_logger().error(str(ex))
            return Point()
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().error(f"TF ERROR frame: {target_frame} from {source_frame}")
            self.get_logger().error(str(ex))
            return Point()
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f"TF ERROR frame: {target_frame} from {source_frame}")
            self.get_logger().error(str(ex))
            return Point()
        except Exception as ex:
            self.get_logger().error(f"TF UNEXPECTED ERROR frame: {target_frame} from {source_frame}")
            self.get_logger().error(str(ex))
            return Point()

def main(args=None):
    rclpy.init(args=args)
    tf_listener2_node_for_debug = TfListener2NodeForDebug()
    try:
        while rclpy.ok():
            tf_listener2_node_for_debug.get_tf("person", "map")
            # tf_listener2_node_for_debug.get_tf("person", "base_footprint")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        tf_listener2_node_for_debug.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()