import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage

class PersonPositionTracker(Node):
    def __init__(self):
        super().__init__('person_position_tracker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_person_position = None # 初期位置の取得
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.initial_tf_received = False
        self.robot_frame = 'base_footprint'
        self.person_frame = 'person_0'

    def tf_callback(self, msg):
        if not self.initial_tf_received:
            for transform in msg.transforms:
                if transform.child_frame_id == self.person_frame and transform.header.frame_id == self.robot_frame:
                    self.initial_person_position = Point()
                    self.initial_person_position.x = transform.transform.translation.x
                    self.initial_person_position.y = transform.transform.translation.y
                    self.initial_person_position.z = transform.transform.translation.z
                    self.get_logger().info(f'Initial person position from /tf: x={self.initial_person_position.x}, y={self.initial_person_position.y}, z={self.initial_person_position.z}')
                    self.initial_tf_received = True
                    return  # 初期位置を取得したら以降の処理をスキップ

        if self.initial_person_position is None:
            self.get_logger().warn('Initial person position not yet received from /tf.')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.person_frame,
                rclpy.time.Time()
            )

            current_person_x = transform.transform.translation.x
            current_person_y = transform.transform.translation.y

            initial_person_x_robot = self.initial_person_position.x
            initial_person_y_robot = self.initial_person_position.y

            delta_y = current_person_y - initial_person_y_robot

            if delta_y > 0.1:  # ロボットから見て左に移動したと判断する閾値
                # self.get_logger().info('Person moved to the right.')
                self.get_logger().info('Person moved to the left.')
            elif delta_y < -0.1: # ロボットから右に移動したと判断する閾値
                self.get_logger().info('Person moved to the right.')
                # self.get_logger().info('Person moved to the left.')
            else:
                self.get_logger().info('Person has not moved significantly left or right.')

        except TransformException as ex:
            self.get_logger().error(f'Could not transform {self.person_frame} to {self.robot_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()