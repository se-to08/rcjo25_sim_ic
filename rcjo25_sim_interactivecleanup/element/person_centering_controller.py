import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point, Twist
from tf2_msgs.msg import TFMessage
import math

class PersonCenteringController(Node):
    def __init__(self):
        super().__init__('person_centering_controller')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_frame = 'base_footprint'
        self.person_frame = 'person_0'
        self.cmd_vel_publisher = self.create_publisher(Twist, '/hsrb/command_velocity', 10)
        self.control_timer = self.create_timer(0.1, self.control_callback) # 10Hzで制御

        # 制御パラメータ
        self.angular_speed_factor = 0.5  # 左右のずれに対する回転速度の係数
        self.centering_threshold_y = 0.05 # これ以下のずれなら回転しない (単位: メートル)

    def control_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.person_frame,
                rclpy.time.Time()
            )

            person_y_relative_to_robot = transform.transform.translation.y

            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0
            twist_cmd.linear.y = 0.0
            twist_cmd.linear.z = 0.0
            twist_cmd.angular.x = 0.0
            twist_cmd.angular.y = 0.0
            twist_cmd.angular.z = 0.0

            if abs(person_y_relative_to_robot) > self.centering_threshold_y:
                # Y軸のずれが大きい場合、回転する
                angular_z = person_y_relative_to_robot * self.angular_speed_factor

                # 回転方向を調整 (人が右にいる場合は左に回転, 人が左にいる場合は右に回転)
                twist_cmd.angular.z = -angular_z

                self.get_logger().info(f'Person is at y: {person_y_relative_to_robot:.2f}, rotating with angular z: {twist_cmd.angular.z:.2f}')
            else:
                self.get_logger().info('Person is centered (within threshold).')

            self.cmd_vel_publisher.publish(twist_cmd)

        except TransformException as ex:
            self.get_logger().error(f'Could not transform {self.person_frame} to {self.robot_frame}: {ex}')
            # 変換に失敗した場合は、速度指令を停止する
            twist_cmd = Twist()
            self.cmd_vel_publisher.publish(twist_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PersonCenteringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()