import rclpy
from rclpy.node import Node
import numpy as np
from sobits_interfaces.msg import KeyPoint
from sobits_interfaces.msg import KeyPointArray

class HumanPositionNode(Node):
    def __init__(self):
        super().__init__('human_position_node')
        self.subscription = self.create_subscription(
            KeyPointArray,
            '/human_pose/keypoint_3d_array',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a timer to call the output function every 3 seconds
        self.timer = self.create_timer(3.0, self.output_position)

        # Initialize variables to store position information
        self.human_position = None

    def listener_callback(self, msg):
        # Initialize key points
        nose = neck = right_shoulder = left_shoulder = None

        # Extract key points if they exist
        if len(msg.key_points_array) > 0:
            key_points = msg.key_points_array[0].key_points
            if len(key_points) > 0:
                nose = np.array([key_points[0].x, key_points[0].y, key_points[0].z])
            if len(key_points) > 1:
                neck = np.array([key_points[1].x, key_points[1].y, key_points[1].z])
            if len(key_points) > 2:
                right_shoulder = np.array([key_points[2].x, key_points[2].y, key_points[2].z])
            if len(key_points) > 5:
                left_shoulder = np.array([key_points[5].x, key_points[5].y, key_points[5].z])

        # Calculate the average x position of the key points to determine human position
        x_positions = []
        if nose is not None:
            x_positions.append(nose[0])
        if neck is not None:
            x_positions.append(neck[0])
        if right_shoulder is not None:
            x_positions.append(right_shoulder[0])
        if left_shoulder is not None:
            x_positions.append(left_shoulder[0])

        if x_positions:
            avg_x_position = np.mean(x_positions)
            
            # Determine the position based on the average x position
            if avg_x_position < -1.0/3.0:
                self.human_position = "left"
            elif avg_x_position > 1.0/3.0:
                self.human_position = "right"
            else:
                self.human_position = "center"
        else:
            self.human_position = None

    def output_position(self):
        if self.human_position:
            self.get_logger().info(f'The human is positioned to the {self.human_position}.')

def main(args=None):
    rclpy.init(args=args)
    human_position_node = HumanPositionNode()
    rclpy.spin(human_position_node)
    human_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
