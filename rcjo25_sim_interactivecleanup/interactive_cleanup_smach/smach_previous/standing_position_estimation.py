#!/usr/bin/env python3
# coding: utf-8
import sys
import rclpy
from rclpy.node import Node
from standing_position_estimator.srv import EstimateStandingPosition
from geometry_msgs.msg import Pose

class EstimatePositionClient(Node):

    def __init__(self):
        super().__init__('estimate_position_client')

    def processing(self, target_name, distance):
        self.cli = self.create_client(EstimateStandingPosition, '/standing_position_estimator/estimate_standing_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = EstimateStandingPosition.Request()
        req.target_name = target_name
        req.current_pose = Pose()  # ROS2ではPose型のオブジェクトを明示的に作成する必要があります
        req.distance = distance
        self.get_logger().info(f'Estimate_position -> Target_name [{target_name}]')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().estimated_pos
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return None

def main(args=None):
    rclpy.init(args=args)
    estimate_position_client = EstimatePositionClient()
    estimated_pose = estimate_position_client.processing('target', 0.8)
    if estimated_pose:
        estimate_position_client.get_logger().info(f'Estimated position: {estimated_pose}')
    else:
        estimate_position_client.get_logger().warn('Failed to get estimated position.')
    estimate_position_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()