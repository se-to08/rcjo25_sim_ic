import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sobits_interfaces.action import MoveToPose

class MoveToPoseClient(Node):
    def __init__(self):
        super().__init__('test_grasp')
        self._action_client = ActionClient(self, MoveToPose, '/hsr_sim/move_to_pose')
        self.goal_reached = False  # 目的地に到着したかどうかを示すフラグ
        self.goal_status = None  # ゴールのステータスを保存する変数

        self._action_client.wait_for_server()

    def send_goal(self, pose_name):
        goal_msg = MoveToPose.Goal()
        goal_msg.pose_name = pose_name
        goal_msg.time_allowance.sec = 5
        goal_msg.time_allowance.nanosec = 0

        self.get_logger().info(f'Sending goal: {pose_name}')
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            self.goal_status = False
            return

        self.get_logger().info('Goal accepted by the server. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: success = {result.success}')
        self.goal_status = result.success
        self.goal_reached = result.success

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')
    
    def change_pose(self, pose_name):
        self.send_goal(pose_name)
        while rclpy.ok() and self.goal_status is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        return self.goal_reached


def main(args=None):
    rclpy.init(args=args)

    move_to_pose_client = MoveToPoseClient()
    pose_name = "initial_pose"
    #   - initial_pose
    #   - detecting_pose
    #   - low_detecting_pose
    #   - lowest_detecting_pose
    #   - mesurment_pose
    #   - detecting_box_pose
    result = move_to_pose_client.change_pose(pose_name)
    print(result)
    rclpy.shutdown()

if __name__ == '__main__':
    main()