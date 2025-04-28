#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
import time
import subprocess
from std_msgs.msg import String, Bool
from hsr_ros.srv import OdomBase
from smach_files import send_robot_motion  # Assuming this is converted to ROS2

class TrashBoxDetector(Node):
    def __init__(self):
        super().__init__('trash_box_detector')
        self.detect_obj_flag = False
        self.get_detect_msg_count = 0
        self.sub_detect_obj_flag = self.create_subscription(
            Bool,
            "/pcl_detect_msg",
            self.cb_detect_obj,
            1
        )
        self.base_move_client = self.create_client(OdomBase, '/robot_ctrl/base_ctrl')

    def cb_detect_obj(self, msg):
        self.detect_obj_flag = msg.data
        self.get_logger().info(f"Received detect message: {msg.data}")

        if not msg.data:
            temp_move_length = 0.02  # [m]
            if self.get_detect_msg_count % 6 == 0:
                self.base_move("X:" + str(-temp_move_length))
            elif self.get_detect_msg_count % 6 == 1:
                self.base_move("Y:" + str(-temp_move_length * 0.5))
            elif self.get_detect_msg_count % 6 == 2:
                self.base_move("X:" + str(temp_move_length))
            elif self.get_detect_msg_count % 6 == 3:
                self.base_move("Y:" + str(temp_move_length))
            elif self.get_detect_msg_count % 6 == 4:
                self.base_move("X:" + str(-temp_move_length))
            elif self.get_detect_msg_count % 6 == 5:
                self.base_move("Y:" + str(-temp_move_length * 0.5))

        self.get_detect_msg_count += 1

    def node_kill(self):
        subprocess.Popen(['ros2', 'node', 'kill', '/pcl_obj_recogniser'])  # ゴミ箱検出ノードの停止
        self.get_logger().info("Killed node: /pcl_obj_recogniser")

    def base_move(self, order_str):
        while not self.base_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('base_ctrl service not available, waiting again...')
        req = OdomBase.Request()
        req.motion_str = str(order_str)
        future = self.base_move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().res_str
        else:
            self.get_logger().error(f"Service call failed for base_ctrl: {future.exception()}")
            return None

    def processing(self, timeout_sec):
        self.detect_obj_flag = False
        self.get_detect_msg_count = 0
        self.get_logger().info("DETECTING_POSE")
        send_robot_motion.processing("DETECTING_POSE")  # Assuming this ROS1 function is adapted for ROS2

        subprocess.Popen(['ros2', 'launch', 'pcl_matcher', 'trashbox_recog.launch'])
        self.get_logger().info("Launched pcl_matcher/trashbox_recog.launch")

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).to_sec() < timeout_sec:
            if self.detect_obj_flag:
                self.get_logger().info("DETECTED OBJECT")
                time.sleep(2.0)
                return 'SUCCESS'
            time.sleep(0.5)

        # タイムアウトして見つからなかった場合
        # self.node_kill()
        # time.sleep(0.1)
        self.get_logger().info("Object detection timed out.")
        return 'FAILURE'

def main(args=None):
    rclpy.init(args=args)
    trash_box_detector = TrashBoxDetector()
    try:
        result = trash_box_detector.processing(3000.0)
        trash_box_detector.get_logger().info(f"Detection processing result: {result}")
        # trash_box_detector.base_move("T:3.14159") # 180 degrees in radians
        # trash_box_detector.base_move("X:-0.05") # -5 cm in meters
    finally:
        trash_box_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()