#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sobit_common_interfaces.srv import RunCtrl  # ROS 2 のサービス定義に合わせてください
# from rc2021_sim_sigverse_ic.msg import ObjectListMsg # ROS 2 のメッセージ定義に合わせてください
# from smach_files import send_robot_motion # ROS 2 では直接利用できない
# from smach_files import task_commom # ROS 2 では直接利用できない
import subprocess
import asyncio

# グローバル変数の宣言 (ROS 2 ではパラメータとして扱う方が推奨されます)
OBJECT_LIST_PARAM = 'object_list'

# send_robot_motion の代替として、簡単なString Publisher
ROBOT_MOTION_TOPIC = '/robot_motion_command'
BASE_CTRL_TOPIC = '/base_ctrl_command'

class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')
        self.object_list = []
        self.tf_save_ctrl_pub = self.create_publisher(Bool, '/rc2021_sim_sigverse_ic/tf_static_ctrl', 10)
        self.yolo_run_ctrl_client = self.create_client(RunCtrl, '/yolov5_ros/run_ctrl')
        self.object_list_sub = self.create_subscription(String, '/rc2021_sim_sigverse_ic/object_tf_list', self.cb_object_list, 10)
        self.motion_publisher = self.create_publisher(String, ROBOT_MOTION_TOPIC, 10)
        self.base_ctrl_publisher = self.create_publisher(String, BASE_CTRL_TOPIC, 10)
        self.declare_parameter(OBJECT_LIST_PARAM, []) # パラメータとしてオブジェクトリストを宣言

    def get_object_list(self):
        return self.get_parameter(OBJECT_LIST_PARAM).get_parameter_value().string_array_value

    def start_tf_broadcaster(self):
        try:
            subprocess.Popen(['ros2', 'run', 'rc2021_sim_sigverse_ic', 'tf_static_broadcaster.py'])
            asyncio.sleep(1.0)
        except FileNotFoundError:
            self.get_logger().error("tf_static_broadcaster.py not found. Make sure the package is built and sourced.")

    def cb_object_list(self, msg):
        self.get_logger().info(f"Received object list: {msg.data}")
        self.set_parameters([rclpy.parameter.Parameter(OBJECT_LIST_PARAM, [msg.data])])

    async def yolo_detect_ctrl(self, ctrl):
        while not self.yolo_run_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /yolov5_ros/run_ctrl service...')
        request = RunCtrl.Request()
        request.request = ctrl
        future = self.yolo_run_ctrl_client.call_async(request)
        await future
        if future.result() is not None:
            return future.result().response
        else:
            self.get_logger().error('Service call failed for /yolov5_ros/run_ctrl')
            return False

    def send_robot_motion(self, motion_command):
        msg = String()
        msg.data = motion_command
        self.motion_publisher.publish(msg)
        self.get_logger().info(f"Published robot motion command: {motion_command}")

    def send_base_ctrl(self, base_command):
        msg = String()
        msg.data = base_command
        self.base_ctrl_publisher.publish(msg)
        self.get_logger().info(f"Published base control command: {base_command}")

    # task_commom の代替として、パラメータでタスク失敗状態を管理する例
    def get_task_failed_state(self):
        return self.get_parameter('task_failed').get_parameter_value().bool_value

    async def adjustment(self):
        self.send_robot_motion('DETECTING_POSE')
        # await self.yolo_detect_ctrl(True)
        self.get_logger().info("detect_start.")
        tf_save_ctrl_flag = Bool()
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(8)

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3) #ここの時間を少し長めにとると良い

        self.send_robot_motion('LOWEST_DETECTING_POSE')
        await asyncio.sleep(3)

        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(8)

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3)

        # await self.yolo_detect_ctrl(False) #yolo停止

    async def adjustment_high(self):
        self.send_robot_motion('DETECTING_POSE')
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(True)
        self.get_logger().info("detect_start.")
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag = Bool()
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(13)
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(False)
        if self.get_task_failed_state():
            return
        self.send_base_ctrl("T:-29")
        if self.get_task_failed_state():
            return
        self.send_robot_motion('INITIAL_POSE')

        return

    async def adjustment_low(self):
        self.send_robot_motion('DETECTING_POSE2')
        self.send_base_ctrl("T:29")
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(True)
        self.get_logger().info("detect_start.")
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag = Bool()
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(13)
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(False)
        if self.get_task_failed_state():
            return
        self.send_base_ctrl("T:-29")
        if self.get_task_failed_state():
            return
        self.send_robot_motion('INITIAL_POSE')

        return

    async def processing(self):#target_obj
        # self.send_base_ctrl("X:20")
        self.send_robot_motion('DETECTING_POSE')
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(True)
        self.get_logger().info("detect_start")
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag = Bool()
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(13)
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(6)
        if self.get_task_failed_state():
            return

        self.send_robot_motion('DETECTING_POSE2')
        await asyncio.sleep(3)
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(13)
        if self.get_task_failed_state():
            return

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(2)
        if self.get_task_failed_state():
            return

        # await self.yolo_detect_ctrl(False)
        if self.get_task_failed_state():
            return
        self.send_base_ctrl("T:-29")
        if self.get_task_failed_state():
            return
        self.send_robot_motion('INITIAL_POSE')
        return

    async def processing_front(self):#target_obj
        self.send_base_ctrl("T:30")
        self.send_robot_motion('DETECTING_POSE')
        # await self.yolo_detect_ctrl(True)
        self.get_logger().info("detect_start.")

        tf_save_ctrl_flag = Bool()
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3)

        tf_save_ctrl_flag.data = False
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(2)

        self.send_robot_motion('HIGH_DETECTING_POSE')
        tf_save_ctrl_flag.data = True
        self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(3)

        # tf_save_ctrl_flag.data = False
        # self.tf_save_ctrl_pub.publish(tf_save_ctrl_flag)
        await asyncio.sleep(2)

        # await self.yolo_detect_ctrl(False) #yolo停止

        self.send_robot_motion('INITIAL_POSE')

async def main(args=None):
    rclpy.init(args=args)
    object_recognition_node = ObjectRecognitionNode()
    try:
        object_recognition_node.start_tf_broadcaster()
        await object_recognition_node.processing()
        # await object_recognition_node.adjustment()
    finally:
        object_recognition_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())