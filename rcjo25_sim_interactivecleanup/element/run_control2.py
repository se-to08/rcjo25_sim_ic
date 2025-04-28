import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class RunControl(Node):
    def __init__(self):
        super().__init__('run_control_node')
        self.yolo_client = self.create_client(SetBool, '/yolo_ros/run_ctrl')
        self.yolo_position_client = self.create_client(SetBool, '/yolo_ros/position/run_ctr')

        self.ssd_client = self.create_client(SetBool, '/ssd_ros/run_ctr')
        self.ssd_position_client = self.create_client(SetBool, '/ssd_ros/position/run_ctr')

        self.box_entry_client = self.create_client(SetBool, '/box_detection_node/run_ctrl')

        self.lightweight_client = self.create_client(SetBool, '/human_pose/run_ctr')
        self.lightweight_keypoints_client = self.create_client(SetBool, '/human_pose/keypoints/run_ctr')


    def send_request(self, package_name, enable):
        self.req = SetBool.Request()  # req属性を初期化
        self.req.data = enable
        if package_name == "yolo":
            while not self.yolo_client.wait_for_service(timeout_sec=1.0) and not self.yolo_position_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('yoloサービスの待機中...')
            self.yolo_future = self.yolo_client.call_async(self.req)
            self.yolo_pos_future = self.yolo_position_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.yolo_future)
            rclpy.spin_until_future_complete(self, self.yolo_pos_future)
            return self.yolo_future.result()

        elif package_name == "ssd":
            while not self.ssd_client.wait_for_service(timeout_sec=1.0) and not self.ssd_position_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('ssdサービスの待機中...')
            self.ssd_future = self.ssd_client.call_async(self.req)
            self.ssd_pos_future = self.ssd_position_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.ssd_future)
            rclpy.spin_until_future_complete(self, self.ssd_pos_future)
            return self.ssd_future.result()

        elif package_name == "box_entry":
            while not self.box_entry_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('box_entryサービスの待機中...')
            self.box_entry_future = self.box_entry_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.box_entry_future)
            return self.box_entry_future.result()

        elif package_name == "lightweight":
            while not self.lightweight_client.wait_for_service(timeout_sec=1.0) and not self.lightweight_keypoints_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('yoloサービスの待機中...')
            self.lightweight_future = self.lightweight_client.call_async(self.req)
            self.lightweight_keypoints_future = self.lightweight_keypoints_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.lightweight_future)
            rclpy.spin_until_future_complete(self, self.lightweight_keypoints_future)
            return self.lightweight_future.result()
        else:
            return False

    #全体をまとめる関数
    def process_run_ctr(msg):
        if msg == "start":
            try:
                # yolov8_run_control(msg)
                ssd_run_control(msg)
                light_weight_run_control(msg)
            except rospy.ROSInterruptException:
                pass
        elif msg == "stop":
            try:
                yolov8_run_control(msg)
                ssd_run_control(msg)
                light_weight_run_control(msg)
                human_pose_kill()
            except rospy.ROSInterruptException:
                pass

def main(args=None):
    rclpy.init(args=args)
    run_control = RunControl()

    # package_name = "lightweight"  # 使用するパッケージ名を指定
    # enable = True  # 有効化するかどうかを指定

    # response = run_control.send_request(package_name, enable)  # TrueまたはFalseを送信

    # run_control.get_logger().info(f'結果: {response.success}')

    try:
        rclpy.spin(run_control)
    except KeyboardInterrupt:
        pass
    finally:
        run_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
