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

    def ssd_run_control(self, aizu):
        req = SetBool.Request()
        if aizu == "start":
            req.data = True
        elif aizu == "stop":
            req.data = False

        while not self.ssd_client.wait_for_service(timeout_sec=1.0) and not self.ssd_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ssdサービスの待機中...')

        ssd_future = self.ssd_client.call_async(req)
        ssd_pos_future = self.ssd_position_client.call_async(req)
        rclpy.spin_until_future_complete(self, ssd_future)
        rclpy.spin_until_future_complete(self, ssd_pos_future)
        return [ssd_future.result(), ssd_pos_future.result()]

    def lightweight_run_control(self, aizu):
        req = SetBool.Request()
        if aizu == "start":
            req.data = True
        elif aizu == "stop":
            req.data = False

        while not self.lightweight_client.wait_for_service(timeout_sec=1.0) and not self.lightweight_keypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('lightweightサービスの待機中...')

        lw_future = self.lightweight_client.call_async(req)
        lw_key_future = self.lightweight_keypoints_client.call_async(req)
        rclpy.spin_until_future_complete(self, lw_future)
        rclpy.spin_until_future_complete(self, lw_key_future)
        return [lw_future.result(), lw_key_future.result()]

    def yolo_run_control(self, aizu):
        req = SetBool.Request()
        if aizu == "start":
            req.data = True
        elif aizu == "stop":
            req.data = False

        while not self.yolo_client.wait_for_service(timeout_sec=1.0) and not self.yolo_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('yoloサービスの待機中...')

        yolo_future = self.yolo_client.call_async(req)
        yolo_pos_future = self.yolo_position_client.call_async(req)
        rclpy.spin_until_future_complete(self, yolo_future)
        rclpy.spin_until_future_complete(self, yolo_pos_future)
        return [yolo_future.result(), yolo_pos_future.result()]

    def process_run_ctr(self, msg):
        result = []
        if msg == "start":
            try:
                result.extend(self.ssd_run_control(msg))
                result.extend(self.lightweight_run_control(msg))
            except Exception as e:
                self.get_logger().error(f"start時にエラー: {e}")
        elif msg == "stop":
            try:
                # result.extend(self.yolo_run_control(msg))
                result.extend(self.ssd_run_control(msg))
                result.extend(self.lightweight_run_control(msg))
            except Exception as e:
                self.get_logger().error(f"stop時にエラー: {e}")
        return result


def main(args=None):
    rclpy.init(args=args)
    run_control = RunControl()

    try:
        message = "start"  # "start" または "stop"
        responses = run_control.process_run_ctr(message)

        for idx, res in enumerate(responses):
            if res is not None:
                run_control.get_logger().info(f"[応答{idx}] 成功: {res.success} / メッセージ: {res.message}")
            else:
                run_control.get_logger().warn(f"[応答{idx}] 応答がありませんでした。")
    except KeyboardInterrupt:
        pass
    finally:
        run_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
