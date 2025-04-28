import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class RosNodeController(Node):
    def __init__(self):
        super().__init__('ros_node_controller')
        # self.process_control_subscription = self.create_subscription(
        #     String,
        #     '/process_control',
        #     self.processing_callback,
        #     10
        # )
        self.get_logger().info("Ros Node Controller initialized.")

    # def processing_callback(self, msg):
    #     self.processing(msg.data)

    def processing(self, msg):
        # Nav2関連ノード名リスト（終了対象）
        nav2_nodes = [
            '/map_server', '/amcl', '/behavior_server', '/bt_navigator',
            '/bt_navigator_navigate_through_poses_rclcpp_node',
            '/bt_navigator_navigate_to_pose_rclcpp_node',
            '/controller_server', '/global_costmap/global_costmap',
            '/lifecycle_manager_localization', '/lifecycle_manager_navigation',
            '/local_costmap/local_costmap', '/location_tf_broadcaster',
            '/map_server', '/planner_server', '/rviz', '/rviz_navigation_dialog_action_client',
            '/waypoint_follower', '/smoother_server', '/velocity_smoother'
        ]

        # hsr関連ノード名リスト（終了対象）
        hsr_sim_nodes = [
            '/camera/container', '/camera/points_xyzrgb', '/hsr_sim/joint_action_server', 
            '/hsr_sim/transform_listener_impl_5767ec37d760', '/hsr_sim/wheel_action_server',
            '/launch_ros_530', '/robot_state_publisher', '/rviz2', '/transform_listener_impl_6435286c9ed0', 
        ]

        if msg == "start":
            ##### HSR #####
            cmd = "ros2 launch hsr_sim_common minimal.launch.py"
            subprocess.Popen(cmd, shell=True)
            time.sleep(3)

            ##### slam #####
            cmd = "ros2 launch sobits_navigation hsr_nav2.launch.py "
            subprocess.Popen(cmd, shell=True)
            time.sleep(3)

            ### 立ち位置算出 ###
            # subprocess.Popen(['ros2', 'launch', 'standing_position_estimator', 'standing_position_estimator.launch.py'])
            # time.sleep(3)

            ### pose_proposal ###
            # subprocess.Popen(['ros2', 'launch', 'pose_proposal_network_ros', 'video_3d_hsr.launch.py'])
            # time.sleep(3)

            ##### yolo #####
            # cmd = "ros2 run chainer_yolov3_ros chainer_yolo_obj_point_publisher"
            # subprocess.Popen(cmd, shell=True)
            # time.sleep(3)

            self.get_logger().info('ROS nodes started.')

        elif msg == "kill":
            # # sobit_navigation_stuck
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/controller_server', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/smoother_server', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/planner_server', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/behavior_server', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/bt_navigator', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/waypoint_follower', 'shutdown'])
            # subprocess.Popen(['ros2', 'lifecycle', 'set', '/velocity_smoother', 'shutdown'])

            # # hsr_ros
            # subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/hsr_controller'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/joint_controller'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/sub_obj_depth'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_point_cloud_xyz'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_point_cloud_xyzrgb'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_register'])

            # standing_position_estimator
            # subprocess.Popen(['ros2', 'node', 'kill', '/standing_position_estimator/standing_position_estimator_node'])

            # subprocess.Popen(['ros2', 'node', 'kill', '/tf_static_broadcaster'])

            # pose_proposal_network_ros
            # subprocess.Popen(['ros2', 'node', 'kill', '/pose_proposal_network_ros/pose_2d_node'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/pose_proposal_network_ros/video_3d_node'])

            # placeable_position_estimator
            # subprocess.Popen(['ros2', 'node', 'kill', '/placeable_position_estimator/placeable_position_estimater_node'])
            # box_entry_gate_detection
            # subprocess.Popen(['ros2', 'node', 'kill', '/box_entry_gate_detection/box_detection_node'])

            # chainer_yolov3_ros
            # subprocess.Popen(['ros2', 'node', 'kill', '/chainer_yolov3_ros/obj_point_publisher_node'])
            # subprocess.Popen(['ros2', 'node', 'kill', '/chainer_yolov3_ros/yolov3_node'])

            # 各ノードを強制終了する
            for node_name in nav2_nodes:
                self.terminate_node(node_name)

            time.sleep(3.0)

            for node_name in hsr_sim_nodes:
                self.terminate_node(node_name)

            time.sleep(5.0)
            subprocess.Popen(['ros2', 'node', 'cleanup'])
            time.sleep(5)

            self.get_logger().info('************* ROS_NODE RESET FINISH *************')
        else:
            self.get_logger().error("ros_node_ctrl: unknown msg...")

    # 指定したノード名に関連するプロセスを終了させる関数
    def terminate_node(self, node_name):
        try:
            result = subprocess.run(['pgrep', '-f', node_name], stdout=subprocess.PIPE)
            pids = result.stdout.decode().split()  # PIDのリストを取得
            self.get_logger().info(f"{node_name} のPID: {pids}")
            for pid in pids:
                os.kill(int(pid), signal.SIGTERM)  # SIGTERMでプロセスを終了
                self.get_logger().info(f"プロセス {pid} を終了しました")
        except Exception as e:
            self.get_logger().error(f"{node_name} の終了中にエラーが発生: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros_node_ctrl = RosNodeController()

    try:
        message = "kill"
        #   - start
        #   - kill        
        # rclpy.spin(ros_node_ctrl)
        ros_node_ctrl.processing(message)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node_ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()