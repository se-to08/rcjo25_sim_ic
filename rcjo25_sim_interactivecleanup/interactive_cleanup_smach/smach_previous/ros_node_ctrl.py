#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time

class NodeController(Node):

    def __init__(self):
        super().__init__('node_controller')
        self.subscription = self.create_subscription(
            String,
            'node_control',
            self.control_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'node_status', 10)

    def person_follow_launch(self):
        self.get_logger().info('Launching person_follower.launch')
        subprocess.Popen(['ros2', 'launch', 'rc2021_sim_sigverse_ic', 'person_follower.launch'])
        time.sleep(5.0)
        self.publish_status('person_follow_launched')

    def person_follow_kill(self):
        self.get_logger().info('Killing person follower nodes')
        subprocess.Popen(['ros2', 'node', 'kill', '/ssd_object_detect/nodelet_manager'])
        subprocess.Popen(['ros2', 'node', 'kill', '/ssd_object_detect/ssd_nodelet'])
        time.sleep(1.0)
        self.publish_status('person_follow_killed')

    def human_pose_launch(self):
        self.get_logger().info('Launching human_pose_estimation.launch')
        subprocess.Popen(["ros2", "launch", "lightweight_human_pose_estimation", "human_pose_estimation.launch"])
        time.sleep(2.0)
        self.publish_status('human_pose_launched')

    def human_pose_kill(self):
        self.get_logger().info('Killing human pose estimation nodes')
        subprocess.Popen(['ros2', 'node', 'kill', '/human_pose_estimation/human_pose_tf_broadcaster'])
        subprocess.Popen(['ros2', 'node', 'kill', '/human_pose_estimation/lightweight_human_pose_estimation'])
        time.sleep(1.0)
        self.publish_status('human_pose_killed')

    def processing(self, msg):
        self.get_logger().info(f'Received control command: "{msg}"')
        if msg == "start":
            ##### HSR #####
            cmd = "ros2 launch hsr_ros minimal.launch"
            subprocess.Popen(cmd, shell=True)
            time.sleep(3)

            ##### slam #####
            cmd = "ros2 launch hsr_navigation hsr_navigation.launch"
            subprocess.Popen(cmd, shell=True)
            time.sleep(3)

            ### 立ち位置算出 ###
            subprocess.Popen(['ros2', 'launch', 'standing_position_estimator', 'standing_position_estimator.launch'])
            time.sleep(3)

            ### pose_proposal ###
            # subprocess.Popen(['ros2', 'launch', 'pose_proposal_network_ros', 'video_3d_hsr.launch'])
            # time.sleep(3)

            ##### yolo #####
            # cmd = "ros2 run chainer_yolov3_ros chainer_yolo_obj_point_publisher"
            # subprocess.Popen(cmd, shell=True)
            # time.sleep(3)
            self.publish_status('nodes_started')

        elif msg == "kill":
            # sobit_navigation_stuck
            subprocess.Popen(['ros2', 'node', 'kill', '/amcl'])
            subprocess.Popen(['ros2', 'node', 'kill', '/map_server'])
            subprocess.Popen(['ros2', 'node', 'kill', '/move_base'])

            # hsr_ros
            subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/hsr_controller'])
            subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/joint_controller'])
            subprocess.Popen(['ros2', 'node', 'kill', '/robot_ctrl/sub_obj_depth'])
            subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_point_cloud_xyz'])
            subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_point_cloud_xyzrgb'])
            subprocess.Popen(['ros2', 'node', 'kill', '/depth_image_proc_register'])

            # standing_position_estimator
            subprocess.Popen(['ros2', 'node', 'kill', '/standing_position_estimator/standing_position_estimator_node'])

            subprocess.Popen(['ros2', 'node', 'kill', '/tf_static_broadcaster'])

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

            time.sleep(8.0)
            subprocess.Popen(['ros2', 'node', 'cleanup'])
            time.sleep(5)
            self.get_logger().info('************* ROS_NODE RESET FINISH *************')
            self.publish_status('nodes_killed')
        else:
            self.get_logger().error(f"Unknown control message: {msg}")
            self.publish_status('unknown_command')

    def control_callback(self, msg):
        self.processing(msg.data)

    def publish_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    rclpy.spin(node_controller)
    node_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()