import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from std_msgs.msg import Header
from sobits_interfaces.srv import MoveHandToTargetTF
from sobits_interfaces.action import MoveWheelRotate, MoveWheelLinear, MoveJoint

robot_name = "hsr_sim"

class GraspingRobot(Node):
    def __init__(self):
        super().__init__('grasping_robot')
        self.cli = self.create_client(MoveHandToTargetTF, robot_name+'/move_hand_to_tf')
        self.action_client_joint = ActionClient(self, MoveJoint, robot_name+'/move_joint')
        self.action_client_linear = ActionClient(self, MoveWheelLinear, robot_name+'/move_wheel_linear')
        self.final_result = ""

    def move_hand_to_target_TF(self, target_name, diff_x=0.0, diff_y=0.0, diff_z=0.0):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service available, sending request...')

        req = MoveHandToTargetTF.Request()
        req.target_frame = target_name

        transform = TransformStamped()
        transform.header = Header()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_footprint"
        transform.transform.translation.x = diff_x
        transform.transform.translation.y = diff_y
        transform.transform.translation.z = diff_z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        req.tf_differential = transform

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        euler_angles = self.quaternion_to_euler(response.move_pose.orientation)
        return euler_angles[2], response.move_pose.position, response.target_joint_names, response.target_joint_rad, response.success

    def quaternion_to_euler(self, goal_info):
        q = np.array([goal_info.w, goal_info.x, goal_info.y, goal_info.z])
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp) if math.fabs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [roll, pitch, yaw]

    def move_joint(self, target_joint_names, target_joint_rad):
        while not self.action_client_joint.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        goal_msg = MoveJoint.Goal()
        goal_msg.target_joint_names = target_joint_names
        goal_msg.target_joint_rad = target_joint_rad
        goal_msg.time_allowance = Duration(seconds=5.0).to_msg()

        future = self.action_client_joint.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.success

    def move_wheel_linear(self, move_pose):
        while not self.action_client_linear.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        goal_msg = MoveWheelLinear.Goal()
        goal_msg.target_point.x = move_pose.x
        goal_msg.target_point.y = move_pose.y
        goal_msg.target_point.z = move_pose.z
        goal_msg.time_allowance = Duration(seconds=5.0).to_msg()

        future = self.action_client_linear.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.success

    def grasp(self, object_name):
        diff_x = -0.1
        diff_y = 0.0
        diff_z = 0.05
        # 目標到達に必要な情報の獲得：水平移動・回転・関節角度
        target_yaw, target_point, target_joint_names, target_joint_rad, is_success = self.move_hand_to_target_TF(object_name, diff_x, diff_y, diff_z)
        if not is_success:
            return False

        # ハンドを開く
        print("ハンド開く")
        if not self.move_joint(["hand_motor_joint"], [1.2]):
            print("ハンドを開くのに失敗した")
            return False
        time.sleep(0.5) #同じ関数であるためsleepを入れる

        # 関節角度の調整
        print("関節角度の調整")
        if not self.move_joint(target_joint_names, target_joint_rad):
            print("関節角度の調整に失敗した")
            return False
        
        # 水平移動
        print("水平移動")
        target_point.x *= 1.02
        target_point.y *= 1.02
        if not self.move_wheel_linear(target_point):
            print("水平移動に失敗した")
            return False
        time.sleep(0.5)
        
        # 水平移動
        print("水平移動")
        if not self.move_wheel_linear(Point(x=0.08, y=0.0, z=0.0)):
            print("水平移動に失敗した")
            return False

        # ハンドを閉じる
        print("ハンドを閉じる")
        if self.move_joint(["hand_motor_joint"], [0.0]):
            print("何も掴めなかった")
            return False
        print("ハンドを閉じるのに失敗した。つまり、掴めたということだ✌")

        # 進んだ分だけ後退
        print("水平移動")
        if not self.move_wheel_linear(Point(x=-target_point.x - 0.08 - 0.05, y=0.0, z=0.0)):
            print("水平移動")
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    grasping_robot = GraspingRobot()
    object_name = "rabbit_doll"
    result = grasping_robot.grasp(object_name)
    print(result)
    rclpy.shutdown()

if __name__ == '__main__':
    main()