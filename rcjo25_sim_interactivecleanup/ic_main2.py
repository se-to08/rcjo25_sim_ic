#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
import smach, smach_ros
import time

from geometry_msgs.msg import *
from rcjo25_sim_interactivecleanup.element.ros_node_control import RosNodeController
# from rcjo25_sim_interactivecleanup.element.ros_node_control3 import RosNodeController
from rcjo25_sim_interactivecleanup.element.run_control import RunControl
from rcjo25_sim_interactivecleanup.element.tf_listener5 import TfListenerNode
from rcjo25_sim_interactivecleanup.interactive_cleanup_smach.smach_previous.task_common import TaskCommon
from rcjo25_sim_interactivecleanup.element.person_region_estimator import PersonRegionEstimator
from rcjo25_sim_interactivecleanup.element.pointing_direction_estimator5 import PointingDirectionEstimator
from rcjo25_sim_interactivecleanup.element.change_pose import MoveToPoseClient
# from smach_files import *
# from smach_files import move, object_recog, pointing_estimate, ros2_node_ctrl, run_ctrl, send_robot_motion, task_commom, standing_position_estimation, base_ctrl,grasp


# グローバル変数
Next_State = None
Target_Obj_Frame_Name = []
Grasp_Challenge_Times = 3
Target_Put_Frame_Name = ""
Move_Challenge_Times = 3 		# 移動回数の最大施行回数
Receive_Challenge_Times = 2 	# 指差しを要求する最大施行回数
Grasp_Challenge_Times = 3 		# 1オブジェクトに対する把持の最大施行回数



###ros 2###
class InteractiveCleanup(Node):
    def __init__(self):
        super().__init__('sim_ic_2025')

    def execute(self):
        sm = smach.StateMachine(outcomes=['finish'])

        with sm:
            smach.StateMachine.add('Start', Start(self),transitions={'start_OK':'Check'})
            smach.StateMachine.add('Check', Check(self), transitions={'receive':'Receive'})
            # smach.StateMachine.add('Check', Check(self), transitions={'receive':'Receive','search': 'Search','grasp':'Grasp','send':'Send'})
            smach.StateMachine.add('Receive', Receive(self),transitions={'finish':'Is_finish'})
            # smach.StateMachine.add('Search', Search(self),transitions={'finish':'Is_finish'})
            # smach.StateMachine.add('Grasp', Grasp(self),transitions={'finish':'Is_finish'})
            # smach.StateMachine.add('Send', Send(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Is_finish', Is_finish(self),transitions={'continue_task': 'Check', 'session_finish': 'Start'})

        sm.execute()
######

###smachクラス###
class Start(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["start_OK"],
            )
        self.node = node

        self.task_common_node = TaskCommon()

        self.ros_node_control_node = RosNodeController()
        # self.ros_node_control3_node = RosNodeController() # ros_node_control3のとき
        self.run_ctrl_node = RunControl()

    def execute(self, userdata):
        global Next_State


        self.ros_node_control_node.kill_nodes()
        # self.ros_node_control3_node.processing("kill") # ros_node_control3のとき
        self.run_ctrl_node.process_run_ctr("stop")

        time.sleep(5)

        self.ros_node_control_node.start_nodes()
        # self.ros_node_control3_node.processing("start") # ros_node_control3のとき
        self.run_ctrl_node.process_run_ctr("start")

        self.task_common_node.reset_task_flag()

        time.sleep(10)
        
        Next_State = 'receive'
        return 'start_OK'


class Check(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['receive'])
        self.node = node

    def execute(self, userdata):
        global Next_State
        self.node.get_logger().info(' STATE : Next State Check ')
        if Next_State == 'Next_State': return 'session_finish'


class Receive(smach.State):
    def __init__(self, node):
        smach.State.__init__(
            self, 
            input_keys=['Object_Area', 'Pointing_Position1', 'Pointing_Position2',
                        'Initial_Position', 'Pointing_Direction1', 'Pointing_Direction2'],
            output_keys=['Object_Area', 'Pointing_Position1', 'Pointing_Position2',
                         'Initial_Position', 'Pointing_Direction1', 'Pointing_Direction2'],
            outcomes=['finish']
            )
        self.node = node

        self.task_common_node = TaskCommon()
        self.Receive_Challenge_Times = 1
        self.tf_listener_node = TfListenerNode()
        self.person_region_estimator_node = PersonRegionEstimator()
        self.pointing_direction_node = PointingDirectionEstimator()
        self.change_pose_node = MoveToPoseClient()

    def _is_zero_point(self, point):
        # Pointがゼロかどうかを判定するプライベートメソッド
        return point.x == 0.0 and point.y == 0.0 and point.z == 0.0

    def _try_get_valid_pose(self, retries=3, delay=1.0):
        # 有効な座標を取得するためのリトライ機能
        for _ in range(retries):
            pose = self.tf_listener_node.get_transformed_pose()
            if pose and not self._is_zero_point(pose):
                return pose
            time.sleep(delay)
        return Point()

    def execute(self, userdata):
        global Next_State

        userdata.Object_Area = ""
        userdata.Pointing_Position1 = Point()
        userdata.Pointing_Position2 = Point()
        userdata.Initial_Position = Point()

        self.node.get_logger().info("11111111111111111111111111111111111111111111111111111111")
        self.node.get_logger().info("ready送信")
        self.task_common_node.wait_ready_and_send_ready_command() # Are_you_ready?を待ってI_am_readyを送る     


        self.node.get_logger().info("222222222222222222222222222222222222222222222222222222222222222")
        self.node.get_logger().info("person フレーム")
        self.Initial_Position = self._try_get_valid_pose()  # プライベートメソッドを使う
        print("Person_Initial_position")
        print(self.Initial_Position)

        # tf変換は一旦無視
        # userdata.Initial_Position = self.tf_listener2_for_debug_node.get_tf("person", "map")
        if userdata.Initial_Position == Point():
            userdata.Initial_Position.x = 2.25
            userdata.Initial_Position.y = 0.0
        print("Person_Initial_position")
        print(userdata.Initial_Position)        

        self.node.get_logger().info("33333333333333333333333333333333333333333333333333333333")
        self.node.get_logger().info("指差し認識開始１")
        for receive_challenge in range(self.Receive_Challenge_Times):

            if self.task_common_node.get_task_failed_state() == True:
            	Next_State = "time_is_up"	
            	return 'finish'

            self.change_pose_node.change_pose('initial_pose') #pose変える

            if self.task_common_node.get_task_failed_state() == True:
            	Next_State = "time_is_up"
            	return 'finish'

            self.task_common_node.wait_pick_it_up_command()  # 命令待機
            # rclpy.spin_once(self.task_common_node)  # コールバックを処理

            # userdata.Pointing_Position1 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            userdata.Pointing_Position1 = self._try_get_valid_pose()  # プライベートメソッドを使う
            print(userdata.Pointing_Position1)
            time.sleep(5.0)


            if self._is_zero_point(userdata.Pointing_Position1):
                time.sleep(1.0)
                userdata.Pointing_Position1 = self._try_get_valid_pose()  # プライベートメソッドを使う
            print("POINTING_POSITION1")
            print(userdata.Pointing_Position1)

            if userdata.Initial_Position != (0.0, 0.0, 0.0) and userdata.Pointing_Position1 != (0.0, 0.0, 0.0):
                userdata.Object_Area = self.person_region_estimator_node.search_person_region(userdata.Pointing_Position1, userdata.Initial_Position)
                print("Object_Area:", userdata.Object_Area)

            time.sleep(1.0)

            ######指差し認識
            print("指差し開始")

            userdata.Pointing_Direction1 = self.pointing_direction_node.pointing_direction()
            # if Pointing_Direction1 == "FAILURE":
            # 	Next_State = "time_is_up"
            # 	return 'finish'

            self.task_common_node.wait_clean_up_command()

            userdata.Pointing_Position2 = self._try_get_valid_pose()  # プライベートメソッドを使う
            print(userdata.Pointing_Position2)
            time.sleep(2.0)

            userdata.Pointing_Position2 = self._try_get_valid_pose()  # プライベートメソッドを使う
            print(userdata.Pointing_Position2)
            time.sleep(5.0)

           
            if self._is_zero_point(userdata.Pointing_Position2):
                time.sleep(1.5)
                userdata.Pointing_Position2 = self._try_get_valid_pose()  # プライベートメソッドを使う
            print("POINTING_POSITION2")
            print(userdata.Pointing_Position2)
            time.sleep(2.0)

            userdata.Pointing_Direction2 = self.pointing_direction_node.pointing_direction()

            if self._is_zero_point(userdata.Pointing_Position1) and self._is_zero_point(userdata.Pointing_Position2):
                print("終わりだよ")
                Next_State = 'give_up'
                return 'finish'
            
            Next_State = 'search'
            return 'finish'

class Is_finish(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['continue_task', 'session_finish'])
        self.node = node

        self.task_common_node = TaskCommon()

    def execute(self, userdata):
        global Next_State

        if Next_State == "search": # 位置の推定が完了
            self.node.get_logger().info(' STATE : Is_finish\nSuccessful acquisition of skeleton!!!')
            return 'continue_task'
        
        elif Next_State == "give_up":
            self.node.get_logger().info(' STATE : Is_finish\nGive up.')
            self.task_common_node.pub_task_msg("Give_up")
            # rclpy.spin_once(self.task_common_node, timeout_sec=0.1)  # 明示的にノードを回す
            # move.move_cancel()
            # ros_node_ctrl.processing("kill")
            # run_ctrl.process_run_ctr("stop")
            time.sleep(5)
            return 'session_finish'
        else:
            self.node.get_logger().info(' STATE : Is_finish\nTime is up.')
     
            time.sleep(2)
            return 'session_finish'


######



def main():
    rclpy.init()
    node = InteractiveCleanup()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()