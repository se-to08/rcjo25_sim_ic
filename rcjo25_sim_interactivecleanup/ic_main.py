#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
import smach, smach_ros
import time

from rcjo25_sim_interactivecleanup.element.ros_node_control3 import RosNodeController
from rcjo25_sim_interactivecleanup.element.run_control import RunControl
from rcjo25_sim_interactivecleanup.element.tf_listener2_for_debug import TfListener2NodeForDebug
from rcjo25_sim_interactivecleanup.element.task_common import TaskCommon
from rcjo25_sim_interactivecleanup.element.person_region_estimator import PersonRegionEstimator
from rcjo25_sim_interactivecleanup.element.pointing_direction_estimator5 import PointingDirectionEstimator
from rcjo25_sim_interactivecleanup.element.change_pose import MoveToPoseClient
# from smach_files import *
# from smach_files import move, object_recog, pointing_estimate, ros2_node_ctrl, run_ctrl, send_robot_motion, task_commom, standing_position_estimation, base_ctrl,grasp


# グローバル変数
Next_State = None
# Object_Area = ""
# Initial_Position = Point()
# Pointing_Position1 = Point()
# Pointing_Position2 = Point()
# Pointing_Direction1 = None
# Pointing_Direction2 = None
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
        sm = smach.StateMachine(outocomes=['finish'])

        with sm:
            smach.StateMachine.add('Start', Start(self),transitions={'start_OK':'Check'})
            smach.StateMachine.add('Check', Check(), transitions={'receive':'Receive','search': 'Search','grasp':'Grasp','send':'Send'})
            smach.StateMachine.add('Receive', Receive(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Search', Search(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Grasp', Grasp(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Send', Send(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Is_finish', Is_finish(self),transitions={'continue_task': 'Check', 'session_finish': 'Start'})

        sm.execute()
######

###smachクラス###
class Start(smach.state):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["start_OK"],
            )
        self.node = node

    def execute(self):
        global Next_State

        self.ros_node_control3_node = RosNodeController()
        self.run_ctrl_node = RunControl()

        self.ros_node_control3_node.processing("kill")
        self.run_ctrl_node.process_run_ctr("stop")

        self.ros_node_control3_node.processing("start")
        self.run_ctrl_node.process_run_ctr("start")

        time.sleep(10)
        
        Next_State = 'receive'
        return 'start_OK'


class Check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['receive' ,'search', 'grasp', 'send'])
    def execute(self, userdata):
        global Next_State
        self.get_logger().info(' STATE : Next State Check ')
        if Next_State == 'receive': return 'receive'#指差し認識
        if Next_State == 'search': 	return 'search'	#物体探索
        if Next_State == 'grasp': 	return 'grasp'	#物体把持
        if Next_State == 'send':	return 'send'	#片付け

class Receive(smach.state):
    def __init__(self, node):
        smach.State.__init__(
            self, 
            output_keys = ['Object_Area', 'Pointing_Position1', 'Pointing_Position2',
            'Initial_Position', 'Pointing_Direction1', 'Pointing_Direction2'],
            outcomes=['finish']
            )
        self.node = node

        self.task_common_node = TaskCommon()
        self.Receive_Challenge_Times = 2
        self.tf_listener2_for_debug_node = TfListener2NodeForDebug()
        self.person_region_estimator_node = PersonRegionEstimator()
        self.pointing_direction_node = PointingDirectionEstimator()
        self.change_pose_node = MoveToPoseClient()

    def execute(self, userdata):
        global Next_State

        userdata.Object_Area = ""
		userdata.Pointing_Position1 = Point()
		userdata.Pointing_Position2 = Point()
		userdata.Initial_Position = Point()

        self.get_logger().info("11111111111111111111111111111111111111111111111111111111")
        self.get_logger().info("person フレーム")
        userdata.Initial_Position = self.tf_listener2_for_debug_node.get_tf("person", "map")
        if userdata.Initial_Position == Point():
            self.Initial_Position.x = 2.25
            self.Initial_Position.y = 0.0
        print("Person_Initial_position")
        print(userdata.Initial_Position)        
        
        self.get_logger().info("22222222222222222222222222222222222222222222222222222222")
        self.get_logger().info("ready送信")
        while True:
            msg = self.task_common_node.get_msg()
            if msg is not None:
                self.get_logger().info(f"Received message: '{msg.message}'")
                if msg.message == "Are_you_ready?":
                    self.get_logger().info("Are_you_ready? received, sending I_am_ready.")
                    self.task_common_node.send_msg("I_am_ready", "")
                    break
            else:
                self.get_logger().info("Waiting for message...")

            rclpy.spin_once(self.task_common_node)
            time.sleep(0.5)  # メッセージのチェック間隔を調整

        self.get_logger().info("33333333333333333333333333333333333333333333333333333333")
        self.get_logger().info("指差し認識開始１")
        for receive_challenge in range(self.Receive_Challenge_Times):

            # if task_commom.get_task_failed_state() == True:
            # 	Next_State = "time_is_up"	
            # 	return 'finish'

            self.change_pose_node.change_pose('initial_pose') #pose変える

            # if task_commom.get_task_failed_state() == True:
            # 	Next_State = "time_is_up"
            # 	return 'finish'

            self.task_common_node.wait_pick_it_up_command()  # 命令待機
            rclpy.spin_once(self.task_common_node)  # コールバックを処理

            userdata.Pointing_Position1 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(userdata.Pointing_Position1)

            time.sleep(5.0)

            if userdata.Pointing_Position1 == Point():
                time.sleep(1.0)
                userdata.Pointing_Position1 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION1")
            print(userdata.Pointing_Position1)
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

            userdata.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(self.Pointing_Position2)
            time.sleep(2.0)

            userdata.Pointing_Position2 = self.tf_listener2_for_debug_node.get_tf("person", "map")
            print(userdata.Pointing_Position2)
            time.sleep(5.0)

            if userdata.Pointing_Position2 == Point():
                time.sleep(1.5)
                userdata.Pointing_Position2 = userdata.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION2")
            print(userdata.Pointing_Position2)
            time.sleep(2.0)

            if userdata.Pointing_Position2 == Point():
                time.sleep(1.5)
                userdata.Pointing_Position2 = userdata.tf_listener2_for_debug_node.get_tf("person", "map")
            print("POINTING_POSITION2")
            print(userdata.Pointing_Position2)
            time.sleep(2.0)

            userdata.Pointing_Direction2 = self.pointing_direction_node.pointing_direction()
            # if Pointing_Direction1 == "FAILURE":
            # 	Next_State = "time_is_up"
            # 	return 'finish'
            
            Next_State = 'search'
            return 'receive_finish'



class Search(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['finish'])
        self.node = node

    def execute(self, userdata):
        #処理

class Grasp(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['finish'])
        self.node = node

    def execute(self, userdata):
        #処理

class Send(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['finish'])
        self.node = node

    def execute(self, userdata):
        #処理

class Is_finish(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['continue_task', 'session_finish'])
        self.node = node

    def execute(self, userdata):
        global Next_State

		if Next_State == "search": # 位置の推定が完了
			self.get_logger().info(' STATE : Is_finish\nSuccessful acquisition of skeleton!!!')
			return 'continue_task'
		if Next_State == "grasp": # 位置の推定が完了
			self.get_logger().info(' STATE : Is_finish\nSuccessful search object!!!')
			return 'continue_task'
		elif Next_State == "send": # 物体把持の処理が完了
			self.get_logger().info(' STATE : Is_finish\nSuccessful object gripping!!!')
			rospy.sleep(1.0)
			if task_commom.get_task_failed_state() == False: # 把持成功
				self.get_logger().info(' STATE : Is_finish\nSuccessful object gripping!!!')
				move._cancel()
				ros_node_ctrl.processing("kill")
				run_ctrl.process_run_ctr("stop")
				rospy.sleep(2)
				return 'continue_task'
			else: # 把持失敗
				self.get_logger().info(' STATE : Is_finish\nFailure to grasp objects.')
				move.move_cancel()
				# ros_node_ctrl.processing("kill")
				# run_ctrl.process_run_ctr("stop")
				rospy.sleep(2)
				return 'session_finish'
		elif Next_State == "finish": # 物体片付け処理が完了
			rospy.sleep(1.0)
			if task_commom.get_task_failed_state() == False: # 物を指定場所に置くのに成功
				self.get_logger().info(' STATE : Is_finish\nSuccessful to put a target object!!!')
				# ros_node_ctrl.processing("kill")
				# run_ctrl.process_run_ctr("stop")
				rospy.sleep(2)
			else:
				self.get_logger().info(' STATE : Is_finish\nFailed to place the target object.')
				# ros_node_ctrl.processing("kill")
				# run_ctrl.process_run_ctr("stop")
				rospy.sleep(2)
			return 'session_finish'
		elif Next_State == "give_up":
			self.get_logger().info(' STATE : Is_finish\nGive up.')
			task_commom.pub_task_msg("Give_up")
			# move.move_cancel()
			# ros_node_ctrl.processing("kill")
			# run_ctrl.process_run_ctr("stop")
			rospy.sleep(2)
			return 'session_finish'
		else:
			self.get_logger().info(' STATE : Is_finish\nTime is up.')
			# move.move_cancel()
			# ros_node_ctrl.processing("kill")
			# run_ctrl.process_run_ctr("stop")
			rospy.sleep(2)
			return 'session_finish'


######



def main():
    rclpy.init()
    node = InteractiveCleanup()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()