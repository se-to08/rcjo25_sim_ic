#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
import smach, smach_ros

from .states.task_common import TaskCommon
# from smach_files import *
# from smach_files import move, object_recog, pointing_estimate, ros2_node_ctrl, run_ctrl, send_robot_motion, task_commom, standing_position_estimation, base_ctrl,grasp


###ros 2###
class InteractiveCleanup(Node):
    def __init__(self):
        super().__init__('sim_ic_2025')

    def execute(self):
        sm = smach.StateMachine(outocomes=['finish'])

        with sm:
            smach.StateMachine.add('Start', Start(self),transitions={'start_OK':'Receive'})
            smach.StateMachine.add('Receive', Receive(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Search', Search(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Grasp', Grasp(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Send', Send(self),transitions={'finish':'Is_finish'})
            smach.StateMachine.add('Is_finish', Is_finish(self),transitions={'continue_search':'Search', 'continue_grasp':'Grasp', 'continue_send':'Send', 'session_finish':'Start'})

        sm.execute()
######

###smachクラス###
class Start(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start_OK'])
        self.node = node
    
    def execute(self, userdata):
        # #処理
        # ros2_node_ctrl.processing("kill")
        # run_ctrl.process_run_ctr("stop")

        # ros2_node_ctrl.processing("start")
        # run_ctrl.process_run_ctr("start")
		 
        # task_commom.reset_task_flag() # task管理用のフラグの初期化
        # person_follow.person_follow_initialize()
        # overlaytext.processing(' STATE : Challenge Start ')
		# #move.move_regist()
        # rclpy.sleep(10)
        # return 'start_OK'

class Receive(smach.state):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['finish'])
        self.node = node

    def execute(self, userdata):
        #処理
        global Next_State, Receive_Challenge_Times, Object_Area, Pointing_Position1, Pointing_Position2, Initial_Position, Head_Direction1, Head_Direction2, Pointing_Direction1, Pointing_Direction2, joint_co
		
        self.task_common = TaskCommon()


		Object_Area = ""
		Pointing_Position1 = Point()
		Pointing_Position2 = Point()
		Initial_Position = Point()
		rospy.sleep(1)
		print("=========================")
		### モーションの初期化 ###
		overlaytext.processing(' STATE : Receive Instruction')
		run_ctrl.process_run_ctr("start") # ノード立ち上げ
		rospy.sleep(1)
		print("=========================")

		### アバター待機 ###
		#if task_commom.get_task_failed_state() == True:
		#		Next_State = "time_is_up"
		#		return 'finish'
		overlaytext.processing(' STATE : Receive Instruction\nWait for Are_you_ready message.')
		self.task_commom.wait_ready_and_send_ready_command()
		overlaytext.processing(' STATE : Receive Instruction\nSend I_am_ready message.')

		# ros_node_ctrl.pointing_person_launch()
		# base_ctrl.processing("X:50")
  

		# rospy.sleep(2.0)

		print("1回目")
		self.task_commom.wait_for_frame_exists('person',10)
		Initial_Position = pointing_estimate.get_tf("person_first")
		if Initial_Position == Point():
			Initial_Position.x = 2.25
			Initial_Position.y = 0.0
		print("Person_Initial_position")
		print(Initial_Position)
		print(type(Initial_Position))


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
        smach.State.__init__(self, outcomes=['continue_search', 'continue_grasp', 'continue_send', 'session_finish'])
        self.node = node

    def execute(self, userdata):
        #処理

######



def main():
    rclpy.init()
    node = InteractiveCleanup()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()