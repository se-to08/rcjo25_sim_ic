#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
import smach, smach_ros
from .element.tf_listener2_for_debug import TfListener2NodeForDebug
from .element.task_common_send_ready import TaskCommonSendReady
from .element.person_region_estimator import PersonRegionEstimator
from .element.pointing_direction_estimator import PointingDirectionEstimator


class Receive(Node):
    def __init__(self):
        super().__init__('receive')
        self.Object_Area = ""
        self.Pointing_Position1 = Point()
        self.Pointing_Position2 = Point()
        self.Initial_Position = Point()
        self.msg = task_common_send_ready.TaskCommonSendReady()
        self.Receive_Challenge_Times = 2

    def execute(self):
        self.get_logger().info("11111111111111111111111111111111111111111111111111111111")
        self.get_logger().info("person フレーム")
        self.Initial_Position = tf_listener2_for_debug.get_tf("person", "base_footprint") # map座標が取れないのでpersonが取れるかの確認。要変更
        if self.Initial_Position == Point():
            self.Initial_Position.x = 2.25
            self.Initial_Position.y = 0.0
            
        self.get_logger().info("22222222222222222222222222222222222222222222222222222222")
        self.get_logger().info("ready送信")
        while self.msg.get_msg().message == "Are_you_ready?"
            self.msg.send_msg("I_am_ready", "")

        self.get_logger().info("33333333333333333333333333333333333333333333333333333333")
        self.get_logger().info("指差し認識１")
        for receive_challenge in range(Receive_Challenge_Times):
			while self.msg.get_msg().message == ""
                self.msg.send_msg("I_am_ready", "")
			print("2回目")
            ###↑pick it upメッセージを待つ処理
# 



			# Pointing_Position1 = pointing_estimate.get_tf("person")	#指差し中の人のpositionを取得Point
			# print(Pointing_Position1)
			# # person_follow.face_direction_ctrl(True) #人の向き推定
			# rospy.sleep(2.0)
			# rospy.sleep(2.5)
			# if Pointing_Position1 == Point():
			# 	rospy.sleep(1.5)
			# 	print("3回目")
			# 	Pointing_Position1 = pointing_estimate.get_tf("person")
			# print("POINTING_POSITION1")
			# print(Pointing_Position1)
			# Object_Area = pointing_estimate.search_person_region(Pointing_Position1, Initial_Position)
			# print("Object_Area:",Object_Area)

			# rospy.sleep(1)
			# ######指差し認識
			# print("指差し開始")
            
			# # ros_node_ctrl.human_pose_launch()
			# print("1回目の認識")
			# # person_detail_ = rospy.Subscriber("/human_3d/pose_array",KeyPoint3DArray,person_follow.callback_point,queue_size=10)
			# msg = KeyPoint3DArray()
			# Pointing_Direction1 = person_follow.pointing_direction(msg)
			# if Pointing_Direction1 == "FAILURE":
			# 	Next_State = "time_is_up"
			# 	return 'finish'

			# # person_follow.face_direction_ctrl(False)
			
			# overlaytext.processing(' STATE : Receive Instruction\nWait for Clean_up! message.')
			# task_commom.wait_clean_up_command() #命令待機
			# print("4回目")
			# Pointing_Position2 = pointing_estimate.get_tf("person")
			# print(Pointing_Position2)
			# # person_follow.face_direction_ctrl(True) #人の向き推定
			# rospy.sleep(2.0)
			# print("4回目")
			# Pointing_Position2 = pointing_estimate.get_tf("person")
			# print(Pointing_Position2)
			# # person_follow.face_direction_ctrl(True) #人の向き推定
			# rospy.sleep(2.0)
			# rospy.sleep(2.5)
			# if Pointing_Position2 == Point():
			# 	rospy.sleep(1.5)
			# 	print("5回目")
			# 	Pointing_Position2 = pointing_estimate.get_tf("person")
			# print("POINTING_POSITION2")
			# print(Pointing_Position2)
			# rospy.sleep(1)
			# rospy.sleep(2.5)
			# if Pointing_Position2 == Point():
			# 	rospy.sleep(1.5)
			# 	print("5回目")
			# 	Pointing_Position2 = pointing_estimate.get_tf("person")
			# print("POINTING_POSITION2")
			# print(Pointing_Position2)
			# rospy.sleep(1)
			# print("2回目の認識")
			# Pointing_Direction2 = person_follow.pointing_direction(Pointing_Position2)
			# if Pointing_Direction1 == "FAILURE":
			# 	Next_State = "time_is_up"
			# 	return 'finish'
		
			# # ros_node_ctrl.human_pose_kill()

			# Next_State = 'search'
			# return 'finish'


        

def main():
    rclpy.init()
    node = Receive()
    node.execute()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()