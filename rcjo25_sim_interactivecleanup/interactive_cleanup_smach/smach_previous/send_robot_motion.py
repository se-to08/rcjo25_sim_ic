#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import String,Bool
from hsr_ros.srv import *

def processing(motion_type):
	rospy.wait_for_service('/robot_ctrl/motion_ctrl')
	try:
		motion = rospy.ServiceProxy("/robot_ctrl/motion_ctrl", robot_motion)
		res = motion(motion_type)
		return res.is_moved
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)

def processing_detect(motion_height):
	rospy.wait_for_service('/robot_ctrl/detect_ctrl')
	try:
		motion = rospy.ServiceProxy("/robot_ctrl/detect_ctrl", detect_ctrl)
		res = motion(motion_height)
		return res.res_bool
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)

def processing_base_ctrl(motion_str):
	rospy.wait_for_service('/robot_ctrl/base_ctrl')
	try:
		motion = rospy.ServiceProxy("/robot_ctrl/base_ctrl", odom_base)
		res = motion(motion_str)
		return res.res_str
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)

def processing_put(put_position, motion_height):
	rospy.wait_for_service('/robot_ctrl/put_ctrl')
	try:
		motion = rospy.ServiceProxy("/robot_ctrl/put_ctrl", put_ctrl)
		res = motion(put_position, motion_height)
		return res.res_bool
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)

def processing_watch(tf_name):
	rospy.wait_for_service('/robot_ctrl/watch_motion')
	try:
		motion = rospy.ServiceProxy('/robot_ctrl/watch_motion', watch_motion)
		res = motion(tf_name)
		return res.res_bool
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)
	return False

if __name__ == '__main__':	#この部分は「$ python sub.py」の時には実行される
	print ("do processing() @robot_motion.py")
	processing("LOWEST_DETECTING_POSE")
	# processing_base_ctrl("T:30")