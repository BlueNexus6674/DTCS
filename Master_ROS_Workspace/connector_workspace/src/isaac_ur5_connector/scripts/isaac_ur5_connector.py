#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy

import std_msgs.msg 
import control_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg

import actionlib
import actionlib_msgs

from actionlib_msgs.msg import GoalID

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from math import pi, tau, dist, fabs, cos

#Global
global joint_names
global joint_command
global joint_states

joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
joint_command = ["0", "-1.5707", "0", "-1.5707", "0", "0"]
joint_states = ["0", "-1.5707", "0", "-1.5707", "0", "0"]
joint_goal = ["0", "-1.5707", "0", "-1.5707", "0", "0"]



print("----- Important Message -----")
print("Setup Complete Complete")
print("----- END Important Message END -----")

def Connector():
	global joint_names
	global joint_command
	global joint_states
	
	CycleNum = 0
	
	rospy.init_node("my_node")
	rospy.get_time()
	
	client = actionlib.SimpleActionClient("/scaled_pos_joint_traj_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	client.wait_for_server()
	
	print("Connected to server")
	#Vars
	Rate = rospy.Rate(25) 
	#ROSPublisher = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', JointTrajectory, queue_size=1)

	
	#Initiation Successful

	
	#Starting Main Loop
	while not rospy.is_shutdown():
		#Get Joint_Command topic
		rospy.Subscriber("/joint_command", JointState, JCMsgData, queue_size=1)
		
		#Get Joint_States topic
		rospy.Subscriber("/joint_states", JointState, JSMsgData, queue_size=1)
		
		#Interpolate
		joint_goal[0] = float(joint_command[2])
		joint_goal[1] = float(joint_command[1])
		joint_goal[2] = float(joint_command[0])
		joint_goal[3] = float(joint_command[3])
		joint_goal[4] = float(joint_command[4])
		joint_goal[5] = float(joint_command[5])
		
		#Prepare the Trajectory
		FollowJointTrajectoryMsg = JointTrajectory()
		
		FollowJointTrajectoryMsg.header = Header()
		FollowJointTrajectoryMsg.header.stamp = rospy.Time.now()
		
		FollowJointTrajectoryMsg.joint_names = joint_names
		
		FollowJointTrajectoryMsgPoints = JointTrajectoryPoint()
		FollowJointTrajectoryMsgPoints.positions = joint_goal
		FollowJointTrajectoryMsgPoints.time_from_start = rospy.Duration(2)
		
		FollowJointTrajectoryMsg.points.append(FollowJointTrajectoryMsgPoints)
		
		#Prepare Goal Msg
		goal1 = FollowJointTrajectoryGoal()
		goal1.trajectory = (FollowJointTrajectoryMsg)
		
		client.send_goal(goal1)
		#wait = client.wait_for_result()
		#if not wait:
		#	print("Error Waiting")
		#else:
		#	result = (client.get_result())
		#	resultcode = str(result)[0:13]
		#	if resultcode == "error_code: 0":
		#		print("Successful Publish")
		#	else:
		#		print("Publishing Error")
		#		print("-- FULL ERROR --")
		#		print(result)
		#		print("-- END ERROR --")
		print("Publish Cycle Complete: " + str(CycleNum))
		CycleNum = CycleNum + 1
		
		#Sleep
		Rate.sleep()





def JCMsgData(JCMsg):
	global joint_command
	
	#Recieve and sort msg data
	#print("--- Debug: Recieved Joint Command Data ---")
	joint_names = JCMsg.name
	joint_command = JCMsg.position

def JSMsgData(JSMsg):
	global joint_names
	global joint_states
	
	#Recieve and sort msg data
	#print("--- Debug: Recieved Joint State Data ---")
	joint_names = JSMsg.name
	joint_states = JSMsg.position	
	
	
if __name__ == '__main__':
	try:
		Connector()
	except rospy.ROSInterruptException:
		pass
	


