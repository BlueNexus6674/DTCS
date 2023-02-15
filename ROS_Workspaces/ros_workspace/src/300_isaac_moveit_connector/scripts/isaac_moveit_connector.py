#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg

import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

try:
	from math import pi, tau, dist, fabs, cos
except:
	from math import pi, fabs, cos, sqrt
	tau = 2.0*pi
	def dist(p, q):
		return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


print("----- Important Message -----")
print("Imports Complete")
print("----- END Important Message END -----")

#Vars
PlanningGroupName = "urarm"

#Init MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("isaac_moveit_connector", anonymous=True)

#Define Interfaces
UR5_Commander = moveit_commander.RobotCommander()

PlanningSceneInterface = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander(PlanningGroupName)




def Connector():
	
	#Define DisplayTrajectory ROS publisher for RVIZ
	#display_trajectory_publisher = rospy.Publisher(
	#	"/move_group/display_planned_path",
	#	moveit_msgs.msg.DisplayTrajectory,
	#	queue_size=20,
	#)

	#Initiation Successful
	print("----- Important Message -----")
	print("Init Complete")
	print("----- END Important Message END -----")
	rospy.get_time()
	
	#Starting Main Loop
	while not rospy.is_shutdown():
		#Get Joint_Command topic
		rospy.Subscriber("/joint_command", JointState, JSMsgData, queue_size=1)
		
		#Sleep
		rospy.spin()

def JSMsgData(JSMsg):
	#Recieve and sort msg data
	print("--- Debug: Recieved Data ---")
	JointNames = JSMsg.name
	JointPositions = JSMsg.position
	
	#Set Joint_Command as goal
	print("--- Debug: Setting Goal ---")
	joint_goal = move_group.get_current_joint_values()
	joint_goal[0] = JointPositions[0]
	joint_goal[1] = JointPositions[1]
	joint_goal[2] = JointPositions[2]
	joint_goal[3] = JointPositions[3]
	joint_goal[4] = JointPositions[4]
		
	#Plan and execute goal
	print("--- Debug: Executing Goal ---")
	move_group.go(joint_goal, wait=True)
	
	print("--- Debug: Goal Achieved ---")
	#move_group.stop()


if __name__ == '__main__':
	try:
		Connector()
	except rospy.ROSInterruptException:
		pass
	


