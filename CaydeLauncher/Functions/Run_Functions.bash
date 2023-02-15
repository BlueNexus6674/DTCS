#!/bin/bash

Run_roscore () {
	roscore
}

Run_UR_ROS_Driver () {
	echo "Launching ROS UR Driver"
	
	RobotType=ur5
	RobotIP=192.168.56.101
	KinConfig=${DTCSRepoPath}/IRL_Cayde_Kin_Config.yaml
	
	echo "Settings:"
	echo "<robot_type>: $RobotType"
	echo "robot_ip: $RobotIP"
	echo "KinematicsConfig: $KinConfig"
	echo ""
	echo ""
	echo ""
	roslaunch ur_robot_driver ${RobotType}_bringup.launch robot_ip:=${RobotIP} 
	#roslaunch ur_robot_driver ${RobotType}_bringup.launch robot_ip:=${RobotIP} kinematics_config:=${KinConfig}
}

Run_UR_ROS2_Driver () {
	echo "WIP: Dev in progress"
}

Run_ROS_Isaac_Action_Connector () {
	echo "Launching isaac_ur5_connector:"
	roslaunch isaac_ur5_connector isaac_ur5_connector.launch
}

Run_ROS_Isaac_Moveit_Connector () {
	echo "WIP: Dev in progress"
}

