#!/bin/bash

Run_roscore () {
	roscore
}

Run_UR_ROS_Driver_NKC () {
	echo "Launching ROS UR Driver (No KinConfig)"
	echo "Settings:"
	echo "<robot_type>: $RobotType"
	echo "robot_ip: $RobotIP"
	echo "KinematicsConfig: None"
	echo ""
	echo ""
	echo ""
	roslaunch ur_robot_driver ${RobotType}_bringup.launch robot_ip:=${RobotIP} 
}

Run_UR_ROS_Driver_KC () {
	echo "Launching ROS UR Driver (KinConfig)"
	echo "Settings:"
	echo "<robot_type>: $RobotType"
	echo "robot_ip: $RobotIP"
	echo "KinematicsConfig: $KinConfig"
	echo ""
	echo ""
	echo ""
	roslaunch ur_robot_driver ${RobotType}_bringup.launch robot_ip:=${RobotIP} kinematics_config:=${KinConfig}
}


Run_ROS_Isaac_Action_Connector () {
	echo "Launching isaac_ur5_connector:"
	roslaunch isaac_ur5_connector isaac_ur5_connector.launch
}

Run_Isaac_ROS_Source () {
	echo "Launching IsaacSIM with ros_workspace sourced..."
	${IsaacPath}/isaac-sim.sh
}

Run_ROS_Isaac_Moveit_Connector () {
	echo "WIP: Dev in progress"
}

Run_UR_ROS2_Driver () {
	echo "WIP: Dev in progress"
}


