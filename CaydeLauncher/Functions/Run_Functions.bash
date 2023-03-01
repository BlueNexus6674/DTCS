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

Run_ROS_Isaac_Camera_Start () {	
	echo "Launching isaac_ur5_camera_show"
	roslaunch isaac_ur5_cam cam_both_show.launch LDV:=${LeftDevVideo} RDV:=${RightDevVideo} ResX:=${ResX} ResY:=${ResY}
}

Run_ROS_Isaac_Camera_Start_Calibrated () {	
	echo "Launching isaac_ur5_camera calibrated"
	roslaunch isaac_ur5_cam cam_both_calibrated.launch LDV:=${LeftDevVideo} RDV:=${RightDevVideo} ResX:=${ResX} ResY:=${ResY}
}

Run_ROS_Isaac_Camera_Stereo_Start () {
	echo "Launching stereo_image_proc"
	ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True

}

Run_ROS_Isaac_Camera_Stereo_View () {
	echo "Viewing Stereo"
	rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color _approximate_sync:=True
}

Run_ROS_Isaac_Camera_Calibration () {
	rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.0350 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
}

Run_ROS_Isaac_Camera_Stereo_Calibration () {
	rosrun rqt_reconfigure rqt_reconfigure
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


