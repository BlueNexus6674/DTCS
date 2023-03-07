#!/bin/bash

Menu_Run () {
	echo ""
	echo "----- Run Menu -----"
	echo "1: roscore"
	echo "2: UR ROS Driver "
	echo "3: ROS Isaac Action Connector"
	echo "4: ROS Isaac Camera Start"
	echo "10: IsaacSIM with ros_workspace sourced"
	echo "80: Change LDV/RDV Cam"
	echo "81: ROS Isaac Camera Calibration"
	echo "90: ROS Isaac Moveit Connector"
	echo "91: UR ROS2 Driver"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Run_roscore; break;;
			2) Run_UR_ROS_Driver_KC; break;;
			3) Run_ROS_Isaac_Action_Connector; break;;
			4) Run_ROS_Isaac_Camera_Start; break;;
			10) Run_Isaac_ROS_Source; break;;
			80) Run_ChangeCam; break;;
			81) Run_ROS_Isaac_Camera_Calibration; break;;
			90) Run_ROS_Isaac_Moveit_Connector; break;;
			91) Run_UR_ROS2_Driver; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Cortex () {
	echo ""
	echo "----- Isaac Cortex (SA) Menu -----"
	echo "1: Cayde (Main, RobotPeckDelayed, SIM Bridge Disabled)"
	echo "2: Cayde (Main, RobotPeckDelayed, SIM Bridge Enabled)"
	echo "3: Cayde (Main, RobotPeckDelayed, SIM Bridge Disabled)"
	echo "4: Cayde (Main, RobotPeckDelayed, SIM Bridge Enabled)"
	echo "5: Cayde (Follow_Example)"
	echo "6: Franka (peck_state_machine)"
	echo "7: Franka (peck_game)"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Cortex_Cayde_Main_RobotPeckDelayed_BridgeDisabled; break;;
			2) Cortex_Cayde_Main_RobotPeckDelayed_BridgeEnabled; break;;
			3) Cortex_Cayde_Main_Vision_RPD_BridgeDisabled; break;;
			4) Cortex_Cayde_Main_Vision_RPD_BridgeEnabled; break;;
			5) Cortex_Cayde_FollowExample; break;;
			6) Cortex_Franka_PeckStateMachine; break;;
			7) Cortex_Franka_PeckGame; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Gym () {
	echo ""
	echo "----- Isaac Gym Menu -----"
	echo "1: Cartpole"
	echo "2: Quadcopter"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Gym_Cartpole; break;;
			2) Gym_Quadcopter; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Folder () {
	echo ""
	echo "----- Folder Menu -----"
	echo "1: Extension Examples (User Examples)"
	echo "2: Standalone Examples (User Examples)"
	echo "3: Motion Policy Configs (no longer needed)"
	echo "4: OmniIsaacGymEnv"
	echo "5: Cayde (behaviour)"
	echo "6: IsaacSIM_Python"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Folder_Ext; break;;
			2) Folder_SA; break;;
			3) Folder_MPC; break;;
			4) Folder_OmniIsaacGymEnv; break;;
			5) Folder_CaydeBehaviour; break;;
			6) Folder_IsaacSIM_Python; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Utility () {
	echo ""
	echo "----- Util Menu -----"
	echo "1: This Menu Is Currently Unused"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) echo ""; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Install () {
	echo ""
	echo "----- Install Enviroment Menu -----"
	echo "Main Install:"
	echo "0: Install CaydeLauncher"
	echo "1: Auto-Install (Completes Main Install)"
	echo "2: Linux Setup"
	echo "3: Install ROS Noetic"
	echo "4: Install UR ROS Driver"
	echo "5: Install dev_ros_workspace into ros_workspace (Catkin_Make and Source)"
	echo "6: Install ROS Control"
	
	echo ""
	echo "Isaac SIM:"
	echo "REQUIRES IsaacSIM to be installed and to have been launched at least once before installing"
	echo "7: Install ROS (Noetic) Isaac SIM Workspace"
	echo "8: Install ROS2 (Foxy) Isaac SIM Workspace"
	echo "9: Install IsaacSIM Gym Env"
	echo "10: Install IsaacSIM_Python (Add-On)"
	
	echo ""
	echo "MoveIT and WIP:"
	echo "98: Install MoveIT"
	echo "99: Install moveit_workspace ROS (Catkin_Make and Source) "
	echo "998: Install ROS2 Foxy (untested)"
	echo "999: Install UR ROS2 Driver (not yet implemented)"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			0) Install_CaydeLauncher; break;;
			1) Install_Auto_Main_ROS_Menu; break;;
			2) Install_Linux_Config; break;;
			3) Install_ROS_Noetic; break;;
			4) Install_UR_ROS_Noetic_Driver; break;;
			5) Install_dev_ros_workspace; break;;
			6) Install_ROS_Control; break;;
			7) Install_IsaacSIM_ROS_Noetic_Workspace; break;;
			8) Install_IsaacSIM_ROS2_Foxy_Workspace; break;;
			9) Install_IsaacSIM_Gym_Env; break;;
			10) Install_IsaacSIM_Python; break;;
			98) Install_ROS_Moveit; break;;
			99) Install_moveit_workspace_ROS; break;;
			998) Install_ROS2_Foxy; break;;
			999) Install_UR_ROS2_Foxy_Driver; break;;
			[Xx]*) break;;
			* ) echo ${WAM};;
		esac
	done
}
