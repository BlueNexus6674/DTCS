#!/bin/bash

Menu_Run () {
	echo ""
	echo "----- Run Menu -----"
	echo "1: roscore"
	echo "2: UR ROS Driver"
	echo "3: UR ROS2 Driver"
	echo "4: ROS Isaac Action Connector"
	echo "5: ROS Isaac Moveit Connector"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Run_roscore; break;;
			2) Run_UR_ROS_Driver; break;;
			3) Run_UR_ROS2_Driver; break;;
			4) Run_ROS_Isaac_Action_Connector; break;;
			5) Run_ROS_Isaac_Moveit_Connector; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}

Menu_Cortex () {
	echo ""
	echo "----- Isaac Cortex (SA) Menu -----"
	echo "1: Cayde (FullStack, peck_game)"
	echo "2: Cayde (peck_state_machine)"
	echo "3: Cayde (peck_game)"
	echo "4: Franka (peck_state_machine)"
	echo "5: Franka (peck_game)"
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Cortex_Cayde_FullStack_PeckGame; break;;
			2) Cortex_Cayde_PeckStateMachine; break;;
			3) Cortex_Cayde_PeckGame; break;;
			4) Cortex_Franka_PeckStateMachine; break;;
			5) Cortex_Franka_PeckGame; break;;
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
	echo "1: Install CaydeLauncher"
	echo "2: Linux Setup"
	echo "3: Install ROS Noetic"
	echo "4: Install ROS2 Foxy"
	echo "5: Install UR ROS Driver"
	echo "6: Install UR ROS2 Driver"
	echo "7: Install ROS (Noetic) Isaac SIM Workspace"
	echo "8: Install ROS2 (Foxy) Isaac SIM Workspace"
	echo "9: Install IsaacSIM Gym Env"
	echo "10: Install IsaacSIM_Python (Add-On)"
	echo "80: Install connector ROS package (Catkin_Make and Source)"
	echo "81: Install customrobot_workspace ROS (Catkin_Make and Source)"
	echo "82: Install moveit_workspace ROS (Catkin_Make and Source) "
	echo "98: Install ROS Control"
	echo "99: Install MoveIT"
	echo ""
	echo ""
	echo ""
	echo "Please only enter option 7 or 8 after installing the Omniverse AppImage provided by Nvidia, and then installing and launching IsaacSIM at least once"
	echo ""
	echo ""
	echo ""
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Install_CaydeLauncher; break;;
			2) Install_Linux_Config; break;;
			3) Install_ROS_Noetic; break;;
			4) Install_ROS2_Foxy; break;;
			5) Install_UR_ROS_Noetic_Driver; break;;
			6) Install_UR_ROS2_Foxy_Driver; break;;
			7) Install_IsaacSIM_ROS_Noetic_Workspace; break;;
			8) Install_IsaacSIM_ROS2_Foxy_Workspace; break;;
			9) Install_IsaacSIM_Gym_Env; break;;
			10) Install_IsaacSIM_Python; break;;
			80) Install_connector_workspace_ROS; break;;
			81) Install_customrobot_workspace_ROS; break;;
			82) Install_moveit_workspace_ROS; break;;
			98) Install_ROS_Control; break;;
			99) Install_ROS_Moveit; break;;
			[Xx]*) break;;
			* ) echo ${WAM};;
		esac
	done
}
