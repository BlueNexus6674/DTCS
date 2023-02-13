#!/bin/bash

#Paths
IsaacPath=~/.local/share/ov/pkg/isaac_sim-2022.2.0
IsaacPythonPath=~/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh
CaydeRepoPath=~/Documents/CaydeRepo

#Sources That May Not Exist
source /opt/ros/noetic/setup.bash
source /home/cayde/Documents/ros_workspace/devel/setup.bash
#source ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/devel/setup.bash
#source ${CaydeRepoPath}/Master_ROS_Workspace/connector_workspace/devel/setup.bash
#source ${CaydeRepoPath}/Master_ROS_Workspace/customrobot_workspace/devel/setup.bash
#source ${CaydeRepoPath}/Master_ROS_Workspace/moveit_workspace/devel/setup.bash


#clear

# Source Functions
source ${CaydeRepoPath}/CaydeLauncher/Functions/Menu_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Run_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Cortex_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Gym_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Folder_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Utility_Functions.bash
source ${CaydeRepoPath}/CaydeLauncher/Functions/Install_Functions.bash

#Wrong Answer Message
WAM="Invalid Answer, Please Try Again"

cd ~

ecode=0
while [ $ecode -eq 0 ]
do
	echo "----- Cayde Launcher -----"

	echo "Options: "
	echo ""
	echo "1: Run Menu"
	echo "2: Isaac Cortex (SA) Menu"
	echo "3: Isaac GYM Menu"
	echo "4: Launch A Folder Menu"
	echo "5: Utility Menu"
	echo "6: Install Enviroment Menu"
	echo ""
	

	while true; do read -p "Select Option: " select
	    case $select in
		1) Menu_Run; break;;
		2) Menu_Cortex; break;;
		3) Menu_Gym; break;;
		4) Menu_Folder; break;;
		5) Menu_Utility; break;;
		6) Menu_Install; break;;
		[Xx]* ) ecode=1; echo "Exiting..."; break;;
		* ) echo ${WAM};;
	    esac
	done
	
	echo "----- ----- -----"
	echo "Operation Complete"
	echo "----- ----- -----"
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
	echo ""
done










