#!/bin/bash

#Paths
UserName=${USER}
IsaacPath=/home/${UserName}/.local/share/ov/pkg/isaac_sim-2022.2.0
IsaacPythonPath=/home/${UserName}/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh
DTCSRepoPath=/home/${UserName}/Documents/DTCS

GitClonePath=https://github.com/BlueNexus6674/DTCS.git
DTCSRepoLauncherPath=${DTCSRepoPath}"/CaydeLauncher/Config/CaydeLauncher.desktop"
DestinationLauncherPath="/home/${UserName}/.local/share/applications/CaydeLauncher.desktop"

#Vars
RobotType=ur5
RobotIP=192.168.0.100
KinConfig=${DTCSRepoPath}/IRL_Cayde_Kin_Config.yaml

#Sources That May Not Exist
source /opt/ros/noetic/setup.bash
source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash

#clear

# Source Functions
source ${DTCSRepoPath}/CaydeLauncher/Functions/Menu_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Run_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Cortex_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Gym_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Folder_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Utility_Functions.bash
source ${DTCSRepoPath}/CaydeLauncher/Functions/Install_Functions.bash

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










