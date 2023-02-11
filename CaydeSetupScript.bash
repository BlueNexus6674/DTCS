#!/bin/bash
#My Script
clear

GitClonePath=https://github.com/BlueNexus6674/Cayde_ROS_Isaac.git
CaydeRepoPath=~/Documents/CaydeRepo/

ecode=0
while [ $ecode -eq 0 ]
do
	echo "----- Cayde Setup Script -----"

	echo "Options: "
	echo ""
	echo "1: Auto-Install"
	echo "2: Clone Repo"
	echo "3: Install Cayde Launcher"
	echo "X: Exit"
	echo ""
	

	while true; do read -p "Select Option: " select
	    case $select in
		1) option=1; break;;
		2) option=2; break;;
		3) option=3; break;;
		[Xx]* ) ecode=1; echo "Exiting..."; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done


	if [ $option -eq  1 ]
	then
		echo ""
		echo "Step: Cloning Repo"
		echo ""
		git clone $GitClonePath $CaydeRepoPath
		
		echo ""
		echo "Step: Installing CaydeLauncher"
		echo ""
		CaydeRepoLauncherPath=${CaydeRepoPath}"CaydeLauncher/Config/CaydeLauncher.desktop"
		DestinationLauncherPath="~/.local/share/applications/CaydeLauncher.desktop"
		cp $CaydeLauncherPath $DestinationLauncherPath
	fi
	
	if [ $option -eq  2 ]
	then
		echo ""
		echo "Step: Cloning Repo"
		echo ""
		git clone $GitClonePath $CaydeRepoPath
	fi
	
	if [ $option -eq  3 ]
	then
		echo ""
		echo "Step: Installing CaydeLauncher"
		echo ""
		CaydeRepoLauncherPath=${CaydeRepoPath}"CaydeLauncher/Config/CaydeLauncher.desktop"
		DestinationLauncherPath="~/.local/share/applications/CaydeLauncher.desktop"
		cp $CaydeLauncherPath $DestinationLauncherPath
	fi
	
	
	

	
	#----------------------------------------------------------------- Quit Loop
	
	echo "----- ----- -----"
	echo "Operation Complete"
	echo "----- ----- -----"
	echo ""

done

