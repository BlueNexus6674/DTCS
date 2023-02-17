#!/bin/bash
#My Script
clear

UserName=${USER}

GitClonePath=https://github.com/BlueNexus6674/DTCS.git
DTCSRepoPath=/home/${UserName}/Documents/DTCS/

DTCSRepoLauncherPath=${DTCSRepoPath}"CaydeLauncher/Config/CaydeLauncher.desktop"
DestinationLauncherPath="/home/${UserName}/.local/share/applications/CaydeLauncher.desktop"

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
		[Xx]* ) option=0; ecode=1; echo "Exiting..."; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done


	if [ $option -eq  1 ]
	then
		#Step 1/3
		sudo apt-get install git
		echo ""
		echo "Step 1/3: Cloning Repo"
		echo ""
		git clone $GitClonePath $CaydeRepoPath
		
		#Step 2/3
		echo ""
		echo "Step 2/3: Adding USER parameters to desktop file"
		echo ""
		#Write File
		ExecInfo='Exec="/home/'${USER}'/Documents/DTCS/CaydeLauncher/CaydeLauncher.bash"'
		echo "[Desktop Entry]" > $DTCSRepoLauncherPath
		echo "Version=1.0" >> $DTCSRepoLauncherPath
		echo "Type=Application" >> $DTCSRepoLauncherPath
		echo "Terminal=true" >> $DTCSRepoLauncherPath
		echo $ExecInfo >> $DTCSRepoLauncherPath
		echo "Name=CaydeLauncher" >> $DTCSRepoLauncherPath
		echo "Comment=CaydeLauncher" >> $DTCSRepoLauncherPath
		echo "Icon=/home/${USER}/Documents/DTCS/CaydeLauncher/Config/CaydeLauncher.png" >> $DTCSRepoLauncherPath
		
		#Step 3/3
		echo ""
		echo "Step 3/3: Installing CaydeLauncher"
		echo ""
		cp $DTCSRepoLauncherPath $DestinationLauncherPath
	fi
	
	if [ $option -eq  2 ]
	then
		sudo apt-get install git
		echo ""
		echo "Step: Cloning Repo"
		echo ""
		git clone $GitClonePath $CaydeRepoPath
	fi
	
	if [ $option -eq  3 ]
	then
		#Write File
		ExecInfo='Exec="/home/'${USER}'/Documents/DTCS/CaydeLauncher/CaydeLauncher.bash"'
		echo "[Desktop Entry]" > $DTCSRepoLauncherPath
		echo "Version=1.0" >> $DTCSRepoLauncherPath
		echo "Type=Application" >> $DTCSRepoLauncherPath
		echo "Terminal=true" >> $DTCSRepoLauncherPath
		echo $ExecInfo >> $DTCSRepoLauncherPath
		echo "Name=CaydeLauncher" >> $DTCSRepoLauncherPath
		echo "Comment=CaydeLauncher" >> $DTCSRepoLauncherPath
		echo "Icon=/home/${USER}/Documents/DTCS/CaydeLauncher/Config/CaydeLauncher.png" >> $DTCSRepoLauncherPath
		
		#Install
		echo ""
		echo "Step: Installing CaydeLauncher"
		echo ""
		cp $DTCSRepoLauncherPath $DestinationLauncherPath
	fi
	
	
	

	
	#----------------------------------------------------------------- Quit Loop
	
	echo "----- ----- -----"
	echo "Operation Complete"
	echo "----- ----- -----"
	echo ""

done
