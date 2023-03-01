#!/bin/bash

#Paths
DefaultConfig=/home/${USER}/Documents/DTCS/CaydeLauncher/Config/DefaultConfig.yaml
DTCSRepoPath=/home/${USER}/Documents/DTCS

source ${DTCSRepoPath}/CaydeLauncher/Functions/Source_Functions.bash

eval $(Config_Parse $DefaultConfig)
eval $(Config_Parse $CustomConfig)

while [ $ecode -eq 0 ]
do
	echo "----- Cayde Launcher -----"

	echo "Options: "
	echo ""
	echo "1: Run Menu"
	echo "2: Isaac Cortex (SA) Menu"
	echo "3: Isaac GYM Menu"
	echo "4: Launch A Folder Menu"
	echo "5: Install Enviroment Menu"
	echo ""
	

	while true; do read -p "Select Option: " select
	    case $select in
		1) Menu_Run; break;;
		2) Menu_Cortex; break;;
		3) Menu_Gym; break;;
		4) Menu_Folder; break;;
		5) Menu_Install; break;;
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










