#!/bin/bash

#----- Install Functions -----#
Install_Auto_Main_ROS_Menu (){

	echo ""
	echo " --- Auto-Install ---"
	echo "Pre-requisites:"
	echo "- IsaacSIM MUST be installed BEFORE running this auto-installer"
	
	echo ""
	
	echo "Method:"
	echo "- Update System"
	echo "- Install ROS Noetic"
	echo "- Install UR_ROS_Noetic_Driver into ros_workspace"
	echo "- Install dev_ros_workspace into ros_workspace (developed by LW)"
	#echo "- Install moveit_workspace"
	echo "- Install ROS Control"
	#echo "- Install ROS moveit"
	echo "- Install IsaacSIM Features"
	echo "--- Install Isaac SIM ROS Workspace"
	echo "--- Install Isaac SIM Gym Environment"
	echo "--- Install IsaacSIM_Python (add-on, developed by LW)"
	
	echo ""
	
	echo "Options:"
	echo "1: Install IsaacSIM"
	echo "2: Continue with Auto-Install"
	echo "X: Quit"
	
	while true; do read -p "Select Option: " select
		case $select in
			1) Install_IsaacSIM; break;;
			2) Install_Auto_Main_ROS; break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done	
}

Install_Auto_Main_ROS () {
	# Setup
	Install_Linux_Config
	Install_ROS_Noetic
	
	# ros_workspace
	Install_UR_ROS_Noetic_Driver
	Install_dev_ros_workspace
	
	#Other workspaces
	#Install_moveit_workspace_ROS
	
	# ROS packages
	Install_ROS_Control
	#Install_ROS_Moveit
	
	# Isaac SIM
	Install_IsaacSIM_ROS_Noetic_Workspace
	Install_IsaacSIM_Gym_Env
	Install_IsaacSIM_Python
}

Install_IsaacSIM () {
	echo ""
	echo " --- Install IsaacSIM --- "
	echo ""
	echo "IsaacSIM must be installed manually"
	echo ""
	echo "Official Instructions: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html"
	echo ""
	echo "Instructions:"
	echo "- Check system requirements / driver requirements"
	echo "- Download Omniverse App Image"
	echo "- ENSURE Nvidia Driver is installed on system (Author is currently using 'nvidia-driver-525-open' installed by 'Software and Updates')"
	echo "- Open Omniverse App Image (may require permissions / making executable)"
	echo "- Install 'Cache' through Omniverse App Image"
	echo "- Install 'Nucleous' through Omniverse App Image"
	echo "- Install IsaacSIM through Omniverse App Image"
	echo "- OPEN IsaacSIM (this finishes the install process) "
	echo ""
	echo "Download Location: https://www.nvidia.com/en-us/omniverse/download/"
	echo ""
	
	while true; do read -p "Finished (Y/X to exit): " select
		case $select in
			[Yy]* ) break;;
			[Xx]* ) break;;
			* ) echo ${WAM};;
		esac
	done
}





Install_CaydeLauncher () {
	
	echo ""
	echo "----- Installing CaydeLauncher -----"
	echo ""
	#Step 1/3
	sudo apt-get install git
	echo ""
	echo "Step 1/3: Cloning Repo"
	echo ""
	git clone $GitClonePath $DTCSRepoPath
	
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
}

Install_Test () {
	echo ""
	echo "Installing Test"
	echo "Installing Test"
	echo "Installing Test"
	echo ""
}

Install_Linux_Config () {
	echo ""
	echo "Config Linux (Updating)"
	echo ""
	sudo apt update
	sudo apt upgrade -y
	
	sudo apt -y install v4l-utils 
	
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	echo "source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash" >> ~/.bashrc
}

Install_ROS_Noetic () {
	echo ""
	echo "ROS Noetic Install"
	echo ""
	sudo apt update && sudo apt upgrade
	#Instructions from http://wiki.ros.org/noetic/Installation/Ubuntu
	 
	#Setup your source list
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	 
	#Set up your keys
	sudo apt -y install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

	#Update Package Index
	sudo apt update

	#Installation
	sudo apt -y install ros-noetic-desktop-full
	 
	#Source ROS
	source /opt/ros/noetic/setup.bash
	#echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

	#Dependencies for building packages
	sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

	#Initialize rosdep
	sudo apt -y install python3-rosdep
	sudo rosdep init
	rosdep update
}

Install_ROS2_Foxy () {
	echo ""
	echo "ROS2 Foxy Install"
	echo ""
	#Instructions from https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

	sudo apt update
	sudo apt upgrade -y

	#Setup sources
	sudo apt -y install software-properties-common
	sudo add-apt-repository universe
	 
	#Add ROS2 GPG Key
	sudo apt update
	sudo apt -y install curl
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	 
	#Add repository to sources list
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	 
	#Install ROS2 packages
	sudo apt update
	sudo apt upgrade -y
	sudo apt -y install ros-foxy-desktop python3-argcomplete

	#Source setup script
	#source /opt/ros/foxy/setup.bash
	#echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
	
	#For rosdep install command
	sudo apt -y install ros-dev-tools
	 
	#For colcon build command
	sudo apt -y install python3-colcon-common-extensions
}

Install_UR_ROS_Noetic_Driver () {
	echo ""
	echo "Universal_Robots_ROS-Driver (Noetic) Install"
	echo ""
	#Instructions from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
	 
	#Source ROS Noetic
	source /opt/ros/noetic/setup.bash
	 
	#Move to Catkin Workspace
	cd ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/src

	#Clone the Driver
	git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git Universal_Robots_ROS_Driver

	#Clone the UR Description (At the time of writing,the melodic-devel branch must be used)
	git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git universal_robot
		 
	#Install Dependencies
	cd ${DTCSRepoPath}/ROS_Workspaces/ros_workspace
	sudo apt update -qq
	rosdep update
	rosdep install --from-paths src --ignore-src -y
	 
	#Build ROS_Workspace
	catkin_make
	 
	#Source ROS_Workspace
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash 
}

Install_UR_ROS2_Foxy_Driver () {
	echo ""
	echo "Universal_Robots_ROS2-Driver (Foxy) WIP Install"
	echo "Currently not working..."
	echo ""
}

Install_IsaacSIM_ROS_Noetic_Workspace () {
	echo ""
	echo "Isaac SIM Workspace (Noetic) Install Install"
	echo ""
	#Instructions from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html
	 
	#Move IsaacSIM Workspace from Isaac Sim Install to /DTCSRepoPath/ROS_Workspaces/ros_workspace/src
	cp -R ~/.local/share/ov/pkg/isaac_sim-2022.2.0/ros_workspace/src/* ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/src/
	
	#Source ROS
	source /opt/ros/noetic/setup.bash
	 
	#Resolve dependencies
	cd ${DTCSRepoPath}/ROS_Workspaces/ros_workspace
	rosdep install -i --from-path src --rosdistro noetic -y

	#Build ros_Workspace
	catkin_make
	 
	#Source ros_workspace
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash 
}

Install_IsaacSIM_ROS2_Foxy_Workspace () {
	echo ""
	echo "Isaac SIM Workspace (Foxy) Install Install"
	echo ""
	#Instructions from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html
	 
	#Move Isaac SIM Workspace2 from Isaac Sim Install to ~/DTCSRepoPath/ROS_Workspaces/ros2_workspace/src
	cp -R ~/.local/share/ov/pkg/isaac_sim-2022.2.0/ros_workspace2/src/* ${DTCSRepoPath}/ROS_Workspaces/ros2_workspace/src/
	 
	#Source ROS
	source /opt/ros/foxy/setup.bash
	 
	#Resolve dependencies
	cd ~/Documents/ros2_workspace
	rosdep install -i --from-path src --rosdistro foxy -y

	#Build ros2_workspace
	colcon build
	 
	#Source ros2_workspace
	source ${DTCSRepoPath}/ROS_Workspaces/ros2_workspace/install/setup.bash 
}

Install_IsaacSIM_Gym_Env () {
	echo ""
	echo "IsaacSIM_Gym_Env Install"
	echo ""
	cd /home/${UserName}/
	git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
	cd /home/${UserName}/OmniIsaacGymEnvs/
	$IsaacPythonPath -m pip install -e .
}

Install_IsaacSIM_Python () {
	echo ""
	echo "IsaacSIM_Python (Add-on) Install"
	echo ""
	cd ${DTCSRepoPath}/IsaacSIM_Python
	$IsaacPythonPath -m pip install -e .
}

Install_dev_ros_workspace () {

	echo ""
	echo "connector_workspace"
	cp -R ${DTCSRepoPath}/ROS_Workspaces/dev_ros_workspace/src/* ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/src
	
	#Source ROS
	source /opt/ros/noetic/setup.bash
	 
	#Resolve dependencies
	cd ${DTCSRepoPath}/ROS_Workspaces/ros_workspace
	rosdep install -i --from-path src --rosdistro noetic -y

	#Build ros_Workspace
	catkin_make
	 
	#Source ros_workspace
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash
}

Install_moveit_workspace_ROS () {
	echo ""
	echo "moveit_workspace"
	echo ""
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash
	cd ${DTCSRepoPath}/ROS_Workspaces/moveit_workspace
	rosdep install -i --from-path src --rosdistro noetic -y
	catkin_make
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash
}

Install_ROS_Control () {
	echo ""
	echo "ROS Control Install"
	echo ""
	sudo apt-get -y install ros-noetic-ros-control ros-noetic-ros-controllers
}

Install_ROS_Moveit () {
	echo ""
	echo "ROS Moveit Install"
	echo ""
	sudo apt -y install ros-noetic-moveit
}
