#!/bin/bash

#----- Install Functions -----#
	
Install_CaydeLauncher () {
	echo ""
	echo "Installing CaydeLauncher"
	echo ""
	DTCSRepoLauncherPath=${DTCSRepoPath}/CaydeLauncher/Config/CaydeLauncher.desktop
	DestinationLauncherPath=~/.local/share/applications/CaydeLauncher.desktop
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
	sudo apt update && sudo apt upgrade
	
	echo "${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash" >> ~/.bashrc
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
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

	#Update Package Index
	sudo apt update

	#Installation
	sudo apt install ros-noetic-desktop-full
	 
	#Source ROS
	source /opt/ros/noetic/setup.bash
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

	#Dependencies for building packages
	sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

	#Initialize rosdep
	sudo apt install python3-rosdep
	sudo rosdep init
	rosdep update
}

Install_ROS2_Foxy () {
	echo ""
	echo "ROS2 Foxy Install"
	echo ""
	#Instructions from https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

	sudo apt update && sudo apt upgrade

	#Setup sources
	sudo apt install software-properties-common
	sudo add-apt-repository universe
	 
	#Add ROS2 GPG Key
	sudo apt update && sudo apt install curl
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	 
	#Add repository to sources list
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	 
	#Install ROS2 packages
	sudo apt update
	sudo apt upgrade
	sudo apt install ros-foxy-desktop python3-argcomplete

	#Source setup script
	#source /opt/ros/foxy/setup.bash
	#echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
	
	#For rosdep install command
	sudo apt install ros-dev-tools
	 
	#For colcon build command
	sudo apt install python3-colcon-common-extensions
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
	cd ~
	git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
	cd ~/OmniIsaacGymEnvs/
	$IsaacPythonPath -m pip install -e .
}

Install_IsaacSIM_Python () {
	echo ""
	echo "IsaacSIM_Python (Add-on) Install"
	echo ""
	cd ${DTCSRepoPath}/IsaacSIM_Python
	$IsaacPythonPath -m pip install -e .
}

Install_connector_workspace_ROS () {

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

Install_customrobot_workspace_ROS () {

	echo ""
	echo "customrobot_workspace"
	echo ""
	cd ${DTCSRepoPath}/DTCS/RobotConfiguration/customrobot_workspace
	rosdep install -i --from-path src --rosdistro noetic -y
	catkin_make
	#source ${DTCSRepoPath}/DTCS/RobotConfiguration/customrobot_workspace/devel/setup.bash 
	#echo "source  ${DTCSRepoPath}/DTCS/RobotConfiguration/customrobot_workspace/devel/setup.bash" >> ~/.bashrc
}

Install_moveit_workspace_ROS () {
	echo ""
	echo "moveit_workspace"
	echo ""
	source ${DTCSRepoPath}/DTCS/RobotConfiguration/customrobot_workspace/devel/setup.bash 
	cd ${DTCSRepoPath}/DTCS/RobotConfiguration/moveit_workspace
	rosdep install -i --from-path src --rosdistro noetic -y
	catkin_make
	source ${DTCSRepoPath}/ROS_Workspaces/ros_workspace/devel/setup.bash
	#source ${DTCSRepoPath}/DTCS/RobotConfiguration/moveit_workspace/devel/setup.bash 
	#echo "source ${DTCSRepoPath}/DTCS/RobotConfiguration/moveit_workspace/devel/setup.bash" >> ~/.bashrc
}

Install_ROS_Control () {
	echo ""
	echo "ROS Control Install"
	echo ""
	sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
}

Install_ROS_Moveit () {
	echo ""
	echo "ROS Moveit Install"
	echo ""
	sudo apt install ros-noetic-moveit
}

