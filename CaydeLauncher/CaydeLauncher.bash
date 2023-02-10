#!/bin/bash
#My Script
clear

IsaacPythonPath=~/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh
CaydeRepoPath=~/Documents/CaydeRepo/

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
	echo "5: Util Menu"
	echo "6: Install Enviroment Menu"
	echo ""
	

	while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=1; break;;
		[2]* ) option=2; break;;
		[3]* ) option=3; break;;
		[4]* ) option=4; break;;
		[5]* ) option=5; break;;
		[6]* ) option=6; break;;
		[Xx]* ) ecode=1; echo "Exiting..."; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done

	#--------------------------------------------------------------------- Switch
	
	if [ $option -eq 1 ]
	then
		echo ""
		echo "----- Run Menu -----"
		echo "1: roscore"
		#echo "2: Cayde (peck_game)"
		#echo "3: Franka (peck_state_machine)"
		#echo "4: Franka (peck_game)"
		echo ""
		
		#while true; do read -p "Select Option: " select
	    #case $select in
		#[1]* ) option=201; break;;
		#[2]* ) option=202; break;;
		#[3]* ) option=203; break;;
		#[4]* ) option=204; break;;
		#[Xx]* ) option=0; break;;
		#* ) echo "Please answer a correct answer.";;
	    #esac
	#done
	fi
	
	if [ $option -eq 2 ]
	then
		echo ""
		echo "----- Isaac Cortex (SA) Menu -----"
		echo "1: Cayde (peck_state_machine)"
		echo "2: Cayde (peck_game)"
		echo "3: Franka (peck_state_machine)"
		echo "4: Franka (peck_game)"
		echo ""
		
		while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=201; break;;
		[2]* ) option=202; break;;
		[3]* ) option=203; break;;
		[4]* ) option=204; break;;
		[Xx]* ) option=0; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done
	fi
	
	if [ $option -eq 3 ]
	then
		echo ""
		echo "----- Isaac Gym Menu -----"
		echo "1: Cartpole"
		echo "2: Quadcopter"
		echo ""
		
		while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=301; break;;
		[2]* ) option=302; break;;
		[Xx]* ) option=0; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done
	fi
	
	if [ $option -eq 4 ]
	then
		echo ""
		echo "----- Folder Menu -----"
		echo "1: Extension Examples (User Examples)"
		echo "2: Standalone Examples (User Examples)"
		echo "3: Motion Policy Configs (no longer needed)"
		echo "4: OmniIsaacGymEnv"
		echo "5: CustomIsaacRepo (Cayde)"
		echo "6: CustomIsaacRepo (custom_isaac_repo)"
		echo ""
		
		while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=401; break;;
		[2]* ) option=402; break;;
		[3]* ) option=403; break;;
		[4]* ) option=404; break;;
		[5]* ) option=405; break;;
		[6]* ) option=406; break;;
		[Xx]* ) option=0; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done
	fi
	
	if [ $option -eq 5 ]
	then
		echo ""
		echo "----- Util Menu -----"
		echo "1: Clone Isaac Gym"
		echo "2: Install Isaac Gym"
		echo "3: Install CustomIsaacRepo"
		echo ""
		
		while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=501; break;;
		[2]* ) option=502; break;;
		[3]* ) option=503; break;;
		[Xx]* ) option=0; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done
	fi
	
	if [ $option -eq 6 ]
	then
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
		echo "9: Install IsaacSIM_Python (Add-On)"
		echo "80: Install customrobot_workspace ROS (Catkin_Make and Source)"
		echo "81: Install moveit_workspace ROS (Catkin_Make and Source) "
		echo "98: Install ROS Control"
		echo "99: Install MoveIT"
		echo ""
		echo ""
		echo ""
		echo "Please only enter option 7 or 8 after installing the Omniverse AppImage provided by Nvidia, and then installing and launching IsaacSIM at least once"
		echo ""
		echo ""
		echo""
		
		while true; do read -p "Select Option: " select
	    case $select in
		[1]* ) option=601; break;;
		[2]* ) option=602; break;;
		[3]* ) option=603; break;;
		[4]* ) option=604; break;;
		[5]* ) option=605; break;;
		[6]* ) option=606; break;;
		[7]* ) option=607; break;;
		[8]* ) option=608; break;;
		[9]* ) option=609; break;;
		[80]* ) option=680; break;;
		[81]* ) option=681; break;;
		[98]* ) option=698; break;;
		[99]* ) option=699; break;;
		[Xx]* ) option=0; break;;
		* ) echo "Please answer a correct answer.";;
	    esac
	done
	fi
	
	cd ~
	
	
	
	#----- Isaac Cortex -----#
	if [ $option -eq  201 ]
	then
		$IsaacPythonPath ~/Documents/CustomIsaacRepo/Cayde/Cayde.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_state_machine.py
	fi
	
	if [ $option -eq  202 ]
	then
		$IsaacPythonPath ~/Documents/CustomIsaacRepo/Cayde/Cayde.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_game.py
	fi
	
	if [ $option -eq 203 ]
	then
		$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_state_machine.py
	fi

	if [ $option -eq  204 ]
	then
		$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_game.py
	fi
	
	
	
	#----- Isaac Gym -----#
	if [ $option -eq  301 ]
	then
		$IsaacPythonPath ~/OmniIsaacGymEnvs/omniisaacgymenvs/scripts/rlgames_train.py task=Cartpole
	fi
	
	if [ $option -eq  302 ]
	then
		$IsaacPythonPath ~/OmniIsaacGymEnvs/omniisaacgymenvs/scripts/rlgames_train.py task=Quadcopter
	fi
	
	
	
	#----- Folder -----#
	if [ $option -eq  401 ]
	then
		nautilus ~/.local/share/ov/pkg/isaac_sim-2022.2.0/extension_examples/user_examples/
	fi

	if [ $option -eq  402 ]
	then
		nautilus ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/user_examples/
	fi
	
	if [ $option -eq  403 ]
	then
		nautilus ~/.local/share/ov/pkg/isaac_sim-2022.2.0/exts/omni.isaac.motion_generation/motion_policy_configs/
	fi
	
	if [ $option -eq  404 ]
	then
		nautilus ~/OmniIsaacGymEnvs/
	fi
	
	if [ $option -eq  405 ]
	then
		nautilus ~/Documents/CustomIsaacRepo/Cayde/
	fi
	
	if [ $option -eq  406 ]
	then
		nautilus ~/Documents/CustomIsaacRepo/custom_isaac_repo/
	fi
	
	
	
	#----- Util -----#
	if [ $option -eq  501 ]
	then
		git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
	fi
	
	if [ $option -eq  502 ]
	then
		cd ~/OmniIsaacGymEnvs/
		$IsaacPythonPath -m pip install -e .
		cd ~
	fi
	
	if [ $option -eq  503 ]
	then
		cd ~/Documents/CustomIsaacRepo/custom_isaac_repo/
		$IsaacPythonPath -m pip install -e .
		cd ~
	fi
	
	
	
	#----- Install Menu -----#
	
	if [ $option -eq  601 ]
	then
		echo ""
		echo "Installing CaydeLauncher"
		echo ""
		CaydeRepoLauncherPath=${CaydeRepoPath}"CaydeLauncher/CaydeLauncher.desktop"
		DestinationLauncherPath="~/.local/share/applications/CaydeLauncher.desktop"
		cp $CaydeLauncherPath $DestinationLauncherPath
	fi
	
	if [ $option -eq  602 ]
	then
		echo ""
		echo "Config Linux (Updating)"
		echo ""
		sudo apt update && sudo apt upgrade
		
		echo "${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/devel/setup.bash" >> ~/.bashrc
		#cd ~/Documents/CaydeRepo
		#echo "HelpMe" >> ~/.bashrc
		

		#Work Folders
		#mkdir -p CustomIsaacRepo
		#mkdir -p IsaacSimURDF

		#Catkin workspace creation
		#mkdir -p ros_workspace/src
		#mkdir -p ros2_workspace/src
		#mkdir -p customrobot_workspace/src
		#mkdir -p moveit_workspace/src/ur5_with_rg2_moveit_configuration

		#Source workspaces in all terminals
		#nano ~/.bashrc
		
		#Add the following at the end for ROS
		#source /opt/ros/noetic/setup.bash
		#source ~/Documents/ros_workspace/devel/setup.bash
		#source ~/Documents/customrobot_workspace/devel/setup.bash
		#source ~/Documents/moveit_workspace/devel/setup.bash

		#Add the following at the end for ROS2
		#source /opt/ros/foxy/setup.bash
		#source ~/Documents/ros2_workspace/install/setup.bash
		#source ~/Documents/customrobot_workspace/install/setup.bash
		#source ~/Documents/moveit_workspace/install/setup.bash
	fi
	
	if [ $option -eq  603 ]
	then
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
	fi
	
	if [ $option -eq  604 ]
	then
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
	fi
	
	if [ $option -eq  605 ]
	then
		echo ""
		echo "Universal_Robots_ROS-Driver (Noetic) Install"
		echo ""
		#Instructions from https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
		 
		#Source ROS Noetic
		source /opt/ros/noetic/setup.bash
		 
		#Move to Catkin Workspace
		cd ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/src

		#Clone the Driver
		git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git Universal_Robots_ROS_Driver

		#Clone the UR Description (At the time of writing,the melodic-devel branch must be used)
		git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git universal_robot
			 
		#Install Dependencies
		cd ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace
		sudo apt update -qq
		rosdep update
		rosdep install --from-paths src --ignore-src -y
		 
		#Build ROS_Workspace
		catkin_make
		 
		#Source ROS_Workspace
		source ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/devel/setup.bash 
	fi
	
	if [ $option -eq  606 ]
	then
		echo ""
		echo "Universal_Robots_ROS2-Driver (Foxy) WIP Install"
		echo "Currently not working..."
		echo ""
	fi
	
	if [ $option -eq  607 ]
	then
		echo ""
		echo "Isaac SIM Workspace (Noetic) Install Install"
		echo ""
		#Instructions from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html
		 
		#Move IsaacSIM Workspace from Isaac Sim Install to /CaydeRepoPath/Master_ROS_Workspace/ros_workspace/src
		cp -R ~/.local/share/ov/pkg/isaac_sim-2022.2.0/ros_workspace/src/* ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/src/
		
		#Source ROS
		source /opt/ros/noetic/setup.bash
		 
		#Resolve dependencies
		cd ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace
		rosdep install -i --from-path src --rosdistro noetic -y

		#Build ros_Workspace
		catkin_make
		 
		#Source ros_workspace
		source ${CaydeRepoPath}/Master_ROS_Workspace/ros_workspace/devel/setup.bash 
	fi
	
	if [ $option -eq  608 ]
	then
		echo ""
		echo "Isaac SIM Workspace (Foxy) Install Install"
		echo ""
		#Instructions from https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html
		 
		#Move Isaac SIM Workspace2 from Isaac Sim Install to ~/CaydeRepoPath/Master_ROS2_Workspace/ros2_workspace/src
		cp -R ~/.local/share/ov/pkg/isaac_sim-2022.2.0/ros_workspace2/src/* ${CaydeRepoPath}/Master_ROS2_Workspace/ros2_workspace/src/
		 
		#Source ROS
		source /opt/ros/foxy/setup.bash
		 
		#Resolve dependencies
		cd ~/Documents/ros2_workspace
		rosdep install -i --from-path src --rosdistro foxy -y

		#Build ros2_workspace
		colcon build
		 
		#Source ros2_workspace
		source ${CaydeRepoPath}/Master_ROS2_Workspace/ros2_workspace/install/setup.bash 
	fi
	
	if [ $option -eq  609 ]
	then
		echo ""
		echo "IsaacSIM_Python (Add-on) Install"
		echo ""
		cd ${CaydeRepoPath}/IsaacSIM_Python
		$IsaacPythonPath -m pip install -e .
	fi
	
	if [ $option -eq  680 ]
	then
		echo ""
		echo "customrobot_workspace"
		echo ""
		cd ${CaydeRepoPath}/Master_ROS_Workspace/customrobot_workspace
		rosdep install -i --from-path src --rosdistro noetic -y
		catkin_make
		source ${CaydeRepoPath}/Master_ROS_Workspace/customrobot_workspace/devel/setup.bash 
		echo "${CaydeRepoPath}/Master_ROS_Workspace/customrobot_workspace/devel/setup.bash" >> ~/.bashrc
		

	fi
	
	if [ $option -eq  681 ]
	then
		echo ""
		echo "moveit_workspace"
		echo ""
		cd ${CaydeRepoPath}/Master_ROS_Workspace/moveit_workspace
		rosdep install -i --from-path src --rosdistro noetic -y
		catkin_make
		source ${CaydeRepoPath}/Master_ROS_Workspace/moveit_workspace/devel/setup.bash 
		echo "${CaydeRepoPath}/Master_ROS_Workspace/moveit_workspace/devel/setup.bash" >> ~/.bashrc
		
	fi
	
	if [ $option -eq  698 ]
	then
		echo ""
		echo "ROS Control Install"
		echo ""
		sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
	fi
	
	if [ $option -eq  699 ]
	then
		echo ""
		echo "ROS Moveit Install"
		echo ""
		sudo apt install ros-noetic-moveit
		
	fi
	
	if [ $option -eq  999 ]
	then
		#--- Blank ---#
		echo ""
	fi

	
	

	
	#----------------------------------------------------------------- Quit Loop
	
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

 #./standalone_examples/user_examples/sa_lab_script.py
