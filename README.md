# Cayde_ROS_Isaac
## Description
Functions:
- Provides a launcher (CaydeLauncher) to automate tasks
  - Running common ROS features
  - Running Isaac SIM Standalone Applications (commonly Cortex / Gym)
  - Launching complex folders
  - Utility Menu
  - General Install Menu:
- IsaacSIM_Python
  - Adds a custom cayde_robot.py that makes it easy to add custom robots and control them with custom RMPFlows
  - Makes it easy to add custom .py scripts to Isaac SIM Python
- Cayde
  - Folder of behaviours and descriptions for a UR5 nicknamed Cayde
- ROS / ROS2 Master Workspaces allowing for easy organisation and seperation of scripts

Currently only compatible with Ubuntu 20.04, support provided for:
- Isaac SIM 2022.2
- ROS Noetic
- ROS2 Foxy

Authored by LW, Student Undergraduate of Mechatronic Engineering at Lancaster University for the dissertation titled:

```Developing and Implementing a Digital Twin Control System (DTCS) for a UR5 utilising ROS and Nvidia ISaac SIM```

## Usage:
<p align="center">
 <img src="https://user-images.githubusercontent.com/65248566/218260574-a83cd6ab-07f8-4f88-8f2c-bebf3f48dbf3.png" width=100 height=100 />
</p>

<p align="center">
 <img src="https://user-images.githubusercontent.com/65248566/218261000-a43e2090-6c92-48e2-840b-360386d21f69.png" />
</p>


## To Do:
- Add auto username changer

## Install Instructions:

1. Download CaydeSetupScript.bash and place in ```~/Documents```

2. Make executable and launch CaydeSetupScript.bash
```bash
cd ~/Documents
chmod u+x CaydeSetupScript.bash
~/Documents/CaydeSetupScript.bash
```

3. Select ```Auto-Install```

4. Add CaydeLauncher to Favourites from AppDrawer

5. Optionally delete CaydeSetupScript.bash

6. Open CaydeLauncher

7. Select ```Install Enviroment Menu```

8. Install Enviroment Menu options as needed




