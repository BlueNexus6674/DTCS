# DTCS
## Description
Functions:
- Provides a launcher (CaydeLauncher) to automize:
  - Running (Executing) common ROS commands.
  - Launching Isaac SIM Standalone Applications (commonly Cortex / Gym).
  - Opening obscure folders.
  - Installing the dev environment (such as ROS, ROS2, IsaacSIM, etc).
- IsaacSIM_Python
  - Adds cayde_robot.py, a purpose-built script that makes it easy to add custom robots in IsaacSIM.
- DTCS
  - Folder of behaviours and descriptions for a UR5 nicknamed Cayde
- ROS/ROS2 Master Workspaces. Contains dev and active workspaces.

Currently only compatible with Ubuntu 20.04, support is provided for:
- Isaac SIM 2022.2
- ROS Noetic

Authored by LW, Student Undergraduate of Mechatronic Engineering at Lancaster University for the dissertation titled:

```Developing and Implementing a Digital Twin Control System (DTCS) for a UR5 utilising ROS and Nvidia ISaac SIM```


## Usage:
<p align="center">
 <img src="https://user-images.githubusercontent.com/65248566/218260574-a83cd6ab-07f8-4f88-8f2c-bebf3f48dbf3.png" width=100 height=100 />
</p>

<p align="center">
 <img src="https://user-images.githubusercontent.com/65248566/218261000-a43e2090-6c92-48e2-840b-360386d21f69.png" />
</p>

# Install and Config Instructions:
## Install:
1. Download, make executable, and run CaydeSetupScript.bash
```bash
cd ~/Documents
wget https://raw.githubusercontent.com/BlueNexus6674/DTCS/main/CaydeSetupScript.bash
sudo chmod u+x CaydeSetupScript.bash
~/Documents/CaydeSetupScript.bash
```

2. Select ```Auto-Install```

3. Post-install, restart.

4. Add CaydeLauncher to Favourites from AppDrawer

5. Optionally delete CaydeSetupScript.bash 

6. Open CaydeLauncher

7. Select ```Install Enviroment Menu```

8. Install Enviroment Menu options as needed (OR USE AUTO-INSTALL)

It's recommended to install and launch IsaacSIM at least once before using this repository. 
