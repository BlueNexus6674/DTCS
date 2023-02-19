# Description
Folder for organising ROS/ROS2 Workspaces:

- The `dev_ros/ros2_workspace` is used to develop and store custom packages created for the DTCS project.
- The `ros/ros2_workspace` is the active ROS workspaces.

## ros_workspace
CaydeLauncher sources this workspace. It is purposefully empty as the contents are setup using CaydeLauncher.

## dev_ros_workspace
Contains:
- `ur5withrg2` - Defines the UR5 with RG2 robot setup.
- `isaac_ur5_connector` - Relays joint goals from the '/joint_command' topic to the Universal_Robot_ROS_Driver action server.
- `isaac_moveit_connector` - this works but causes stuttering due to MoveIt Go being a blocking command. 

## ros2_workspace
Currently unused.

## dev_ros2_workspace
Currently unused.
