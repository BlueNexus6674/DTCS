#!/bin/bash
#----- Isaac Cortex -----#
Cortex_Cayde_Main_RobotPeckDelayed_BridgeDisabled () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_Main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/Robot_Peck_Delayed.py --bridge=False
}

Cortex_Cayde_Main_RobotPeckDelayed_BridgeEnabled () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_Main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/Robot_Peck_Delayed.py --bridge=True
}

Cortex_Cayde_Main_Vision_RPD_BridgeDisabled () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_Main_Vision.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/Robot_Peck_Delayed.py --bridge=False
}

Cortex_Cayde_Main_Vision_RPD_BridgeEnabled () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_Main_Vision.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/Robot_Peck_Delayed.py --bridge=True
}

Cortex_Cayde_FollowExample () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_Follow_Example.py
}

Cortex_Franka_PeckStateMachine () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/unused/peck_state_machine.py
}

Cortex_Franka_PeckGame () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/unused/peck_game.py
}
