#!/bin/bash
#----- Isaac Cortex -----#
Cortex_Cayde_FullStack_PeckGame () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde_FullStack.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/peck.py --bridge=False
}

Cortex_Cayde_PeckStateMachine () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/peck_state_machine.py
}

Cortex_Cayde_PeckGame () {
	$IsaacPythonPath ${DTCSRepoPath}/DTCS/IsaacSIM/Cayde.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/peck_game.py
}

Cortex_Franka_PeckStateMachine () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/peck_state_machine.py
}

Cortex_Franka_PeckGame () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=${DTCSRepoPath}/DTCS/IsaacSIM/RobotBehaviors/peck_game.py
}
