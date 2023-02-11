#!/bin/bash
#----- Isaac Cortex -----#
Cortex_Cayde_PeckStateMachine () {
	$IsaacPythonPath ~/Documents/CustomIsaacRepo/Cayde/Cayde.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_state_machine.py
}

Cortex_Cayde_PeckGame () {
	$IsaacPythonPath ~/Documents/CustomIsaacRepo/Cayde/Cayde.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_game.py
}

Cortex_Franka_PeckStateMachine () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_state_machine.py
}

Cortex_Franka_PeckGame () {
	$IsaacPythonPath ~/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/omni.isaac.cortex/franka_examples_main.py --behavior=/home/cayde/Documents/CustomIsaacRepo/Cayde/behaviors/peck_game.py
}
