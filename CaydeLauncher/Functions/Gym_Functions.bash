#!/bin/bash
#----- Isaac Gym -----#
Gym_Cartpole () {
	$IsaacPythonPath ~/OmniIsaacGymEnvs/omniisaacgymenvs/scripts/rlgames_train.py task=Cartpole
}

Gym_Quadcopter () {
	$IsaacPythonPath ~/OmniIsaacGymEnvs/omniisaacgymenvs/scripts/rlgames_train.py task=Quadcopter
}
