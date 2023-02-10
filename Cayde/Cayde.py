# Isaac SIM Standalone Header
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

#-------------- Cortex --------------#
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.cortex.cortex_world import CortexWorld, LogicalStateMonitor, Behavior
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.cortex.cortex_utils import load_behavior_module
#-------------- END Cortex END --------------#

#-------------- CustomIsaacRepo --------------#
from custom_isaac_repo.cayde_robot import add_cayde_to_stage
#-------------- END CustomIsaacRepo END --------------#

#-------------- General -------------#
from omni.isaac.core import World

import numpy as np
import argparse
#-------------- END General END --------------#

class CubeSpec:
	def __init__(self, name, color):
		self.name = name
		self.color = np.array(color)

def main():
	#Dirs
	Config_Dir = "/home/cayde/Documents/CustomIsaacRepo/Cayde/"
	UR10_Path = Config_Dir + "UR10.usd"

	#RMP Args
	ArgsParser = argparse.ArgumentParser()
	ArgsParser.add_argument("--behavior", type=str, default=Config_Dir + "behaviors/block_stacking_behavior.py", help="Cayde Help Me")
	Cayde_Args = ArgsParser.parse_args()

	#Create world
	world = CortexWorld()
	world.scene.add_default_ground_plane()
	
	#Add Robot
	robot = world.add_robot(add_cayde_to_stage("cayde", "/World/ur10", Config_Dir))

	#Add Cubes
	obs_specs = [CubeSpec("RedCube", [0.7, 0.0, 0.0]), CubeSpec("BlueCube", [0.0, 0.0, 0.7]), CubeSpec("YellowCube", [0.7, 0.7, 0.0]), CubeSpec("GreenCube", [0.0, 0.7, 0.0])]
	width = 0.0515
	for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
		obj = world.scene.add(DynamicCuboid(prim_path="/World/Obs/{}".format(spec.name), name=spec.name, size=width, color=spec.color, position=np.array([x, 0.4, width / 2])))
		robot.register_obstacle(obj)
	
	#Load Cortex Behaviour
	print("")
	print("loading behavior: {}".format(Cayde_Args.behavior))
	print("")

	decider_network = load_behavior_module(Cayde_Args.behavior).make_decider_network(robot)
	world.add_decider_network(decider_network)

	world.run(simulation_app)
	simulation_app.close()

if __name__ == "__main__":
    main()


