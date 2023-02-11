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
from IsaacSIM_Python.cayde_robot import add_cayde_to_stage
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
	Base_Dir = "/home/cayde/Documents/CaydeRepo/Cayde/"
	Behaviour_Config_Dir = Base_Dir + "behaviours/"
	Robot_Config_Dir = Base_Dir + "RobotDescription/"
	USD_Path_ur5withrg2 = "/home/cayde/Documents/CaydeRepo/IsaacSimURDF/USD/ur5withrg2/ur5withrg2.usd"
	#USD_Path_ur5withrg2 = Robot_Config_Dir + "/ur5withrg2/ur5withrg2.usd"
	

	#RMP Args
	ArgsParser = argparse.ArgumentParser()
	ArgsParser.add_argument("--behavior", type=str, default=Behaviour_Config_Dir + "block_stacking_behavior.py", help="Cayde Help Me")
	Cayde_Args = ArgsParser.parse_args()

	#Create world
	world = CortexWorld()
	world.scene.add_default_ground_plane()
	
	#Add Robot
	robot = world.add_robot(add_cayde_to_stage(
		name = "ur5withrg2",
		prim_path = "/World/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "Cayde_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	))

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


