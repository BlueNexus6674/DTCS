# Isaac SIM Standalone Header
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ----- CUSTOM RMP Import ------#
from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
# ------ END -----#

# ----- Cortex -----
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.cortex.cortex_world import CortexWorld, LogicalStateMonitor, Behavior
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.cortex.cortex_utils import load_behavior_module
# ----- End Cortex -----

#-------------- CustomRobot --------------#
from omni.isaac.cortex.robot import CortexRobot
from custom_isaac_repo.cayde_robot import MotionCommandedRobot
#-------------- CustomRobot --------------#

# ----- General -----
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage

import numpy as np
import os
import argparse
# ----- End General -----

#Dirs
RMP_Config_Dir = "/home/cayde/Documents/CustomIsaacRepo/RMP/"
UR10_Path = "/home/cayde/Documents/CustomIsaacRepo/RMP/UR10.usd"
#UR10_Path = RMP_Config_Dir + "UR10.usd"

#RMP Args
RMPParser = argparse.ArgumentParser()
RMPParser.add_argument("--urdf_path",	type=str, default="Cayde.urdf")
RMPParser.add_argument("--rmpflow_config_path", type=str, default="Cayde_rmpflow_config.yaml")
RMPParser.add_argument("--end_effector_frame_name", type=str, default="tool0")
RMPParser.add_argument("--behavior", type=str, default=RMP_Config_Dir + "behaviors/block_stacking_behavior.py", help="Cayde Help Me")
RMP_Args = RMPParser.parse_args()

#Initialize an RmpFlow object
RMPFlow = RmpFlow(
    robot_description_path = os.path.join(RMP_Config_Dir, "Cayde_description.yaml"),
    urdf_path = os.path.join(RMP_Config_Dir, RMP_Args.urdf_path),
    rmpflow_config_path = os.path.join(RMP_Config_Dir, RMP_Args.rmpflow_config_path),
    end_effector_frame_name = RMP_Args.end_effector_frame_name, #This frame name must be present in the URDF
    maximum_substep_size = .0034
)

class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)

#Create world
world = CortexWorld()

#Add Robot
add_reference_to_stage(usd_path=UR10_Path, prim_path="/World/ur10")
#robot = world.scene.add(Robot(prim_path="/World/ur10/ur10", name="ur10"))
#robot = world.add_robot(Robot(prim_path="/World/ur10/ur10", name="ur10"))
CR = CortexRobot("ur10", "/World/ur10/ur10")
MCR = MotionCommandedRobot("ur10", "/World/ur10/ur10", RMPFlow)
#URMCR = 
CortexRobot = world.add_robot(MCR)



#Make a target to follow
target_cube = cuboid.VisualCuboid("/World/target", position = np.array([0, 0.5, 0.7]), orientation = np.array([0, -1, 0, 0]), color = np.array([1, 0, 0]), size = .1)



#RMPFlow Articulation Controller
#Uncomment this line to visualize the collision spheres in the robot_description YAML file
#rmpflow.visualize_collision_spheres()

#physics_dt = 1/60.
#articulation_rmpflow = ArticulationMotionPolicy(robot, RMPFlow, physics_dt)
#articulation_controller = robot.get_articulation_controller()


world.reset()
world.reset()

while simulation_app.is_running():
	world.step(render=True)
	if world.is_playing():
		if world.current_time_step_index == 0:
			world.reset()
		if world.current_time_step_index == 10:
			world.scene.add_default_ground_plane()
			obs_specs = [
				CubeSpec("RedCube", [0.7, 0.0, 0.0]),
				CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
				CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
				CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
			]
			width = 0.0515
			for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
				obj = world.scene.add(
					DynamicCuboid(
					prim_path="/World/Obs/{}".format(spec.name),
					name=spec.name,
					size=width,
					color=spec.color,
					position=np.array([x, 0.4, width / 2]),
				    )
				)
			#RMPFlow.add_obstacle(obj)
			CortexRobot.register_obstacle(obj)
			
			print("")
			print("loading behavior: {}".format(RMP_Args.behavior))
			print("")
			decider_network = load_behavior_module(RMP_Args.behavior).make_decider_network(CortexRobot)
			world.add_decider_network(decider_network)

	#Set rmpflow target to be the current position of the target cube.
	#RMPFlow.set_end_effector_target(target_position=target_cube.get_world_pose()[0], target_orientation=target_cube.get_world_pose()[1])
	#RMPFlow.set_end_effector_target(target_position=target_cube.get_world_pose()[0])
	  
	#actions = articulation_rmpflow.get_next_articulation_action()
	#articulation_controller.apply_action(actions)

simulation_app.close()


