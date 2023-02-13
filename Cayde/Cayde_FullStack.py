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

#Cortex Sync
from omni.isaac.core.prims import XFormPrim
from omni.isaac.cortex.cortex_object import CortexObject
from omni.isaac.core.robots import Robot

#Omnigraph
import omni.ext
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

#ros
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros_bridge")

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

	#Cayde_FullStack Args
	ArgsParser = argparse.ArgumentParser()
	ArgsParser.add_argument("--bridge", type=bool, default=True, help="Cayde Help Me")
	ArgsParser.add_argument("--behavior", type=str, default=Behaviour_Config_Dir + "block_stacking_behavior.py", help="Cayde Help Me")
	Cayde_Args = ArgsParser.parse_args()

	#Create world
	world = CortexWorld()
	world.scene.add_default_ground_plane()
	
	#Control Prim
	control_prim = XFormPrim(prim_path="/World/Control")
	control_prim.set_world_pose(position=np.array([0.17, 1, 0.0]))
	
	#Add Real Robot
	real_prim = XFormPrim(prim_path="/World/Real")
	real_prim.set_world_pose(position=np.array([-1, 0.0, 0.0]))
	real_MCR = add_cayde_to_stage(
		name = "ur5withrg2_real",
		prim_path = "/World/Real/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "Cayde_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	)
	real_robot = world.scene.add(Robot(name="ur5withrg2_real", prim_path="/World/Real/ur5withrg2"))
	
	#Add Sim Robot
	sim_prim = XFormPrim(prim_path="/World/Sim")
	sim_prim.set_world_pose(position=np.array([1.5, 0.0, 0.0]))
	sim_MCR = add_cayde_to_stage(
		name = "ur5withrg2_sim",
		prim_path = "/World/Sim/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "Cayde_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	)
	sim_robot = world.add_robot(sim_MCR)

	#Add Cubes
	obs_specs = [
        CubeSpec("RedCube", [0.7, 0.0, 0.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
	]
	width = 0.0515
	for i, (x, spec) in enumerate(zip(np.linspace(-0.4, 0.4, len(obs_specs)), obs_specs)):
		control_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Control/{}".format(spec.name),
				name=spec.name + "_control",
				size=width,
				color=spec.color,
				translation=np.array([x, 0, width / 2]),
			)
		)
		
		real_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Real/{}".format(spec.name),
				name=spec.name,
				size=width,
				color=spec.color,
				translation=np.array([x, 0.7, width / 2]),
			)
		)


		sim_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Sim/{}".format(spec.name),
				name="{}_sim".format(spec.name),
				size=width,
				color=spec.color,
				translation=np.array([x, 0.7, width / 2]),
			)
		)
		sim_robot.register_obstacle(control_obj)
		sim_robot.register_obstacle(sim_obj)
		
	#Setup Omnigraph----------------------------------------------------------------
	global OGController
	global OGContext
	global CGP
	OGController = og.GraphController()
	OGController.create_graph({"graph_path": "/controller_graph", "evaluator_name": "execution"})
	ControllerGraphPath = "/controller_graph/"
	CGP = ControllerGraphPath



	#Creating Nodes -------------------------------------
	OGController.create_node(CGP + "OnTick", "omni.graph.action.OnTick")
	OGController.create_node(CGP + "IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime")
	OGController.create_node(CGP + "ArticulationController_1", "omni.isaac.core_nodes.IsaacArticulationController")
	#OGController.create_node(CGP + "ArticulationController_2", "omni.isaac.core_nodes.IsaacArticulationController")
	OGController.create_node(CGP + "ROS1PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock")
	OGController.create_node(CGP + "ROS1PublishTransformTree", "omni.isaac.ros_bridge.ROS1PublishTransformTree")
	OGController.create_node(CGP + "ROS1PublishJointState_1", "omni.isaac.ros_bridge.ROS1PublishJointState")
	OGController.create_node(CGP + "ROS1PublishJointState_2", "omni.isaac.ros_bridge.ROS1PublishJointState")
	OGController.create_node(CGP + "ROS1SubscribeJointState_1", "omni.isaac.ros_bridge.ROS1SubscribeJointState")
	OGController.create_node(CGP + "ROS1SubscribeJointState_2", "omni.isaac.ros_bridge.ROS1SubscribeJointState")
	
	
	
	#Create Var
	OGController.create_variable("/controller_graph", "Key1Pressed", "Bool")
	OGController.create_node(CGP + "write_variable_1", "omni.graph.core.WriteVariable")
	
	#Setting Nodes ------------------------------------
	#JointPublisher_1 UR5_Target to Joint_Command
	NodePath = CGP + "ROS1PublishJointState_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_command")
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/ur5withrg2"])
	
	#JointPublisher_2 UR5_Target to Joint_States (Simulation Bridge)
	if Cayde_Args.bridge:
		
		NodePath = CGP + "ROS1PublishJointState_2"
		og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")
		og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
		set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/ur5withrg2"])

	#Publish TransformTree
	NodePath = CGP + "ROS1PublishTransformTree"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/ur5withrg2"])

	#SubscribeJointState_1 UR5_Real joint_states
	NodePath = CGP + "ROS1SubscribeJointState_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")

	#ArticulationController_1 - UR5_real
	NodePath = CGP + "ArticulationController_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:robotPath"), "/World/Real/ur5withrg2")
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Real/ur5withrg2"])

	#ArticulationController_2
	#NodePath = CGP + "ArticulationController_2/"
	#og.Controller.set(og.ObjectLookup.attribute(NodePath + "inputs:robotPath"), "/World/ur5_with_rg2_target")

	#Connecting Nodes -------------------------------------
	#ExecIn's
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ArticulationController_1.inputs:execIn")
	#OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ArticulationController_2.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishClock.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishTransformTree.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishJointState_1.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishJointState_2.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1SubscribeJointState_1.inputs:execIn")
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1SubscribeJointState_2.inputs:execIn")

	#Publish TimeStamps
	OGController.connect(CGP + "IsaacReadSimulationTime.outputs:simulationTime", CGP + "ROS1PublishClock.inputs:timeStamp")
	OGController.connect(CGP + "IsaacReadSimulationTime.outputs:simulationTime", CGP + "ROS1PublishTransformTree.inputs:timeStamp")
	OGController.connect(CGP + "IsaacReadSimulationTime.outputs:simulationTime", CGP + "ROS1PublishJointState_1.inputs:timeStamp")
	OGController.connect(CGP + "IsaacReadSimulationTime.outputs:simulationTime", CGP + "ROS1PublishJointState_2.inputs:timeStamp")

	#Subscriber Joint State_1 to Articulation Controller1
	OGController.connect(CGP + "ROS1SubscribeJointState_1.outputs:effortCommand", CGP + "ArticulationController_1.inputs:effortCommand")
	OGController.connect(CGP + "ROS1SubscribeJointState_1.outputs:positionCommand", CGP + "ArticulationController_1.inputs:positionCommand")
	OGController.connect(CGP + "ROS1SubscribeJointState_1.outputs:velocityCommand", CGP + "ArticulationController_1.inputs:velocityCommand")
	OGController.connect(CGP + "ROS1SubscribeJointState_1.outputs:jointNames", CGP + "ArticulationController_1.inputs:jointNames")


	#Subscriber Joint State_2 to Articulation Controller2
	#OGController.connect(CGP + "ROS1SubscribeJointState_2.outputs:effortCommand", CGP + "ArticulationController_2.inputs:effortCommand")
	#OGController.connect(CGP + "ROS1SubscribeJointState_2.outputs:positionCommand", CGP + "ArticulationController_2.inputs:positionCommand")
	#OGController.connect(CGP + "ROS1SubscribeJointState_2.outputs:velocityCommand", CGP + "ArticulationController_2.inputs:velocityCommand")
	#OGController.connect(CGP + "ROS1SubscribeJointState_2.outputs:jointNames", CGP + "ArticulationController_2.inputs:jointNames")

	#Omnigraph END -----------------------------------------------------------------
	
	#Load Cortex Behaviour
	print("")
	print("loading behavior: {}".format(Cayde_Args.behavior))
	print("")

	decider_network = load_behavior_module(Cayde_Args.behavior).make_decider_network(sim_robot)
	world.add_decider_network(decider_network)
	#real_robot.add_decider_network(decider_network)

	world.run(simulation_app)
	simulation_app.close()

if __name__ == "__main__":
    main()


