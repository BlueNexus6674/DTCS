# Isaac SIM Standalone Header
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

#----- General Imports -----
import numpy as np
import argparse

#----- IsaacSIM_Python Imports -----
from IsaacSIM_Python.cayde_robot import add_cayde_to_stage

#----- Core Imports -----
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage

#----- Cortex Imports -----
from omni.isaac.cortex.cortex_world import CortexWorld, LogicalStateMonitor, Behavior
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.cortex.cortex_utils import load_behavior_module

#----- OmniGraph Imports -----
import omni.ext
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

#----- ROS Imports -----
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros_bridge")






class CubeSpec:
	def __init__(self, name, color):
		self.name = name
		self.color = np.array(color)






def main():
	#Dirs
	Base_Dir = "/home/cayde/Documents/DTCS/DTCS/"
	Behaviour_Config_Dir = Base_Dir + "/IsaacSIM/RobotBehaviours/"
	Robot_Config_Dir = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_RobotDescription/"
	USD_Path_ur5withrg2 = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_URDF/USD/ur5withrg2/ur5withrg2.usd"
	USD_Path_table = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_URDF/USD/assem_table/assem_table.usd"

	#Cayde_FullStack Args
	ArgsParser = argparse.ArgumentParser()
	ArgsParser.add_argument("--bridge", type=str, default="False", help="ROS Simulation Bridge Enabled (Y/n)")
	ArgsParser.add_argument("--behavior", type=str, default="", help="Which behavior to run")
	Cayde_Args = ArgsParser.parse_args()
	
	print ("--- Bridge: " + Cayde_Args.bridge + " ---")

	#Create world
	world = CortexWorld()
	world.scene.add_default_ground_plane()
	
	#Create OmniGraph
	OmniGraphSetup(Cayde_Args.bridge)
	
	#Control Prim
	control_prim = XFormPrim(prim_path="/World/Control")
	control_prim.set_world_pose(position=np.array([0.5, 1.5, 0.0]))
	
	#Add Real
	real_prim = XFormPrim(prim_path="/World/Real")
	real_prim.set_world_pose(position=np.array([-1, 0.0, 0.0]))
	
	add_reference_to_stage(usd_path=USD_Path_table, prim_path="/World/Real/Table")
	real_table_prim = XFormPrim(prim_path="/World/Real/Table")
	real_table_prim.set_local_pose(np.array([-0.6, 0.0, 0.88]))
	
	real_robot_prim = XFormPrim(prim_path="/World/Real/RobotCell")
	real_robot_prim.set_local_pose(np.array([0.24, 0.11, 0.92]))
	
	real_MCR = add_cayde_to_stage(
		name = "ur5withrg2_real",
		prim_path = "/World/Real/RobotCell/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "Cayde_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	)
	real_robot = world.scene.add(Robot(name="ur5withrg2_real", prim_path="/World/Real/RobotCell/ur5withrg2"))
	
	
	
	#Add Sim
	sim_prim = XFormPrim(prim_path="/World/Sim")
	sim_prim.set_world_pose(position=np.array([1.5, 0.0, 0.0]))
	
	add_reference_to_stage(usd_path=USD_Path_table, prim_path="/World/Sim/Table")
	sim_table_prim = XFormPrim(prim_path="/World/Sim/Table")
	sim_table_prim.set_local_pose(np.array([-0.6, 0.0, 0.88]))
	
	sim_robot_prim = XFormPrim(prim_path="/World/Sim/RobotCell")
	sim_robot_prim.set_local_pose(np.array([0.24, 0.11, 0.92]))
	
	sim_MCR = add_cayde_to_stage(
		name = "ur5withrg2_sim",
		prim_path = "/World/Sim/RobotCell/ur5withrg2",
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
	for i, (x, spec) in enumerate(zip(np.linspace(-0.6, 0.2, len(obs_specs)), obs_specs)):
		control_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Control/{}".format(spec.name),
				name="Control_{}".format(spec.name),
				size=width,
				color=spec.color,
				translation=np.array([x, 0, width / 2]),
			)
		)
		
		real_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Real/RobotCell/{}".format(spec.name),
				name=spec.name,
				size=width,
				color=spec.color,
				translation=np.array([x, 0.35, width / 2]),
			)
		)
		
		sim_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Sim/RobotCell/{}".format(spec.name),
				name="Sim_{}".format(spec.name),
				size=width,
				color=spec.color,
				translation=np.array([x, 0.35, width / 2]),
			)
		)
		sim_robot.register_obstacle(control_obj)
		sim_robot.register_obstacle(sim_obj)
	
	#Load Cortex Behaviour
	print("")
	print("loading behavior: {}".format(Cayde_Args.behavior))
	print("")
	
	if (Cayde_Args.behavior != ""):
		decider_network = load_behavior_module(Cayde_Args.behavior).make_decider_network(sim_robot)
		world.add_decider_network(decider_network)

	world.run(simulation_app)
	simulation_app.close()





def OmniGraphSetup (SimulationBridge):
	#----- Setup Omnigraph -----
	global OGController
	global OGContext
	global CGP
	OGController = og.GraphController()
	OGController.create_graph({"graph_path": "/controller_graph", "evaluator_name": "execution"})
	ControllerGraphPath = "/controller_graph/"
	CGP = ControllerGraphPath

	#----- Creating Nodes -----
	OGController.create_node(CGP + "OnTick", "omni.graph.action.OnTick")
	OGController.create_node(CGP + "IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime")
	OGController.create_node(CGP + "ArticulationController_1", "omni.isaac.core_nodes.IsaacArticulationController")
	OGController.create_node(CGP + "ROS1PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock")
	OGController.create_node(CGP + "ROS1PublishTransformTree", "omni.isaac.ros_bridge.ROS1PublishTransformTree")
	OGController.create_node(CGP + "ROS1PublishJointState_1", "omni.isaac.ros_bridge.ROS1PublishJointState")
	OGController.create_node(CGP + "ROS1PublishJointState_2", "omni.isaac.ros_bridge.ROS1PublishJointState")
	OGController.create_node(CGP + "ROS1SubscribeJointState_1", "omni.isaac.ros_bridge.ROS1SubscribeJointState")
	OGController.create_node(CGP + "ROS1SubscribeJointState_2", "omni.isaac.ros_bridge.ROS1SubscribeJointState")
	
	#----- Setting Nodes -----
	#JointPublisher_1 UR5_Target to Joint_Command
	NodePath = CGP + "ROS1PublishJointState_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_command")
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/RobotCell/ur5withrg2"])
	
	#JointPublisher_2 UR5_Target to Joint_States (Simulation Bridge)
	if (SimulationBridge == "True"):
		NodePath = CGP + "ROS1PublishJointState_2"
		og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")
		og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
		set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/RobotCell/ur5withrg2"])

	#Publish TransformTree
	NodePath = CGP + "ROS1PublishTransformTree"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:queueSize"), 1)
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/RobotCell/ur5withrg2"])

	#SubscribeJointState_1 UR5_Real joint_states
	NodePath = CGP + "ROS1SubscribeJointState_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")

	#ArticulationController_1 - UR5_real
	NodePath = CGP + "ArticulationController_1"
	og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:robotPath"), "/World/Real/RobotCell/ur5withrg2")
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Real/RobotCell/ur5withrg2"])

	#----- Connecting Nodes -----
	#ExecIn's
	OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ArticulationController_1.inputs:execIn")
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






if __name__ == "__main__":
    main()


