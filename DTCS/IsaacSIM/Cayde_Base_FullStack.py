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


def main():
	#Dirs
	Base_Dir = "/home/cayde/Documents/DTCS/DTCS/"
	Behaviour_Config_Dir = Base_Dir + "/IsaacSIM/RobotBehaviours/"
	Robot_Config_Dir = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_RobotDescription/"
	USD_Path_ur5withrg2 = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_URDF/USD/ur5withrg2/ur5withrg2.usd"

	#Cayde_FullStack Args
	ArgsParser = argparse.ArgumentParser()
	ArgsParser.add_argument("--bridge", type=str, default="False", help="ROS Simulation Bridge Enabled (Y/n)")
	ArgsParser.add_argument("--behavior", type=str, default="", help="Which behavior to run")
	Cayde_Args = ArgsParser.parse_args()

	#Create world
	world = CortexWorld()
	world.scene.add_default_ground_plane()
	
	#Create OmniGraph
	OmniGraphSetup(Cayde_Args.behavior)
	
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
	set_target_prims(primPath=NodePath, targetPrimPaths=["/World/Sim/ur5withrg2"])
	
	#JointPublisher_2 UR5_Target to Joint_States (Simulation Bridge)
	if (SimulationBridge == "True"):
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


