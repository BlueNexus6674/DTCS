# Isaac SIM Standalone Header
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

#----- General Imports -----
import numpy as np
import argparse
import os

#----- IsaacSIM_Python Imports -----
from IsaacSIM_Python.cayde_robot import add_cayde_to_stage

#----- Core Imports -----
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path

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
import rospy
from geometry_msgs.msg import Point

simulation_app.update()

global X
global Y
global XOffset
global YOffset
global AcceptanceRange

X = [0, 0, 0, 0]
Y = [0, 0, 0, 0]
XOffset = 1.12
YOffset = 0.5
XOld = [0, 0, 0, 0]
YOld = [0, 0, 0, 0]
AcceptanceRange = 0.01


class CubeSpec:
	def __init__(self, name, color):
		self.name = name
		self.color = np.array(color)

def main():
	global X
	global Y
	#Dirs
	UserName = os.getlogin()
	
	Base_Dir = "/home/" + UserName + "/Documents/DTCS/"
	
	Behaviour_Config_Dir = Base_Dir + "DTCS/IsaacSIM/RobotBehaviours/"
	Robot_Config_Dir = Base_Dir + "DTCS/RobotConfiguration/"
	
	USD_Path_ur5withrg2 = Base_Dir + "/ROS_Workspaces/ros_workspace/src/ur5withrg2/USD/ur5withrg2/ur5withrg2.usd"
	USD_Path_table = Base_Dir + "Assets/USD/assem_table/assem_table.usd"

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
	control_prim.set_world_pose(position=np.array([0.4, 1.5, 0.0]))
	
	#Add Real
	real_prim = XFormPrim(prim_path="/World/Real")
	real_prim.set_world_pose(position=np.array([-1.05, 0, 0.0]))
	
	add_reference_to_stage(usd_path=USD_Path_table, prim_path="/World/Real/Table")
	real_table_prim = XFormPrim(prim_path="/World/Real/Table")
	real_table_prim.set_local_pose(np.array([-0.6, 0.0, 0.88]))
	
	real_robot_prim = XFormPrim(prim_path="/World/Real/RobotCell")
	real_robot_prim.set_local_pose(np.array([0.24, 0.11, 0.92]))
	
	sim_robot_prim = XFormPrim(prim_path="/World/Real/RobotCell/Obstacle")
	#sim_robot_prim.set_local_pose(np.array([0.35, -0.11, 0]))
	sim_robot_prim.set_local_pose(np.array([0.35, -0.11, 0]))
	real_MCR = add_cayde_to_stage(
		name = "ur5withrg2_real",
		prim_path = "/World/Real/RobotCell/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "ur5withrg2_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	)
	real_robot = world.scene.add(Robot(name="ur5withrg2_real", prim_path="/World/Real/RobotCell/ur5withrg2"))
	
	
	
	#Add Sim
	sim_prim = XFormPrim(prim_path="/World/Sim")
	sim_prim.set_world_pose(position=np.array([1.45, 0, 0.0]))
	
	add_reference_to_stage(usd_path=USD_Path_table, prim_path="/World/Sim/Table")
	sim_table_prim = XFormPrim(prim_path="/World/Sim/Table")
	sim_table_prim.set_local_pose(np.array([-0.6, 0.0, 0.88]))
	
	sim_robot_prim = XFormPrim(prim_path="/World/Sim/RobotCell")
	sim_robot_prim.set_local_pose(np.array([0.24, 0.11, 0.92]))
	
	sim_robot_prim = XFormPrim(prim_path="/World/Sim/RobotCell/Obstacle")
	sim_robot_prim.set_local_pose(np.array([0.35, -0.11, 0]))
	
	sim_MCR = add_cayde_to_stage(
		name = "ur5withrg2_sim",
		prim_path = "/World/Sim/RobotCell/ur5withrg2",
		usd_path = USD_Path_ur5withrg2,
		urdf_path = Robot_Config_Dir + "ur5withrg2.urdf",
		lula_robot_description_path = Robot_Config_Dir + "ur5withrg2_lula_description.yaml",
		rmpflow_config_path = Robot_Config_Dir + "ur5withrg2_rmpflow_config.yaml",
		end_effector_name = "RG2tool0",
	)
	sim_robot = world.add_robot(sim_MCR)
	
	
	
	
	#Add Cubes
	obs_specs = [
        CubeSpec("RedCube", [1.0, 1.0, 1.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.8, 0.3, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
	]
	width = 0.06
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
		sim_robot.register_obstacle(control_obj)
		
		
		
	for i, (x, spec) in enumerate(zip(np.linspace(-0.85, -0.25, len(obs_specs)), obs_specs)):
		real_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Real/RobotCell/Obstacle/{}".format(spec.name),
				name=spec.name,
				size=width,
				color=spec.color,
				translation=np.array([x, 0.40, width / 2]),
			)
		)
		
		sim_obj = world.scene.add(
			DynamicCuboid(
				prim_path="/World/Sim/RobotCell/Obstacle/{}".format(spec.name),
				name="Sim_{}".format(spec.name),
				size=width,
				color=spec.color,
				translation=np.array([x, 0.40, width / 2]),
			)
		)
		sim_robot.register_obstacle(sim_obj)
	
	# ROSPY
	ROS_Sub = rospy.Subscriber("/MATLAB/Cube1", Point, Cube1Callback, queue_size=1)
	ROS_Sub = rospy.Subscriber("/MATLAB/Cube2", Point, Cube2Callback, queue_size=1)
	ROS_Sub = rospy.Subscriber("/MATLAB/Cube3", Point, Cube3Callback, queue_size=1)
	ROS_Sub = rospy.Subscriber("/MATLAB/Cube4", Point, Cube4Callback, queue_size=1)
	
	#Load Cortex Behaviour
	print("")
	print("loading behavior: {}".format(Cayde_Args.behavior))
	print("")
	
	if (Cayde_Args.behavior != ""):
		decider_network = load_behavior_module(Cayde_Args.behavior).make_decider_network(sim_robot)
		world.add_decider_network(decider_network)
	
	XFormCubeSim = [0, 0, 0, 0]
	XFormCubeSim[0] = XFormPrim("/World/Sim/RobotCell/Obstacle/RedCube")
	XFormCubeSim[1]  = XFormPrim("/World/Sim/RobotCell/Obstacle/BlueCube")
	XFormCubeSim[2]  = XFormPrim("/World/Sim/RobotCell/Obstacle/YellowCube")
	XFormCubeSim[3]  = XFormPrim("/World/Sim/RobotCell/Obstacle/GreenCube")
	
	XFormCubeReal = [0, 0, 0, 0]
	XFormCubeReal[0] = XFormPrim("/World/Real/RobotCell/Obstacle/RedCube")
	XFormCubeReal[1]  = XFormPrim("/World/Real/RobotCell/Obstacle/BlueCube")
	XFormCubeReal[2]  = XFormPrim("/World/Real/RobotCell/Obstacle/YellowCube")
	XFormCubeReal[3]  = XFormPrim("/World/Real/RobotCell/Obstacle/GreenCube")
	
	world.reset()
	while (simulation_app.is_running()):
		i = 0
		for i in range(4):
			if ((X[i] != XOld[i]) or (Y[i] != YOld[i])):
				XFormCubeReal[i].set_local_pose([X[i], Y[i], 0.03])
				XOld[i] = X[i]
				YOld[i] = Y[i]
			XFormCubeSim[i].set_local_pose(XFormCubeReal[i].get_local_pose()[0], XFormCubeReal[i].get_local_pose()[1])
		world.step(render=True, step_sim=True)
	#world.run(simulation_app)
	simulation_app.close()

def Cube1Callback(Data):
	global X
	global Y
	global XOffset
	global YOffset
	global AcceptanceRange
	
	TX = (Data.x/1000)-XOffset
	TY = (-Data.y/1000)+YOffset
	
	AXL = X[0] - AcceptanceRange
	AXH = X[0] + AcceptanceRange 
	AYL = Y[0] - AcceptanceRange
	AYH = Y[0] + AcceptanceRange
	
	if ((TX > AXH) or (TX < AXL) or (TY > AYH) or (TY < AYL)):
		X[0] = TX
		Y[0] = TY
	

def Cube2Callback(Data):
	global X
	global Y
	global XOffset
	global YOffset
	global AcceptanceRange
	
	TX = (Data.x/1000)-XOffset
	TY = (-Data.y/1000)+YOffset
	
	AXL = X[1] - AcceptanceRange
	AXH = X[1] + AcceptanceRange 
	AYL = Y[1] - AcceptanceRange
	AYH = Y[1] + AcceptanceRange
	
	if ((TX > AXH) or (TX < AXL) or (TY > AYH) or (TY < AYL)):
		X[1] = TX
		Y[1] = TY
	
def Cube3Callback(Data):
	global X
	global Y
	global XOffset
	global YOffset
	global AcceptanceRange
	
	TX = (Data.x/1000)-XOffset
	TY = (-Data.y/1000)+YOffset
	
	AXL = X[2] - AcceptanceRange
	AXH = X[2] + AcceptanceRange 
	AYL = Y[2] - AcceptanceRange
	AYH = Y[2] + AcceptanceRange
	
	if ((TX > AXH) or (TX < AXL) or (TY > AYH) or (TY < AYL)):
		X[2] = TX
		Y[2] = TY

def Cube4Callback(Data):
	global X
	global Y
	global XOffset
	global YOffset
	global AcceptanceRange
	
	TX = (Data.x/1000)-XOffset
	TY = (-Data.y/1000)+YOffset
	
	AXL = X[3] - AcceptanceRange
	AXH = X[3] + AcceptanceRange 
	AYL = Y[3] - AcceptanceRange
	AYH = Y[3] + AcceptanceRange
	
	if ((TX > AXH) or (TX < AXL) or (TY > AYH) or (TY < AYL)):
		X[3] = TX
		Y[3] = TY	

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
    rospy.init_node("IsaacNode", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    main()


