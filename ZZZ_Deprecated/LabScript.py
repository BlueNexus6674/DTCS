# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
 
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core_nodes.scripts.utils import set_target_prims
 
from pxr import Sdf
 
import numpy as np
 
import omni.ext
import omni.graph.core as og
 
global Rotation
global Direction
global RotationEnable
global SimulatedRobotBool
global OGController
global CGP
 
Rotation = 0
Direction = 1
RotationEnable = False
SimulatedRobotBool = True
 
# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html
 
class LabScript(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return
 
    def setup_scene(self):
 	UserName = os.getlogin()
 	
        world = self.get_world()
        world.scene.add_default_ground_plane()
 
        #UR5_with_RG2 Inclusion
        ur5_with_rg2_path="/home/" + UserName + "/Documents/DTCS/ROS_Workspaces/ros_workspace/src/ur5withrg2/USD/ur5withrg2/ur5withrg2.usd"
 
        #Add UR_1
        add_reference_to_stage(usd_path=ur5_with_rg2_path, prim_path="/World/ur5_with_rg2_target")
        ur5_with_rg2_robot_target = world.scene.add(Robot(prim_path="/World/ur5_with_rg2_target", name="ur5_with_rg2_target", position=np.array([-0.5, 0, 0])))
         
        #Add UR_2
        add_reference_to_stage(usd_path=ur5_with_rg2_path, prim_path="/World/ur5_with_rg2_real")
        ur5_with_rg2_robot_real = world.scene.add(Robot(prim_path="/World/ur5_with_rg2_real", name="ur5_with_rg2_real", position=np.array([0.5, 0, 0])))
 
 
        #OmniGraph
        global OGController
        global OGContext
        global CGP
        OGController = og.GraphController()
        OGController.create_graph({"graph_path": "/controller_graph", "evaluator_name": "execution"})
        ControllerGraphPath = "/controller_graph/"
        CGP = ControllerGraphPath
 
 
        #Creating Vars
         
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
         
        #Setting Nodes -------------------------------------
        #JointPublisher_1 UR5_Target to Joint_Command
        NodePath = CGP + "ROS1PublishJointState_1"
        og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_command")
        set_target_prims(primPath=NodePath, targetPrimPaths=["/World/ur5_with_rg2_target"])
 
        #JointPublisher_2 UR5_Target to Joint_States (Simulation Bridge)
        NodePath = CGP + "ROS1PublishJointState_2"
        og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")
        set_target_prims(primPath=NodePath, targetPrimPaths=["/World/ur5_with_rg2_target"])
 
        #SubscribeJointState_1 UR5_Real joint_states
        NodePath = CGP + "ROS1SubscribeJointState_1"
        og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:topicName"), "/joint_states")
 
        #ArticulationController_1 - UR5_real
        NodePath = CGP + "ArticulationController_1"
        og.Controller.set(og.ObjectLookup.attribute(NodePath + "/inputs:robotPath"), "/World/ur5_with_rg2_real")
        set_target_prims(primPath=NodePath, targetPrimPaths=["/World/ur5_with_rg2_real"])
 
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
        return
 
    async def setup_post_load(self):
        global Rotation
        self._world = self.get_world()
        self._ur5_with_rg2_target = self._world.scene.get_object("ur5_with_rg2_target")
        self._ur5_with_rg2_target_articulation_controller = self._ur5_with_rg2_target.get_articulation_controller()
         
        #self._ur5_with_rg2_real = self._world.scene.get_object("ur5_with_rg2_real")
        #self._ur5_with_rg2_real_articulation_controller = self._ur5_with_rg2_real.get_articulation_controller()
 
        #Home Robots
        Rotation = 0
        self._ur5_with_rg2_target_articulation_controller.apply_action(ArticulationAction(joint_positions=[Rotation, -1.5707, 0, -1.5707, 0, 0, 0, 0]))
        #self._ur5_with_rg2_real_articulation_controller.apply_action(ArticulationAction(joint_positions=[Rotation, -1.5707, 0, -1.5707, 0, 0, 0, 0]))
         
        #Add Callbacks
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return
     
    def send_robot_actions(self, step_size):
        global Rotation
        global Direction
        global RotationEnable
        if (RotationEnable == True):
            if (Direction == 1):
                if(Rotation > 6):
                    Direction = -1
 
            if (Direction == -1):
                if (Rotation < -6):
                    Direction = 1
             
            Rotation = Rotation + (Direction*0.01)
         
        self._ur5_with_rg2_target_articulation_controller.apply_action(ArticulationAction(joint_positions=[Rotation, -1.5707, 0, -1.5707, 0, 0, 0, 0]))
        #self._ur5_with_rg2_real_articulation_controller.apply_action(ArticulationAction(joint_positions=[Rotation, -1.5707, 0, -1.5707, 0, 0, 0, 0]))
 
    async def _on_rotate_event_async(self):
        global RotationEnable
        RotationEnable = not(RotationEnable)
        return
 
    async def _on_simulated_robot_event_async(self):
        global OGController
        global CGP
        global SimulatedRobotBool
 
        SimulatedRobotBool = not(SimulatedRobotBool)
         
        if (SimulatedRobotBool == True):
            OGController.connect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishJointState_2.inputs:execIn")
        else:
            OGController.disconnect(CGP + "OnTick.outputs:tick", CGP + "ROS1PublishJointState_2.inputs:execIn")
        return
 
    async def _on_home_event_async(self):
        global Rotation
        Rotation = 0
        return
 
 
 
    async def setup_pre_reset(self):
        return
 
    async def setup_post_reset(self):
        return
 
    def world_cleanup(self):
        return
