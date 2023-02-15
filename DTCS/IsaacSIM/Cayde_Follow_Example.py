# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

#----- General Imports -----
import numpy as np
from omni.isaac.core.objects import VisualSphere
from omni.isaac.cortex.cortex_world import CortexWorld
from omni.isaac.cortex.df import DfNetwork, DfState, DfStateMachineDecider
from omni.isaac.cortex.dfb import DfContext
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage

#----- IsaacSIM_Python Imports -----
from IsaacSIM_Python.cayde_robot import add_cayde_to_stage

class FollowState(DfState):
    """ The context object is available as self.context. We have access to everything in the context
    object, which in this case is everything in the robot object (the command API and the follow
    sphere).
    """

    @property
    def robot(self):
        return self.context.robot

    @property
    def follow_sphere(self):
        return self.context.robot.follow_sphere

    def enter(self):
        #self.robot.gripper.close()
        self.follow_sphere.set_local_pose(*self.robot.arm.get_fk_pq().as_tuple())

    def step(self):
        target_position, _ = self.follow_sphere.get_local_pose()
        self.robot.arm.send_end_effector(target_position=target_position)
        return self  # Always transition back to this state.


def main():
	#Dirs
	Base_Dir = "/home/cayde/Documents/DTCS/DTCS/"
	Behaviour_Config_Dir = Base_Dir + "/IsaacSIM/RobotBehaviours/"
	Robot_Config_Dir = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_RobotDescription/"
	USD_Path_ur5withrg2 = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_URDF/USD/ur5withrg2/ur5withrg2.usd"
	USD_Path_table = Base_Dir + "RobotConfiguration/IsaacSIM/IsaacSIM_URDF/USD/assem_table/assem_table.usd"
	
	world = CortexWorld()

	#Add Sim Objects
	sim_prim = XFormPrim(prim_path="/World/Sim")
	sim_prim.set_world_pose(position=np.array([1.5, 0.0, 0.0]))
	
	add_reference_to_stage(usd_path=USD_Path_table, prim_path="/World/Sim/Table")
	sim_table_prim = XFormPrim(prim_path="/World/Sim/Table")
	sim_table_prim.set_local_pose(np.array([-0.6, 0.0, 0.88]))
	
	#Add Robot To Stage
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
	robot = world.add_robot(sim_MCR)

	robot.follow_sphere = world.scene.add(
	VisualSphere(
	    name="follow_sphere", prim_path="/World/Sim/RobotCell/FollowSphere", radius=0.02, color=np.array([0.7, 0.0, 0.7])
	)
	)
	world.scene.add_default_ground_plane()

	world.add_decider_network(DfNetwork(DfStateMachineDecider(FollowState()), context=DfContext(robot)))

	world.run(simulation_app)
	simulation_app.close()


if __name__ == "__main__":
    main()
