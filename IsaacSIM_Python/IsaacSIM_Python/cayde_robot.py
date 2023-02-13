# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

""" Proposal for ControlledArticulation interface for adding command APIs to Articulation objects.

Collaborative systems functional pipeline:
1. perception: sensor data --> entities and transforms
2. world model: entities and transforms --> USD
3. logical state monitoring: USD --> logical state
4. decisions: USD and logical state --> commands
5. command API: commands --> articulation actions
6. control: articulation --> actions to movement

These tools implement the command API. The command API is an API for commanding different subsets of
the articulation's joints. For instance, the MotionCommander (see motion_commander.py) gives a
command API for specifying target poses for the end-effector along with approach parameters and
C-space resolution parameters. Likewise, the GripperCommander (see below) gives a command API for
moving the gripper to a desired width at a specific speed. It can also close the gripper until it
feels a desired force.

These command APIs are available through the robot object added to the world and accessible from
the decision layer (inside state machines and decider networks).

See:
- commander.py for the base class interface.
- motion_commander.py and GripperCommander (below) for examples
- standalone_examples/cortex/task/{nullspace,peck_decider_netwrok,cortex_control_example}.py for
  example usage inside behaviors.
"""


from abc import abstractmethod
from collections import OrderedDict
import numpy as np
from typing import Optional, Sequence

from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.cortex.commander import Commander
from omni.isaac.cortex.cortex_world import Behavior, CommandableArticulation, CortexWorld
from omni.isaac.cortex.motion_commander import MotionCommander
from omni.isaac.manipulators.grippers.surface_gripper import SurfaceGripper
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.motion_generation.lula import RmpFlow
import omni.isaac.motion_generation.interface_config_loader as icl
import omni.physics.tensors


#---------------------------------------------------------------------------------------------------------------------------------------#

def add_cayde_to_stage(	#Adds a UR5/10 (Cayde) to the stage at the specified prim_path, then wraps it as a CortexRobot, then a MotionCommandedRobot.
	name: str,
	prim_path: str,
	usd_path: str,
	urdf_path: str,
	lula_robot_description_path: str,
	rmpflow_config_path: str,
	end_effector_name: str,
	position: Optional[Sequence[float]] = None,
	orientation: Optional[Sequence[float]] = None,
):

	if position is None:
		position = np.zeros(3)

	RMPFlow = RmpFlow(
		robot_description_path = lula_robot_description_path,
		urdf_path = urdf_path,
		rmpflow_config_path = rmpflow_config_path,
		end_effector_frame_name = end_effector_name,
		maximum_substep_size = .0034
	)
	

	add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
	CR = CortexRobot(name, prim_path)
	MCR = MotionCommandedRobot(name, prim_path, RMPFlow)
	
	#CR = CortexRobot(name, prim_path, position)
	#MCR = MotionCommandedRobot(name, prim_path, RMPFlow, position)
	
	return MCR

#---------------------------------------------------------------------------------------------------------------------------------------#

class CortexRobot(CommandableArticulation):
    """ A robot is an Articulation with a collection of commanders commanding the collection of
    joints.

    Note: In the future, a robot will be multiple articulations (such as a mobile base, an arm, and
    a separate gripper. But for now we restrict it to a single Articulation which represents a
    single PhysX articulation.

    Note that position and orientation are both relative to the prim the robot sits on.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ):
        if position is None:
            position = np.zeros(3)
        super().__init__(name=name, prim_path=prim_path, translation=position, orientation=orientation)

        self.commanders_step_dt = CortexWorld.instance().get_physics_dt()
        self.commanders_reset_needed = False
        self.commanders = OrderedDict()

    def add_commander(self, name, commander, make_attr=True):
        if make_attr:
            # Makes attribute self.<name> containing the commander.
            setattr(self, name, commander)
        self.commanders[name] = commander

    def set_commanders_step_dt(self, commanders_step_dt):
        """ Set the internal dt member which is passed to each commander during their step(dt)
        calls.
        """
        self.commanders_step_dt = commanders_step_dt

    def flag_commanders_for_reset(self):
        self.commanders_reset_needed = True

    def step_commanders(self):
        if CortexWorld.instance().is_playing():
            self._reset_commanders_if_needed()
            for _, commander in self.commanders.items():
                commander.step(self.commanders_step_dt)

    def reset_commanders(self):
        for _, commander in self.commanders.items():
            commander.post_reset()

    def _reset_commanders_if_needed(self):
        """ Reset only if flagged.
        """
        if self.commanders_reset_needed:
            self.reset_commanders()
            self.commanders_reset_needed = False

#---------------------------------------------------------------------------------------------------------------------------------------#

class MotionCommandedRobot(CortexRobot):
    class Settings:
        def __init__(self, active_commander=True, smoothed_rmpflow=True, smoothed_commands=True):
            self.active_commander = active_commander
            self.smoothed_rmpflow = smoothed_rmpflow
            self.smoothed_commands = smoothed_commands

    def __init__(
        self,
        name: str,
        prim_path: str,
        motion_policy_config: RmpFlow,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        settings: Optional[Settings] = Settings(),
    ):
        super().__init__(name=name, prim_path=prim_path, position=position, orientation=orientation)
        self.settings = settings

        #if settings.smoothed_rmpflow:
        #    self.motion_policy = RmpFlowSmoothed(**motion_policy_config)
        #else:
        #    self.motion_policy = RmpFlow(**motion_policy_config)
        self.motion_policy = motion_policy_config
        if self.settings.active_commander:
            articulation_motion_policy = ArticulationMotionPolicy(
                robot_articulation=self, motion_policy=self.motion_policy, default_physics_dt=self.commanders_step_dt
            )
            target_prim = VisualCuboid("/World/motion_commander_target", size=0.01, color=np.array([0.15, 0.15, 0.15]))
            self.arm_commander = MotionCommander(
                self, articulation_motion_policy, target_prim, use_smoothed_commands=self.settings.smoothed_commands
            )
        else:
            self.arm_commander = DirectSubsetCommander(ArticulationSubset(self, self.motion_policy.get_active_joints()))
        self.add_commander("arm", self.arm_commander)

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None):
        super().initialize(physics_sim_view)
        self.disable_gravity()
        self.set_joints_default_state(positions=self.default_config)

    @property
    def default_config(self):
        q = np.zeros(self.num_dof)
        indices = self.arm.articulation_subset.joint_indices
        q[indices] = self.motion_policy.get_default_cspace_position_target()
        return q

    @property
    def registered_obstacles(self):
        return self.arm_commander.obstacles

    def register_obstacle(self, obs):
        self.arm_commander.add_obstacle(obs)
        
#---------------------------------------------------------------------------------------------------------------------------------------#
        
class DirectSubsetCommander(Commander):
    class Command:
        def __init__(self, q, qd=None):
            self.q = q
            self.qd = qd

    def step(self, dt):
        if self.command is not None:
            self.articulation_subset.apply_action(self.command.q, self.command.qd)
