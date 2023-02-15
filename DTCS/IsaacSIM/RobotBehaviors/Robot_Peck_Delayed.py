# Copyright (c) 2022, NVIDIA  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

""" This script gives an example of a behavior programmed entirely as a decider network (no state
machines). The behavior will monitor the blocks for movement, and whenever a block moves it will
reach down and peck it. It will always switch to the most recently moved block, aborting its
previous peck behavior if a new block is moved.

The top level Dispatch decider has three actions: peck, lift, and go_home. See the Dispatch
decider's decide() method for the specific implementation of choice of action. Simply put, if
there's an active block, then peck at it. If it doesn't have an active block, and it's currently too
close to the block, then lift a bit away from it. Otherwise, if none of that is true, just go home.

Crticial to the simplicity of this decision description is the monitoring of the relevant logical
information. The context object sets up a collection of monitors which monitor whether there's an
active block (one that's been moved, but hasn't yet been pecked), and whether the end-effector is
close to a block.

Note that the active block is automatically detected as the latest block that's moved. Likewise, the
context monitors also simply monitor to see whether that block is touched by the end-effector. When
the monitor observes that the active block has been touched, it deactivates the block. This
separation between observability and choice of action to make an observable change is a core
principle in decider network design for inducing reactivitiy.
"""

import numpy as np
import time

from omni.isaac.cortex.df import DfLogicalState, DfNetwork, DfDecider, DfDecision, DfAction
from omni.isaac.cortex.dfb import DfLift, DfCloseGripper, make_go_home
import omni.isaac.cortex.math_util as math_util
from omni.isaac.cortex.motion_commander import MotionCommand, ApproachParams, PosePq

import omni.ext
import omni.graph.core as og

class PeckContext(DfLogicalState):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        self.monitors = [
            PeckContext.monitor_block_movement,
            #PeckContext.monitor_control_block_movement,
            PeckContext.monitor_active_target_p,
            PeckContext.monitor_active_block,
            PeckContext.monitor_eff_block_proximity,
            PeckContext.monitor_diagnostics,
        ]

    def reset(self):
        self.blocks = []
        self.controlblocks = []
        self.simblocks = []
        for _, block in self.robot.registered_obstacles.items():
            self.blocks.append(block)
            if ((_.rsplit("_")[0]) == "Control"):
                self.controlblocks.append(block)
                
            if ((_.rsplit("_")[0]) == "Sim"):
                self.simblocks.append(block)
                
        self.control_block_positions = self.get_latest_control_block_positions()
        self.sim_block_positions = self.get_latest_sim_block_positions()
        
        self.active_block = None
        self.active_target_p = None
        self.is_eff_close_to_inactive_block = None

        self.timer = None
        self.timerwait = 5 #5 1
        self.time_at_last_diagnostics_print = None
        
        self.peckheight = 0.05

    @property
    def has_active_block(self):
        return self.active_block is not None

    def clear_active_block(self):
        self.active_block = None
        self.active_target_p = None
        
    def get_latest_control_block_positions(self):
        block_positions = []
        for block in self.controlblocks:
            #block_p, _ = block.get_world_pose()
            block_p, _ = block.get_local_pose()
            block_positions.append(block_p)
        return block_positions
        
    def get_latest_sim_block_positions(self):
        block_positions = []
        for block in self.simblocks:
            #block_p, _ = block.get_world_pose()
            block_p, _ = block.get_local_pose()
            block_positions.append(block_p)
        return block_positions
                
    def monitor_block_movement(self):
        control_block_positions = self.get_latest_control_block_positions()
        sim_block_positions = self.get_latest_sim_block_positions()
        
        for i in range(len(sim_block_positions)):
            if np.linalg.norm(sim_block_positions[i] - self.sim_block_positions[i]) > 0.01:
                self.sim_block_positions[i] = sim_block_positions[i]
        
        for i in range(len(control_block_positions)):
            if np.linalg.norm(control_block_positions[i] - self.control_block_positions[i]) > 0.01:
                self.control_block_positions[i] = control_block_positions[i]
                self.active_block = self.simblocks[i]

    def monitor_active_target_p(self):
        if self.active_block is not None:
            #p, _ = self.active_block.get_world_pose()
            p, _ = self.active_block.get_local_pose()
            self.active_target_p = p + np.array([0.0, 0.0, self.peckheight])

    def monitor_active_block(self):
        if self.active_target_p is not None:
            eff_p = self.robot.arm.get_fk_p()
            dist = np.linalg.norm(eff_p - self.active_target_p)
            if np.linalg.norm(eff_p - self.active_target_p) < 0.01:
                now = time.time()
                
                if self.timer is None or ((now - self.timer) >= self.timerwait):
                    if self.timer is not None:
                        if self.active_block is not None:
                            print("")
                            print("--- Time(" + str(now)[6:13] + "): Peck Timer Finished")
                            print("--- Time(" + str(now)[6:13] + "): Removing Active Block:", self.active_block.name)
                            print("")
                            self.clear_active_block()
                            self.timer = None
                    else:
                        print("")
                        print("--- Time(" + str(now)[6:13] + "): Peck Timer Started")
                        print("")
                        self.timer = now
   
    def monitor_eff_block_proximity(self):
        self.is_eff_close_to_inactive_block = False

        eff_p = self.robot.arm.get_fk_p()
        for block in self.blocks:
            if block != self.active_block:
                #block_p, _ = block.get_world_pose()
                block_p, _ = block.get_local_pose()
                if np.linalg.norm(eff_p - block_p) < 0.07:
                    self.is_eff_close_to_inactive_block = True
                    return

    def monitor_diagnostics(self):
        now = time.time()
        if self.time_at_last_diagnostics_print is None or (now - self.time_at_last_diagnostics_print) >= 0.5:
            if self.active_block is not None:
                if self.timer is not None:
                	print("--- Time(" + str(now)[6:13] + "): Active Block: " + self.active_block.name + ", Timer Remaining: " + str(self.timerwait - (now - self.timer))[0:3])
                else:
                       print("--- Time(" + str(now)[6:13] + "): Active Block: " + self.active_block.name)
            self.time_at_last_diagnostics_print = now


class PeckAction(DfAction):
    def enter(self):
        self.block = self.context.active_block
        self.context.robot.arm.disable_obstacle(self.block)

    def step(self):
        target_p = self.context.active_target_p
        target_q = math_util.matrix_to_quat(
            math_util.make_rotation_matrix(az_dominant=np.array([0.0, 0.0, -1.0]), ax_suggestion=-target_p)
        )
        target = PosePq(target_p, target_q)
        approach_params = ApproachParams(direction=np.array([0.0, 0.0, -0.1]), std_dev=0.04)

        # Send the command each cycle so exponential smoothing will converge.
        self.context.robot.arm.send_end_effector(target, approach_params=approach_params)
        target_dist = np.linalg.norm(self.context.robot.arm.get_fk_p() - target.p)

    def exit(self):
        self.context.robot.arm.enable_obstacle(self.block)


class Dispatch(DfDecider):
    def enter(self):
        self.add_child("peck", PeckAction())
        self.add_child("lift", DfLift(height=0.1))
        self.add_child("go_home", make_go_home())

    def decide(self):
        if self.context.is_eff_close_to_inactive_block:
            return DfDecision("lift")

        if self.context.has_active_block:
            return DfDecision("peck")

        # If we aren't doing anything else, always just go home.
        return DfDecision("go_home")


def make_decider_network(robot):
    return DfNetwork(Dispatch(), context=PeckContext(robot))
