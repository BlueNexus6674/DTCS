# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
 
import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import LabScript
 
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder
from omni.isaac.ui.ui_utils import cb_builder
 
class LabScriptExtension(BaseSampleExtension):
 
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="LabScript",
            title="LabScript",
            doc_link="https://google.com",
            overview="Leo Walker Dissertation",
            file_path=os.path.abspath(__file__),
            sample=LabScript(),
        )
        self.task_ui_elements = [1]
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        return
 
    def _on_rotate_button_event(self, value):
        asyncio.ensure_future(self.sample._on_rotate_event_async())
        return
 
    def _on_home_button_event(self):
        asyncio.ensure_future(self.sample._on_home_event_async())
        return
 
    def _on_simulated_robot_button_event(self, value):
        asyncio.ensure_future(self.sample._on_simulated_robot_event_async())
        return
 
    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                 
                #CheckButton Rotating
                dict = {
                    "label": "Rotation",
                    "type": "checkbox",
                    "default_val" : False,
                    "tooltip" : "Start Rotation",
                    "on_clicked_fn": self._on_rotate_button_event,
                }
                self.task_ui_elements["Rotation"] = cb_builder(**dict)
                #self.task_ui_elements["Rotation"].enabled = True
 
                #Button Home
                dict = {
                    "label" : "Home",
                    "type" : "button",
                    "text" : "Home Robot",
                    "tooltip" : "Home Robot",
                    "on_clicked_fn" : self._on_home_button_event,
                }
                self.task_ui_elements["Home"] = btn_builder(**dict)
                #self.task_ui_elements["Home"].enabled = True
 
                #CheckButton SimulatedRobot
                dict = {
                    "label": "SimulatedRobot",
                    "type": "checkbox",
                    "default_val" : True,
                    "tooltip" : "SimulatedRobot",
                    "on_clicked_fn": self._on_simulated_robot_button_event,
                }
                self.task_ui_elements["SimulatedRobot"] = cb_builder(**dict)
                #self.task_ui_elements["SimulatedRobot"].enabled = True

 
