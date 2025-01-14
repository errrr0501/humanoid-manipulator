# -*- coding: utf-8 -*-
# Copyright (c) 2020-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# Copyright (c) 2023 PickNik, LLC. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import re
import os

import carb
import numpy as np
from pathlib import Path

# In older versions of Isaac Sim (prior to 4.0), SimulationApp is imported from
# omni.isaac.kit rather than isaacsim.
try:
    from isaacsim import SimulationApp
except:
    from omni.isaac.kit import SimulationApp


ROBOT_STAGE_PATH = "/World/GR1_T2"
ROBOT_STAGE_PATH1 = "/World"
# UR_USD_PATH = "/ur5e_gripper/ur5e_gripper_close.usd"
ROBOT_USD_PATH = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Robots/FourierIntelligence/GR-1/GR1_T2.usd"
# UR_USD_PATH = "/ur5e_gripper/ur5e_home.usd"

BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/ur5e_gripper/testbed_table.usd"
BACKGROUND_USD_PATH = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Environments/Grid/default_environment.usd"

GRAPH_PATH = "/ActionGraph"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}


# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)


from omni.isaac.version import get_version

# Check the major version number of Isaac Sim to see if it's four digits, corresponding
# to Isaac Sim 2023.1.1 or older.  The version numbering scheme changed with the
# Isaac Sim 4.0 release in 2024.
is_legacy_isaacsim = len(get_version()[2]) == 4


import omni.graph.core as og  # noqa E402
from omni.isaac.core import SimulationContext  # noqa E402
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    prims,
    stage,
    viewports,
)
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf  # noqa E402
from omni.isaac.core import World


extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# physics_dt=1.0 / 60.0,
# rendering_dt=1.0 / 60.0
# my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)

def main():
    # # create a curobo motion gen instance:
    # num_targets = 0
    # # assuming obstacles are in objects_path:
    # my_world = World(stage_units_in_meters=1.0)
    # stage = my_world.stage

    # xform = stage.DefinePrim("/World", "Xform")
    # stage.SetDefaultPrim(xform)
    # # stage.DefinePrim("/curobo", "Xform")
    # # my_world.stage.SetDefaultPrim(my_world.stage.GetPrimAtPath("/World"))
    # stage = my_world.stage
    # # stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    stage.add_reference_to_stage(
        BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
    )


    # Put robot into world
    prims.create_prim(
        ROBOT_STAGE_PATH,
        "Xform",
        position=np.array([0.00, 0.0, 0.953]),
        orientation=([1.0, 0.0, 0.0, 0.0]),
        usd_path=ROBOT_USD_PATH,
    )

    # my_world.scene.add(ROBOT_STAGE_PATH)

    simulation_app.update()


    try:
        ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
        print("Using ROS_DOMAIN_ID: ", ros_domain_id)
    except ValueError:
        print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
        ros_domain_id = 0
    except KeyError:
        print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
        ros_domain_id = 0

    # Create an action graph with ROS component nodes
    try:
        og_keys_set_values = [
            ("Context.inputs:domain_id", ros_domain_id),
            # Set the /Franka target prim to Articulation Controller node
            ("ArticulationController.inputs:robotPath", ROBOT_STAGE_PATH+"/base"),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            # ("createViewport.inputs:name", REALSENSE_VIEWPORT_NAME),
            # ("createViewport.inputs:viewportId", 1),
            # ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            # ("cameraHelperRgb.inputs:topicName", "rgb"),
            # ("cameraHelperRgb.inputs:type", "rgb"),
            # ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            # ("cameraHelperInfo.inputs:topicName", "camera_info"),
            # ("cameraHelperInfo.inputs:type", "camera_info"),
            # ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            # ("cameraHelperDepth.inputs:topicName", "depth"),
            # ("cameraHelperDepth.inputs:type", "depth"),
        ]

        # In older versions of Isaac Sim, the articulation controller node contained a
        # "usePath" checkbox input that should be enabled.
        if is_legacy_isaacsim:
            og_keys_set_values.insert(1, ("ArticulationController.inputs:usePath", True))

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    (
                        "SubscribeJointState",
                        "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "ArticulationController",
                        "omni.isaac.core_nodes.IsaacArticulationController",
                    ),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    (
                        "getRenderProduct",
                        "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                    ),
                    ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                    (
                        "OnImpulseEvent.outputs:execOut",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    (
                        "SubscribeJointState.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    # ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    # ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    # ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "setCamera.inputs:renderProductPath",
                    # ),
                    # ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    # ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    # ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    # ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                    # ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                    # ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperRgb.inputs:renderProductPath",
                    # ),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperInfo.inputs:renderProductPath",
                    # ),
                    # (
                    #     "getRenderProduct.outputs:renderProductPath",
                    #     "cameraHelperDepth.inputs:renderProductPath",
                    # ),
                ],
                og.Controller.Keys.SET_VALUES: og_keys_set_values,
            },
        )
    except Exception as e:
        print(e)


    # Setting the /GR1_T2 target prim to Publish JointState node
    set_target_prims(
        primPath="/ActionGraph/PublishJointState", targetPrimPaths=[ROBOT_STAGE_PATH + "/base"]
    )
    # try:
    #     og.Controller.edit(
    #         {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    #         {
    #             og.Controller.Keys.CREATE_NODES: [
    #                 ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
    #                 ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
    #                 ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
    #                 ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
    #                 (
    #                     "SubscribeJointState",
    #                     "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
    #                 ),
    #                 (
    #                     "ArticulationController",
    #                     "omni.isaac.core_nodes.IsaacArticulationController",
    #                 ),
    #                 ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
    #             ],
    #             og.Controller.Keys.CONNECT: [
    #                 ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
    #                 ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
    #                 ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
    #                 (
    #                     "OnImpulseEvent.outputs:execOut",
    #                     "ArticulationController.inputs:execIn",
    #                 ),
    #                 ("Context.outputs:context", "PublishJointState.inputs:context"),
    #                 ("Context.outputs:context", "SubscribeJointState.inputs:context"),
    #                 ("Context.outputs:context", "PublishClock.inputs:context"),
    #                 (
    #                     "ReadSimTime.outputs:simulationTime",
    #                     "PublishJointState.inputs:timeStamp",
    #                 ),
    #                 ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
    #                 (
    #                     "SubscribeJointState.outputs:jointNames",
    #                     "ArticulationController.inputs:jointNames",
    #                 ),
    #                 (
    #                     "SubscribeJointState.outputs:positionCommand",
    #                     "ArticulationController.inputs:positionCommand",
    #                 ),
    #                 (
    #                     "SubscribeJointState.outputs:velocityCommand",
    #                     "ArticulationController.inputs:velocityCommand",
    #                 ),
    #                 (
    #                     "SubscribeJointState.outputs:effortCommand",
    #                     "ArticulationController.inputs:effortCommand",
    #                 ),
    #             ],
    #             og.Controller.Keys.SET_VALUES: [
    #                 ("Context.inputs:useDomainIDEnvVar", 1),
    #                 # Setting the /UR target prim to Articulation Controller node
    #                 # ("ArticulationController.inputs:usePath", True),
    #                 ("ArticulationController.inputs:robotPath", ROBOT_STAGE_PATH+ "/base"),
    #                 ("PublishJointState.inputs:topicName", "isaac_joint_states"),
    #                 ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
    #             ],
    #         },
    #     )

    # except Exception as e:
    #     print(e)

    # # Setting the /UR target prim to Publish JointState node
    # set_target_prims(
    #     primPath="/ActionGraph/PublishJointState", targetPrimPaths=[ROBOT_STAGE_PATH + "/base"]
    # )

    simulation_app.update()


    # need to initialize physics getting any articulation..etc
    simulation_context.initialize_physics()

    # simulation_context.play()
    # input()

    while simulation_app.is_running():

        # Run with a fixed step size
        simulation_context.step(render=True)

        # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
        og.Controller.set(
            og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
        )
    simulation_context.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
