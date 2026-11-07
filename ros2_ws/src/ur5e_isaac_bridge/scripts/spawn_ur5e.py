# Spawn the official UR description into Isaac Sim and publish joint state
# over the ROS 2 bridge. Run with Isaac Sim's Python, for example:
#   ~/isaacsim/python.sh spawn_ur5e.py --xacro /path/ur_isaac.urdf.xacro

import argparse
import os
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--xacro", required=True, help="Path to ur_isaac.urdf.xacro")
parser.add_argument("--ur-type", default="ur5e")
parser.add_argument("--joint-commands-topic", default="/isaac_joint_commands")
parser.add_argument("--joint-states-topic", default="/isaac_joint_states")
parser.add_argument("--headless", action="store_true")
args, _unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# Wrist camera settings (Problem 1).
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FRAME_ID = "camera_optical_frame"
# The sim steps at 60 Hz. Skipping 3 frames publishes every 4th, i.e. 15 Hz.
CAM_FRAME_SKIP = 3

# Target cube (Problem 2). Position is the cube centre in metres. The robot base
# sits at the world origin and world -> base_link is identity, so these numbers
# are also the cube position in base_link. Keep them for the projection check.
CUBE_POSITION = (0.5, 0.0, 0.05)
CUBE_SIZE = 0.1  # edge length in metres


def expand_urdf() -> str:
    """Expand the official xacro with absolute mesh paths for the importer."""
    out_dir = tempfile.mkdtemp(prefix="ur5e_isaac_urdf_")
    urdf_path = os.path.join(out_dir, "ur5e.urdf")
    command = [
        "xacro",
        args.xacro,
        f"ur_type:={args.ur_type}",
        "name:=ur",
        "force_abs_paths:=true",
        f"joint_commands_topic:={args.joint_commands_topic}",
        f"joint_states_topic:={args.joint_states_topic}",
    ]
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise RuntimeError(
            "xacro failed. Source the ROS 2 workspace before launching Isaac Sim."
        )
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(result.stdout)
    return urdf_path


def articulation_path(hint: str) -> str:
    stage = stage_utils.get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(hint)
    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return hint
    if prim and prim.IsValid():
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(child.GetPath())
    return hint


def find_prim(root: str, name: str) -> str:
    """Return the path of the first prim called `name` under `root`."""
    stage = stage_utils.get_current_stage(backend="usd")
    root_prim = stage.GetPrimAtPath(root)
    if root_prim and root_prim.IsValid():
        for prim in Usd.PrimRange(root_prim):
            if prim.GetName() == name:
                return str(prim.GetPath())
    raise RuntimeError(
        f"Prim '{name}' not found under {root}. Check that the xacro defines it "
        "and that the URDF importer kept the link."
    )


def create_wrist_camera(robot_root: str) -> str:
    """Create a USD camera under camera_optical_frame so it moves with the arm.

    A USD camera looks down -Z with +Y up. A ROS optical frame looks down +Z
    with +Y down. Rotating the USD camera 180 deg about X inside the optical
    frame aligns the two.
    """
    stage = stage_utils.get_current_stage(backend="usd")
    optical_path = find_prim(robot_root, CAM_FRAME_ID)
    cam_path = optical_path + "/wrist_cam"
    camera = UsdGeom.Camera.Define(stage, cam_path)
    xformable = UsdGeom.Xformable(camera)
    xformable.ClearXformOpOrder()
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
    # Intrinsics: fx = focal * width / horizontal_aperture (about 554 px here).
    camera.GetFocalLengthAttr().Set(18.15)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955 * CAM_HEIGHT / CAM_WIDTH)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    print(f"Wrist camera: {cam_path}")
    return cam_path


def create_camera_graph(cam_path: str) -> None:
    """Publish /wrist_cam/image_raw and /wrist_cam/camera_info at 15 Hz."""
    og.Controller.edit(
        {"graph_path": "/CameraGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("InfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "RgbHelper.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "InfoHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "RgbHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "InfoHelper.inputs:renderProductPath"),
                ("Context.outputs:context", "RgbHelper.inputs:context"),
                ("Context.outputs:context", "InfoHelper.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(cam_path)]),
                ("RenderProduct.inputs:width", CAM_WIDTH),
                ("RenderProduct.inputs:height", CAM_HEIGHT),
                ("RgbHelper.inputs:type", "rgb"),
                ("RgbHelper.inputs:topicName", "wrist_cam/image_raw"),
                ("RgbHelper.inputs:frameId", CAM_FRAME_ID),
                ("RgbHelper.inputs:frameSkipCount", CAM_FRAME_SKIP),
                ("InfoHelper.inputs:topicName", "wrist_cam/camera_info"),
                ("InfoHelper.inputs:frameId", CAM_FRAME_ID),
                ("InfoHelper.inputs:frameSkipCount", CAM_FRAME_SKIP),
            ],
        },
    )
    # Same optional domain-ID handling as the joint graph. Images use
    # simulation time because useSystemTime defaults to False.
    try:
        og.Controller.set(
            og.Controller.attribute("/CameraGraph/Context.inputs:useDomainIDEnvVar"), True
        )
    except Exception as exc:
        print(f"Left /CameraGraph/Context.inputs:useDomainIDEnvVar at its default: {exc}")


def create_target_cube() -> None:
    """Add a fixed red cube at CUBE_POSITION for the camera to look at."""
    stage = stage_utils.get_current_stage(backend="usd")
    cube = UsdGeom.Cube.Define(stage, "/target_cube")
    cube.GetSizeAttr().Set(CUBE_SIZE)
    UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(*CUBE_POSITION))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])
    # Collider only, no rigid body: the cube stays where it is if the arm bumps it.
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print(f"Target cube at {CUBE_POSITION} m (base_link), edge {CUBE_SIZE} m")


def main() -> None:
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=np.array([1.6, 1.6, 1.2]),
        target=np.array([0.0, 0.0, 0.4]),
    )

    assets_root = get_assets_root_path()
    if assets_root:
        stage_utils.add_reference_to_stage(
            assets_root + "/Isaac/Environments/Simple_Room/simple_room.usd",
            "/background",
        )

    urdf_path = expand_urdf()
    usd_dir = tempfile.mkdtemp(prefix="ur5e_isaac_usd_")
    importer = URDFImporter(
        URDFImporterConfig(
            urdf_path=urdf_path,
            usd_path=usd_dir,
            fix_base=True,
            merge_fixed_joints=False,
            collision_from_visuals=False,
            joint_drive_type="force",
            joint_target_type="position",
            override_joint_stiffness=100000.0,
            override_joint_damping=10000.0,
            run_asset_transformer=False,
        )
    )
    usd_path = importer.import_urdf()
    robot_root = "/ur5e"
    stage_utils.add_reference_to_stage(usd_path, robot_root)
    simulation_app.update()
    robot_path = articulation_path(robot_root)
    print(f"UR5e articulation: {robot_path}")

    commands_topic = args.joint_commands_topic.lstrip("/")
    states_topic = args.joint_states_topic.lstrip("/")
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
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
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", robot_path),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(robot_path)]),
                ("PublishJointState.inputs:topicName", states_topic),
                ("SubscribeJointState.inputs:topicName", commands_topic),
            ],
        },
    )
    # Optional on this Isaac Sim build. The joint graph above matches the
    # shipped ROS 2 bridge sample and must still come up if these inputs differ.
    for attr_path, value in (
        ("/ActionGraph/Context.inputs:useDomainIDEnvVar", True),
        ("/ActionGraph/PublishClock.inputs:topicName", "clock"),
    ):
        try:
            og.Controller.set(og.Controller.attribute(attr_path), value)
        except Exception as exc:
            print(f"Left {attr_path} at its default: {exc}")

    # Wrist camera: USD camera under camera_optical_frame, then its ROS graph.
    cam_path = create_wrist_camera(robot_root)
    simulation_app.update()
    create_camera_graph(cam_path)
    create_target_cube()

    simulation_app.update()
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()