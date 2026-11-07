# UR5e Isaac Sim demo

ROS 2 Jazzy workspace that launches a UR5e into NVIDIA Isaac Sim and brings up MoveIt against the simulator's ROS 2 bridge.

Isaac Sim is expected at `~/isaacsim` (this setup was run with Isaac Sim 6.0). The ROS side uses the official Universal Robots description and MoveIt config. Joint commands and joint states cross the bridge on `/isaac_joint_commands` and `/isaac_joint_states`, with simulation time on `/clock`.

> **Wrist camera exercise:** this repository also adds a wrist camera and an image-capture check. See [Wrist camera exercise](#wrist-camera-exercise). Results are in [`results/`](results/).

## Packages

| Package | Role |
|---|---|
| `ur_description` | Official URDF, meshes, and kinematics from [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), branch `jazzy`. |
| `ur_moveit_config` | Official MoveIt config. `Universal_Robots_ROS2_MoveIt_Config` is no longer a separate repository; the package lives in [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), branch `jazzy`. This workspace keeps a sparse checkout of `ur_moveit_config` only. |
| `topic_based_ros2_control` | ros2_control hardware interface from [PickNikRobotics/topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control). It publishes joint commands and subscribes to joint states so MoveIt can drive the simulated arm. |
| `ur5e_isaac_bridge` | Launch package for this demo. It imports the UR5e URDF into Isaac Sim, creates the ROS 2 bridge action graph, and starts `ros2_control` and MoveIt after `/clock` and `/isaac_joint_states` are publishing. |
| `ur5e_wrist_cam_capture` | Added for the wrist camera exercise. `capture_poses` moves the arm to 5 joint goals through MoveIt and saves an image, joint angles and camera pose at each. `project_check` projects the cube into the saved images. |

Upstream pins are listed in `ros2_ws/ur5e_isaac.repos`.

## Fetch upstream sources

The three upstream repositories listed in `ros2_ws/ur5e_isaac.repos` are not stored in this repo. Fetch them once after cloning:

```bash
sudo apt install python3-vcstool python3-opencv python3-yaml
cd ros2_ws
vcs import src < ur5e_isaac.repos
git -C src/Universal_Robots_ROS2_Driver sparse-checkout set ur_moveit_config
```

Only `ur_moveit_config` is needed from `Universal_Robots_ROS2_Driver`, so the sparse checkout keeps the build small. `python3-opencv` and `python3-yaml` are used by the new `ur5e_wrist_cam_capture` package.

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`topic_based_ros2_control` configures tests that need `ros-jazzy-ros-testing`. The `-DBUILD_TESTING=OFF` argument skips those test-only dependencies.

## Launch

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py
```

The launch file starts Isaac Sim with `~/isaacsim/python.sh`, imports the UR5e, and presses Play inside the standalone script. MoveIt and RViz start after the bridge topics appear. In RViz, plan with the `ur_manipulator` group.

Useful arguments:

| Argument | Default | Meaning |
|---|---|---|
| `ur_type` | `ur5e` | Robot model passed to the Universal Robots description. |
| `launch_isaac` | `true` | Start Isaac Sim from this launch. Set `false` to attach MoveIt to a simulator that is already running. |
| `launch_rviz` | `true` | Start RViz with the official MoveIt display config. |
| `use_sim_time` | `true` | Use `/clock` from Isaac Sim. |
| `headless` | `false` | Run the Isaac Sim app without a window. |
| `isaac_python` | `~/isaacsim/python.sh` | Isaac Sim Python launcher. |
| `joint_commands_topic` | `/isaac_joint_commands` | Commands from ros2_control into Isaac Sim. |
| `joint_states_topic` | `/isaac_joint_states` | Joint states from Isaac Sim into ros2_control. |

The first Isaac Sim startup can take longer than the launch's 180 second wait. If the log reports that the bridge timed out and MoveIt was not launched, leave the simulator running and start MoveIt against it:

```bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py launch_isaac:=false
```

The ROS terminal and Isaac Sim need the same `ROS_DOMAIN_ID`. The bridge graph uses the domain from that environment variable.

# Wrist camera exercise

Adds an RGB camera on the UR5e wrist, publishes it over ROS 2, captures 5 images at 5 MoveIt poses, and checks them by projecting the cube's known 3D position into each image.

## What was added

| File | Change |
|---|---|
| `ros2_ws/src/ur5e_isaac_bridge/urdf/ur_isaac.urdf.xacro` | `camera_link` and `camera_optical_frame` on `tool0`. |
| `ros2_ws/src/ur5e_isaac_bridge/scripts/spawn_ur5e.py` | USD camera under the optical frame, a second action graph that publishes it, and the target cube. |
| `ros2_ws/src/ur5e_wrist_cam_capture/` | New package with `capture_poses` and `project_check`. |
| `tools/find_poses.py` | Offline script that found the 5 joint poses (needs numpy and scipy). Not needed to run the demo. |
| `results/` | Output of one full run: images, `poses.jsonl`, `camera_info.yaml`, `scene.yaml`, `projected/`. |

## Run

Build and launch as above. In RViz, switch off **MotionPlanning → Planning Request → Query Goal State** so the goal marker does not sit in front of the camera view.

**1. Check the camera (Problem 1).** In a second terminal:

```bash
ros2 topic hz /wrist_cam/image_raw --use-sim-time        # 14 to 15 Hz, see Known issues
ros2 topic echo /wrist_cam/camera_info --once            # 640x480, frame_id camera_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_optical_frame # changes when the arm moves
```

In RViz add a **Camera** display on `/wrist_cam/image_raw` (fixed frame `base_link`). With the RobotModel overlay on, the robot outline lines up with the image.

**2. Capture 5 images (Problem 2).**

```bash
ros2 run ur5e_wrist_cam_capture capture_poses --ros-args -p output_dir:=capture_output
```

For each pose in `config/poses.yaml` it plans and executes through MoveIt's `MoveGroup` action, waits until the arm has stopped, and saves one image. It writes `capture_output/images/`, `poses.jsonl`, `camera_info.yaml` and `scene.yaml`.

**3. Check the images.**

```bash
ros2 run ur5e_wrist_cam_capture project_check --data capture_output --wireframe
```

This draws the projected cube centre (cyan ring, green cross) and the cube edges on each image in `capture_output/projected/`, prints a table, and exits non-zero if a point misses the cube.

## Results

The files in [`results/`](results/) come from these exact commands. The projected cube centre in each image:

| Image | Pose | u, v (px) |
|---|---|---|
| `00_top_down` | camera above the cube | 320, 238 |
| `01_from_robot` | from the robot side | 318, 241 |
| `02_from_left` | from the left | 321, 242 |
| `03_from_right` | from the right | 318, 240 |
| `04_far_side` | from the far side, looking back at the robot | 319, 241 |

In all five images the point lands on the cube, and the projected cube edges follow the real cube edges. The point is the cube's 3D centre, so in oblique views it can sit on the edge between two faces.

Each line of `poses.jsonl` holds the image path, simulation timestamp, the six joint angles in radians, and `base_link → camera_optical_frame` as translation and quaternion `(x, y, z, w)`.

## Design choices

**Frames.** `camera_link` follows REP 103 (x forward, y left, z up) and is fixed to `tool0` at `xyz = (0, -0.06, 0.04)` with `rpy = (1.5708, -1.5708, 0)`. The −90° pitch points camera x along the tool0 z axis. The 90° roll turns the image about the viewing axis. `camera_optical_frame` is the usual optical frame (z forward, x right, y down) with `rpy = (-1.5708, 0, -1.5708)` from `camera_link`. TF comes from `robot_state_publisher`, as for the rest of the robot, so nothing in Isaac Sim publishes TF.

**USD camera orientation.** A USD camera looks down −Z with +Y up. The ROS optical frame looks down +Z with +Y down. The USD camera is created as a child of the `camera_optical_frame` prim and rotated 180° about X, so the two agree. The roll lives only in the xacro, so TF and the rendered image cannot disagree.

**Intrinsics.** Focal length 18.15 mm and horizontal aperture 20.955 mm give fx = fy ≈ 554.3 px at 640 px width, with the principal point at the image centre and no distortion. `camera_info.yaml` stores what Isaac Sim publishes.

**Rate and time.** The simulation steps at 60 Hz. `frameSkipCount = 3` on both camera helpers publishes every 4th frame, which targets 15 Hz, and keeps each image paired with its `camera_info`. `useSystemTime` stays off, so stamps are simulation time. `frame_id` is `camera_optical_frame`.

**Target cube.** A fixed red cube with a 0.1 m edge, centre `(0.5, 0, 0.05)`. `world → base_link` is the identity, so the position is the same in both frames.

**Choice of poses.** The poses were found offline with the UR5e kinematics from `ur_description` and the camera mount above (`tools/find_poses.py`). It searches for joint angles that put the camera at five positions around the cube, looking at its centre with a level horizon, and keeps those with the whole cube inside the image. They are joint goals, as the brief allows.

**Capture logic.** MoveIt reports success when the controller finishes, which can be before the arm is really still. So `capture_poses` also waits until all joints are within 0.01 rad of the goal and slower than 0.03 rad/s for 1 s of simulation time. It then accepts only an image stamped at least 0.2 s after that moment, and reads `base_link → camera_optical_frame` from TF at the image's own timestamp. If the arm starts moving again before the image arrives, it retries (up to 3 times).

**The check.** For each image the saved pose is inverted to move the cube centre into the camera frame, then `u = fx·X/Z + cx`, `v = fy·Y/Z + cy` (via `cv2.projectPoints`). A point passes if it falls on the largest red region of the image. The cube position is the only thing shared between the ROS side and the renderer, so the check compares the saved pose and intrinsics with what the renderer actually drew.

## Known issues

- **Joints never fully stop in Isaac Sim.** At rest they keep moving at about 0.01 rad/s. The default `settle_velocity` is therefore 0.03 rad/s. A lower value makes `capture_poses` time out. Override with `-p settle_velocity:=...`.
- **The cube is not in MoveIt's planning scene.** MoveIt does not know about it and would not avoid it. The 5 poses keep the arm away from the cube, but this was only checked with a rough model offline.
- **Views 3 to 5 look alike.** The cube is symmetric and all red, so different viewing directions give similar pictures. The numbers in `poses.jsonl` differ.
- **No lens distortion.** The camera is an ideal pinhole, so this check cannot reveal distortion errors.
- **The image rate is slightly under 15 Hz.** `ros2 topic hz --use-sim-time` reports about 14.3 Hz. Most gaps between images are 0.067 s, but some are 0.133 s, so roughly one image in 15 is skipped. I did not find the cause. Images and `camera_info` are still stamped with simulation time, and `capture_poses` waits for a fresh image, so the captured data is not affected.
- **Depth images (bonus) are not done.**
- **First start of Isaac Sim can exceed the 180 s bridge wait.** See the note under Launch.
- **Tested on one machine** (Ubuntu 24.04, ROS 2 Jazzy, Isaac Sim 6.0). There is no automated test.

## AI tools used

Claude (Anthropic) helped with the design and drafted the xacro changes, the `spawn_ur5e.py` changes, the `ur5e_wrist_cam_capture` package and this README. It also computed the five joint poses with an offline kinematics search (`tools/find_poses.py`); I entered them in RViz and checked them. I ran the simulation, capture and projection check on my machine, checked the results, and went through the code so that I can explain it.