#!/usr/bin/env python3
"""Move the UR5e to saved joint poses with MoveIt and save one wrist-camera image at each.

For every pose it writes:
  images/<index>_<name>.png   the camera image
  poses.jsonl                 one line: joint angles + base_link -> camera_optical_frame
and once:
  camera_info.yaml            intrinsics as published on /wrist_cam/camera_info
  scene.yaml                  cube position and size (used by the projection check)

Run (with the launch file already running):
  ros2 run ur5e_wrist_cam_capture capture_poses --ros-args -p output_dir:=capture_output
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener

NS = 1_000_000_000


def stamp_ns(stamp) -> int:
    return stamp.sec * NS + stamp.nanosec


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference between two angles in radians."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image (rgb8, rgba8, bgr8, bgra8) to an OpenCV BGR array."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = raw[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img.copy()


class CaptureNode(Node):
    def __init__(self) -> None:
        # Everything runs on Isaac Sim's /clock.
        super().__init__(
            "capture_poses",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        share = get_package_share_directory("ur5e_wrist_cam_capture")
        self.declare_parameter("poses_file", os.path.join(share, "config", "poses.yaml"))
        self.declare_parameter("output_dir", "capture_output")
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("image_topic", "/wrist_cam/image_raw")
        self.declare_parameter("camera_info_topic", "/wrist_cam/camera_info")
        self.declare_parameter("velocity_scale", 0.3)
        self.declare_parameter("settle_time_s", 1.0)       # sim seconds with no motion
        self.declare_parameter("settle_velocity", 0.005)   # rad/s counted as "stopped"
        self.declare_parameter("goal_tolerance", 0.01)     # rad, final joint error
        self.declare_parameter("settle_timeout_s", 90.0)   # wall-clock seconds to wait for it
        self.declare_parameter("cube_position", [0.5, 0.0, 0.05])
        self.declare_parameter("cube_size", 0.1)

        self.latest_js = None
        self.latest_js_ns = 0
        self.last_moving_ns = 0
        self.prev_js = None
        self.latest_image = None
        self.latest_info = None
        self.last_speed = 0.0

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._on_info, qos_profile_sensor_data
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = ActionClient(self, MoveGroup, "move_action")

    # ---------------------------------------------------------------- callbacks
    def _on_joint_state(self, msg: JointState) -> None:
        now_ns = stamp_ns(msg.header.stamp)
        positions = dict(zip(msg.name, msg.position))
        if len(msg.velocity) == len(msg.name):
            speed = max((abs(v) for v in msg.velocity), default=0.0)
        elif self.prev_js is not None and now_ns > self.prev_js[0]:
            dt = (now_ns - self.prev_js[0]) / NS
            speed = max(
                (abs(angle_diff(p, self.prev_js[1].get(n, p))) / dt for n, p in positions.items()),
                default=0.0,
            )
        else:
            speed = 0.0
        self.last_speed = speed
        if speed > self.get_parameter("settle_velocity").value or self.last_moving_ns == 0:
            self.last_moving_ns = now_ns
        self.prev_js = (now_ns, positions)
        self.latest_js = msg
        self.latest_js_ns = now_ns

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def _on_info(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    # ------------------------------------------------------------------ helpers
    def spin_until(self, condition, timeout_s: float, what: str) -> None:
        """Spin the node until condition() is true. The timeout is wall-clock time."""
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not condition():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Timed out waiting for {what}")
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def joint_positions(self) -> dict:
        return dict(zip(self.latest_js.name, self.latest_js.position))

    # ------------------------------------------------------------------ motion
    def move_to(self, joint_names, q) -> None:
        """Plan and execute a joint goal through MoveIt's MoveGroup action."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.get_parameter("group_name").value
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.get_parameter("velocity_scale").value
        req.max_acceleration_scaling_factor = self.get_parameter("velocity_scale").value
        constraints = Constraints()
        for name, value in zip(joint_names, q):
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=float(value),
                    tolerance_above=1e-3,
                    tolerance_below=1e-3,
                    weight=1.0,
                )
            )
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False  # plan AND execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self.move_group.send_goal_async(goal)
        self.spin_until(send_future.done, 20.0, "MoveGroup to accept the goal")
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("MoveGroup rejected the goal")
        result_future = handle.get_result_async()
        self.spin_until(result_future.done, 120.0, "MoveGroup to finish planning and execution")
        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveGroup failed with error code {code}")

    def wait_until_stopped(self, joint_names, q) -> int:
        """Wait until the arm reached the goal and stayed still. Returns the settle time (ns).

        MoveIt reports success when the controller finishes, which can be before the
        arm has really stopped, so we check joint states ourselves.
        """
        settle_ns = int(self.get_parameter("settle_time_s").value * NS)
        tol = self.get_parameter("goal_tolerance").value

        def stopped() -> bool:
            if self.latest_js is None:
                return False
            if self.latest_js_ns - self.last_moving_ns < settle_ns:
                return False
            pos = self.joint_positions()
            return all(abs(angle_diff(pos[n], v)) < tol for n, v in zip(joint_names, q))

        try:
            self.spin_until(stopped, self.get_parameter("settle_timeout_s").value, "the arm to stop")
        except TimeoutError:
            pos = self.joint_positions()
            errors = {n: abs(angle_diff(pos[n], v)) for n, v in zip(joint_names, q)}
            worst = max(errors, key=errors.get)
            still_s = (self.latest_js_ns - self.last_moving_ns) / NS
            raise TimeoutError(
                f"Arm did not settle. Worst joint error: {worst} {errors[worst]:.4f} rad "
                f"(allowed {tol}). Still for {still_s:.2f} s of sim time (need "
                f"{settle_ns / NS:.1f}). Latest max joint speed: {self.last_speed:.4f} rad/s "
                f"(allowed {self.get_parameter('settle_velocity').value})."
            )
        return self.latest_js_ns

    # ----------------------------------------------------------------- capture
    def capture_one(self, joint_names, q):
        """Return (image msg, joint dict, TF msg) for a settled arm, retrying if it moves."""
        base = self.get_parameter("base_frame").value
        cam = self.get_parameter("camera_frame").value
        settle_ns = int(self.get_parameter("settle_time_s").value * NS)
        for _attempt in range(3):
            t_settled = self.wait_until_stopped(joint_names, q)
            moving_before = self.last_moving_ns
            # Only accept an image rendered after the arm had stopped.
            self.spin_until(
                lambda: self.latest_image is not None
                and stamp_ns(self.latest_image.header.stamp) > t_settled + int(0.2 * NS),
                15.0,
                "a fresh camera image",
            )
            image = self.latest_image
            t_img = Time.from_msg(image.header.stamp)
            # The TF at the image time comes from robot_state_publisher, which may lag a little.
            self.spin_until(lambda: self.tf_buffer.can_transform(base, cam, t_img), 10.0, f"TF {base} -> {cam}")
            tf = self.tf_buffer.lookup_transform(base, cam, t_img)
            joints = self.joint_positions()
            # Make sure the arm did not start moving again while we were capturing.
            if self.last_moving_ns == moving_before and (
                stamp_ns(image.header.stamp) - self.last_moving_ns >= settle_ns
            ):
                return image, joints, tf
            self.get_logger().warn("Arm moved during capture, retrying")
        raise RuntimeError("Could not capture a settled image after 3 attempts")

    def write_camera_info(self, path: str) -> None:
        info = self.latest_info
        data = {
            "image_width": int(info.width),
            "image_height": int(info.height),
            "camera_name": self.get_parameter("camera_frame").value,
            "frame_id": info.header.frame_id,
            "distortion_model": info.distortion_model,
            "distortion_coefficients": {"rows": 1, "cols": len(info.d), "data": [float(x) for x in info.d]},
            "camera_matrix": {"rows": 3, "cols": 3, "data": [float(x) for x in info.k]},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [float(x) for x in info.r]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": [float(x) for x in info.p]},
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def run(self) -> None:
        with open(self.get_parameter("poses_file").value, encoding="utf-8") as f:
            config = yaml.safe_load(f)
        joint_names = config["joint_names"]
        out_dir = self.get_parameter("output_dir").value
        os.makedirs(os.path.join(out_dir, "images"), exist_ok=True)

        self.get_logger().info("Waiting for /clock, joint states, camera and MoveGroup...")
        self.spin_until(lambda: self.sim_now_ns() > 0, 60.0, "/clock (is Isaac Sim playing?)")
        self.spin_until(lambda: self.latest_js is not None, 30.0, "/joint_states")
        self.spin_until(lambda: self.latest_image is not None, 30.0, self.get_parameter("image_topic").value)
        self.spin_until(lambda: self.latest_info is not None, 30.0, self.get_parameter("camera_info_topic").value)
        if not self.move_group.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("MoveGroup action server /move_action not available")

        self.write_camera_info(os.path.join(out_dir, "camera_info.yaml"))
        with open(os.path.join(out_dir, "scene.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {
                    "frame": self.get_parameter("base_frame").value,
                    "cube_center": [float(x) for x in self.get_parameter("cube_position").value],
                    "cube_size": float(self.get_parameter("cube_size").value),
                },
                f,
                sort_keys=False,
            )

        with open(os.path.join(out_dir, "poses.jsonl"), "w", encoding="utf-8") as jsonl:
            for index, pose in enumerate(config["poses"]):
                name, q = pose["name"], pose["q"]
                self.get_logger().info(f"[{index + 1}/{len(config['poses'])}] moving to '{name}'")
                self.move_to(joint_names, q)
                image, joints, tf = self.capture_one(joint_names, q)

                rel_path = os.path.join("images", f"{index:02d}_{name}.png")
                cv2.imwrite(os.path.join(out_dir, rel_path), image_to_bgr(image))
                t = tf.transform.translation
                r = tf.transform.rotation
                record = {
                    "index": index,
                    "name": name,
                    "image": rel_path,
                    "stamp": {"sec": image.header.stamp.sec, "nanosec": image.header.stamp.nanosec},
                    "joint_names": joint_names,
                    "joint_positions": [float(joints[n]) for n in joint_names],
                    "parent_frame": tf.header.frame_id,
                    "child_frame": tf.child_frame_id,
                    "translation": [t.x, t.y, t.z],
                    "quaternion_xyzw": [r.x, r.y, r.z, r.w],
                }
                jsonl.write(json.dumps(record) + "\n")
                jsonl.flush()
                self.get_logger().info(f"    saved {rel_path}")
        self.get_logger().info(f"Done. Output in {os.path.abspath(out_dir)}")


def main() -> None:
    rclpy.init()
    node = CaptureNode()
    try:
        node.run()
    except (TimeoutError, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()