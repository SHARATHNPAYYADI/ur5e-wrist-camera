#!/usr/bin/env python3
"""Project the cube's 3D position into every saved image and draw it.

Inputs (written by capture_poses) in the data directory:
  poses.jsonl       per image: base_link -> camera_optical_frame (translation + quaternion)
  camera_info.yaml  intrinsics
  scene.yaml        cube centre and size in base_link
Outputs:
  <data>/projected/<image>.png   image with the projected point drawn
  <data>/projected/report.json   numbers for the README

The check, per image:
  p_cam = inverse(T_base_cam) @ p_base          move the cube point into the camera frame
  u = fx * X / Z + cx ,  v = fy * Y / Z + cy    pinhole model (cv2.projectPoints, same maths)
The point should land on the red cube.

  ros2 run ur5e_wrist_cam_capture project_check --data capture_output
"""

import argparse
import itertools
import json
import os
import sys

import cv2
import numpy as np
import yaml


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a quaternion (x, y, z, w)."""
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def project(points_base: np.ndarray, translation, quaternion, K: np.ndarray, D: np.ndarray):
    """Project Nx3 points given in base_link. Returns (pixels Nx2, depth N in metres).

    translation/quaternion describe the camera pose in base_link (camera -> base).
    """
    R = quat_to_matrix(*quaternion)
    t = np.asarray(translation, dtype=float)
    R_bc = R.T                      # base -> camera rotation
    t_bc = -R.T @ t                 # base -> camera translation
    depth = (R_bc @ points_base.T).T[:, 2] + t_bc[2]
    rvec, _ = cv2.Rodrigues(R_bc)
    pixels, _ = cv2.projectPoints(points_base.reshape(-1, 1, 3), rvec, t_bc, K, D)
    return pixels.reshape(-1, 2), depth


def red_blob(img_bgr: np.ndarray):
    """Mask of the largest strongly red region (the cube) and its centroid."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 150, 50), (8, 255, 255)) | cv2.inRange(hsv, (172, 150, 50), (180, 255, 255))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None, None
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest, centroids[largest]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", default="capture_output", help="directory written by capture_poses")
    parser.add_argument("--out", default=None, help="output directory (default: <data>/projected)")
    parser.add_argument("--wireframe", action="store_true", help="also draw the projected cube edges")
    args = parser.parse_args()
    out_dir = args.out or os.path.join(args.data, "projected")
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(args.data, "camera_info.yaml"), encoding="utf-8") as f:
        info = yaml.safe_load(f)
    with open(os.path.join(args.data, "scene.yaml"), encoding="utf-8") as f:
        scene = yaml.safe_load(f)
    K = np.array(info["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    D = np.array(info["distortion_coefficients"]["data"], dtype=float)
    cube = np.array(scene["cube_center"], dtype=float)
    half = scene["cube_size"] / 2.0
    corners = np.array([cube + half * np.array(s) for s in itertools.product((-1, 1), repeat=3)])
    edges = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]

    report = []
    with open(os.path.join(args.data, "poses.jsonl"), encoding="utf-8") as f:
        records = [json.loads(line) for line in f if line.strip()]

    print(f"cube centre {cube.tolist()} m in {scene['frame']}, K fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"{'image':<26}{'u':>8}{'v':>8}{'depth m':>9}  {'in image':<9}{'on cube':<9}{'dist to red centroid px':>24}")
    for rec in records:
        img_path = os.path.join(args.data, rec["image"])
        img = cv2.imread(img_path)
        if img is None:
            print(f"cannot read {img_path}")
            continue
        h, w = img.shape[:2]
        (uv,), (depth,) = project(cube.reshape(1, 3), rec["translation"], rec["quaternion_xyzw"], K, D)
        u, v = float(uv[0]), float(uv[1])
        in_front = depth > 0
        in_image = in_front and 0 <= u < w and 0 <= v < h

        mask, centroid = red_blob(img)
        on_cube = bool(in_image and mask is not None and mask[int(round(v)), int(round(u))])
        dist = float(np.hypot(u - centroid[0], v - centroid[1])) if (centroid is not None and in_front) else None

        vis = img.copy()
        if args.wireframe and in_front:
            pix, dep = project(corners, rec["translation"], rec["quaternion_xyzw"], K, D)
            for i, j in edges:
                if dep[i] > 0 and dep[j] > 0:
                    cv2.line(vis, tuple(int(round(c)) for c in pix[i]), tuple(int(round(c)) for c in pix[j]), (255, 200, 0), 1, cv2.LINE_AA)
        if in_front:
            centre = (int(round(u)), int(round(v)))
            cv2.circle(vis, centre, 9, (255, 255, 0), 2, cv2.LINE_AA)        # cyan ring
            cv2.drawMarker(vis, centre, (0, 255, 0), cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)  # green cross
        cv2.putText(vis, f"{rec['name']}  u={u:.0f} v={v:.0f}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        out_path = os.path.join(out_dir, os.path.basename(rec["image"]))
        cv2.imwrite(out_path, vis)

        print(f"{rec['image']:<26}{u:>8.1f}{v:>8.1f}{depth:>9.3f}  {str(in_image):<9}{str(on_cube):<9}{'' if dist is None else f'{dist:>24.1f}'}")
        report.append(
            {"image": rec["image"], "u": u, "v": v, "depth_m": float(depth), "in_image": bool(in_image),
             "on_cube": on_cube, "dist_to_red_centroid_px": dist}
        )

    with open(os.path.join(out_dir, "report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    passed = sum(r["on_cube"] for r in report)
    print(f"\n{passed}/{len(report)} projected points land on the cube. Annotated images: {out_dir}")
    if passed != len(report):
        sys.exit(1)


if __name__ == "__main__":
    main()