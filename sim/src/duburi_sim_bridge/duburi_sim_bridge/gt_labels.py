#!/usr/bin/env python3

"""Project SAUVC prop AABBs into camera frames as YOLO labels.

Uses course YAML prop placements + vehicle ground-truth odometry and a simple
pinhole model from CameraInfo / configs. Good enough for OD bootstrap; not a
full Gazebo visibility check.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import yaml

# YOLO class order written to classes.txt
CLASSES = [
    'qual_gate',
    'final_gate',
    'orange_flare',
    'flare_red',
    'flare_yellow',
    'flare_blue',
    'drum_red',
    'drum_blue',
    'drum_red_pinger',
    'starting_zone',
]

MODEL_TO_CLASS = {
    'sauvc_qual_gate': 'qual_gate',
    'sauvc_final_gate': 'final_gate',
    'sauvc_orange_flare': 'orange_flare',
    'sauvc_flare_red': 'flare_red',
    'sauvc_flare_yellow': 'flare_yellow',
    'sauvc_flare_blue': 'flare_blue',
    'sauvc_drum_red': 'drum_red',
    'sauvc_drum_blue': 'drum_blue',
    'sauvc_drum_red_pinger': 'drum_red_pinger',
    'sauvc_starting_zone': 'starting_zone',
}

# Approximate axis-aligned half-extents in metres (x, y, z) in prop frame.
# Origin conventions match prop_library anchors (surface or floor).
PROP_HALF_EXTENTS = {
    'sauvc_qual_gate': (0.05, 0.75, 0.80),
    'sauvc_final_gate': (0.05, 0.75, 0.50),
    'sauvc_orange_flare': (0.08, 0.08, 0.80),
    'sauvc_flare_red': (0.04, 0.04, 0.40),
    'sauvc_flare_yellow': (0.04, 0.04, 0.40),
    'sauvc_flare_blue': (0.04, 0.04, 0.40),
    'sauvc_drum_red': (0.30, 0.30, 0.15),
    'sauvc_drum_blue': (0.30, 0.30, 0.15),
    'sauvc_drum_red_pinger': (0.30, 0.30, 0.15),
    'sauvc_starting_zone': (0.70, 0.70, 0.02),
}

# Camera extrinsics relative to base_link (from configs.yaml); optical: x right, y down, z forward.
FRONT_CAM_POSE = (0.2, 0.0, 0.0)  # x,y,z in base_link
BOTTOM_CAM_POSE = (0.0, 0.0, -0.1)


@dataclass
class PropInstance:
    name: str
    model: str
    xyz: tuple[float, float, float]
    yaw: float


def class_id(model: str) -> Optional[int]:
    label = MODEL_TO_CLASS.get(model)
    if label is None:
        return None
    return CLASSES.index(label)


def load_course_props(course_yaml: Path, pool_depth: float = 1.6) -> list[PropInstance]:
    with open(course_yaml) as f:
        course = yaml.safe_load(f)
    props = []
    for entry in course.get('props') or []:
        model = entry['model']
        name = entry.get('name', model)
        x, y = entry.get('xy', [0.0, 0.0])
        yaw = float(entry.get('yaw', 0.0))
        if 'z' in entry:
            z = float(entry['z'])
        else:
            # Surface-anchored gates/zones sit at z=0; floor props at -depth.
            if model in ('sauvc_qual_gate', 'sauvc_starting_zone'):
                z = 0.0 + float(entry.get('z_offset', 0.0))
            else:
                z = -pool_depth + float(entry.get('z_offset', 0.0))
        props.append(PropInstance(name, model, (float(x), float(y), z), yaw))
    return props


def _quat_to_R(qx, qy, qz, qw) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _yaw_R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _base_to_optical(cam: str) -> np.ndarray:
    """Rotation base_link (FLU) → optical (RDF: x right, y down, z forward)."""
    # FLU → RDF: x_opt=y_base, y_opt=-z_base, z_opt=x_base for forward cam.
    if cam == 'front':
        return np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
    # Bottom camera looks -z: x_opt=y_base, y_opt=x_base, z_opt=-z_base (approx).
    return np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64)


def project_props_yolo(
    props: Iterable[PropInstance],
    vehicle_xyz: tuple[float, float, float],
    vehicle_quat: tuple[float, float, float, float],
    cam: str,
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> list[tuple[int, float, float, float, float]]:
    """Return YOLO rows (cls, xc, yc, w, h) normalized 0..1."""
    R_wb = _quat_to_R(*vehicle_quat)
    t_wb = np.array(vehicle_xyz, dtype=np.float64)
    cam_t = np.array(FRONT_CAM_POSE if cam == 'front' else BOTTOM_CAM_POSE, dtype=np.float64)
    R_bo = _base_to_optical(cam)

    rows = []
    for prop in props:
        cid = class_id(prop.model)
        half = PROP_HALF_EXTENTS.get(prop.model)
        if cid is None or half is None:
            continue
        R_wp = _yaw_R(prop.yaw)
        t_wp = np.array(prop.xyz, dtype=np.float64)
        # 8 corners of AABB in prop frame (surface/floor origin).
        hx, hy, hz = half
        if prop.model in ('sauvc_qual_gate', 'sauvc_starting_zone'):
            zs = np.array([0.0, -2 * hz])  # hangs downward from surface
        else:
            zs = np.array([0.0, 2 * hz])  # stands upward from floor
        corners_p = []
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                for sz in zs:
                    corners_p.append([sx, sy, sz])
        corners_p = np.array(corners_p, dtype=np.float64)
        corners_w = (R_wp @ corners_p.T).T + t_wp

        # World → base → optical.
        corners_b = (R_wb.T @ (corners_w - t_wb).T).T - cam_t
        corners_o = (R_bo @ corners_b.T).T

        # Keep points in front of the camera.
        front = corners_o[:, 2] > 0.15
        if not np.any(front):
            continue
        pts = corners_o[front]
        u = fx * (pts[:, 0] / pts[:, 2]) + cx
        v = fy * (pts[:, 1] / pts[:, 2]) + cy
        u0, u1 = float(u.min()), float(u.max())
        v0, v1 = float(v.min()), float(v.max())
        # Clip to image.
        u0, u1 = max(0.0, u0), min(width - 1.0, u1)
        v0, v1 = max(0.0, v0), min(height - 1.0, v1)
        if u1 - u0 < 2 or v1 - v0 < 2:
            continue
        xc = ((u0 + u1) / 2.0) / width
        yc = ((v0 + v1) / 2.0) / height
        bw = (u1 - u0) / width
        bh = (v1 - v0) / height
        if bw * bh < 0.0005:
            continue
        rows.append((cid, xc, yc, bw, bh))
    return rows


def write_classes_file(path: Path) -> None:
    path.write_text('\n'.join(CLASSES) + '\n')


def format_yolo_line(row: tuple[int, float, float, float, float]) -> str:
    cid, xc, yc, w, h = row
    return f'{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}'
