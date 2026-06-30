#!/usr/bin/env python3
"""Visualize Drawing_Demo replay files as Franka end-effector target positions."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


NORMAL_DRAWING_RETREAT_DISTANCE = 0.025


def read_pose(path: Path):
    values = [float(value) for value in path.read_text(encoding="utf-8-sig").split()]
    if len(values) < 16:
        raise ValueError(f"{path} must contain at least 16 pose values")
    return np.array(values[:16], dtype=float)


def read_robot_calibration(config_dir: Path):
    points = []
    for index in (1, 2, 3):
        pose = read_pose(config_dir / f"Robot_Calibration_Point_{index}.txt")
        points.append(pose[12:15])
    return points


def fit_plane(points):
    center = np.mean(points, axis=0)
    centered = np.array(points) - center
    _, _, vh = np.linalg.svd(centered)
    normal = vh[-1]
    d = -float(np.dot(normal, center))
    if d > 0:
        normal = -normal
        d = -d
    return np.array([normal[0], normal[1], normal[2], d], dtype=float)


def project_to_plane(point, plane):
    normal = plane[:3]
    return point - (np.dot(normal, point) + plane[3]) * normal


def target_from_xy(x, y, calibration_points, plane):
    p1, p2, p3 = calibration_points
    k1 = min(max(x, 0.0), 1.0)
    k2 = min(max(y, 0.0), 1.0)
    point = k1 * (p3 - p2) + k2 * (p1 - p2) + p2
    return project_to_plane(point, plane)


def read_replay(path: Path, max_samples: int | None):
    samples = []
    with path.open("r", encoding="utf-8-sig") as handle:
        first = handle.readline()
        for line_no, line in enumerate(handle, start=2):
            clean = line.strip()
            if not clean:
                continue
            parts = clean.split()
            if len(parts) < 4:
                raise ValueError(f"Replay line {line_no} needs index x y pen: {line.rstrip()}")
            sample_index = int(float(parts[0]))
            x = float(parts[1])
            y = float(parts[2])
            pen = int(float(parts[3]))
            samples.append((sample_index, x, y, pen))
            if max_samples is not None and len(samples) >= max_samples:
                break
    if not samples:
        raise ValueError(f"No replay samples found in {path}")
    return samples


def compute_end_effector_samples(samples, calibration_points, robot_plane, replay_y_flip: bool):
    positions = []
    for sample_index, x, replay_y, pen in samples:
        y = 1.0 - replay_y if replay_y_flip else replay_y
        plane = robot_plane.copy()
        if pen == 0:
            plane[3] += NORMAL_DRAWING_RETREAT_DISTANCE
        point = target_from_xy(x, y, calibration_points, plane)
        positions.append(
            {
                "sample": sample_index,
                "canvas_x": x,
                "canvas_y": y,
                "replay_y": replay_y,
                "pen": pen,
                "ee_x": float(point[0]),
                "ee_y": float(point[1]),
                "ee_z": float(point[2]),
            }
        )
    return positions


def draw_2d(positions, output: Path, title: str):
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_x = []
    draw_y = []
    move_x = []
    move_y = []
    for item in positions:
        target = (draw_x, draw_y) if item["pen"] else (move_x, move_y)
        target[0].append(item["ee_x"])
        target[1].append(item["ee_y"])

    ax.plot(move_x, move_y, ".", color="#9aa0a6", markersize=1, label="pen up")
    ax.plot(draw_x, draw_y, ".", color="#111111", markersize=1, label="pen down")
    ax.set_title(title)
    ax.set_xlabel("end-effector x (m)")
    ax.set_ylabel("end-effector y (m)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def draw_3d(positions, output: Path, title: str):
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    draw = [item for item in positions if item["pen"]]
    move = [item for item in positions if not item["pen"]]
    if move:
        ax.scatter(
            [item["ee_x"] for item in move],
            [item["ee_y"] for item in move],
            [item["ee_z"] for item in move],
            s=1,
            c="#9aa0a6",
            label="pen up",
        )
    if draw:
        ax.scatter(
            [item["ee_x"] for item in draw],
            [item["ee_y"] for item in draw],
            [item["ee_z"] for item in draw],
            s=1,
            c="#111111",
            label="pen down",
        )

    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.legend(loc="best")
    set_axes_equal(ax, positions)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def set_axes_equal(ax, positions):
    xs = np.array([item["ee_x"] for item in positions])
    ys = np.array([item["ee_y"] for item in positions])
    zs = np.array([item["ee_z"] for item in positions])
    ranges = np.array([xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min()])
    radius = max(float(ranges.max()) / 2.0, 1e-6)
    centers = np.array([(xs.max() + xs.min()) / 2.0, (ys.max() + ys.min()) / 2.0, (zs.max() + zs.min()) / 2.0])
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def write_csv(positions, output: Path):
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["sample", "canvas_x", "canvas_y", "replay_y", "pen", "ee_x", "ee_y", "ee_z"],
        )
        writer.writeheader()
        writer.writerows(positions)


def summarize(positions):
    pen_down = sum(1 for item in positions if item["pen"])
    xs = [item["ee_x"] for item in positions]
    ys = [item["ee_y"] for item in positions]
    zs = [item["ee_z"] for item in positions]
    travel = 0.0
    for previous, current in zip(positions, positions[1:]):
        travel += math.dist(
            (previous["ee_x"], previous["ee_y"], previous["ee_z"]),
            (current["ee_x"], current["ee_y"], current["ee_z"]),
        )
    print(f"Samples: {len(positions)} | pen-down: {pen_down} | pen-up: {len(positions) - pen_down}")
    print(f"EE x range: {min(xs):.6f} to {max(xs):.6f} m")
    print(f"EE y range: {min(ys):.6f} to {max(ys):.6f} m")
    print(f"EE z range: {min(zs):.6f} to {max(zs):.6f} m")
    print(f"Approx target travel length: {travel:.3f} m")


def build_parser():
    parser = argparse.ArgumentParser(description="Visualize a Drawing_Demo replay file as Franka EE target positions.")
    parser.add_argument("--replay", type=Path, required=True, help="Replay file, for example Data/square.txt")
    parser.add_argument("--config-dir", type=Path, default=Path("Config"), help="Directory with robot calibration points")
    parser.add_argument("--output-prefix", type=Path, required=True, help="Output path prefix without extension")
    parser.add_argument("--max-samples", type=int, help="Limit samples for faster plotting")
    parser.add_argument("--no-replay-y-flip", action="store_true", help="Disable the y flip used by C++ replay mode")
    parser.add_argument("--csv", action="store_true", help="Also write end-effector positions as CSV")
    return parser


def main():
    args = build_parser().parse_args()
    if not args.replay.exists():
        raise SystemExit(f"Replay file does not exist: {args.replay}")
    if not args.config_dir.exists():
        raise SystemExit(f"Config directory does not exist: {args.config_dir}")

    calibration_points = read_robot_calibration(args.config_dir)
    robot_plane = fit_plane(calibration_points)
    samples = read_replay(args.replay, args.max_samples)
    positions = compute_end_effector_samples(
        samples,
        calibration_points,
        robot_plane,
        replay_y_flip=not args.no_replay_y_flip,
    )

    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    path_2d = prefix.with_name(prefix.name + "_ee_xy.png")
    path_3d = prefix.with_name(prefix.name + "_ee_3d.png")
    draw_2d(positions, path_2d, f"{args.replay.name} end-effector XY")
    draw_3d(positions, path_3d, f"{args.replay.name} end-effector XYZ")
    if args.csv:
        write_csv(positions, prefix.with_name(prefix.name + "_ee_positions.csv"))

    summarize(positions)
    print(f"2D plot: {path_2d}")
    print(f"3D plot: {path_3d}")
    if args.csv:
        print(f"CSV: {prefix.with_name(prefix.name + '_ee_positions.csv')}")


if __name__ == "__main__":
    raise SystemExit(main())
