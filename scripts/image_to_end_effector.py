#!/usr/bin/env python3
"""Convert an image into Drawing_Demo replay and Franka EE target visualizations."""

import argparse
import os
import re
from pathlib import Path

_MPL_CACHE = Path("/tmp/drawing_demo_matplotlib")
_MPL_CACHE.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE))

from make_drawing_path import (
    fit_strokes_to_physical_area,
    samples_from_strokes,
    section_strokes_by_physical_length,
    stretch_samples_to_count,
    strokes_from_image,
    write_preview,
    write_replay,
)
from visualize_end_effector_path import (
    compute_end_effector_samples,
    draw_2d,
    draw_3d,
    fit_plane,
    read_replay,
    read_robot_calibration,
    summarize,
    write_csv,
)


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
REPLAY_SAMPLE_SECONDS = 0.005


def clean_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned.strip("._-") or "drawing"


def resolve_input_path(value: str) -> Path:
    path = Path(value).expanduser()
    if path.exists():
        return path.resolve()
    if not path.is_absolute():
        project_path = PROJECT_ROOT / path
        if project_path.exists():
            return project_path.resolve()
    raise FileNotFoundError(f"Image does not exist: {value}")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Convert an image to replay points, calibrated EE XYZ positions, and preview plots."
    )
    parser.add_argument(
        "--image",
        default=str(PROJECT_ROOT / "Images" / "square.png"),
        help="Image path. Defaults to Images/square.png.",
    )
    parser.add_argument("--name", help="Output base name. Defaults to the image filename stem.")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "Data", help="Output directory.")
    parser.add_argument("--config-dir", type=Path, default=PROJECT_ROOT / "Config", help="Calibration config directory.")
    parser.add_argument(
        "--width",
        type=int,
        help="Trace canvas width in pixels. Defaults to the calibrated plane ratio when omitted.",
    )
    parser.add_argument("--height", type=int, default=300, help="Trace canvas height in pixels.")
    parser.add_argument(
        "--no-calibrated-aspect",
        action="store_true",
        help="Keep the legacy 220px trace width instead of deriving width from robot calibration.",
    )
    parser.add_argument(
        "--no-fit-to-calibration",
        action="store_true",
        help="Do not scale traced strokes into the calibrated robot drawing area.",
    )
    parser.add_argument(
        "--square-fit",
        action="store_true",
        help="Fit strokes into the largest centered square inside the calibrated robot drawing area.",
    )
    parser.add_argument(
        "--calibration-margin-m",
        type=float,
        default=0.005,
        help="Physical margin to keep inside the calibrated drawing area when fitting strokes.",
    )
    parser.add_argument("--threshold", type=int, help="Optional binary threshold for image tracing.")
    parser.add_argument("--canny-sigma", type=float, default=0.33, help="Automatic Canny threshold sigma.")
    parser.add_argument("--blur", type=int, default=3, help="Gaussian blur kernel; 0 disables.")
    parser.add_argument("--min-contour-points", type=int, default=10, help="Ignore shorter contours.")
    parser.add_argument("--simplify-px", type=float, default=1.0, help="Contour simplification in pixels.")
    parser.add_argument(
        "--greyscale-output",
        type=Path,
        help="Prepared greyscale PNG used for tracing. Defaults to Data/<name>_greyscale.png.",
    )
    parser.add_argument("--no-greyscale-output", action="store_true", help="Do not save the prepared greyscale image.")
    parser.add_argument(
        "--no-contrast-enhance",
        action="store_true",
        help="Disable CLAHE contrast, denoise, and light sharpening before edge tracing.",
    )
    parser.add_argument("--travel-step", type=float, default=0.001, help="Max normalized step while pen is lifted.")
    parser.add_argument("--draw-step", type=float, default=0.00075, help="Max normalized step while drawing.")
    parser.add_argument("--lift-samples", type=int, default=30, help="Pen-up dwell samples before travel.")
    parser.add_argument("--hover-samples", type=int, default=40, help="Pen-up dwell samples before pressing.")
    parser.add_argument("--press-samples", type=int, default=80, help="Pen-down dwell samples before a stroke.")
    parser.add_argument("--final-lift-samples", type=int, default=300, help="Pen-up dwell samples at the end.")
    parser.add_argument(
        "--section-length-m",
        type=float,
        help="Break traced lines into pen-down sections of this physical length.",
    )
    parser.add_argument(
        "--section-gap-m",
        type=float,
        default=0.001,
        help="Physical pen-up gap between sectioned line pieces.",
    )
    parser.add_argument(
        "--target-replay-minutes",
        type=float,
        help="Stretch replay samples to at least this duration, based on the 5 ms replay tick.",
    )
    parser.add_argument("--max-visual-samples", type=int, help="Limit visualized samples for faster plotting.")
    return parser


def main():
    args = build_parser().parse_args()
    image = resolve_input_path(args.image)
    output_dir = args.output_dir.expanduser()
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir
    config_dir = args.config_dir.expanduser()
    if not config_dir.is_absolute():
        config_dir = PROJECT_ROOT / config_dir

    output_name = clean_name(args.name or image.stem)
    output_dir.mkdir(parents=True, exist_ok=True)

    replay_path = output_dir / f"{output_name}.txt"
    source_preview_path = output_dir / f"{output_name}_preview.png"
    ee_prefix = output_dir / output_name
    ee_xy_path = output_dir / f"{output_name}_ee_xy.png"
    ee_3d_path = output_dir / f"{output_name}_ee_3d.png"
    ee_csv_path = output_dir / f"{output_name}_ee_positions.csv"
    greyscale_path = None
    if not args.no_greyscale_output:
        greyscale_path = args.greyscale_output or output_dir / f"{output_name}_greyscale.png"
        greyscale_path = greyscale_path.expanduser()
        if not greyscale_path.is_absolute():
            greyscale_path = PROJECT_ROOT / greyscale_path

    calibration_points = read_robot_calibration(config_dir)
    p1, p2, p3 = calibration_points
    calibration_width_m = float(((p3 - p2) ** 2).sum() ** 0.5)
    calibration_height_m = float(((p1 - p2) ** 2).sum() ** 0.5)

    trace_width = args.width
    if trace_width is None:
        if args.no_calibrated_aspect:
            trace_width = 220
        else:
            trace_width = max(2, int(round(args.height * calibration_width_m / calibration_height_m)))

    trace_args = argparse.Namespace(
        image=image,
        width=trace_width,
        height=args.height,
        blur=args.blur,
        threshold=args.threshold,
        canny_sigma=args.canny_sigma,
        min_contour_points=args.min_contour_points,
        simplify_px=args.simplify_px,
        greyscale_output=greyscale_path,
        no_contrast_enhance=args.no_contrast_enhance,
    )

    strokes = strokes_from_image(trace_args)
    fit_info = None
    if not args.no_fit_to_calibration:
        strokes, fit_info = fit_strokes_to_physical_area(
            strokes,
            calibration_width_m,
            calibration_height_m,
            args.calibration_margin_m,
            square=args.square_fit,
        )
    section_info = None
    if args.section_length_m:
        strokes, section_info = section_strokes_by_physical_length(
            strokes,
            calibration_width_m,
            calibration_height_m,
            args.section_length_m,
            args.section_gap_m,
        )
    samples = samples_from_strokes(
        strokes,
        travel_step=args.travel_step,
        draw_step=args.draw_step,
        lift_samples=args.lift_samples,
        hover_samples=args.hover_samples,
        press_samples=args.press_samples,
        final_lift_samples=args.final_lift_samples,
    )
    target_sample_count = None
    original_sample_count = len(samples)
    if args.target_replay_minutes:
        target_sample_count = int(round(args.target_replay_minutes * 60.0 / REPLAY_SAMPLE_SECONDS))
        samples = stretch_samples_to_count(samples, target_sample_count)
    write_replay(samples, replay_path)
    write_preview(samples, source_preview_path, 1000)

    robot_plane = fit_plane(calibration_points)
    replay_samples = read_replay(replay_path, args.max_visual_samples)
    positions = compute_end_effector_samples(replay_samples, calibration_points, robot_plane, replay_y_flip=True)

    draw_2d(positions, ee_xy_path, f"{replay_path.name} end-effector XY")
    draw_3d(positions, ee_3d_path, f"{replay_path.name} end-effector XYZ")
    write_csv(positions, ee_csv_path)

    print(f"Image: {image}")
    if greyscale_path:
        print(f"Prepared greyscale trace input: {greyscale_path}")
    print(f"Replay file for robot: {replay_path}")
    print(f"Image path preview: {source_preview_path}")
    print(f"End-effector XY plot: {ee_xy_path}")
    print(f"End-effector 3D plot: {ee_3d_path}")
    print(f"End-effector CSV: {ee_csv_path}")
    print(f"Calibrated robot area: {calibration_width_m:.3f} m wide x {calibration_height_m:.3f} m tall")
    print(f"Trace canvas: {trace_width} x {args.height} px")
    if fit_info:
        print(
            "Fitted drawing size: "
            f"{fit_info['fitted_width_m']:.3f} m x {fit_info['fitted_height_m']:.3f} m "
            f"(scale {fit_info['scale']:.3f}, margin {fit_info['margin_m']:.3f} m"
            f"{', square fit' if fit_info['square'] else ''})"
        )
    if section_info:
        print(
            "Sectioned strokes: "
            f"{section_info['sections']} sections, "
            f"{section_info['section_length_m']:.4f} m draw / "
            f"{section_info['gap_length_m']:.4f} m gap"
        )
    if target_sample_count:
        print(
            "Replay timing: "
            f"{len(samples) * REPLAY_SAMPLE_SECONDS / 60.0:.2f} min "
            f"({original_sample_count} raw samples stretched to {len(samples)})"
        )
    print()
    summarize(positions)
    print()
    print("Run this replay with:")
    print("cd /home/airocs/Desktop/Projects/Drawing_Demo/Visit/build")
    print(
        "./Drawing_Demo --robot-ip 127.0.0.1 --mios-port 12000 "
        f"--local-ip 192.168.3.1 --start-pose 0 --replay-file {replay_path.name}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
