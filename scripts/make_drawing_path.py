#!/usr/bin/env python3
"""Create Drawing_Demo replay files from images or normalized coordinates."""

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DEFAULT_WIDTH = 300
DEFAULT_HEIGHT = 400
DEFAULT_START = (0.5, 0.5)


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    pen: int


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def clamp_margin(value: float, width_m: float, height_m: float) -> float:
    max_margin = max(0.0, min(width_m, height_m) * 0.45)
    return max(0.0, min(value, max_margin))


def distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def physical_distance(a: Point, b: Point, width_m: float, height_m: float) -> float:
    return math.hypot((a.x - b.x) * width_m, (a.y - b.y) * height_m)


def interpolate(a: Point, b: Point, max_step: float):
    steps = max(1, int(math.ceil(distance(a, b) / max(max_step, 1e-6))))
    for i in range(1, steps + 1):
        t = i / steps
        yield Point(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)


def add_hold(samples, point: Point, pen: int, count: int):
    for _ in range(max(0, count)):
        samples.append(Sample(point.x, point.y, pen))


def add_move(samples, current: Point, target: Point, pen: int, max_step: float):
    for point in interpolate(current, target, max_step):
        samples.append(Sample(point.x, point.y, pen))
    return target


def order_strokes_nearest_neighbor(strokes):
    if not strokes:
        return []

    remaining = [list(stroke) for stroke in strokes]
    current = Point(*DEFAULT_START)
    ordered = []
    while remaining:
        best_index = 0
        best_reverse = False
        best_distance = float("inf")
        for index, stroke in enumerate(remaining):
            start_distance = distance(current, stroke[0])
            end_distance = distance(current, stroke[-1])
            if start_distance < best_distance:
                best_distance = start_distance
                best_index = index
                best_reverse = False
            if end_distance < best_distance:
                best_distance = end_distance
                best_index = index
                best_reverse = True
        stroke = remaining.pop(best_index)
        if best_reverse:
            stroke.reverse()
        ordered.append(stroke)
        current = stroke[-1]
    return ordered


def stroke_bounds(strokes):
    points = [point for stroke in strokes for point in stroke]
    if not points:
        raise ValueError("No stroke points available for fitting.")
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def fit_strokes_to_physical_area(strokes, width_m: float, height_m: float, margin_m: float, square: bool = False):
    """Scale strokes in calibrated physical coordinates, then map back to 0..1 replay coordinates."""
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError("Calibrated width and height must be positive.")

    min_x, max_x, min_y, max_y = stroke_bounds(strokes)
    if square:
        source_scale_x_m = min(width_m, height_m)
        source_scale_y_m = source_scale_x_m
    else:
        source_scale_x_m = width_m
        source_scale_y_m = height_m
    source_min_x_m = min_x * source_scale_x_m
    source_max_x_m = max_x * source_scale_x_m
    source_min_y_m = min_y * source_scale_y_m
    source_max_y_m = max_y * source_scale_y_m
    source_width_m = max(source_max_x_m - source_min_x_m, 1e-9)
    source_height_m = max(source_max_y_m - source_min_y_m, 1e-9)

    margin_m = clamp_margin(margin_m, width_m, height_m)
    if square:
        target_size_m = max(min(width_m, height_m) - 2.0 * margin_m, 1e-9)
        target_width_m = target_size_m
        target_height_m = target_size_m
    else:
        target_width_m = max(width_m - 2.0 * margin_m, 1e-9)
        target_height_m = max(height_m - 2.0 * margin_m, 1e-9)
    scale = min(target_width_m / source_width_m, target_height_m / source_height_m)

    fitted_width_m = source_width_m * scale
    fitted_height_m = source_height_m * scale
    target_min_x_m = (width_m - target_width_m) * 0.5 + (target_width_m - fitted_width_m) * 0.5
    target_min_y_m = (height_m - target_height_m) * 0.5 + (target_height_m - fitted_height_m) * 0.5

    fitted = []
    for stroke in strokes:
        fitted_stroke = []
        for point in stroke:
            point_x_m = point.x * source_scale_x_m
            point_y_m = point.y * source_scale_y_m
            fitted_x_m = target_min_x_m + (point_x_m - source_min_x_m) * scale
            fitted_y_m = target_min_y_m + (point_y_m - source_min_y_m) * scale
            fitted_stroke.append(Point(clamp01(fitted_x_m / width_m), clamp01(fitted_y_m / height_m)))
        fitted.append(fitted_stroke)

    return fitted, {
        "scale": scale,
        "margin_m": margin_m,
        "source_width_m": source_width_m,
        "source_height_m": source_height_m,
        "fitted_width_m": fitted_width_m,
        "fitted_height_m": fitted_height_m,
        "square": square,
    }


def section_strokes_by_physical_length(strokes, width_m: float, height_m: float, section_length_m: float, gap_length_m: float):
    if section_length_m <= 0.0:
        return strokes, {"sections": len(strokes), "section_length_m": 0.0, "gap_length_m": 0.0}

    gap_length_m = max(0.0, gap_length_m)
    sectioned = []
    for stroke in strokes:
        if len(stroke) < 2:
            continue

        drawing = True
        remaining_m = section_length_m
        current_section = [stroke[0]]
        cursor = stroke[0]

        for segment_end in stroke[1:]:
            segment_left_m = physical_distance(cursor, segment_end, width_m, height_m)
            if segment_left_m <= 1e-12:
                cursor = segment_end
                continue

            while segment_left_m > 1e-12:
                step_m = min(remaining_m, segment_left_m)
                ratio = step_m / segment_left_m
                next_point = Point(
                    cursor.x + (segment_end.x - cursor.x) * ratio,
                    cursor.y + (segment_end.y - cursor.y) * ratio,
                )

                if drawing:
                    if not current_section:
                        current_section = [cursor]
                    if distance(current_section[-1], next_point) > 1e-12:
                        current_section.append(next_point)

                cursor = next_point
                segment_left_m -= step_m
                remaining_m -= step_m

                if remaining_m <= 1e-12:
                    if drawing:
                        if len(current_section) >= 2:
                            sectioned.append(current_section)
                        current_section = []
                        if gap_length_m > 0.0:
                            drawing = False
                            remaining_m = gap_length_m
                        else:
                            drawing = True
                            remaining_m = section_length_m
                            current_section = [cursor]
                    else:
                        drawing = True
                        remaining_m = section_length_m
                        current_section = [cursor]

        if drawing and len(current_section) >= 2:
            sectioned.append(current_section)

    return sectioned, {
        "sections": len(sectioned),
        "section_length_m": section_length_m,
        "gap_length_m": gap_length_m,
    }


def stretch_samples_to_count(samples, target_count: int):
    if target_count <= len(samples) or len(samples) < 2:
        return samples

    stretched = []
    source_count = len(samples)
    for index in range(target_count):
        source_position = index * (source_count - 1) / max(target_count - 1, 1)
        left_index = int(math.floor(source_position))
        right_index = min(left_index + 1, source_count - 1)
        fraction = source_position - left_index
        left = samples[left_index]
        right = samples[right_index]
        stretched.append(
            Sample(
                left.x + (right.x - left.x) * fraction,
                left.y + (right.y - left.y) * fraction,
                left.pen,
            )
        )
    return stretched


def samples_from_strokes(
    strokes,
    travel_step: float,
    draw_step: float,
    lift_samples: int,
    hover_samples: int,
    press_samples: int,
    final_lift_samples: int,
):
    samples = []
    current = Point(*DEFAULT_START)
    add_hold(samples, current, 0, lift_samples)

    for stroke in order_strokes_nearest_neighbor(strokes):
        if len(stroke) < 2:
            continue

        start = stroke[0]
        add_hold(samples, current, 0, lift_samples)
        current = add_move(samples, current, start, 0, travel_step)
        add_hold(samples, current, 0, hover_samples)
        add_hold(samples, current, 1, press_samples)

        for point in stroke[1:]:
            current = add_move(samples, current, point, 1, draw_step)

    add_hold(samples, current, 0, final_lift_samples)
    return samples


def resize_to_canvas(gray, width: int, height: int):
    image_height, image_width = gray.shape[:2]
    scale = min(width / image_width, height / image_height)
    resized_width = max(1, int(round(image_width * scale)))
    resized_height = max(1, int(round(image_height * scale)))
    resized = cv2.resize(gray, (resized_width, resized_height), interpolation=cv2.INTER_AREA)

    canvas = np.full((height, width), 255, dtype=np.uint8)
    x0 = (width - resized_width) // 2
    y0 = (height - resized_height) // 2
    canvas[y0 : y0 + resized_height, x0 : x0 + resized_width] = resized
    return canvas


def to_uint8(image):
    if image.dtype == np.uint8:
        return image
    if np.issubdtype(image.dtype, np.integer):
        max_value = np.iinfo(image.dtype).max
        scaled = image.astype(np.float32) * (255.0 / max(float(max_value), 1.0))
        return np.clip(scaled, 0, 255).astype(np.uint8)

    image = image.astype(np.float32)
    finite = image[np.isfinite(image)]
    if finite.size == 0:
        return np.zeros(image.shape, dtype=np.uint8)
    low = float(finite.min())
    high = float(finite.max())
    if 0.0 <= low and high <= 1.0:
        return np.clip(image * 255.0, 0, 255).astype(np.uint8)
    if low == high:
        return np.full(image.shape, 255 if high > 0 else 0, dtype=np.uint8)
    scaled = (image - low) * (255.0 / (high - low))
    return np.clip(scaled, 0, 255).astype(np.uint8)


def as_uint8_grayscale(image):
    image = to_uint8(image)
    if image.ndim == 2:
        gray = image
    elif image.ndim == 3 and image.shape[2] == 4:
        alpha = image[:, :, 3:4].astype(np.float32) / 255.0
        bgr = image[:, :, :3].astype(np.float32)
        composited = bgr * alpha + 255.0 * (1.0 - alpha)
        gray = cv2.cvtColor(composited.astype(np.uint8), cv2.COLOR_BGR2GRAY)
    elif image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(f"Unsupported image shape: {image.shape}")

    return gray


def read_grayscale(image_path: Path):
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Unable to read image: {image_path}")
    return as_uint8_grayscale(image)


def enhance_grayscale_for_tracing(gray):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    denoised = cv2.bilateralFilter(enhanced, d=5, sigmaColor=40, sigmaSpace=40)
    detail = cv2.GaussianBlur(denoised, (0, 0), 1.0)
    sharpened = cv2.addWeighted(denoised, 1.35, detail, -0.35, 0)
    return sharpened


def prepare_grayscale_canvas(args):
    gray = read_grayscale(args.image)
    canvas = resize_to_canvas(gray, args.width, args.height)
    if not getattr(args, "no_contrast_enhance", False):
        canvas = enhance_grayscale_for_tracing(canvas)
    if args.blur > 0:
        kernel = args.blur if args.blur % 2 == 1 else args.blur + 1
        canvas = cv2.GaussianBlur(canvas, (kernel, kernel), 0)
    greyscale_output = getattr(args, "greyscale_output", None)
    if greyscale_output:
        greyscale_output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(greyscale_output), canvas)
    return canvas


def automatic_canny(gray, sigma: float):
    median = float(np.median(gray))
    lower = int(max(0, (1.0 - sigma) * median))
    upper = int(min(255, (1.0 + sigma) * median))
    return cv2.Canny(gray, lower, upper)


def strokes_from_image(args):
    canvas = prepare_grayscale_canvas(args)
    if args.threshold is not None:
        _, binary = cv2.threshold(canvas, args.threshold, 255, cv2.THRESH_BINARY_INV)
        edges = cv2.Canny(binary, 50, 150)
    else:
        edges = automatic_canny(canvas, args.canny_sigma)

    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    strokes = []
    for contour in contours:
        if len(contour) < args.min_contour_points:
            continue
        epsilon = max(0.0, args.simplify_px)
        if epsilon > 0:
            contour = cv2.approxPolyDP(contour, epsilon, closed=False)
        points = []
        for item in contour.reshape(-1, 2):
            px, py = float(item[0]), float(item[1])
            x = clamp01(px / max(args.width - 1, 1))
            y = clamp01(1.0 - py / max(args.height - 1, 1))
            points.append(Point(x, y))
        if len(points) >= 2:
            strokes.append(points)

    if not strokes:
        raise ValueError("No drawable contours found. Try lowering --threshold or --min-contour-points.")
    return strokes


def parse_coord_line(line: str, line_no: int):
    clean = line.split("#", 1)[0].strip()
    if not clean:
        return None
    parts = [part for part in re.split(r"[,\s]+", clean) if part]
    if len(parts) < 2:
        raise ValueError(f"Coordinate line {line_no} needs at least x and y: {line.rstrip()}")
    x = float(parts[0])
    y = float(parts[1])
    pen = int(float(parts[2])) if len(parts) >= 3 else 1
    return x, y, 1 if pen else 0


def strokes_from_coords(args):
    strokes = []
    current = []
    input_size = args.input_size

    with args.coords.open("r", encoding="utf-8-sig") as handle:
        for line_no, line in enumerate(handle, start=1):
            parsed = parse_coord_line(line, line_no)
            if parsed is None:
                continue

            x, y, pen = parsed
            if input_size:
                x = x / max(input_size[0], 1.0)
                y = y / max(input_size[1], 1.0)
            if args.origin == "top-left":
                y = 1.0 - y

            point = Point(clamp01(x), clamp01(y))
            if pen:
                current.append(point)
            elif current:
                if len(current) >= 2:
                    strokes.append(current)
                current = []

    if len(current) >= 2:
        strokes.append(current)
    if not strokes:
        raise ValueError("No drawable coordinate strokes found.")
    return strokes


def write_replay(samples, output: Path):
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        handle.write(f"{len(samples)}\n")
        for index, sample in enumerate(samples, start=1):
            handle.write(f"{index} {sample.x:.6f} {sample.y:.6f} {sample.pen}\n")


def write_preview(samples, preview: Path, size: int):
    preview.parent.mkdir(parents=True, exist_ok=True)
    image = np.full((size, size, 3), 255, dtype=np.uint8)
    last = None
    for sample in samples:
        point = (
            int(round(sample.x * (size - 1))),
            int(round((1.0 - sample.y) * (size - 1))),
        )
        if sample.pen and last is not None:
            cv2.line(image, last, point, (0, 0, 0), 1, cv2.LINE_AA)
        last = point if sample.pen else None
    cv2.imwrite(str(preview), image)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Generate Drawing_Demo Data/*.txt replay paths from an image/photo or coordinate file."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="Input image/photo to trace into contour strokes")
    source.add_argument("--coords", type=Path, help="Input coordinate file with x y [pen] per line")
    parser.add_argument("--output", "-o", type=Path, required=True, help="Output replay file, usually Data/name.txt")
    parser.add_argument("--preview", type=Path, help="Optional PNG preview of the robot replay path")
    parser.add_argument("--preview-size", type=int, default=1000, help="Preview image size in pixels")

    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Image trace canvas width in pixels")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Image trace canvas height in pixels")
    parser.add_argument("--threshold", type=int, help="Optional binary threshold for image tracing")
    parser.add_argument("--canny-sigma", type=float, default=0.33, help="Automatic Canny threshold sigma")
    parser.add_argument("--blur", type=int, default=3, help="Gaussian blur kernel for image tracing; 0 disables")
    parser.add_argument("--min-contour-points", type=int, default=20, help="Ignore shorter image contours")
    parser.add_argument("--simplify-px", type=float, default=1.0, help="Contour simplification in image pixels")
    parser.add_argument("--greyscale-output", type=Path, help="Optional PNG of the prepared greyscale trace input")
    parser.add_argument(
        "--no-contrast-enhance",
        action="store_true",
        help="Disable CLAHE contrast, denoise, and light sharpening before edge tracing",
    )

    parser.add_argument("--origin", choices=("bottom-left", "top-left"), default="bottom-left",
                        help="Origin for --coords input before writing replay coordinates")
    parser.add_argument("--input-size", nargs=2, type=float, metavar=("WIDTH", "HEIGHT"),
                        help="Treat --coords x/y as pixels and normalize by this size")

    parser.add_argument("--travel-step", type=float, default=0.001, help="Max normalized step while pen is lifted")
    parser.add_argument("--draw-step", type=float, default=0.00075, help="Max normalized step while drawing")
    parser.add_argument("--lift-samples", type=int, default=30, help="Pen-up dwell samples before travel")
    parser.add_argument("--hover-samples", type=int, default=40, help="Pen-up dwell samples before pressing")
    parser.add_argument("--press-samples", type=int, default=80, help="Pen-down dwell samples before a stroke")
    parser.add_argument("--final-lift-samples", type=int, default=300, help="Pen-up dwell samples at the end")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.width < 2 or args.height < 2:
        parser.error("--width and --height must be at least 2")
    if args.image and not args.image.exists():
        parser.error(f"--image does not exist: {args.image}")
    if args.coords and not args.coords.exists():
        parser.error(f"--coords does not exist: {args.coords}")

    strokes = strokes_from_image(args) if args.image else strokes_from_coords(args)
    samples = samples_from_strokes(
        strokes,
        travel_step=args.travel_step,
        draw_step=args.draw_step,
        lift_samples=args.lift_samples,
        hover_samples=args.hover_samples,
        press_samples=args.press_samples,
        final_lift_samples=args.final_lift_samples,
    )
    write_replay(samples, args.output)
    if args.preview:
        write_preview(samples, args.preview, args.preview_size)

    pen_down = sum(1 for sample in samples if sample.pen)
    print(f"Wrote {len(samples)} samples to {args.output}")
    print(f"Strokes: {len(strokes)} | pen-down samples: {pen_down}")
    if args.preview:
        print(f"Preview: {args.preview}")


if __name__ == "__main__":
    raise SystemExit(main())
