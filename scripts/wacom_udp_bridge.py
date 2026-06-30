#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import socket
import subprocess
import sys
import time


DEFAULT_ALTITUDE = 1.57079632679

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)


def run_command(args, env):
    try:
        return subprocess.run(args, env=env, check=False, capture_output=True, text=True)
    except FileNotFoundError:
        return None


def discover_stylus_id(env):
    if shutil.which("xsetwacom"):
        result = run_command(["xsetwacom", "--list", "devices"], env)
        if result and result.returncode == 0:
            for line in result.stdout.splitlines():
                if "type: STYLUS" in line:
                    match = re.search(r"id:\s*(\d+)", line)
                    if match:
                        return match.group(1)

    result = run_command(["xinput", "list"], env)
    if result and result.returncode == 0:
        for line in result.stdout.splitlines():
            if "Wacom" in line and "stylus" in line.lower():
                match = re.search(r"id=(\d+)", line)
                if match:
                    return match.group(1)

    return None


def read_tablet_area(device_id, env):
    result = run_command(["xinput", "list-props", str(device_id)], env)
    if not result or result.returncode != 0:
        return 0.0, 0.0, 44800.0, 29600.0

    for line in result.stdout.splitlines():
        if "Wacom Tablet Area" not in line:
            continue
        numbers = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", line.split(":", 1)[-1])]
        if len(numbers) >= 4:
            return numbers[0], numbers[1], numbers[2], numbers[3]

    return 0.0, 0.0, 44800.0, 29600.0


def is_wacom_stylus(device_id, env):
    result = run_command(["xinput", "list-props", str(device_id)], env)
    if not result or result.returncode != 0:
        return False

    text = result.stdout.lower()
    return "wacom" in text and ("tool type" in text and "stylus" in text)


def read_state(device_id, env):
    result = run_command(["xinput", "--query-state", str(device_id)], env)
    if not result or result.returncode != 0:
        detail = result.stderr.strip() if result else "xinput command not found"
        raise RuntimeError(detail or "unable to read xinput state")

    values = {}
    for line in result.stdout.splitlines():
        match = re.search(r"valuator\[(\d+)\]=(-?\d+(?:\.\d+)?)", line)
        if match:
            values[int(match.group(1))] = float(match.group(2))
    return values


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def build_packet(values, area, pressure_threshold):
    min_x, min_y, max_x, max_y = area
    raw_x = values.get(0, min_x)
    raw_y = values.get(1, min_y)
    raw_pressure = values.get(2, 0.0)

    x = clamp((raw_x - min_x) / max(max_x - min_x, 1.0))
    y = clamp((raw_y - min_y) / max(max_y - min_y, 1.0))
    pressure = clamp(raw_pressure / 65536.0)
    button = 1 if pressure >= pressure_threshold else 0

    return f"Tablet {x:.6f} {y:.6f} {pressure:.6f} {DEFAULT_ALTITUDE:.12f} 0.000000 {button}"


def main():
    parser = argparse.ArgumentParser(description="Send Wacom stylus data to Drawing_Demo over UDP.")
    parser.add_argument("--host", default="127.0.0.1", help="Drawing_Demo tablet UDP host")
    parser.add_argument("--port", type=int, default=3333, help="Drawing_Demo tablet UDP port")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"), help="X display to read")
    parser.add_argument("--device-id", help="xinput device id for the Wacom stylus")
    parser.add_argument("--rate", type=float, default=60.0, help="Packets per second")
    parser.add_argument("--pressure-threshold", type=float, default=0.02, help="Normalized pressure needed to draw")
    parser.add_argument("--dry-run", action="store_true", help="Print packets without sending them")
    parser.add_argument("--once", action="store_true", help="Read one sample and exit")
    args = parser.parse_args()

    env = os.environ.copy()
    env["DISPLAY"] = args.display

    if not shutil.which("xinput"):
        print("xinput is required. Install package: xinput", file=sys.stderr)
        return 1

    device_id = args.device_id
    if device_id and not is_wacom_stylus(device_id, env):
        fallback_id = discover_stylus_id(env)
        if fallback_id:
            print(
                f"xinput id {device_id} is not a Wacom stylus; using discovered stylus id {fallback_id}",
                file=sys.stderr,
            )
            device_id = fallback_id
        else:
            print(f"xinput id {device_id} is not a Wacom stylus and no stylus was found.", file=sys.stderr)
            return 1

    if not device_id:
        device_id = discover_stylus_id(env)
    if not device_id:
        print("No Wacom stylus found through xinput/xsetwacom.", file=sys.stderr)
        return 1

    area = read_tablet_area(device_id, env)
    interval = 1.0 / max(args.rate, 1.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)
    last_status_time = 0.0

    print(f"Wacom bridge using xinput id {device_id}, display {args.display}, area {area}")
    if args.dry_run:
        print("Dry run: not sending UDP packets")
    else:
        print(f"Sending tablet packets to {args.host}:{args.port}")

    try:
        while True:
            packet = build_packet(read_state(device_id, env), area, args.pressure_threshold)
            if args.dry_run or args.once:
                print(packet)
            if not args.dry_run:
                sock.sendto(packet.encode("ascii"), destination)

            if args.once:
                return 0

            now = time.monotonic()
            if now - last_status_time >= 2.0:
                print(packet)
                last_status_time = now

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nWacom bridge stopped")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
