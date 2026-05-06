#!/usr/bin/env python3

import argparse
import ast
import math
from pathlib import Path
import sys

def quaternion_to_ypr_deg(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    return tuple(math.degrees(v) for v in (yaw, pitch, roll))


def fmt(value, width=10, precision=4):
    if value is None:
        return f"{'NA':>{width}}"
    return f"{value:>{width}.{precision}f}"


def parse_list(text):
    try:
        value = ast.literal_eval(text.strip())
    except (ValueError, SyntaxError):
        return None
    return value if isinstance(value, list) else None


def parse_bool(text):
    value = text.strip().lower()
    if value == "true":
        return True
    if value == "false":
        return False
    return None


def parse_int(text):
    try:
        return int(text.strip())
    except ValueError:
        return None


def parse_keyframe_summary(path):
    summary = {
        "kf_id": None,
        "position": [None, None, None],
        "orientation_xyzw": [None, None, None, None],
        "has_pose_wb_covariance": False,
        "pose_wb_covariance": None,
    }

    section = None
    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.rstrip("\n")

            if line.startswith("associated_files:") or line.startswith("tracks:"):
                break

            if line.startswith("kf_id:"):
                summary["kf_id"] = parse_int(line.split(":", 1)[1])
                continue

            if line == "optimized_pose_wb:":
                section = "optimized_pose_wb"
                continue

            if line == "optimization:":
                section = "optimization"
                continue

            if section == "optimized_pose_wb":
                if line.startswith("  position:"):
                    parsed = parse_list(line.split(":", 1)[1])
                    if parsed is not None:
                        summary["position"] = parsed
                elif line.startswith("  orientation_xyzw:"):
                    parsed = parse_list(line.split(":", 1)[1])
                    if parsed is not None:
                        summary["orientation_xyzw"] = parsed
                elif line and not line.startswith("  "):
                    section = None

            if section == "optimization":
                if line.startswith("  has_pose_wb_covariance:"):
                    parsed = parse_bool(line.split(":", 1)[1])
                    summary["has_pose_wb_covariance"] = bool(parsed)
                elif line.startswith("  pose_wb_covariance:"):
                    parsed = parse_list(line.split(":", 1)[1])
                    if parsed is not None:
                        summary["pose_wb_covariance"] = parsed
                elif line and not line.startswith("  "):
                    section = None

    return summary


def iter_keyframes(session_dir):
    keyframes_dir = session_dir / "keyframes"
    if not keyframes_dir.is_dir():
        raise FileNotFoundError(f"missing keyframes directory: {keyframes_dir}")

    entries = []
    for path in keyframes_dir.glob("*.yaml"):
        data = parse_keyframe_summary(path)
        kf_id = data.get("kf_id")
        if kf_id is None:
            continue
        entries.append((int(kf_id), path, data))
    entries.sort(key=lambda item: item[0])
    return entries


def print_full_covariance(cov):
    if cov is None or len(cov) != 36:
        print("  pose_wb_covariance: unavailable")
        return
    print("  pose_wb_covariance:")
    for row in range(6):
        vals = cov[row * 6:(row + 1) * 6]
        print("   ", " ".join(f"{value: .6e}" for value in vals))


def main():
    parser = argparse.ArgumentParser(
        description="Inspect optimized poses and logged pose covariance from an online-mapping session.")
    parser.add_argument("session_dir", type=Path, help="Path to logged session directory")
    parser.add_argument(
        "--full-covariance",
        action="store_true",
        help="Print the full 6x6 pose covariance after each keyframe row")
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Print only the first N keyframes after sorting by kf_id")
    args = parser.parse_args()

    try:
        keyframes = iter_keyframes(args.session_dir)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if not keyframes:
        print("No keyframe YAML files found.", file=sys.stderr)
        return 1

    header = (
        f"{'kf_id':>6} "
        f"{'tx':>10} {'ty':>10} {'tz':>10} "
        f"{'yaw':>10} {'pitch':>10} {'roll':>10} "
        f"{'cov_rX':>10} {'cov_rY':>10} {'cov_rZ':>10} "
        f"{'cov_tX':>10} {'cov_tY':>10} {'cov_tZ':>10}"
    )
    print(header)
    print("-" * len(header))

    for index, (kf_id, _path, data) in enumerate(keyframes):
        if args.limit > 0 and index >= args.limit:
            break
        position = data.get("position", [None, None, None])
        quat = data.get("orientation_xyzw", [None, None, None, None])

        yaw = pitch = roll = None
        if len(quat) == 4 and all(v is not None for v in quat):
            yaw, pitch, roll = quaternion_to_ypr_deg(quat[0], quat[1], quat[2], quat[3])

        has_cov = bool(data.get("has_pose_wb_covariance", False))
        cov = data.get("pose_wb_covariance")
        diag = [None] * 6
        if has_cov and isinstance(cov, list) and len(cov) == 36:
            diag = [cov[0], cov[7], cov[14], cov[21], cov[28], cov[35]]

        print(
            f"{kf_id:6d} "
            f"{fmt(position[0])} {fmt(position[1])} {fmt(position[2])} "
            f"{fmt(yaw)} {fmt(pitch)} {fmt(roll)} "
            f"{fmt(diag[0])} {fmt(diag[1])} {fmt(diag[2])} "
            f"{fmt(diag[3])} {fmt(diag[4])} {fmt(diag[5])}"
        )

        if args.full_covariance:
            print_full_covariance(cov if has_cov else None)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
