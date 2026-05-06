#!/usr/bin/env python3

import argparse
import csv
import math
import shutil
from pathlib import Path
import sys


def load_csv_rows(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def parse_float(row, key):
    return float(row[key])


def parse_int(row, key):
    return int(row[key])


def parse_pose(row):
    return {
        "kf_id": parse_int(row, "kf_id"),
        "stamp_ns": parse_int(row, "stamp_ns"),
        "px": parse_float(row, "opt_body_px"),
        "py": parse_float(row, "opt_body_py"),
        "pz": parse_float(row, "opt_body_pz"),
        "qx": parse_float(row, "opt_body_qx"),
        "qy": parse_float(row, "opt_body_qy"),
        "qz": parse_float(row, "opt_body_qz"),
        "qw": parse_float(row, "opt_body_qw"),
    }


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quaternion_angle_deg(a, b):
    aqx, aqy, aqz, aqw = normalize_quaternion(*a)
    bqx, bqy, bqz, bqw = normalize_quaternion(*b)
    dot = aqx * bqx + aqy * bqy + aqz * bqz + aqw * bqw
    dot = min(1.0, max(-1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def position_distance(a, b):
    dx = a["px"] - b["px"]
    dy = a["py"] - b["py"]
    dz = a["pz"] - b["pz"]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def load_online_poses(session_dir):
    manifest_path = session_dir / "keyframe_manifest.csv"
    rows = load_csv_rows(manifest_path)

    poses = []
    for row in rows:
        poses.append(
            {
                "kf_id": parse_int(row, "kf_id"),
                "stamp_ns": parse_int(row, "keyframe_stamp_ns"),
                "px": parse_float(row, "opt_body_px"),
                "py": parse_float(row, "opt_body_py"),
                "pz": parse_float(row, "opt_body_pz"),
                "qx": parse_float(row, "opt_body_qx"),
                "qy": parse_float(row, "opt_body_qy"),
                "qz": parse_float(row, "opt_body_qz"),
                "qw": parse_float(row, "opt_body_qw"),
            }
        )

    return poses


def load_offline_poses(session_dir):
    offline_csv = session_dir / "offline_global_graph" / "optimized_keyframes.csv"
    if not offline_csv.exists():
        return []
    return [parse_pose(row) for row in load_csv_rows(offline_csv)]


def write_viz_csv(path, poses):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            "kf_id,stamp_ns,opt_body_px,opt_body_py,opt_body_pz,"
            "opt_body_qx,opt_body_qy,opt_body_qz,opt_body_qw\n"
        )
        for pose in poses:
            stream.write(
                f"{pose['kf_id']},{pose['stamp_ns']},{pose['px']},{pose['py']},{pose['pz']},"
                f"{pose['qx']},{pose['qy']},{pose['qz']},{pose['qw']}\n"
            )


def summarize_deltas(online_poses, offline_poses):
    offline_by_id = {pose["kf_id"]: pose for pose in offline_poses}
    matched = []
    for online in online_poses:
        offline = offline_by_id.get(online["kf_id"])
        if offline is None:
            continue
        pos_err = position_distance(online, offline)
        rot_err = quaternion_angle_deg(
            (online["qx"], online["qy"], online["qz"], online["qw"]),
            (offline["qx"], offline["qy"], offline["qz"], offline["qw"]),
        )
        matched.append((online["kf_id"], pos_err, rot_err))

    if not matched:
        return None

    pos_sq_sum = sum(pos_err * pos_err for _, pos_err, _ in matched)
    rot_sq_sum = sum(rot_err * rot_err for _, _, rot_err in matched)
    max_pos = max(matched, key=lambda item: item[1])
    max_rot = max(matched, key=lambda item: item[2])

    return {
        "count": len(matched),
        "rms_pos_m": math.sqrt(pos_sq_sum / len(matched)),
        "rms_rot_deg": math.sqrt(rot_sq_sum / len(matched)),
        "max_pos": max_pos,
        "max_rot": max_rot,
        "online_only": len(online_poses) - len(matched),
        "offline_only": len(offline_poses) - len(matched),
        "matched": matched,
    }


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Export logged online optimizer poses into offline_global_graph_viz CSV format "
            "and optionally compare them against offline optimized poses."
        )
    )
    parser.add_argument("session_dir", type=Path, help="Path to the logged session directory")
    parser.add_argument(
        "--export-viz-session-dir",
        type=Path,
        default=None,
        help=(
            "Write a sidecar session directory containing "
            "offline_global_graph/optimized_keyframes.csv for the online poses. "
            "Defaults to <session_dir>_online_viz."
        ),
    )
    parser.add_argument(
        "--copy-offline-tags",
        action="store_true",
        help="Copy offline_global_graph/optimized_tags.yaml into the exported viz session if present.",
    )
    parser.add_argument(
        "--no-compare",
        action="store_true",
        help="Skip the online-vs-offline pose delta summary.",
    )
    parser.add_argument(
        "--per-kf",
        action="store_true",
        help="Print per-keyframe position and rotation deltas between online and offline poses.",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=0,
        help="If > 0 with --per-kf, print only the top K keyframes sorted by largest position delta.",
    )
    args = parser.parse_args()

    session_dir = args.session_dir.resolve()
    if not session_dir.is_dir():
        print(f"error: missing session directory: {session_dir}", file=sys.stderr)
        return 1

    online_poses = load_online_poses(session_dir)
    if not online_poses:
        print("error: no online poses found in keyframe_manifest.csv", file=sys.stderr)
        return 1

    export_session_dir = (
        args.export_viz_session_dir.resolve()
        if args.export_viz_session_dir is not None
        else session_dir.parent / f"{session_dir.name}_online_viz"
    )
    export_csv = export_session_dir / "offline_global_graph" / "optimized_keyframes.csv"
    write_viz_csv(export_csv, online_poses)

    copied_tags = False
    offline_tags = session_dir / "offline_global_graph" / "optimized_tags.yaml"
    if args.copy_offline_tags and offline_tags.exists():
        target_tags = export_session_dir / "offline_global_graph" / "optimized_tags.yaml"
        target_tags.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(offline_tags, target_tags)
        copied_tags = True

    print(f"Wrote online pose CSV for visualization: {export_csv}")
    if copied_tags:
        print(f"Copied offline tags into visualization session: {offline_tags}")
    print(
        "Visualize with:\n"
        f"  ros2 run offline_global_graph_viz offline_global_graph_viz_node "
        f"--ros-args -p session_dir:={export_session_dir}"
    )

    if args.no_compare:
        return 0

    offline_poses = load_offline_poses(session_dir)
    if not offline_poses:
        print("No offline optimized_keyframes.csv found; skipping online-vs-offline comparison.")
        return 0

    summary = summarize_deltas(online_poses, offline_poses)
    if summary is None:
        print("No matching keyframe IDs found between online and offline trajectories.")
        return 0

    print("Online vs offline pose deltas:")
    print(f"  matched keyframes: {summary['count']}")
    print(f"  online-only keyframes: {summary['online_only']}")
    print(f"  offline-only keyframes: {summary['offline_only']}")
    print(f"  RMS position delta: {summary['rms_pos_m']:.4f} m")
    print(f"  RMS rotation delta: {summary['rms_rot_deg']:.3f} deg")
    print(
        f"  Max position delta: {summary['max_pos'][1]:.4f} m "
        f"(kf_id={summary['max_pos'][0]})"
    )
    print(
        f"  Max rotation delta: {summary['max_rot'][2]:.3f} deg "
        f"(kf_id={summary['max_rot'][0]})"
    )

    if args.per_kf:
        matched = summary["matched"]
        if args.top_k > 0:
            matched = sorted(matched, key=lambda item: item[1], reverse=True)[: args.top_k]

        print("Per-keyframe deltas:")
        print(f"{'kf_id':>6} {'pos_delta_m':>12} {'rot_delta_deg':>14}")
        print("-" * 36)
        for kf_id, pos_err, rot_err in matched:
            print(f"{kf_id:6d} {pos_err:12.4f} {rot_err:14.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
