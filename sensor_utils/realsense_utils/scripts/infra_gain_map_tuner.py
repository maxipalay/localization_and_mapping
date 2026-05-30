#!/usr/bin/env python3

import argparse
import sys
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WINDOW_NAME = "infra_gain_map_tuner"


def apply_radial_gain_map(
    image_u8: np.ndarray,
    center_x_norm: float,
    center_y_norm: float,
    inner_radius_norm: float,
    full_gain_radius_norm: float,
    max_gain: float,
    radial_strength: float,
):
    h, w = image_u8.shape[:2]
    cx = center_x_norm * (w - 1)
    cy = center_y_norm * (h - 1)

    yy, xx = np.indices((h, w), dtype=np.float32)
    radii = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    max_radius = float(
        max(
            np.hypot(0.0 - cx, 0.0 - cy),
            np.hypot((w - 1) - cx, 0.0 - cy),
            np.hypot(0.0 - cx, (h - 1) - cy),
            np.hypot((w - 1) - cx, (h - 1) - cy),
        )
    )
    if max_radius < 1e-6:
        max_radius = 1.0

    inner_r = np.clip(inner_radius_norm, 0.0, 1.0) * max_radius
    full_r = np.clip(full_gain_radius_norm, 0.0, 1.0) * max_radius
    full_r = max(full_r, inner_r + 1e-3)
    strength = max(radial_strength, 1e-3)

    t = np.clip((radii - inner_r) / (full_r - inner_r), 0.0, 1.0)
    t = t * t * (3.0 - 2.0 * t)
    t = np.power(t, strength)

    gain = 1.0 + (max_gain - 1.0) * t
    src_f32 = image_u8.astype(np.float32)
    corrected = np.clip(src_f32 * gain, 0.0, 255.0).astype(np.uint8)
    return corrected, gain, (cx, cy), max_radius


def trackbar_to_params():
    center_x = cv2.getTrackbarPos("center_x_x1000", WINDOW_NAME) / 1000.0
    center_y = cv2.getTrackbarPos("center_y_x1000", WINDOW_NAME) / 1000.0
    inner_radius = cv2.getTrackbarPos("inner_r_x1000", WINDOW_NAME) / 1000.0
    full_gain_radius = cv2.getTrackbarPos("full_r_x1000", WINDOW_NAME) / 1000.0
    max_gain = cv2.getTrackbarPos("max_gain_x100", WINDOW_NAME) / 100.0
    radial_strength = cv2.getTrackbarPos("radial_strength_x100", WINDOW_NAME) / 100.0
    full_gain_radius = max(full_gain_radius, inner_radius)
    max_gain = max(max_gain, 1.0)
    radial_strength = max(radial_strength, 0.01)
    return center_x, center_y, inner_radius, full_gain_radius, max_gain, radial_strength


def print_yaml(center_x, center_y, inner_radius, full_gain_radius, max_gain, radial_strength):
    print(
        "\n".join(
            [
                f"center_x_norm: {center_x:.3f}",
                f"center_y_norm: {center_y:.3f}",
                f"inner_radius_norm: {inner_radius:.3f}",
                f"full_gain_radius_norm: {full_gain_radius:.3f}",
                f"max_gain: {max_gain:.3f}",
                f"radial_strength: {radial_strength:.3f}",
            ]
        )
    )


def save_radial_map(
    output_prefix: str,
    image_shape,
    gain: np.ndarray,
    center_x: float,
    center_y: float,
    inner_radius: float,
    full_gain_radius: float,
    max_gain: float,
    radial_strength: float,
):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = Path(f"{output_prefix}_{timestamp}")
    prefix.parent.mkdir(parents=True, exist_ok=True)

    np.save(f"{prefix}.npy", gain.astype(np.float32))

    gain_norm = gain / max(gain.max(), 1e-6)
    gain_u16 = np.clip(gain_norm * 65535.0, 0.0, 65535.0).astype(np.uint16)
    cv2.imwrite(f"{prefix}.png", gain_u16)

    h, w = image_shape[:2]
    yaml_text = "\n".join(
        [
            f"width: {w}",
            f"height: {h}",
            "model: manual_radial_mask",
            f"center_x_norm: {center_x:.4f}",
            f"center_y_norm: {center_y:.4f}",
            f"inner_radius_norm: {inner_radius:.4f}",
            f"full_gain_radius_norm: {full_gain_radius:.4f}",
            f"max_gain: {max_gain:.4f}",
            f"radial_strength: {radial_strength:.4f}",
            f"gain_npy: {prefix.name}.npy",
            f"gain_preview_png: {prefix.name}.png",
        ]
    )
    Path(f"{prefix}.yaml").write_text(yaml_text + "\n", encoding="utf-8")
    print(f"saved correction map: {prefix}.npy")
    print(f"saved preview image:  {prefix}.png")
    print(f"saved metadata:       {prefix}.yaml")


class LiveImageSource:
    def __init__(self, topic: str):
        self._bridge = CvBridge()
        self._frame = None
        self._lock = threading.Lock()
        self._node = rclpy.create_node("infra_gain_map_tuner")
        self._sub = self._node.create_subscription(
            Image, topic, self._image_callback, qos_profile_sensor_data
        )

    def _image_callback(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        with self._lock:
            self._frame = frame.copy()

    def poll(self):
        rclpy.spin_once(self._node, timeout_sec=0.01)
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def close(self):
        self._node.destroy_node()


def render_canvas(image, corrected, gain, gain_max):
    gain_vis = gain / max(gain_max, 1e-6)
    gain_vis = np.clip(gain_vis * 255.0, 0.0, 255.0).astype(np.uint8)

    original_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    corrected_bgr = cv2.cvtColor(corrected, cv2.COLOR_GRAY2BGR)
    gain_color = cv2.applyColorMap(gain_vis, cv2.COLORMAP_TURBO)

    delta = cv2.absdiff(corrected, image)
    delta_color = cv2.applyColorMap(delta, cv2.COLORMAP_INFERNO)

    top = np.hstack([original_bgr, corrected_bgr])
    bottom = np.hstack([gain_color, delta_color])
    canvas = np.vstack([top, bottom])

    h, w = image.shape[:2]
    cv2.putText(canvas, "original", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(canvas, "corrected", (w + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(canvas, "gain map", (20, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(canvas, "delta", (w + 20, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return canvas


def main():
    parser = argparse.ArgumentParser(
        description="Interactive tuner for radial IR gain maps in realsense_utils."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", help="Path to a representative grayscale image.")
    group.add_argument("--topic", help="ROS image topic for live tuning.")
    parser.add_argument(
        "--output-prefix",
        default="/tmp/radial_vignette_map",
        help="Prefix for saved correction maps.",
    )
    args = parser.parse_args()

    live_source = None
    image = None
    if args.image:
        image = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"failed to read image: {args.image}", file=sys.stderr)
            return 1
    else:
        rclpy.init(args=None)
        live_source = LiveImageSource(args.topic)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1800, 700)
    cv2.createTrackbar("center_x_x1000", WINDOW_NAME, 500, 1000, lambda _v: None)
    cv2.createTrackbar("center_y_x1000", WINDOW_NAME, 500, 1000, lambda _v: None)
    cv2.createTrackbar("inner_r_x1000", WINDOW_NAME, 250, 1000, lambda _v: None)
    cv2.createTrackbar("full_r_x1000", WINDOW_NAME, 850, 1000, lambda _v: None)
    cv2.createTrackbar("max_gain_x100", WINDOW_NAME, 200, 400, lambda _v: None)
    cv2.createTrackbar("radial_strength_x100", WINDOW_NAME, 100, 400, lambda _v: None)

    help_text = "q/ESC: quit    p: print YAML snippet    s: save radial map    r: reset defaults"

    while True:
        if live_source is not None:
            latest = live_source.poll()
            if latest is not None:
                image = latest
        if image is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                f"waiting for {args.topic}",
                (20, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )
            cv2.imshow(WINDOW_NAME, blank)
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
            continue

        center_x, center_y, inner_radius, full_gain_radius, max_gain, radial_strength = trackbar_to_params()
        corrected, gain, center_xy, max_radius = apply_radial_gain_map(
            image,
            center_x,
            center_y,
            inner_radius,
            full_gain_radius,
            max_gain,
            radial_strength,
        )

        canvas = render_canvas(image, corrected, gain, max_gain)
        cx, cy = center_xy
        cv2.drawMarker(
            canvas,
            (int(round(cx)), int(round(cy))),
            (0, 255, 255),
            cv2.MARKER_CROSS,
            24,
            2,
        )
        inner_px = inner_radius * max_radius
        full_px = full_gain_radius * max_radius
        cv2.circle(canvas, (int(round(cx)), int(round(cy))), int(round(inner_px)), (255, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(canvas, (int(round(cx)), int(round(cy))), int(round(full_px)), (0, 255, 255), 1, cv2.LINE_AA)
        radial_text = (
            f"center=({center_x:.3f}, {center_y:.3f}) "
            f"inner_r={inner_radius:.3f} full_r={full_gain_radius:.3f}"
        )
        cv2.putText(canvas, radial_text, (20, canvas.shape[0] - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        status = f"max_gain={max_gain:.2f} radial_strength={radial_strength:.2f}"
        cv2.putText(canvas, status, (20, canvas.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, help_text, (20, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("p"):
            print_yaml(center_x, center_y, inner_radius, full_gain_radius, max_gain, radial_strength)
        if key == ord("s"):
            save_radial_map(
                args.output_prefix,
                image.shape,
                gain,
                center_x,
                center_y,
                inner_radius,
                full_gain_radius,
                max_gain,
                radial_strength,
            )
        if key == ord("r"):
            cv2.setTrackbarPos("center_x_x1000", WINDOW_NAME, 500)
            cv2.setTrackbarPos("center_y_x1000", WINDOW_NAME, 500)
            cv2.setTrackbarPos("inner_r_x1000", WINDOW_NAME, 250)
            cv2.setTrackbarPos("full_r_x1000", WINDOW_NAME, 850)
            cv2.setTrackbarPos("max_gain_x100", WINDOW_NAME, 200)
            cv2.setTrackbarPos("radial_strength_x100", WINDOW_NAME, 100)

    cv2.destroyAllWindows()
    if live_source is not None:
        live_source.close()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
