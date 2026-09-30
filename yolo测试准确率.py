# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
"""Run real-time YOLO11 inference with an Orbbec color camera."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from pyorbbecsdk import Config, OBError, OBFormat, OBSensorType, Pipeline
from ultralytics import YOLO

WINDOW_NAME = "YOLO11 Orbbec Realtime Test"
QUIT_KEYS = (27, ord("q"), ord("Q"))


def default_model_path() -> Path:
    """Return the first existing default best.pt path."""
    project_dir = Path(__file__).resolve().parent
    candidates = (
        project_dir / "/home/xie/elevate_ws/best.pt",
        Path.home() / "Downloads/ultralytics-main/runs/detect/keyboard_yolo11n/weights/best.pt",
    )
    return next((path for path in candidates if path.is_file()), candidates[0])


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Use an Orbbec camera to test a YOLO11 best.pt model in real time.")
    parser.add_argument("--model", type=Path, default=default_model_path(), help="Path to the trained best.pt model.")
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence threshold.")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size.")
    parser.add_argument("--device", default="0", help="Inference device, such as 0 or cpu.")
    parser.add_argument("--width", type=int, default=640, help="Requested color stream width.")
    parser.add_argument("--height", type=int, default=480, help="Requested color stream height.")
    parser.add_argument("--fps", type=int, default=30, help="Requested color stream frame rate.")
    return parser.parse_args()


def select_color_profile(pipeline: Pipeline, width: int, height: int, fps: int):
    """Select a requested color profile, falling back to the camera default."""
    profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    for color_format in (OBFormat.RGB, OBFormat.BGR, OBFormat.MJPG, OBFormat.YUYV):
        try:
            return profiles.get_video_stream_profile(width, height, color_format, fps)
        except (OBError, RuntimeError):
            pass
    print(f"未找到 {width}x{height}@{fps} 彩色流，使用相机默认配置。")
    return profiles.get_default_video_stream_profile()


def frame_to_bgr(frame) -> np.ndarray:
    """Convert an Orbbec color frame to an OpenCV BGR image."""
    width, height = frame.get_width(), frame.get_height()
    color_format = frame.get_format()
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)

    if color_format == OBFormat.RGB:
        return cv2.cvtColor(data.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if color_format == OBFormat.BGR:
        return data.reshape(height, width, 3).copy()
    if color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("MJPG 彩色帧解码失败")
        return image
    if color_format == OBFormat.YUYV:
        return cv2.cvtColor(data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2)
    if color_format == OBFormat.UYVY:
        return cv2.cvtColor(data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY)
    if color_format == OBFormat.I420:
        return cv2.cvtColor(data.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_I420)
    if color_format == OBFormat.NV12:
        return cv2.cvtColor(data.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV12)
    if color_format == OBFormat.NV21:
        return cv2.cvtColor(data.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV21)
    raise ValueError(f"不支持的奥比中光彩色格式：{color_format}")


def main() -> int:
    """Start the camera and run YOLO inference until Q or ESC is pressed."""
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        print(f"模型不存在：{model_path}")
        return 1

    print(f"加载模型：{model_path}")
    model = YOLO(str(model_path))
    pipeline = Pipeline()
    started = False

    try:
        color_profile = select_color_profile(pipeline, args.width, args.height, args.fps)
        config = Config()
        config.enable_stream(color_profile)
        pipeline.start(config)
        started = True

        print(
            f"相机已启动：{color_profile.get_width()}x{color_profile.get_height()}"
            f"@{color_profile.get_fps()}，格式 {color_profile.get_format()}"
        )
        print("按 Q 或 ESC 退出。")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        smoothed_fps = 0.0
        previous_time = time.perf_counter()

        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue

            image = frame_to_bgr(color_frame)
            result = model.predict(
                source=image,
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                verbose=False,
            )[0]
            annotated = result.plot()

            now = time.perf_counter()
            current_fps = 1.0 / max(now - previous_time, 1e-6)
            previous_time = now
            smoothed_fps = current_fps if smoothed_fps == 0 else 0.9 * smoothed_fps + 0.1 * current_fps
            inference_ms = result.speed.get("inference", 0.0)
            detections = len(result.boxes) if result.boxes is not None else 0
            cv2.putText(
                annotated,
                f"FPS {smoothed_fps:.1f} | Infer {inference_ms:.1f} ms | Objects {detections}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW_NAME, annotated)
            if cv2.waitKey(1) & 0xFF in QUIT_KEYS:
                break
    except KeyboardInterrupt:
        print("\n用户中断。")
    except (OBError, RuntimeError, ValueError) as error:
        print(f"运行失败：{error}")
        return 1
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
