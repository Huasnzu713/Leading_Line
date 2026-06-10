"""命令行: 打开摄像头实时识别箭头方向.

用法:
    python detect_webcam.py                  # 默认 0 号摄像头
    python detect_webcam.py --camera 1       # 选 1 号摄像头
    python detect_webcam.py --min-conf 0.3   # 最小置信度阈值

按 q 退出, 按 s 保存当前帧到 frame_*.png.
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2

from arrow_detector import detect_arrow, annotate

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="摄像头实时箭头识别")
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号 (默认 0)")
    parser.add_argument("--width", type=int, default=640, help="采集宽度")
    parser.add_argument("--height", type=int, default=480, help="采集高度")
    parser.add_argument("--min-conf", type=float, default=0.2,
                        help="低于该置信度的检测结果不在画面上叠加")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if sys.platform == "win32" else 0)
    if not cap.isOpened():
        print(f"无法打开摄像头 {args.camera}", file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("摄像头已启动. 按 q 退出, 按 s 保存当前帧.")

    saved_count = 0
    prev_t = time.perf_counter()
    fps_smooth = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取摄像头失败", file=sys.stderr)
                break

            result = detect_arrow(frame)
            if result is not None and result.confidence >= args.min_conf:
                frame = annotate(frame, result)
            else:
                cv2.putText(frame, "no arrow", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # FPS 显示
            now = time.perf_counter()
            dt = now - prev_t
            prev_t = now
            inst_fps = 1.0 / dt if dt > 0 else 0.0
            fps_smooth = 0.9 * fps_smooth + 0.1 * inst_fps
            cv2.putText(frame, f"{fps_smooth:4.1f} FPS",
                        (frame.shape[1] - 130, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("arrow webcam", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                fname = f"frame_{saved_count:03d}.png"
                cv2.imwrite(fname, frame)
                print(f"  已保存 {fname}")
                saved_count += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
