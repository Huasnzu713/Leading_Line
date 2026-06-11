"""Leading-Line 算法单机调试入口（双端模式下用 `python -m jetson.main_jetson`）。

读 jetson/config.yaml 的算法段（colors / roi / morphology / path / controller / temporal / visualization），
跑主循环后用 cv2.imshow 显示。

用法：
    python main.py                                    # 默认读 jetson/config.yaml
    python main.py --config my.yaml                   # 指定其它配置
    python main.py --source tests/synth.png           # 离线跑一张图

按 ESC 退出可视化窗口。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from jetson.algo import color_segmenter, controller, path_planner, visualizer
from protocol import select_mode


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def open_camera(cam_cfg: dict) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(cam_cfg["index"]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam_cfg["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam_cfg["height"]))
    cap.set(cv2.CAP_PROP_FPS, int(cam_cfg["fps"]))
    if not cap.isOpened():
        raise RuntimeError(
            f"无法打开摄像头 index={cam_cfg['index']}；"
            "如果是合成图请用 --source 指定图片路径。"
        )
    return cap


def open_source(source: str | None) -> cv2.VideoCapture | "StaticSource":
    if source is None:
        return None  # 摄像头
    p = Path(source)
    if not p.exists():
        raise FileNotFoundError(source)
    if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}:
        img = cv2.imread(str(p))
        if img is None:
            raise RuntimeError(f"无法读图: {source}")
        return StaticSource(img)
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {source}")
    return cap


class StaticSource:
    """把单张图包装成可 read() 的对象，便于主循环统一处理。"""

    def __init__(self, img: np.ndarray) -> None:
        self._img = img
        self._sent = False

    def read(self):
        if self._sent:
            return False, None
        self._sent = True
        return True, self._img.copy()

    def isOpened(self) -> bool:  # noqa: N802 (OpenCV 风格)
        return True

    def release(self) -> None:
        pass


def process_frame(frame: np.ndarray, cfg: dict):
    road_mask, floor_mask = color_segmenter.make_masks(
        frame,
        cfg["colors"]["road"]["hsv_lower"],
        cfg["colors"]["road"]["hsv_upper"],
        cfg["colors"]["floor"]["hsv_lower"],
        cfg["colors"]["floor"]["hsv_upper"],
    )
    road_mask = color_segmenter.clean_mask(
        road_mask,
        int(cfg["morphology"]["kernel_size"]),
        int(cfg["morphology"]["opening_iter"]),
        int(cfg["morphology"]["closing_iter"]),
    )
    # 取道路最大连通块：消除人脸/屏边/墙面等零碎误识别
    min_road_px = int(
        cfg.get("filter", {}).get("min_road_area_px", 0)
    )
    road_mask = color_segmenter.keep_largest_component(
        road_mask, min_area=min_road_px
    )
    edges = path_planner.plan(road_mask, cfg)
    steer, speed, lookahead = controller.decide(
        edges["center"], frame.shape[1], cfg["controller"]
    )
    return road_mask, floor_mask, edges, steer, speed, lookahead


def run(cfg: dict, source: str | None, debug: bool = False) -> None:
    cap = open_source(source)
    is_static = isinstance(cap, StaticSource)
    if cap is None:
        cap = open_camera(cfg["camera"])
    runtime = cfg["runtime"]
    print_to_stdout = bool(runtime.get("print_to_stdout", True))
    window_name = str(runtime.get("window_name", "Leading Line"))
    exit_key = int(runtime.get("exit_key", 27))

    # 时域平滑器：跨帧 EMA，消除摄像头噪声/曝光造成的路径抖动
    temp_cfg = cfg.get("temporal", {"enabled": False})
    smoother = (
        path_planner.PathSmoother(alpha=float(temp_cfg.get("alpha", 0.4)))
        if temp_cfg.get("enabled", False)
        else None
    )
    reset_on_no_road = bool(temp_cfg.get("reset_on_no_road", True))

    # 离线结果目录：自动创建，静态图 / 's' 抓帧结果都落在这里
    result_dir = Path("test_result")
    result_dir.mkdir(exist_ok=True)

    last_vis: np.ndarray | None = None  # 保留最后可视化帧，供静态图退出前保存
    debug_mode = debug  # 'd' 键可在视频/摄像头模式运行时切换

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            road_mask, floor_mask, edges, steer, speed, lookahead = process_frame(
                frame, cfg
            )
            path = edges["center"]

            # 时域平滑
            if smoother is not None:
                if reset_on_no_road and (path is None or np.all(np.isnan(path[:, 0]))):
                    smoother.reset()
                path = smoother.update(path)
                edges = {**edges, "center": path}

            if print_to_stdout:
                print(
                    f"\rsteer={steer:+6.2f}deg  speed={speed:.2f}  "
                    f"road={int((road_mask > 0).sum()):>7d}  "
                    f"floor={int((floor_mask > 0).sum()):>7d}",
                    end="",
                    flush=True,
                )

            # 调试模式：2x2 网格（原图/道路掩码/地面掩码/结果）
            if debug_mode:
                vis = visualizer.draw_debug(
                    frame, road_mask, floor_mask, edges, steer, speed,
                    lookahead, cfg["visualization"],
                )
            else:
                vis = visualizer.draw(
                    frame, road_mask, edges, steer, speed, lookahead, cfg["visualization"],
                    floor_mask=floor_mask,
                )
            last_vis = vis
            cv2.imshow(window_name, vis)

            # 静态图：等用户按键再退出，避免窗口一闪而过看不到结果
            if is_static:
                cv2.waitKey(0)
                break

            key = cv2.waitKey(1) & 0xFF
            if key == exit_key:
                break
            if key == ord("s"):
                cv2.imwrite(str(result_dir / "snapshot.png"), vis)
            if key == ord("d"):
                debug_mode = not debug_mode
                print(f"\n[Debug mode: {'ON' if debug_mode else 'OFF'}]")
    finally:
        # 静态图：把可视化结果落盘到 test_result/<原文件名>_result.<ext>
        if is_static and last_vis is not None and source:
            in_path = Path(source)
            out_path = result_dir / f"{in_path.stem}_result{in_path.suffix}"
            cv2.imwrite(str(out_path), last_vis)
            print(f"\n[已保存] {out_path}")
        cap.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="谷仓场景引导线算法")
    p.add_argument("--config", default="jetson/config.yaml", help="YAML 配置路径")
    p.add_argument(
        "--source",
        default=None,
        help="可选：图片或视频路径；不传则使用摄像头",
    )
    p.add_argument(
        "--debug",
        action="store_true",
        help="开启调试模式：2x2 网格（原图/道路掩码/地面掩码/结果）",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    raw_cfg = load_config(args.config)
    # 单机调试：默认用 blue_path 模式（把 colors/visualization 合并到顶层）
    cfg, _meta = select_mode(raw_cfg, "blue_path")
    run(cfg, args.source, debug=args.debug)
    return 0


if __name__ == "__main__":
    sys.exit(main())
