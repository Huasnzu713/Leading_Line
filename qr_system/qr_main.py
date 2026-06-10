"""QR 系统入口：两种模式。

  camera  实时从摄像头读流，识别二维码，喂给状态机，HUD 上画 (state, steer, speed)
  test    读一张图，离线识别一次，喂给状态机，把结果画到结果图里

用法：
    python qr_main.py --mode camera
    python qr_main.py --mode camera --source tests/qr_state_machine_samples/turn_left.png  # 也可加 source 替代摄像头
    python qr_main.py --mode test  --source tests/qr_state_machine_samples/turn_left.png

按 ESC 退出；按 s 保存当前帧到 qr_result/。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

# 允许在 qr_system/ 目录下直接 python qr_main.py
sys.path.insert(0, str(Path(__file__).parent))

try:
    from .qr_decoder import decode_qr_codes, draw_qr_overlay
    from .qr_state_machine import QRStateMachine
except ImportError:  # 脚本直跑模式
    from qr_decoder import decode_qr_codes, draw_qr_overlay
    from qr_state_machine import QRStateMachine


DEFAULT_CONFIG = str(Path(__file__).parent / "qr_config.yaml")


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_config(arg_path: str) -> str:
    """--config 走用户路径；未传则用脚本同目录的 qr_config.yaml。
    找不到时回退到 CWD 下的同名文件，方便临时覆盖。
    """
    if arg_path and arg_path != DEFAULT_CONFIG:
        return arg_path
    if not Path(DEFAULT_CONFIG).exists():
        cwd_fallback = Path("qr_config.yaml")
        if cwd_fallback.exists():
            return str(cwd_fallback)
    return DEFAULT_CONFIG


# ---------- 模式 1：test ----------
def run_test(source: str, cfg: dict) -> int:
    """离线跑一张图，识别一次，喂给状态机，把结果存图。"""
    p = Path(source)
    if not p.exists():
        print(f"[ERROR] 找不到文件：{source}", file=sys.stderr)
        return 1
    img = cv2.imread(str(p))
    if img is None:
        print(f"[ERROR] 无法读图：{source}", file=sys.stderr)
        return 1

    sm = QRStateMachine(
        policy_timeout_s=float(cfg.get("state_machine", {}).get("policy_timeout_s", 30.0))
    )
    sm.start()
    decoded_list = decode_qr_codes(img)
    vis = img.copy()
    print(f"--- 离线识别 {p.name} ---")
    print(f"识别到 {len(decoded_list)} 个二维码")
    for i, d in enumerate(decoded_list):
        print(f"[{i}] text = {d.text!r}")
        vis = draw_qr_overlay(vis, d)
        sm.on_qr_decoded(d.text)

    # 跑几个 tick，让状态机把 DECODED → POLICY_ACTIVE → REPORTING 走完
    for _ in range(int(cfg.get("state_machine", {}).get("test_ticks", 3))):
        steer, speed = sm.tick(0.1)
        print(f"tick: state={sm.state.value}  steer={steer:+.2f}  speed={speed:.2f}")

    # 在结果图底部加一行 HUD
    h, w = vis.shape[:2]
    cv2.rectangle(vis, (0, h - 40), (w, h), (0, 0, 0), -1)
    hud = f"state={sm.state.value}  steer={sm.last_output()[0]:+.2f}  speed={sm.last_output()[1]:.2f}"
    cv2.putText(vis, hud, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    out_dir = Path("qr_result")
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"{p.stem}_decoded{p.suffix}"
    cv2.imwrite(str(out_path), vis)
    print(f"--- 已保存 {out_path} ---")
    return 0


# ---------- 模式 2：camera ----------
def run_camera(source: Optional[str], cfg: dict) -> int:
    """开摄像头实时识别；source 非空时读图/视频替代摄像头。"""
    cam_cfg = cfg.get("camera", {})
    window_name = str(cfg.get("ui", {}).get("window_name", "QR State Machine"))
    exit_key = int(cfg.get("ui", {}).get("exit_key", 27))
    save_key = ord(str(cfg.get("ui", {}).get("save_key", "s")).lower() or "s")

    if source is None:
        cap = cv2.VideoCapture(int(cam_cfg.get("index", 0)))
    else:
        p = Path(source)
        if not p.exists():
            print(f"[ERROR] 找不到 source：{source}", file=sys.stderr)
            return 1
        if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}:
            img = cv2.imread(str(p))
            if img is None:
                print(f"[ERROR] 无法读图：{source}", file=sys.stderr)
                return 1
            cap = _OneShotSource(img)
        else:
            cap = cv2.VideoCapture(str(p))

    if not cap.isOpened():
        print("[ERROR] 视频源打开失败", file=sys.stderr)
        return 1

    sm = QRStateMachine(
        policy_timeout_s=float(cfg.get("state_machine", {}).get("policy_timeout_s", 30.0))
    )
    sm.start()
    prev_t = time.time()
    out_dir = Path("qr_result")
    out_dir.mkdir(exist_ok=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            now = time.time()
            dt = now - prev_t
            prev_t = now

            # 在 SCANNING / IDLE 状态下持续吃 QR；其它状态忽略新的 QR
            decoded = decode_qr_codes(frame) if sm.state.value in ("IDLE", "SCANNING") else []
            for d in decoded:
                sm.on_qr_decoded(d.text)
            if sm.state.value == "SCANNING" and not decoded:
                sm.on_qr_empty()

            steer, speed = sm.tick(dt)

            # 可视化
            for d in decoded:
                frame = draw_qr_overlay(frame, d)

            h, w = frame.shape[:2]
            cv2.rectangle(frame, (0, 0), (w, 80), (0, 0, 0), -1)
            cv2.putText(frame, f"state : {sm.state.value}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"steer : {steer:+6.2f} deg", (10, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"speed : {speed:5.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            if sm.last_policy is not None:
                cv2.putText(frame, f"policy: {sm.last_policy.name}", (w - 240, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            if sm.last_error:
                cv2.putText(frame, f"err  : {sm.last_error[:40]}", (w - 240, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2, cv2.LINE_AA)

            # 控制台输出
            print(f"\rstate={sm.state.value:<14}  steer={steer:+6.2f}  "
                  f"speed={speed:5.2f}  decoded={len(decoded):d}    ",
                  end="", flush=True)

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == exit_key:
                break
            if key == save_key:
                ts = int(time.time())
                cv2.imwrite(str(out_dir / f"snapshot_{ts}.png"), frame)

            # REPORTING → 短延迟后自动回 IDLE 准备下一轮
            if sm.state.value == "REPORTING":
                sm.ack_report()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print()  # 把 \r 行结束一下
    return 0


class _OneShotSource:
    """把单张图包装成可 read() 的对象。"""

    def __init__(self, img: np.ndarray) -> None:
        self._img = img
        self._sent = False

    def read(self):
        if self._sent:
            return False, None
        self._sent = True
        return True, self._img.copy()

    def isOpened(self) -> bool:  # noqa: N802
        return True

    def release(self) -> None:
        pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="QR + 状态机策略系统")
    p.add_argument("--mode", choices=["camera", "test"], required=True,
                   help="camera=实时摄像头；test=离线读图")
    p.add_argument("--source", default=None,
                   help="test 模式必填图片路径；camera 模式可选（不传走摄像头，传则读图/视频）")
    p.add_argument("--config", default=DEFAULT_CONFIG,
                   help="YAML 配置路径；默认用脚本同目录的 qr_config.yaml，未找到时回退到 CWD")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cfg_path = resolve_config(args.config)
    if not Path(cfg_path).exists():
        print(f"[ERROR] 找不到配置文件：{cfg_path}", file=sys.stderr)
        return 3
    cfg = load_config(cfg_path)
    print(f"[config] {cfg_path}")
    if args.mode == "test":
        if not args.source:
            print("[ERROR] test 模式必须 --source <图片路径>", file=sys.stderr)
            return 2
        return run_test(args.source, cfg)
    return run_camera(args.source, cfg)


if __name__ == "__main__":
    sys.exit(main())
