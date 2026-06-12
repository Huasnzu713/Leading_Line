"""生成测试用 QR 图。

依赖：qrcode 库（pip install qrcode）。
OpenCV 4.13 自带的 QRCodeEncoder 在本环境抛 C++ 异常，所以生成侧
换用 `qrcode` 这个纯 Python 包；解码侧仍然用 cv2.QRCodeDetector，
两边互不干扰。

用法：
    python qr_make_test.py                 # 生成默认几张策略 QR
    python qr_make_test.py --out qr_sample  # 指定输出目录
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import qrcode
from PIL import Image as PILImage


# 默认一组策略用例，覆盖 QR 主流程可能遇到的输入
SAMPLES: list[tuple[str, str]] = [
    ("policy=STOP",                                                       "stop.png"),
    ("policy=TURN_LEFT;steer_deg=-20;speed=0.20;duration_s=1.5",          "turn_left.png"),
    ("policy=TURN_RIGHT;steer_deg=20;speed=0.20;duration_s=1.5",          "turn_right.png"),
    ('{"policy":"SLOW_DOWN","steer_deg":0,"speed":0.10,"duration_s":3.0}', "slow_down.png"),
    ('{"policy":"CRUISE"}',                                               "cruise.png"),
    ("policy=CUSTOM;steer_deg=0;speed=0.0;duration_s=0",                  "custom.png"),
]


def make_qr_image(text: str, size: int = 480) -> np.ndarray:
    """用 qrcode 库生成白底黑码图，转成 OpenCV BGR。"""
    qr = qrcode.QRCode(
        version=None,                       # 自动选最小版本
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=10,
        border=2,
    )
    qr.add_data(text)
    qr.make(fit=True)
    pil_img = qr.make_image(fill_color="black", back_color="white").convert("L")
    pil_img = pil_img.resize((size, size), PILImage.LANCZOS)
    arr = np.array(pil_img, dtype=np.uint8)
    # 加白边 40 px，方便 OpenCV detect
    pad = 40
    bordered = np.full((arr.shape[0] + 2 * pad, arr.shape[1] + 2 * pad), 255, dtype=np.uint8)
    bordered[pad:pad + arr.shape[0], pad:pad + arr.shape[1]] = arr
    return cv2.cvtColor(bordered, cv2.COLOR_GRAY2BGR)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="vehicle/recognition/qr/tests/state_machine_samples",
                   help="输出目录（默认 vehicle/recognition/qr/tests/state_machine_samples/）")
    args = p.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"--- 写入到 {out_dir} ---")
    for text, name in SAMPLES:
        try:
            img = make_qr_image(text)
        except Exception as e:
            print(f"  ! 跳过 {name}: {e}", file=sys.stderr)
            continue
        out_path = out_dir / name
        cv2.imwrite(str(out_path), img)
        print(f"  [ok] {out_path.name:<20}  text={text!r}")
    print("完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
