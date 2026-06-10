"""生成示例箭头图片用于测试和演示.

本文件位于 arrow/ 子目录, 应在 arrow/ 内运行, 默认输出到 ../tests/arrow/.

用法:
    cd arrow
    python generate_samples.py                            # 默认写到 ../tests/arrow/
    python generate_samples.py --out ../tests/arrow       # 同上, 显式指定
    python generate_samples.py --size 300                 # 自定义图片边长

会生成 12 张图: 9 张黑箭头 (4 方向 × {干净, 噪声, 旋转}) + 3 张彩色反例.
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np


# 定义一个朝右的箭头模板, 坐标以图像中心为原点, 单位是"图像宽度的比例"
# 7 个顶点逆时针给出: 尖端 -> 上肩 -> 上颈 -> 左上尾 -> 左下尾 -> 下颈 -> 下肩
_ARROW_RIGHT_TEMPLATE = np.array([
    [ 0.30,  0.00],   # tip
    [ 0.00, -0.18],   # upper shoulder
    [ 0.00, -0.08],   # upper neck
    [-0.25, -0.08],   # upper tail
    [-0.25,  0.08],   # lower tail
    [ 0.00,  0.08],   # lower neck
    [ 0.00,  0.18],   # lower shoulder
], dtype=np.float32)


def _rotate(pts: np.ndarray, deg: float) -> np.ndarray:
    """绕原点旋转 (数学坐标系, 逆时针为正)."""
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    return pts @ R.T


def make_arrow(direction: str, size: int = 300,
               extra_rotation: float = 0.0,
               fg: int = 30, bg: int = 230) -> np.ndarray:
    """生成一张指向 direction 的箭头图.

    direction: 'up' (前) / 'left' / 'right' / 'down'
    extra_rotation: 在标准方向上额外叠加的旋转角度 (度), 用于鲁棒性测试.
    fg/bg: 前景 / 背景灰度.
    """
    base_angle = {"right": 0.0, "up": 90.0, "left": 180.0, "down": -90.0}[direction]
    pts_math = _rotate(_ARROW_RIGHT_TEMPLATE * size, base_angle + extra_rotation)
    # 数学坐标 (+y 向上) 转图像坐标 (+y 向下): 翻转 y, 然后平移到图像中心
    pts_img = pts_math.copy()
    pts_img[:, 1] = -pts_img[:, 1]
    pts_img += [size / 2.0, size / 2.0]
    pts_img = pts_img.astype(np.int32)

    img = np.full((size, size), bg, dtype=np.uint8)
    cv2.fillPoly(img, [pts_img], fg)
    return img


def add_noise(img: np.ndarray, sigma: float = 25.0) -> np.ndarray:
    """加高斯噪声."""
    noise = np.random.normal(0.0, sigma, img.shape).astype(np.float32)
    out = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return out


def make_colored_arrow(direction: str, size: int = 300,
                      fg_bgr: tuple[int, int, int] = (30, 30, 200),
                      bg_bgr: tuple[int, int, int] = (230, 230, 230)) -> np.ndarray:
    """生成一张 BGR 彩色的非黑箭头图, 用于验证"只接受黑色"过滤.

    direction: 'up' / 'left' / 'right' / 'down'
    fg_bgr / bg_bgr: 前景 / 背景的 BGR 颜色.
    """
    base_angle = {"right": 0.0, "up": 90.0, "left": 180.0, "down": -90.0}[direction]
    pts_math = _rotate(_ARROW_RIGHT_TEMPLATE * size, base_angle)
    pts_img = pts_math.copy()
    pts_img[:, 1] = -pts_img[:, 1]
    pts_img += [size / 2.0, size / 2.0]
    pts_img = pts_img.astype(np.int32)

    img = np.full((size, size, 3), bg_bgr, dtype=np.uint8)
    cv2.fillPoly(img, [pts_img], fg_bgr)
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="生成示例箭头图片")
    parser.add_argument("--out", default="../tests/arrow", help="输出目录 (相对 CWD; 默认 ../tests/arrow 需从 arrow/ 子目录运行)")
    parser.add_argument("--size", type=int, default=300, help="图片边长 (像素)")
    parser.add_argument("--seed", type=int, default=42, help="噪声随机种子")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    np.random.seed(args.seed)

    # 干净版本: 四个方向各一张
    for d in ("up", "left", "right", "down"):
        path = os.path.join(args.out, f"arrow_{d}.png")
        cv2.imwrite(path, make_arrow(d, size=args.size))
        print(f"  wrote {path}")

    # 加噪声版本 (验证抗噪)
    for d in ("up", "left", "right"):
        path = os.path.join(args.out, f"arrow_{d}_noisy.png")
        cv2.imwrite(path, add_noise(make_arrow(d, size=args.size), sigma=30.0))
        print(f"  wrote {path}")

    # 小幅旋转版本 (验证扇区分类的容差)
    for d, rot in (("up", 15.0), ("right", -20.0)):
        path = os.path.join(args.out, f"arrow_{d}_rot{int(rot):+d}.png")
        cv2.imwrite(path, make_arrow(d, size=args.size, extra_rotation=rot))
        print(f"  wrote {path}")

    # 彩色反例: 红色箭头, 算法应拒绝 (灰度均值 ~76, 仍可被拒; 蓝/绿色更明显)
    for d in ("up", "right"):
        path = os.path.join(args.out, f"arrow_{d}_red.png")
        cv2.imwrite(path, make_colored_arrow(d, size=args.size, fg_bgr=(30, 30, 200)))
        print(f"  wrote {path}")
    # 浅灰反例: 几乎不黑, 一定被拒
    for d in ("up",):
        path = os.path.join(args.out, f"arrow_{d}_gray.png")
        cv2.imwrite(path, make_colored_arrow(d, size=args.size, fg_bgr=(150, 150, 150)))
        print(f"  wrote {path}")

    print(f"\nDone. {len(os.listdir(args.out))} images in {args.out}/")


if __name__ == "__main__":
    main()
