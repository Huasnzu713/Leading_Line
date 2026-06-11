"""命令行: 对单张/多张图片识别箭头方向.

本文件位于 arrow/ 子目录, 应在 arrow/ 内运行, 样本位于上一级 tests/arrow/.

用法:
    cd arrow
    python detect_image.py ../tests/arrow/arrow_up.png
    python detect_image.py ../tests/arrow/*.png --save-dir out
    python detect_image.py ../tests/arrow/arrow_up.png --show     # 弹窗显示
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2

try:
    from .arrow_detector import detect_arrow, annotate
except ImportError:  # 脚本直跑模式
    from arrow_detector import detect_arrow, annotate

# Windows 控制台默认 GBK, 强制 UTF-8 让中文方向名正常显示
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass


def _expand_paths(patterns: list[str]) -> list[str]:
    """支持 shell 通配符 (Windows 下 shell 不会自动展开)."""
    out: list[str] = []
    for p in patterns:
        matches = glob.glob(p)
        if matches:
            out.extend(matches)
        elif os.path.exists(p):
            out.append(p)
        else:
            print(f"  [warn] 找不到: {p}", file=sys.stderr)
    return sorted(set(out))


def main() -> int:
    parser = argparse.ArgumentParser(description="识别图片中的箭头方向")
    parser.add_argument("images", nargs="+", help="图片路径, 支持通配符")
    parser.add_argument("--save-dir", default=None,
                        help="若给出, 把标注后的图片保存到该目录")
    parser.add_argument("--show", action="store_true",
                        help="弹窗显示每张图 (按任意键继续, q 退出)")
    args = parser.parse_args()

    paths = _expand_paths(args.images)
    if not paths:
        print("没有匹配到任何图片", file=sys.stderr)
        return 1

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    ok = 0
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  [error] 读取失败: {path}", file=sys.stderr)
            continue

        result = detect_arrow(img)
        if result is None:
            print(f"  {path}: 未检测到箭头")
            continue

        print(f"  {path}: 方向={result.direction:<4} "
              f"角度={result.angle_deg:6.1f}°  置信度={result.confidence:.2f}")
        ok += 1

        if args.save_dir or args.show:
            annotated = annotate(img, result)
            if args.save_dir:
                out_path = os.path.join(args.save_dir, os.path.basename(path))
                cv2.imwrite(out_path, annotated)
            if args.show:
                cv2.imshow("arrow", annotated)
                if cv2.waitKey(0) & 0xFF == ord("q"):
                    break

    if args.show:
        cv2.destroyAllWindows()

    print(f"\n共处理 {len(paths)} 张图, 成功识别 {ok} 张.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
