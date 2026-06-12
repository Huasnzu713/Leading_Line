#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
from pathlib import Path

# 包根（scripts/ 的上一层），README 要求把 vehicle/ 和 protocol/ 软链到这里。
_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import rospy  # noqa: E402

# 先建 ROS 上下文，RosBridge(backend="ros") 才能注册到正确节点名。
rospy.init_node("leading_line_pipeline", anonymous=False, log_level=rospy.INFO)

# roslaunch 会在 sys.argv 里塞 __name:=... __log:=... 之类，argparse 不认；
# 用 rospy.myargv 把这些过滤掉，剩下用户传的 --config / --log-level 给 main()。
sys.argv = rospy.myargv(argv=sys.argv)

from vehicle.main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
