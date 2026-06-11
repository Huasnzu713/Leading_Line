#!/usr/bin/env python3
"""roslaunch 包装器：以 ROS 节点身份拉起 jetson/main_jetson.py。

直接用 <node type="jetson/main_jetson.py"> 有两个坑：
  1. main_jetson.py 没 shebang，也不在 scripts/ 下，roslaunch 拉不动；
  2. RosBridge(backend="ros") 需要 rospy.init_node 已经调过，否则
     Publisher 注册到 /unnamed，调试时找不到节点。

本包装器解决这两点：
  a) 自己带 #!/usr/bin/env python3，装在 scripts/ 下供 roslaunch 找；
  b) 先 rospy.init_node("leading_line_pipeline") 建立 ROS 上下文；
  c) 过滤掉 roslaunch 注入的 __name:= / __log:= 等参数，再调主入口。

启动方式（被 leading_line_pc_monitor.launch 包起来）::
    rosrun leading_line run_jetson_pipeline.py \\
        --config $(find leading_line)/jetson/config.yaml --log-level INFO
"""
from __future__ import annotations

import sys
from pathlib import Path

# 包根（scripts/ 的上一层），README 要求把 jetson/ 和 protocol/ 软链到这里。
_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import rospy  # noqa: E402

# 先建 ROS 上下文，RosBridge(backend="ros") 才能注册到正确节点名。
rospy.init_node("leading_line_pipeline", anonymous=False, log_level=rospy.INFO)

# roslaunch 会在 sys.argv 里塞 __name:=... __log:=... 之类，argparse 不认；
# 用 rospy.myargv 把这些过滤掉，剩下用户传的 --config / --log-level 给 main()。
sys.argv = rospy.myargv(argv=sys.argv)

from jetson.main_jetson import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
