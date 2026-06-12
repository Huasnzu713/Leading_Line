# -*- coding: utf-8 -*-
"""引导线算法核心：颜色分割 → 路径规划 → 控制 → 渲染。

全部在 Jetson 端运行。`main.py`（单机调试）和 `vehicle/pipeline.py`
（双端模式）都从这里导入。
"""
