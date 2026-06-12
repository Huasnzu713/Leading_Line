# -*- coding: utf-8 -*-
"""单元测试集。

- test_algorithm.py       : 引导线算法核心 + 鲁棒性（合并原 8 个测试）
- test_qr_policy.py       : QR 策略文本解析
- test_qr_state_machine.py: QR 状态机
- test_qr_e2e.py          : QR 解码 + 状态机端到端
- test_comm.py            : PC ↔ 车辆 socket 双端联通

运行方式（不依赖 pytest）::

    python tests/unit/test_algorithm.py
    python tests/unit/test_qr_policy.py
    python tests/unit/test_qr_state_machine.py
    python tests/unit/test_qr_e2e.py
    python tests/unit/test_comm.py
"""
