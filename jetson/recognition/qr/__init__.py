"""二维码识别 + 状态机驱动的策略系统。

- `qr_decoder.decode_qr_codes(frame)` 返回识别出的 DecodedQR 列表
- `qr_state_machine.QRStateMachine` 解析 + 状态转移 + 输出 (steer, speed)
- `qr_policy.parse_policy(text)` 解析策略文本

被 `jetson.overrides.FrameOverrides` 调用。
"""
