# QR 策略系统

用 QR 码驱动一个状态机。QR 文本 → 解析成 Policy → 状态机执行 → 输出 `(steer, speed)` 给下游。

## 文件

| 文件 | 作用 |
|---|---|
| `qr_decoder.py` | OpenCV `QRCodeDetector` 的薄包装，返回 `DecodedQR` 列表 |
| `qr_policy.py` | 文本解析（JSON / `key=value;` 两种格式）→ `Policy` 对象；策略白名单 |
| `qr_state_machine.py` | 显式状态 + 事件 + 转移表；tick(dt) 输出控制量 |
| `qr_main.py` | 入口；`--mode camera\|test` |
| `qr_config.yaml` | 摄像头 / 状态机 / UI 参数 |
| `qr_make_test.py` | 生成测试 QR 图 |

## 状态机

```
       START                 QR_FOUND                 TICK
IDLE ───────► SCANNING ────────────────► DECODED ─────► POLICY_ACTIVE
              │ ▲                              │                │
              │ │ QR_EMPTY                      │ TICK           │ duration 到 / TICK
              ▼ │                               ▼                ▼
             (stay)                          (stay)         REPORTING ──ACK──► IDLE
                                                                   │
                                                        POLICY_TIMEOUT
                                                                   │
                                                                   ▼
                                                                ERROR ──RESET──► IDLE
```

| 状态 | 行为 | 输出 steer/speed |
|---|---|---|
| `IDLE` | 等外部 START | (0, 0) |
| `SCANNING` | 持续吃 QR | (0, 0) |
| `DECODED` | 一帧后自动 → POLICY_ACTIVE | 取决于策略 |
| `POLICY_ACTIVE` | 按 duration_s 跑；到期 → REPORTING | 策略值 |
| `REPORTING` | 等 ACK | (0, 0) |
| `ERROR` | 停车，等 RESET | (0, 0) |

`CRUISE` 策略特殊：状态机输出 `(NaN, NaN)` 表示"解绑，由引导线算法接管"。

## 策略格式

**JSON**：

```json
{"policy":"TURN_LEFT","steer_deg":-20,"speed":0.20,"duration_s":1.5}
```

**简化 key=value**：

```
policy=TURN_LEFT;steer_deg=-20;speed=0.20;duration_s=1.5
```

**白名单**（`qr_policy.KNOWN_POLICIES`）：`STOP` / `CRUISE` / `TURN_LEFT` / `TURN_RIGHT` / `SLOW_DOWN` / `SPEED_UP` / `REVERSE` / `HOLD` / `CUSTOM`。

## 用法

```bash
# 安装依赖（与原项目一致）
pip install opencv-python PyYAML

# 生成测试 QR 图到 tests/qr_state_machine_samples/
python qr_make_test.py

# 模式 A：test（读图，离线）
python qr_main.py --mode test  --source tests/qr_state_machine_samples/turn_left.png

# 模式 B：camera（实时）
python qr_main.py --mode camera

# 模式 B 变体：拿一张图当摄像头（便于无摄像头环境调试）
python qr_main.py --mode camera --source tests/qr_state_machine_samples/turn_left.png
```

按 **ESC** 退出；按 **s** 把当前帧存到 `qr_result/snapshot_<ts>.png`。

## 与引导线算法对接

`qr_main.py` 当前是独立运行的 demo，状态机只输出 `(steer_deg, speed)`，不直接驱动小车。

接到实车时：
1. 在 `main.py` 的 `process_frame` 之前跑 `qr_state_machine.tick(dt)`，拿到 `(steer, speed)`。
2. 当 `(steer, speed)` 不是 NaN 时，**覆盖**算法本身算出的值推给下层。
3. 当是 NaN（`CRUISE`）时，正常走 `controller.decide(...)`。
4. `qr_decoder.decode_qr_codes` 可以加到摄像头帧读取之后、`process_frame` 之前。

或者把状态机输出的 `(steer, speed)` 通过 CAN / 串口 / MQTT 推给车体控制器，与 `controller.decide` 的输出互不干扰。
