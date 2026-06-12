# debug/ — 调试工具集

把散落的"单文件调试入口"集中到这里，方便新同学上手时一眼找到。

> 这些脚本**不属于**产品运行时。正式部署用的是：
> - PC：`python pc/main.py --config config_pc.yaml`
> - 车辆：`python vehicle/main.py --config config.yaml`
> - ROS 1：见 `ros_pkgs/leading_line/launch/`

## 1. `algo_preview.py` — 单机算法预览

等价于在 PC 端能看到效果但不需要 Qt、不需要联网。读一张图 / 视频 / 摄像头，
跑路径算法 + 渲染，按 ESC 退出。

```bash
# 跑一张合成图（最常用：检查算法对 sample 输入的输出）
python debug/algo_preview.py --source tests/data/synth.png

# 跑一段视频
python debug/algo_preview.py --source path/to/video.mp4

# 摄像头
python debug/algo_preview.py

# 调试模式：2x2 网格（原图 / 道路掩码 / 地面掩码 / 结果）
python debug/algo_preview.py --source tests/data/synth.png --debug

# 离线结果：跑完一张图会自动写到 test_result/<原名>_result.<ext>
```

可视化窗口按键：
- **ESC**：退出
- **s**：保存当前帧到 `test_result/snapshot.png`
- **d**：切换调试模式

## 2. `arrow_image.py` — 离线箭头识别

```bash
# 单张图
python debug/arrow_image.py tests/data/arrow/arrow_up.png

# 批量（支持通配符）
python debug/arrow_image.py "tests/data/arrow/arrow_*.png" --save-dir out/

# 弹窗显示
python debug/arrow_image.py tests/data/arrow/arrow_up.png --show
```

输出形如：`arrow_up.png: 方向=前   角度= 90.0°  置信度=0.93`

## 3. `arrow_webcam.py` — 实时摄像头箭头识别

```bash
python debug/arrow_webcam.py                 # 默认 0 号摄像头
python debug/arrow_webcam.py --camera 1      # 选 1 号
python debug/arrow_webcam.py --min-conf 0.3  # 提高置信度门槛
```

按 `q` 退出；按 `s` 保存当前帧。

## 4. `arrow_samples.py` — 生成箭头测试样本

按固定 seed=42 生成 12 张样本（4 方向 × {干净, 噪声, 旋转} + 3 张彩色反例）。

```bash
python debug/arrow_samples.py                       # 默认写到 tests/data/arrow/
python debug/arrow_samples.py --out /tmp/arrows     # 自定义目录
python debug/arrow_samples.py --size 500            # 自定义图片边长
```

## 5. `qr_preview.py` — QR 状态机调试

两种模式：

```bash
# 摄像头实时：扫二维码 → 状态机 → HUD 上画 (state, steer, speed)
python debug/qr_preview.py --mode camera

# 离线：读一张图，识别一次，喂给状态机，把结果画到 <name>_decoded.png
python debug/qr_preview.py --mode test --source tests/data/qr/qr_state_machine_samples/turn_left.png
```

参数从 `config.yaml` 的 `debug.qr_preview` 段读：
- `policy_timeout_s`：策略超时（默认 30.0）
- `test_ticks`：test 模式跑几轮 tick（默认 3）
- `camera` / `ui` 段：摄像头设备号、窗口名、按键

按 ESC 退出；按 `s` 保存当前帧到 `qr_result/`。

## 6. `qr_samples.py` — 生成 QR 策略样本

```bash
python debug/qr_samples.py                            # 默认写到 tests/data/qr/qr_state_machine_samples/
python debug/qr_samples.py --out /tmp/qrs             # 自定义目录
```

覆盖一组策略用例：`STOP` / `TURN_LEFT` / `TURN_RIGHT` / `SLOW_DOWN` / `CRUISE` / `CUSTOM`。

`tests/unit/test_qr_e2e.py` 跑不通时会自动调它补样本，**日常不需要手动跑**。
