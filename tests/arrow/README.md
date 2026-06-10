# arrow 测试样本

箭头方向识别（`arrow_recongnize/` 模块）的示例图，固定 seed=42 由 `generate_samples.py` 生成。

| 命名 | 含义 |
|---|---|
| `arrow_<dir>.png` | 干净版（黑箭头，4 方向） |
| `arrow_<dir>_noisy.png` | 高斯噪声版（σ=30），验证抗噪 |
| `arrow_<dir>_rot<±deg>.png` | 小幅旋转版，验证扇区分类容差 |
| `arrow_<dir>_red.png` | 彩色反例：红色箭头，应被 `min_darkness` 过滤拒掉 |
| `arrow_<dir>_gray.png` | 彩色反例：浅灰箭头，应被 `min_darkness` 过滤拒掉 |

共 12 张（9 黑 + 3 彩色反例）。
