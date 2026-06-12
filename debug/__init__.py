"""调试工具集合。

本目录里的脚本都是为开发 / 调参 / 排错准备的**单文件入口**，
不属于产品运行时。算法本体和 PC/vehicle 双端架构本身不需要这些
也能跑。

使用方式：::

    python debug/algo_preview.py --source tests/data/synth.png
    python debug/arrow_image.py tests/data/arrow/arrow_up.png
    python debug/qr_preview.py --mode test --source tests/data/qr/qr_state_machine_samples/turn_left.png

详见 debug/README.md。
"""
