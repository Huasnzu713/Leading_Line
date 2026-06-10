"""PC 端 Qt 主窗口。

布局::

    ┌────────────────────────────────────────────────────────────┐
    │  标题：Leading Line — 监控端                                │
    ├──────────────────────────────────────┬─────────────────────┤
    │                                      │  模式（单选）         │
    │                                      │  ◯ 蓝色路径模式       │
    │           视频画面（QLabel）           │  ◯ 绿色路径模式       │
    │                                      │  ◯ 测试模式          │
    │                                      │                     │
    │                                      │  [ 开始 ]  [ 结束 ]  │
    │                                      │                     │
    │                                      │  状态：未连接         │
    │                                      │  帧率：-- FPS         │
    │                                      │  丢包：0              │
    └──────────────────────────────────────┴─────────────────────┘

线程模型：
- UI 线程：所有 Qt 事件 + QTimer（30Hz）拉 latest_frame() 刷新画面
- 视频接收在 daemon 线程（VideoReceiver 内部）
- 命令发送在 daemon 线程（CommandSender 内部）
- 线程间通过 Qt signal/slot 通信，跨线程安全
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from protocol import ALL_MODES, list_modes
from pc.comm.command_sender import CommandSender
from pc.comm.video_receiver import VideoReceiver
from .widgets import frame_to_pixmap_scaled, make_placeholder_pixmap

log = logging.getLogger("pc.ui")


# --------------------------------------------------------------------------
# 单选模式组
# --------------------------------------------------------------------------

class ModeGroup(QGroupBox):
    """模式单选组：初始化时根据 cfg["modes"] 动态生成选项。"""

    mode_selected = Signal(str)   # 选中新模式时发出模式名

    def __init__(self, cfg: dict, parent: Optional[QWidget] = None) -> None:
        super().__init__("模式", parent)
        self._cfg = cfg
        self._buttons: dict[str, QRadioButton] = {}
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._build()

    def _build(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 14, 10, 10)
        layout.setSpacing(6)
        modes = list_modes(self._cfg)
        if not modes:
            # 兜底：用 ALL_MODES 硬列
            modes = [{"name": m, "label": m} for m in ALL_MODES]
        for m in modes:
            rb = QRadioButton(m["label"])
            rb.setProperty("mode_name", m["name"])
            rb.toggled.connect(self._on_toggled)
            self._buttons[m["name"]] = rb
            self._group.addButton(rb)
            layout.addWidget(rb)
        # 默认选第一个
        first = next(iter(self._buttons.values()), None)
        if first:
            first.setChecked(True)
        layout.addStretch(1)

    def _on_toggled(self, checked: bool) -> None:
        if not checked:
            return
        btn = self.sender()
        name = btn.property("mode_name") if btn else None
        if name:
            log.info("UI 模式选择: %s", name)
            self.mode_selected.emit(name)

    def select(self, mode_name: str) -> None:
        btn = self._buttons.get(mode_name)
        if btn and not btn.isChecked():
            btn.setChecked(True)


# --------------------------------------------------------------------------
# 视频显示控件
# --------------------------------------------------------------------------

class VideoView(QFrame):
    """带边框的视频显示区，内部用 QLabel 显示缩放后的画面。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("background-color: #1e1e1e;")
        self._label = QLabel(self)
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._label.setMinimumSize(640, 480)
        self._label.setStyleSheet("color: #cccccc;")
        self._show_placeholder()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._label)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._label.pixmap() is None or self._label.pixmap().isNull():
            self._show_placeholder()
        else:
            # 重绘当前 pixmap 到新尺寸
            cur = self._last_frame
            if cur is not None:
                self.update_frame(cur)

    def _show_placeholder(self) -> None:
        pm = make_placeholder_pixmap(self._label.width() or 640,
                                     self._label.height() or 480)
        self._label.setPixmap(pm)
        self._last_frame = None

    def update_frame(self, frame_bgr: np.ndarray) -> None:
        self._last_frame = frame_bgr
        pm = frame_to_pixmap_scaled(frame_bgr, self._label.width(), self._label.height())
        self._label.setPixmap(pm)


# --------------------------------------------------------------------------
# 主窗口
# --------------------------------------------------------------------------

class MainWindow(QMainWindow):
    """Qt 主窗口：左视频 / 右菜单 / 底部状态栏。"""

    # 跨线程信号（视频/命令线程 → UI 线程）
    status_signal = Signal(str, str)   # kind, payload
    frame_signal = Signal(object)      # ndarray

    def __init__(
        self,
        cfg: dict,
        video_receiver: VideoReceiver,
        command_sender: CommandSender,
    ) -> None:
        super().__init__()
        self.cfg = cfg
        self.video = video_receiver
        self.sender = command_sender
        self._current_mode: Optional[str] = None
        self._build_ui()
        self._wire_signals()
        # 30Hz 拉帧
        self._timer = QTimer(self)
        self._timer.setInterval(int(1000 / 30))
        self._timer.timeout.connect(self._poll_frame)
        self._timer.start()
        # 状态文字刷新（每秒一次）
        self._stats_timer = QTimer(self)
        self._stats_timer.setInterval(1000)
        self._stats_timer.timeout.connect(self._update_stats)
        self._stats_timer.start()
        self.setWindowTitle("Leading Line — 监控端")
        self.resize(1280, 720)

    # ----- UI 构建 -----

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # 左侧：视频
        self.video_view = VideoView()
        root.addWidget(self.video_view, stretch=4)

        # 右侧：菜单
        sidebar = QVBoxLayout()
        sidebar.setSpacing(10)

        # 模式组
        self.mode_group = ModeGroup(self.cfg)
        self.mode_group.mode_selected.connect(self._on_mode_selected)
        sidebar.addWidget(self.mode_group)

        # 控制按钮组
        ctrl_box = QGroupBox("控制")
        ctrl_layout = QVBoxLayout(ctrl_box)
        ctrl_layout.setContentsMargins(10, 14, 10, 10)
        ctrl_layout.setSpacing(8)
        self.btn_start = QPushButton("开始")
        self.btn_start.setMinimumHeight(40)
        self.btn_start.setStyleSheet("font-weight: bold; background: #2e7d32; color: white;")
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_end = QPushButton("结束")
        self.btn_end.setMinimumHeight(40)
        self.btn_end.setStyleSheet("font-weight: bold; background: #c62828; color: white;")
        self.btn_end.clicked.connect(self._on_end_clicked)
        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_end)
        sidebar.addWidget(ctrl_box)

        # 状态显示
        info_box = QGroupBox("状态")
        info_layout = QVBoxLayout(info_box)
        info_layout.setContentsMargins(10, 14, 10, 10)
        info_layout.setSpacing(4)
        self.lbl_connection = QLabel("连接：未连接")
        self.lbl_state = QLabel("小车状态：未知")
        self.lbl_mode = QLabel("当前模式：—")
        self.lbl_fps = QLabel("入站帧率：-- FPS")
        self.lbl_packets = QLabel("入站/丢包：0 / 0")
        for w in (self.lbl_connection, self.lbl_state, self.lbl_mode, self.lbl_fps, self.lbl_packets):
            w.setFont(QFont("Microsoft YaHei", 9))
            info_layout.addWidget(w)
        sidebar.addWidget(info_box)
        sidebar.addStretch(1)

        root.addLayout(sidebar, stretch=1)

        # 底部状态栏
        self.setStatusBar(QStatusBar(self))
        self.statusBar().showMessage("就绪")

    # ----- 信号桥接 -----

    def _wire_signals(self) -> None:
        # 命令线程 → UI 线程（跨线程 signal）
        self.status_signal.connect(self._on_status_event, Qt.QueuedConnection)
        # 把 sender 的回调也接上
        self.sender.start(on_status=lambda k, p: self.status_signal.emit(k, p))

    # ----- 定时器回调 -----

    def _poll_frame(self) -> None:
        frame = self.video.latest_frame()
        if frame is not None:
            self.video_view.update_frame(frame)

    def _update_stats(self) -> None:
        s = self.video.stats()
        self.lbl_fps.setText(f"入站帧率：{s['fps_in']:.1f} FPS")
        self.lbl_packets.setText(f"入站/丢包：{s['decoded']} / {s['drops']}")
        # 心跳 / 状态
        cs = self.sender.stats()
        if cs["connected"]:
            rtt = cs["rtt_ms"]
            rtt_txt = f"延迟：{rtt:.0f} ms" if rtt >= 0 else "延迟：—"
            self.lbl_connection.setText(f"连接：已连 Jetson  |  {rtt_txt}")
            self.lbl_connection.setStyleSheet("color: #2e7d32;")
        else:
            self.lbl_connection.setText("连接：未连接（等待重连）")
            self.lbl_connection.setStyleSheet("color: #c62828;")
        if cs["status_text"]:
            self.lbl_state.setText(f"小车状态：{cs['status_text']}")

    # ----- UI 事件 -----

    def _on_mode_selected(self, mode_name: str) -> None:
        self._current_mode = mode_name
        self.lbl_mode.setText(f"当前模式：{mode_name}")
        if self.sender.is_connected():
            self.sender.send_mode(mode_name)
        else:
            self.statusBar().showMessage(f"模式 {mode_name} 已选，但未连 Jetson，发不出去", 3000)

    def _on_start_clicked(self) -> None:
        if not self.sender.is_connected():
            self.statusBar().showMessage("未连 Jetson，开始命令无法发送", 3000)
            return
        self.sender.send_start()
        self.statusBar().showMessage("已发送 START", 2000)

    def _on_end_clicked(self) -> None:
        if not self.sender.is_connected():
            self.statusBar().showMessage("未连 Jetson，结束命令无法发送", 3000)
            return
        self.sender.send_stop()
        self.statusBar().showMessage("已发送 STOP", 2000)

    def _on_status_event(self, kind: str, payload: str) -> None:
        kind = kind.upper()
        if kind == "STATUS":
            # Jetson 推回的格式: "state=... mode=... fps=... ros=... clients=..."
            # 解析成简短的本地化展示
            short = self._shorten_status(payload)
            self.lbl_state.setText(f"小车状态：{short}")
        elif kind == "INFO":
            self.statusBar().showMessage(f"Jetson: {payload}", 3000)
        elif kind == "ACK":
            self.statusBar().showMessage(f"ACK: {payload}", 1500)
        elif kind == "PONG":
            # 静默；RTT 在 _update_stats 里通过 sender.stats() 读
            pass
        elif kind == "CONNECTED":
            self.statusBar().showMessage(f"已连 Jetson: {payload}", 2000)
        elif kind == "DISCONNECTED":
            self.lbl_state.setText("小车状态：未连接")
        else:
            self.statusBar().showMessage(f"{kind}: {payload}", 2000)

    @staticmethod
    def _shorten_status(payload: str) -> str:
        """把 "state=IDLE mode=blue_path fps=28.5 ros=mock clients=1" 变成 "IDLE 模式 blue_path  28.5 FPS  ROS=mock"."""
        out: list[str] = []
        for part in payload.split():
            if "=" not in part:
                continue
            k, _, v = part.partition("=")
            if k == "state":
                out.append(f"{v}")
            elif k == "mode":
                out.append(f"模式 {v}")
            elif k == "fps":
                out.append(f"{v} FPS")
            elif k == "ros":
                out.append(f"ROS={v}")
        return "  |  ".join(out) if out else payload

    # ----- 关闭事件 -----

    def closeEvent(self, event) -> None:
        log.info("窗口关闭，停止 timer / 通信")
        try:
            self._timer.stop()
            self._stats_timer.stop()
            # 尽量告知 Jetson 一声
            try:
                self.sender.send_stop()
            except Exception:  # noqa: BLE001
                pass
        finally:
            super().closeEvent(event)
