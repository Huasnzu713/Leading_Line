"""端到端：生成 QR → 解码 → 状态机 → 校验输出。

依赖 qr_make_test.py 先生成 tests/qr_state_machine_samples/turn_left.png 等样本；
如果文件不存在，会自己调一次生成（这样跑测试不需要先手动生成）。
"""
import sys
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2

from qr_decoder import decode_qr_codes
from qr_state_machine import QRStateMachine


SAMPLES_DIR = ROOT.parent / "tests" / "qr_state_machine_samples"


def _ensure_samples():
    """若样本不存在，跑一次 qr_make_test.py 生成。"""
    need = ["stop.png", "turn_left.png", "turn_right.png", "slow_down.png", "cruise.png", "custom.png"]
    if not all((SAMPLES_DIR / n).exists() for n in need):
        print("[setup] 生成 QR 样本到", SAMPLES_DIR)
        subprocess.run(
            [sys.executable, str(ROOT / "qr_make_test.py"), "--out", str(SAMPLES_DIR)],
            check=True,
        )


def test_decode_then_run_state_machine(case_name: str, expected_policy: str, expected_steer: float, expected_speed: float):
    _ensure_samples()
    img_path = SAMPLES_DIR / f"{case_name}.png"
    assert img_path.exists(), f"样本缺失：{img_path}"

    img = cv2.imread(str(img_path))
    assert img is not None

    decoded = decode_qr_codes(img)
    assert len(decoded) >= 1, f"未识别出 QR: {case_name}"
    sm = QRStateMachine()
    sm.start()
    sm.on_qr_decoded(decoded[0].text)
    # 第一帧：DECODED → POLICY_ACTIVE
    steer, speed = sm.tick(0.1)
    assert sm.last_policy is not None
    assert sm.last_policy.name == expected_policy, f"期望 {expected_policy}，实际 {sm.last_policy.name}"
    assert abs(steer - expected_steer) < 0.01, f"steer {steer} ≠ {expected_steer}"
    assert abs(speed - expected_speed) < 0.01, f"speed {speed} ≠ {expected_speed}"
    print(f"  ok: {case_name} → policy={expected_policy} steer={steer} speed={speed}")


def main():
    cases = [
        ("stop",       "STOP",       0.0, 0.0),
        ("turn_left",  "TURN_LEFT", -20.0, 0.20),
        ("turn_right", "TURN_RIGHT", 20.0, 0.20),
        ("slow_down",  "SLOW_DOWN",   0.0, 0.10),
    ]
    failed = 0
    for case, policy, steer, speed in cases:
        try:
            test_decode_then_run_state_machine(case, policy, steer, speed)
        except Exception as e:
            print(f"  FAIL: {case}: {e}")
            failed += 1
    print(f"--- {len(cases) - failed}/{len(cases)} 通过 ---")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
