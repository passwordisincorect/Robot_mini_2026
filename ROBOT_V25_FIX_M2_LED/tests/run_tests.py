"""Compile the actual .ino files with hardware stubs; no board or serial port access."""
from pathlib import Path
import argparse
import binascii
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "verification"
parser = argparse.ArgumentParser()
parser.add_argument("--compiler", default=shutil.which("g++"))
args = parser.parse_args()
if not args.compiler:
    parser.error("g++ not found; pass --compiler path/to/g++")
OUT.mkdir(exist_ok=True)
env = os.environ.copy()
env["PATH"] = str(Path(args.compiler).resolve().parent) + os.pathsep + env.get("PATH", "")
log = []

def run(command):
    result = subprocess.run([str(v) for v in command], text=True, capture_output=True, env=env)
    output = result.stdout + result.stderr
    print(output, end="")
    log.append("$ " + " ".join(str(v) for v in command) + "\n" + output)
    if result.returncode:
        raise RuntimeError(f"Command exited {result.returncode}")

try:
    for target in ("mega", "esp32", "yaw", "auto", "pid_v25"):
        exe = OUT / f"test_{target}.exe"
        run([args.compiler, "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-Wpedantic",
             "-I", ROOT / "tests/stubs", ROOT / f"tests/test_{target}.cpp", "-o", exe])
        cases = {
            "mega": ("boot", "protocol", "sequences", "interlocks", "pump_enable", "suction_leds", "r3", "limits", "auto_lift", "auto_lift_safety", "normal_brake",
                     "movement", "soft_brake", "relay_active_high", "four_directions", "observed_mapping", "mecanum_rotation", "speed", "rollover", "log_backpressure", "limit_diagnostics", "monitor_rate", "mpu_config", "mpu_axes", "mpu_errors", "mpu_control", "mpu_monitor", "mpu_rollover", "motor_polarity", "brake_interrupt", "brake_no_reverse", "m2_forward_polarity"),
            "esp32": ("boot", "mapping", "freshness", "disconnect", "backpressure", "rollover", "classification", "stale_recovery", "stale_rollover",
                      "monitor_buttons", "monitor_short_press", "monitor_joystick",
                      "monitor_rate", "monitor_backpressure", "monitor_reconnect", "monitor_link_state", "reset_diagnostics"),
            "yaw": ("calibration", "manual", "modes", "fault", "stale"),
            "auto": ("auto_start", "wait_controller", "limit_pwm", "off_persist", "r3", "uart", "sensor", "spin_guard", "sensor_absent"),
            "pid_v25": ("heading_hold", "once_per_sample", "antiwindup", "turn_reset", "neutral_reset", "low_speed", "sample_gap", "negative_sign", "manual_mode", "telemetry"),
        }[target]
        for name in cases:
            run([exe, name])
    wire = OUT / "bridge_wire.bin"
    run([OUT / "test_esp32.exe", "emit", wire])
    data = wire.read_bytes()
    assert len(data) % 14 == 0
    for start in range(0, len(data), 14):
        packet = data[start:start + 14]
        assert packet[:3] == bytes((0xA5, 0x5A, 1))
        assert int.from_bytes(packet[12:], "little") == binascii.crc_hqx(packet[2:12], 0xFFFF)
    assert data[-11] == 1
    assert int.from_bytes(data[-8:-6], "little") == 0xFFFB
    assert data[-6:-2] == bytes((0, 255, 128, 192))
    print("PASS independent Python CRC and wire-format check")
    log.append("PASS independent Python CRC and wire-format check\n")
    run([OUT / "test_mega.exe", "bridge", wire])
    print("PASS: 73 scenario groups + ESP32-to-Mega integration and independent CRC check.")
    log.append("PASS: 73 scenario groups + ESP32-to-Mega integration and independent CRC check.\n")
except Exception as exc:
    print(f"FAIL: {exc}", file=sys.stderr)
    log.append(f"FAIL: {exc}\n")
    sys.exit(1)
finally:
    (OUT / "native_tests.log").write_text("".join(log), encoding="utf-8")
