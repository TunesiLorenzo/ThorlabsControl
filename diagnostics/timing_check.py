"""
Live timing diagnostic for the move -> burst-read pipeline used by
scan_engine.run_scan.

Runs N single-line moves on the X axis with a real Arduino burst read
attached, using the same MoveStartLagMonitor + burst_read_binary machinery
as the actual scan loop, and reports per trial:

- command issue -> burst-read-loop-start delay (the dead time between
  telling the motor to move and the first sample actually being read),
- the real motor-start lag (position/status-bit based, not assumed),
- the real motor-stop time (when the moving status bit actually cleared),
- when the burst read loop actually stopped,
- and the delta between the two: burst-stop - motor-stop.

burst_read_binary's stop_condition (see ArduinoSampler.burst_read_binary)
is supposed to keep reading past the nominal duration until the motor has
actually stopped, so that delta should always be >= 0. A negative delta
means the row's trailing samples were lost (burst ended before the motor
did); a motor-stop that's never detected means the row hit hard_timeout
before the monitor ever saw the motor stop. Both are real timing bugs
worth investigating, not just modeling slop.

Needs the stage (X axis) and Arduino connected (same hardware as
scan_engine.py/scan_gui.py). Edit the constants below and run:

    python diagnostics/timing_check.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ArduinoSampler import burst_read_binary, close_arduino, open_arduino
from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController
from motion_timing import (
    MoveStartLagMonitor,
    expected_move_time,
    sampling_duration_for_move,
    summarize_latencies,
)

# Hardware
SERIAL = "50865380"
CHANNEL = 1
ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 230400

# Motion, real units (mm, mm/s, mm/s^2)
ACCELERATION = 4.0
MAX_VELOCITY = 4.0
X0 = 1.0
X_SPAN = 1.0

# Acquisition (mirrors scan_engine.run_scan defaults)
READ_SAFETY_FACTOR = 1.10
READ_OVERHEAD_S = 0.05
MOVE_START_LAG_THRESHOLD_MM = 0.0000005  # 0.5 nm; must exceed quantization noise
LAG_MONITOR_TIMEOUT_S = 2.0

NUM_TRIALS = 20
ROW_SETTLE_S = 0.5
STOP_DELTA_WARN_S = 0.0  # flag any trial where the burst stopped before the motor did

SKIP_HOMING_CHECK = True
HOME_TIMEOUT_S = 30.0


def require_homed(motor):
    motor.request_update()
    if not motor.get_status_bits() & 0x00000400:
        raise ThorlabsError(
            "X axis is not homed. Set SKIP_HOMING_CHECK=False or home the "
            "stage before running this check."
        )


def run_trial(motorx, ser, trial_index, x_displacement, move_time, read_duration):
    actual_x_start = motorx.get_position(real_unit=True)
    ser.reset_input_buffer()

    row_wait_timeout_s = max(read_duration + 2.0, move_time * 2.0 + 2.0)
    lag_monitor = MoveStartLagMonitor(
        motorx,
        pos0=actual_x_start,
        position_threshold_mm=MOVE_START_LAG_THRESHOLD_MM,
        timeout_s=row_wait_timeout_s,
        track_full_trace=False,
        watch_for_stop=True,
    ).start()

    command_issue_s = time.perf_counter()
    motorx.move_relative(x_displacement, wait=False, real_unit=True)

    samples, read_timing = burst_read_binary(
        ser=ser,
        duration=read_duration,
        reset_buffer=False,
        return_timing=True,
        stop_condition=lag_monitor.has_stopped,
        hard_timeout=row_wait_timeout_s,
    )

    motorx.wait_until_stopped(timeout_s=row_wait_timeout_s, require_motion_seen=False)
    actual_x_end = motorx.get_position(real_unit=True)

    lag_monitor.stop()
    lag_monitor.join(timeout=LAG_MONITOR_TIMEOUT_S)
    lag_result = lag_monitor.result(command_issue_s)

    burst_start_s = read_timing["start_time"] - command_issue_s
    burst_stop_s = read_timing["end_time"] - command_issue_s

    motor_start_lag_s = None if lag_result is None else lag_result["lag_midpoint_s"]
    motor_stop_lag_s = None if lag_result is None else lag_result["motion_stopped_lag_s"]
    stop_delta_s = None if motor_stop_lag_s is None else burst_stop_s - motor_stop_lag_s

    issues = []
    if lag_result is None:
        issues.append("motor start was never detected (command->burst latency unknown)")
    if lag_result is not None and motor_stop_lag_s is None:
        issues.append("motor stop was never detected before hard_timeout")
    elif stop_delta_s is not None and stop_delta_s < STOP_DELTA_WARN_S:
        issues.append(
            f"burst read stopped {abs(stop_delta_s) * 1000:.2f} ms BEFORE the motor "
            "actually stopped -- row's trailing samples were lost"
        )

    def fmt(value_s, sign=False):
        if value_s is None:
            return "    n/a  "
        return f"{value_s * 1000:+8.2f} ms" if sign else f"{value_s * 1000:8.2f} ms"

    print(
        f"trial {trial_index + 1:2d}: "
        f"cmd->burst-start={fmt(burst_start_s)} | "
        f"motor-start lag={fmt(motor_start_lag_s)} | "
        f"motor-stop={fmt(motor_stop_lag_s)} | "
        f"burst-stop={fmt(burst_stop_s)} | "
        f"stop_delta={fmt(stop_delta_s, sign=True)}"
        + ("  <-- " + "; ".join(issues) if issues else "")
    )

    return {
        "samples": len(samples),
        "x_start": actual_x_start,
        "x_end": actual_x_end,
        "burst_start_s": burst_start_s,
        "burst_stop_s": burst_stop_s,
        "motor_start_lag_s": motor_start_lag_s,
        "motor_stop_lag_s": motor_stop_lag_s,
        "stop_delta_s": stop_delta_s,
        "issues": issues,
    }


def main():
    motorx = ThorlabsModularStepperController(serial=SERIAL, channel=CHANNEL, poll_ms=1)
    ser = None
    failed = True
    results = []

    try:
        motorx.connect()
        if SKIP_HOMING_CHECK:
            require_homed(motorx)
        else:
            print("Homing X...")
            motorx.home(wait=True, timeout_s=HOME_TIMEOUT_S)

        motorx.set_velocity_params(
            acceleration=ACCELERATION, max_velocity=MAX_VELOCITY, real_unit=True
        )
        motorx.move_absolute(X0 - X_SPAN / 2.0, wait=True, real_unit=True)

        ser = open_arduino(port=ARDUINO_PORT, baud=ARDUINO_BAUD)

        move_time = expected_move_time(X_SPAN, MAX_VELOCITY, ACCELERATION)
        read_duration = sampling_duration_for_move(
            distance=X_SPAN,
            max_velocity=MAX_VELOCITY,
            acceleration=ACCELERATION,
            safety_factor=READ_SAFETY_FACTOR,
            overhead_s=READ_OVERHEAD_S,
        )
        print(
            f"{NUM_TRIALS} trial(s); expected move time {move_time * 1000:.1f} ms, "
            f"nominal read duration {read_duration * 1000:.1f} ms.\n"
        )

        for trial_index in range(NUM_TRIALS):
            direction = 1.0 if trial_index % 2 == 0 else -1.0
            results.append(
                run_trial(motorx, ser, trial_index, direction * X_SPAN, move_time, read_duration)
            )
            if trial_index < NUM_TRIALS - 1:
                time.sleep(ROW_SETTLE_S)

        failed = False
    finally:
        if ser is not None:
            close_arduino(ser)
        if failed:
            motorx.safe_shutdown()
        else:
            motorx.disconnect()

    burst_starts = [r["burst_start_s"] for r in results]
    motor_start_lags = [r["motor_start_lag_s"] for r in results if r["motor_start_lag_s"] is not None]
    stop_deltas = [r["stop_delta_s"] for r in results if r["stop_delta_s"] is not None]
    flagged = [(i, r["issues"]) for i, r in enumerate(results) if r["issues"]]

    print("\n" + "=" * 72)
    print("SUMMARY")
    print("=" * 72)
    print(summarize_latencies("command -> burst-read-loop start", burst_starts))
    if motor_start_lags:
        print(summarize_latencies("real motor-start lag", motor_start_lags))
    if stop_deltas:
        print(summarize_latencies("burst-stop - motor-stop (should be >= 0)", stop_deltas))

    if flagged:
        print(f"\n{len(flagged)}/{len(results)} trial(s) flagged:")
        for i, issues in flagged:
            for issue in issues:
                print(f"  trial {i + 1}: {issue}")
    else:
        print(f"\nAll {len(results)} trial(s) clean: burst read always ran until (or past) motor stop.")


if __name__ == "__main__":
    main()
