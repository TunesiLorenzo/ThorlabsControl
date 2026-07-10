import math
import time

import matplotlib.pyplot as plt
import numpy as np

from ArduinoSampler import close_arduino, open_arduino, burst_read_binary
from motion_timing import (
    MoveStartLagMonitor,
    expected_move_time,
    sample_positions_for_motion,
    sampling_duration_for_move,
)
from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController


def build_axis_points(start: float, span: float, spacing: float):
    if spacing <= 0:
        raise ValueError("spacing must be positive")

    start = float(start)
    span = float(span)
    total = abs(span)
    if total == 0:
        return [start]

    direction = 1.0 if span >= 0 else -1.0
    steps = int(math.floor(total / spacing + 1e-9))
    points = [start + direction * i * spacing for i in range(steps + 1)]
    end = start + span

    if abs(points[-1] - end) > 1e-9:
        points.append(end)

    return points


def load_scan(path):
    """Reload scan_rows saved by the __main__ scan loop's np.savez() dump."""
    data = np.load(path, allow_pickle=True)
    return data["scan_rows"].tolist()


def plot_scan(scan_rows):
    plotted_rows = 0
    fig, ax = plt.subplots()
    color_map = plt.get_cmap("viridis")
    color_count = max(1, len(scan_rows) - 1)

    for row_index, row in enumerate(scan_rows):
        samples = row["samples"]
        if not samples:
            continue

        row_xs = sample_positions_for_motion(
            x_start=row["x_start"],
            displacement=row["x_displacement"],
            sample_count=len(samples),
            read_duration_s=row.get("sample_span_s", row["read_duration"]),
            max_velocity=row["max_velocity"],
            acceleration=row["acceleration"],
            motion_start_offset_s=row.get("motion_start_offset_s", 0.0),
        )

        ax.plot(
            row_xs,
            samples,
            linewidth=1.0,
            color=color_map(row_index / color_count),
            label=f"y={row['y']:.3f}",
        )
        plotted_rows += 1

    if plotted_rows == 0:
        print("No samples collected; skipping scan plot.")
        return

    ax.set_xlabel("X position")
    ax.set_ylabel("ADC count")
    ax.set_title("Scanned Lines")
    ax.grid(True)

    if plotted_rows <= 20:
        ax.legend(title="Row", fontsize="small", ncols=2)

    fig.tight_layout()
    plt.show()


def check_connection_and_home(motorx, motory, home_timeout_s: float = 30.0):
    print("Checking motor connections...")
    for label, motor in (("X", motorx), ("Y", motory)):
        motor.request_update()
        status = motor.get_status_bits()
        position = motor.get_position(real_unit=True)
        print(
            f"{label} connected: "
            f"status=0x{status:08X}, "
            f"position={position:.6f}, "
            f"poll={motor.get_polling_duration()} ms"
        )

    print("Homing motors...")
    for label, motor in (("X", motorx), ("Y", motory)):
        print(f"Homing {label} axis...")
        motor.home(wait=True, timeout_s=home_timeout_s)
        motor.request_update()

        status = motor.get_status_bits()
        if not status & 0x00000400:
            raise ThorlabsError(f"{label} axis did not report homed")

        print(f"{label} homed at position {motor.get_position(real_unit=True):.6f}")

    print("Connection and homing check complete.")


def print_scan_timing_stats(scan_rows):
    if not scan_rows:
        return

    move_to_read_delays = [
        row["move_to_read_start_s"]
        for row in scan_rows
        if "move_to_read_start_s" in row
    ]
    command_call_times = [
        row["move_command_call_s"]
        for row in scan_rows
        if "move_command_call_s" in row
    ]
    return_to_read_delays = [
        row["move_return_to_read_start_s"]
        for row in scan_rows
        if "move_return_to_read_start_s" in row
    ]
    sample_spans = [
        row["sample_span_s"]
        for row in scan_rows
        if "sample_span_s" in row
    ]
    motion_start_offsets = [
        row["motion_start_offset_s"]
        for row in scan_rows
        if "motion_start_offset_s" in row
    ]
    measured_lag_lower = [
        row["measured_lag_lower_s"]
        for row in scan_rows
        if row.get("measured_lag_lower_s") is not None
    ]
    measured_lag_upper = [
        row["measured_lag_upper_s"]
        for row in scan_rows
        if row.get("measured_lag_upper_s") is not None
    ]
    measured_lag_midpoint = [
        row["measured_lag_midpoint_s"]
        for row in scan_rows
        if row.get("measured_lag_midpoint_s") is not None
    ]
    measured_moving_bit_lag = [
        row["measured_moving_bit_lag_s"]
        for row in scan_rows
        if row.get("measured_moving_bit_lag_s") is not None
    ]
    n_lag_missed = sum(
        1 for row in scan_rows if row.get("measured_lag_midpoint_s") is None
    )

    def print_stats(label, values):
        if not values:
            return

        avg = sum(values) / len(values)
        print(
            f"{label}: "
            f"avg={avg * 1000:.2f} ms, "
            f"min={min(values) * 1000:.2f} ms, "
            f"max={max(values) * 1000:.2f} ms"
        )

    print("Timing summary:")
    print_stats("Move command issue -> burst read loop start", move_to_read_delays)
    print_stats("Move command return -> burst read loop start", return_to_read_delays)
    print_stats("move_relative() call duration", command_call_times)
    print_stats("Plot compensation offset (measured lag midpoint, used for X projection)", motion_start_offsets)
    print_stats("Buffered sample span used for plotting", sample_spans)
    print_stats("Measured move-start lag, lower bound", measured_lag_lower)
    print_stats("Measured move-start lag, upper bound", measured_lag_upper)
    print_stats("Measured move-start lag, midpoint", measured_lag_midpoint)
    print_stats("Measured move-start lag, moving-bit", measured_moving_bit_lag)
    if n_lag_missed:
        print(
            f"Move-start lag was not detected on {n_lag_missed}/{len(scan_rows)} "
            "row(s); those rows fell back to command-return timing."
        )


if __name__ == "__main__":
    SERIAL = "50865380"
    ARDUINO_PORT = "COM3"
    ARDUINO_BAUD = 230400

    x0 = 1.0
    y0 = 1.0
    x_span = 1.0
    y_span = 0
    line_spacing = 0.1
    default_acceleration = 4.0
    default_max_velocity = 4.0
    row_settle_s = 2
    read_safety_factor = 1.10
    read_overhead_s = 0.05
    home_timeout_s = 30.0
    skip_homing_check = True
    # Position-based move-start-lag detection (see motion_timing.MoveStartLagMonitor).
    move_start_lag_threshold_mm = 0.0000005  # 0.5 nm; must exceed quantization noise
    lag_monitor_timeout_s = 2.0

    # Dump every row's raw samples + timing/lag metadata (including whatever
    # tail was sampled after the motor actually stopped) so they can be
    # re-plotted/re-analyzed without re-running the hardware.
    SAVE_FILE = "scan_last.npz"

    x_start = x0 - x_span / 2
    y_start = y0 - y_span / 2
    y_points = build_axis_points(y_start, y_span, line_spacing)
    scan_rows = []

    motorx = None
    motory = None
    ser = None
    scan_failed = True

    try:
        motorx = ThorlabsModularStepperController(
            serial=SERIAL,
            channel=1,
            poll_ms=1,
        )
        motory = ThorlabsModularStepperController(
            serial=SERIAL,
            channel=2,
            poll_ms=1,
        )
        motorx.connect()
        motory.connect()
        if skip_homing_check:
            print("Skipping motor connection/homing check.")
        else:
            check_connection_and_home(
                motorx=motorx,
                motory=motory,
                home_timeout_s=home_timeout_s,
            )

        motorx.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )
        motory.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )

        motorx.move_absolute(x_start, wait=True, real_unit=True)
        motory.move_absolute(y_start, wait=True, real_unit=True)

        ser = open_arduino(port=ARDUINO_PORT, baud=ARDUINO_BAUD)

        move_time = expected_move_time(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
        )
        read_duration = sampling_duration_for_move(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
            safety_factor=read_safety_factor,
            overhead_s=read_overhead_s,
        )

        print(
            f"Scanning {len(y_points)} rows; "
            f"expected X move time {move_time:.3f}s, "
            f"read duration {read_duration:.3f}s per row."
        )

        for row_index, y_target in enumerate(y_points):
            if row_index > 0:
                motory.move_absolute(y_target, wait=True, real_unit=True)
                time.sleep(row_settle_s)

            direction = 1.0 if row_index % 2 == 0 else -1.0
            x_displacement = direction * x_span
            actual_x_start = motorx.get_position(real_unit=True)
            actual_y = motory.get_position(real_unit=True)

            reset_done_s = time.perf_counter()
            ser.reset_input_buffer()
            sample_window_start_s = time.perf_counter()

            # Watches get_position()/status bits in the background while this
            # thread issues the move and then blocks on the Arduino read, so
            # we get the real per-row motion-start lag instead of assuming
            # command-issue/command-return timing.
            lag_monitor = MoveStartLagMonitor(
                motorx,
                pos0=actual_x_start,
                position_threshold_mm=move_start_lag_threshold_mm,
                timeout_s=lag_monitor_timeout_s,
            ).start()

            move_command_issue_s = time.perf_counter()
            motorx.move_relative(x_displacement, wait=False, real_unit=True)
            move_command_return_s = time.perf_counter()

            samples, read_timing = burst_read_binary(
                ser=ser,
                duration=read_duration,
                reset_buffer=False,
                return_timing=True,
            )
            sample_window_end_s = read_timing["end_time"]

            lag_monitor.join(timeout=lag_monitor_timeout_s)
            lag_result = lag_monitor.result(move_command_issue_s)
            if lag_result is not None:
                motion_start_s = move_command_issue_s + lag_result["lag_midpoint_s"]
            else:
                print(
                    f"Row {row_index}: move-start lag not detected within "
                    f"{lag_monitor_timeout_s:.1f}s; falling back to "
                    "command-return timing for the X projection."
                )
                motion_start_s = move_command_return_s
                lag_result = {}

            motorx.wait_until_stopped(
                timeout_s=max(read_duration + 2.0, move_time * 2.0 + 2.0),
                require_motion_seen=False,
            )
            actual_x_end = motorx.get_position(real_unit=True)

            scan_rows.append(
                {
                    "row": row_index,
                    "y": actual_y,
                    "x_start": actual_x_start,
                    "x_end": actual_x_end,
                    "x_displacement": x_displacement,
                    "motion_time": move_time,
                    "read_duration": read_duration,
                    "sample_span_s": sample_window_end_s - sample_window_start_s,
                    "motion_start_offset_s": motion_start_s - sample_window_start_s,
                    "move_to_read_start_s": (
                        read_timing["start_time"] - move_command_issue_s
                    ),
                    "move_return_to_read_start_s": (
                        read_timing["start_time"] - move_command_return_s
                    ),
                    "move_command_call_s": (
                        move_command_return_s - move_command_issue_s
                    ),
                    "serial_reset_s": sample_window_start_s - reset_done_s,
                    "measured_lag_lower_s": lag_result.get("lag_lower_bound_s"),
                    "measured_lag_upper_s": lag_result.get("lag_upper_bound_s"),
                    "measured_lag_midpoint_s": lag_result.get("lag_midpoint_s"),
                    "measured_moving_bit_lag_s": lag_result.get("moving_bit_lag_s"),
                    "acceleration": default_acceleration,
                    "max_velocity": default_max_velocity,
                    "samples": samples,
                }
            )
            lag_mid_s = scan_rows[-1]["measured_lag_midpoint_s"]
            lag_mid_str = "n/a" if lag_mid_s is None else f"{lag_mid_s * 1000:.2f} ms"
            print(
                f"Row {row_index + 1}/{len(y_points)}: "
                f"y={actual_y:.6f}, "
                f"x={actual_x_start:.6f}->{actual_x_end:.6f}, "
                f"samples={len(samples)}, "
                f"cmd->read={scan_rows[-1]['move_to_read_start_s'] * 1000:.2f} ms, "
                f"measured lag mid={lag_mid_str}"
            )

        print_scan_timing_stats(scan_rows)
        scan_failed = False

    finally:
        if ser is not None:
            close_arduino(ser)

        if scan_failed:
            if motorx is not None:
                motorx.safe_shutdown()
            if motory is not None:
                motory.safe_shutdown()
        else:
            if motorx is not None:
                motorx.disconnect()
            if motory is not None:
                motory.disconnect()

    if SAVE_FILE and scan_rows:
        np.savez(SAVE_FILE, scan_rows=np.array(scan_rows, dtype=object))
        print(f"Saved {len(scan_rows)} row(s) (raw samples + timing/lag metadata) to {SAVE_FILE}")

    plot_scan(scan_rows)
