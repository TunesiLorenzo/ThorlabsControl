import math
import time

import matplotlib.pyplot as plt
import numpy as np

from ArduinoSampler import close_arduino, open_arduino, burst_read_binary
from motion_timing import (
    MoveStartLagMonitor,
    expected_move_time,
    interp_nan,
    sample_positions_for_motion,
    sampling_duration_for_move,
)
from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController

# Below this fraction of the commanded X span captured, run_scan() prints a
# per-row warning (see the coverage spot-check at the end of its row loop).
COVERAGE_WARN_THRESHOLD = 0.97


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


def _row_positions(row):
    """
    Map one row's ADC samples to X positions using the modeled trapezoidal
    profile (motion_timing.sample_positions_for_motion), anchored by the
    row's measured move-start lag (motion_start_offset_s) rather than a
    continuously polled position trace.
    """
    samples = row["samples"]
    if "x_positions" in row:
        return np.asarray(row["x_positions"], dtype=float)
    return np.asarray(
        sample_positions_for_motion(
            x_start=row["x_start"],
            displacement=row["x_displacement"],
            sample_count=len(samples),
            read_duration_s=row.get("sample_span_s", row["read_duration"]),
            max_velocity=row["max_velocity"],
            acceleration=row["acceleration"],
            motion_start_offset_s=row.get("motion_start_offset_s", 0.0),
        )
    )


def build_scan_grid(scan_rows, max_points=200):
    """
    Resample scan_rows onto a common rectangular (x, y) grid so they can be
    imaged as a heatmap or 3D surface. Both axes are capped at max_points by
    picking evenly spaced rows/columns, so large scans stay fast to render --
    this is display-only downsampling, the saved scan_rows keep every raw
    sample.

    Returns (x_grid, y_grid, z) with z.shape == (len(y_grid), len(x_grid)),
    NaN where a grid cell fell outside what was actually sampled.
    """
    rows_with_data = [row for row in scan_rows if len(row["samples"])]
    if not rows_with_data:
        return np.array([]), np.array([]), np.empty((0, 0))

    rows_with_data = sorted(rows_with_data, key=lambda row: row["y"])
    y_full = np.array([row["y"] for row in rows_with_data], dtype=float)

    x_min = min(min(row["x_start"], row["x_end"]) for row in rows_with_data)
    x_max = max(max(row["x_start"], row["x_end"]) for row in rows_with_data)
    n_cols = max(2, min(max_points, max(len(row["samples"]) for row in rows_with_data)))
    x_grid = np.linspace(x_min, x_max, n_cols)

    z_full = np.full((len(rows_with_data), len(x_grid)), np.nan)
    for row_index, row in enumerate(rows_with_data):
        row_xs = _row_positions(row)
        z_full[row_index], _ = interp_nan(row_xs, row["samples"], x_grid)

    if len(y_full) > max_points:
        keep = np.round(np.linspace(0, len(y_full) - 1, max_points)).astype(int)
        y_grid = y_full[keep]
        z_full = z_full[keep]
    else:
        y_grid = y_full

    return x_grid, y_grid, z_full


def build_scan_lines(scan_rows):
    """
    Return one (y, x_positions, samples) tuple per non-empty row, using the
    same X reconstruction as build_scan_grid(), for overlaying each scanned
    line on a single plot instead of resampling onto a shared grid.
    """
    lines = []
    for row in scan_rows:
        samples = row["samples"]
        if not samples:
            continue
        lines.append((row["y"], _row_positions(row), samples))
    return lines


def find_scan_max(scan_rows):
    """
    Return ``(x, y, sample)`` for the highest finite stored sample.

    X is reconstructed with the same timing model used by the plots. Using
    the raw rows here (rather than the display grid) ensures that display
    downsampling cannot move or omit the selected point. Returns ``None``
    when no row contains a finite sample.
    """
    best = None
    for row in scan_rows:
        samples = np.asarray(row.get("samples", []), dtype=float).reshape(-1)
        if samples.size == 0:
            continue

        finite_indices = np.flatnonzero(np.isfinite(samples))
        if finite_indices.size == 0:
            continue

        local_index = int(finite_indices[np.argmax(samples[finite_indices])])
        value = float(samples[local_index])
        if best is not None and value <= best[2]:
            continue

        x_positions = _row_positions(row)
        if local_index >= len(x_positions):
            continue
        best = (float(x_positions[local_index]), float(row["y"]), value)

    return best


def _cached_row_positions(row, positions_cache):
    key = id(row)
    row_xs = positions_cache.get(key)
    if row_xs is None:
        row_xs = _row_positions(row)
        positions_cache[key] = row_xs
    return row_xs


def build_scan_grid_incremental(scan_rows, cache, max_points=200):
    """
    Cached equivalent of build_scan_grid(), for callers that redraw
    repeatedly as scan_rows grows (e.g. a GUI polling loop). `cache` is a
    plain dict the caller owns across calls -- pass a fresh {} to start
    over (e.g. for a new scan or a freshly loaded file); the same dict can
    also be shared with build_scan_lines_incremental() below.

    Without caching, every redraw during a long scan repeats the (fairly
    expensive) position reconstruction + grid interpolation for every row
    seen so far, not just the new ones -- work that grows every tick as the
    scan progresses. That's wall-clock time spent holding the GIL on
    whatever thread calls this, which can starve a concurrently running
    hardware-read thread (see diagnostics/timing_check.py) if this is
    called from a GUI's main thread. Caching each row's finished result
    turns a redraw with no new rows into a cheap dict-lookup pass.

    Row results are cached per (x_min, x_max, n_cols) grid; if any of those
    change (e.g. the commanded span or max_points changes) the grid cache
    is invalidated and rebuilt, but the underlying per-row position
    reconstruction (the expensive part, especially for measured-trace mode)
    is cached independently and survives that.
    """
    rows_with_data = [row for row in scan_rows if len(row["samples"])]
    if not rows_with_data:
        return np.array([]), np.array([]), np.empty((0, 0))

    rows_with_data = sorted(rows_with_data, key=lambda row: row["y"])
    y_full = np.array([row["y"] for row in rows_with_data], dtype=float)

    x_min = min(min(row["x_start"], row["x_end"]) for row in rows_with_data)
    x_max = max(max(row["x_start"], row["x_end"]) for row in rows_with_data)
    n_cols = max(2, min(max_points, max(len(row["samples"]) for row in rows_with_data)))
    x_grid = np.linspace(x_min, x_max, n_cols)

    grid_key = (x_min, x_max, n_cols)
    if cache.get("grid_key") != grid_key:
        cache["grid_key"] = grid_key
        cache["z_rows"] = {}
    z_rows = cache["z_rows"]
    positions = cache.setdefault("positions", {})

    z_full = np.empty((len(rows_with_data), len(x_grid)))
    for row_index, row in enumerate(rows_with_data):
        key = id(row)
        z_row = z_rows.get(key)
        if z_row is None:
            row_xs = _cached_row_positions(row, positions)
            z_row, _ = interp_nan(row_xs, row["samples"], x_grid)
            z_rows[key] = z_row
        z_full[row_index] = z_row

    if len(y_full) > max_points:
        keep = np.round(np.linspace(0, len(y_full) - 1, max_points)).astype(int)
        y_grid = y_full[keep]
        z_full = z_full[keep]
    else:
        y_grid = y_full

    return x_grid, y_grid, z_full


def build_scan_lines_incremental(scan_rows, cache):
    """
    Cached equivalent of build_scan_lines(), sharing the per-row position
    cache with build_scan_grid_incremental() when passed the same `cache`
    dict (the position reconstruction doesn't depend on the grid, so it's
    reused as-is regardless of grid changes).
    """
    positions = cache.setdefault("positions", {})
    lines = []
    for row in scan_rows:
        samples = row["samples"]
        if not samples:
            continue
        lines.append((row["y"], _cached_row_positions(row, positions), samples))
    return lines


def plot_scan(scan_rows):
    fig, ax = plt.subplots()
    lines = build_scan_lines(scan_rows)
    color_map = plt.get_cmap("viridis")
    color_count = max(1, len(lines) - 1)

    for line_index, (y, row_xs, samples) in enumerate(lines):
        ax.plot(
            row_xs,
            samples,
            linewidth=1.0,
            color=color_map(line_index / color_count),
            label=f"y={y:.3f}",
        )

    plotted_rows = len(lines)
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
    extra_read_past_nominal = [
        row["sample_span_s"] - row["read_duration"]
        for row in scan_rows
        if "sample_span_s" in row and "read_duration" in row
    ]
    n_lag_missed = sum(
        1 for row in scan_rows if row.get("measured_lag_midpoint_s") is None
    )
    n_stop_missed = sum(
        1 for row in scan_rows if row.get("measured_motion_stopped_lag_s") is None
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
    print_stats("Extra read time past nominal read_duration (stop_condition)", extra_read_past_nominal)
    if n_lag_missed:
        print(
            f"Move-start lag was not detected on {n_lag_missed}/{len(scan_rows)} "
            "row(s); those rows fell back to command-return timing."
        )
    if n_stop_missed:
        print(
            f"Motion-stop was not detected on {n_stop_missed}/{len(scan_rows)} "
            "row(s); those rows' burst read ran until hard_timeout instead of "
            "stop_condition."
        )


def run_scan(
    *,
    serial,
    arduino_port,
    arduino_baud,
    x0,
    y0,
    x_span,
    y_span,
    line_spacing,
    acceleration,
    max_velocity,
    row_settle_s=2.0,
    read_safety_factor=1.10,
    read_overhead_s=0.05,
    home_timeout_s=30.0,
    skip_homing_check=True,
    # Position-based move-start-lag detection (see motion_timing.MoveStartLagMonitor).
    move_start_lag_threshold_mm=0.0000005,  # 0.5 nm; must exceed quantization noise
    lag_monitor_timeout_s=2.0,
    save_file=None,
    progress_callback=None,
    stop_event=None,
):
    """
    Run one raster scan (X fly-scan per row, stepped in Y) and return
    scan_rows, in the same format plot_scan()/load_scan() expect.

    X position per ADC sample is reconstructed from the modeled trapezoidal
    profile (motion_timing.sample_positions_for_motion), anchored per row by
    a measured move-start lag (MoveStartLagMonitor) rather than assumed
    command-issue timing. The same monitor also watches for the moving
    status bit to actually clear, which burst_read_binary uses to extend a
    row's read past the modeled duration until motion has really finished --
    never cutting a row short just because the model predicted less time
    than the real move took (see COVERAGE_WARN_THRESHOLD below for a
    per-row check that this held).

    progress_callback(row_dict, row_index, total_rows), if given, is called
    after each row is appended to scan_rows -- e.g. so a GUI can update a
    live progress display without waiting for the whole scan to finish.

    stop_event, if given, is checked before each new row starts; if set, the
    scan ends early with whatever rows were already completed (motors are
    left stationary at the end of the last finished row, not mid-move).
    """
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
            serial=serial,
            channel=1,
            poll_ms=1,
        )
        motory = ThorlabsModularStepperController(
            serial=serial,
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
            acceleration=acceleration,
            max_velocity=max_velocity,
            real_unit=True,
        )
        motory.set_velocity_params(
            acceleration=acceleration,
            max_velocity=max_velocity,
            real_unit=True,
        )

        motorx.move_absolute(x_start, wait=True, real_unit=True)
        motory.move_absolute(y_start, wait=True, real_unit=True)

        ser = open_arduino(port=arduino_port, baud=arduino_baud)

        move_time = expected_move_time(
            distance=x_span,
            max_velocity=max_velocity,
            acceleration=acceleration,
        )
        read_duration = sampling_duration_for_move(
            distance=x_span,
            max_velocity=max_velocity,
            acceleration=acceleration,
            safety_factor=read_safety_factor,
            overhead_s=read_overhead_s,
        )

        print(
            f"Scanning {len(y_points)} rows; "
            f"expected X move time {move_time:.3f}s, "
            f"read duration {read_duration:.3f}s per row."
        )

        for row_index, y_target in enumerate(y_points):
            if stop_event is not None and stop_event.is_set():
                print(f"Scan stopped after {row_index}/{len(y_points)} row(s) (requested).")
                break

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

            row_wait_timeout_s = max(read_duration + 2.0, move_time * 2.0 + 2.0)
            # watch_for_stop keeps this monitor running (polling the DLL's
            # cached position/status, no extra device round trip) through the
            # whole row, since burst_read_binary's stop_condition below needs
            # it alive to detect when motion actually finishes -- not just
            # when it starts.
            monitor_timeout_s = row_wait_timeout_s

            # Watches get_position()/status bits in the background while this
            # thread issues the move and then blocks on the Arduino read, so
            # we get the real per-row motion-start lag instead of assuming
            # command-issue/command-return timing.
            lag_monitor = MoveStartLagMonitor(
                motorx,
                pos0=actual_x_start,
                position_threshold_mm=move_start_lag_threshold_mm,
                timeout_s=monitor_timeout_s,
                watch_for_stop=True,
            ).start()

            move_command_issue_s = time.perf_counter()
            motorx.move_relative(x_displacement, wait=False, real_unit=True)
            move_command_return_s = time.perf_counter()

            # duration=read_duration is a minimum, not a hard cutoff: if the
            # row's real motor-start lag eats into that budget, stop_condition
            # keeps the burst going (up to hard_timeout) until lag_monitor
            # confirms the row's motion has actually stopped, instead of
            # silently truncating samples off the row's trailing edge.
            samples, read_timing = burst_read_binary(
                ser=ser,
                duration=read_duration,
                reset_buffer=False,
                return_timing=True,
                stop_condition=lag_monitor.has_stopped,
                hard_timeout=row_wait_timeout_s,
            )
            sample_window_end_s = read_timing["end_time"]

            motorx.wait_until_stopped(
                timeout_s=row_wait_timeout_s,
                require_motion_seen=False,
            )
            actual_x_end = motorx.get_position(real_unit=True)

            # Only now is the move guaranteed finished, so it's safe to end
            # the monitor's polling loop (a no-op if it already stopped
            # itself once it confirmed the moving bit had cleared).
            lag_monitor.stop()
            lag_monitor.join(timeout=lag_monitor_timeout_s)
            lag_result = lag_monitor.result(move_command_issue_s)
            if lag_result is not None:
                motion_start_s = move_command_issue_s + lag_result["lag_midpoint_s"]
            else:
                print(
                    f"Row {row_index}: move-start lag not detected within "
                    f"{monitor_timeout_s:.1f}s; falling back to "
                    "command-return timing for the X projection."
                )
                motion_start_s = move_command_return_s
                lag_result = {}

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
                    "sample_window_start_s": sample_window_start_s,
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
                    "measured_motion_stopped_lag_s": lag_result.get("motion_stopped_lag_s"),
                    "acceleration": acceleration,
                    "max_velocity": max_velocity,
                    "samples": samples,
                }
            )
            lag_mid_s = scan_rows[-1]["measured_lag_midpoint_s"]
            lag_mid_str = "n/a" if lag_mid_s is None else f"{lag_mid_s * 1000:.2f} ms"
            # >0 means this row's read ran past the nominal read_duration
            # waiting for stop_condition (i.e. the fix actually engaged this
            # row); ~0 means the row finished within the nominal budget.
            extra_read_s = scan_rows[-1]["sample_span_s"] - read_duration
            print(
                f"Row {row_index + 1}/{len(y_points)}: "
                f"y={actual_y:.6f}, "
                f"x={actual_x_start:.6f}->{actual_x_end:.6f}, "
                f"samples={len(samples)}, "
                f"cmd->read={scan_rows[-1]['move_to_read_start_s'] * 1000:.2f} ms, "
                f"measured lag mid={lag_mid_str}, "
                f"extra read past nominal={extra_read_s * 1000:.2f} ms"
            )

            # Spot-check this row's actual captured span against what was
            # commanded, using the same X reconstruction the plots use --
            # i.e. did the motor really finish before the read stopped.
            # Should always be ~100% given the stop-detection above; if not,
            # something's still letting a row get cut short.
            row_xs = _row_positions(scan_rows[-1])
            if len(row_xs) and x_span > 0:
                captured = float(np.max(row_xs) - np.min(row_xs))
                coverage = captured / x_span
                if coverage < COVERAGE_WARN_THRESHOLD:
                    print(
                        f"  WARNING: row {row_index} captured only {coverage:.1%} of the "
                        f"commanded span ({captured:.4f}/{x_span:.4f} mm) -- the read may "
                        "have stopped before the motor actually finished."
                    )

            if progress_callback is not None:
                progress_callback(scan_rows[-1], row_index, len(y_points))

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

    if save_file and scan_rows:
        np.savez(save_file, scan_rows=np.array(scan_rows, dtype=object))
        print(f"Saved {len(scan_rows)} row(s) (raw samples + timing/lag metadata) to {save_file}")

    return scan_rows


def run_jog_scan(
    *,
    serial,
    arduino_port,
    arduino_baud,
    x0,
    y0,
    x_span,
    jog_spacing,
    acceleration,
    max_velocity,
    sample_duration_s=0.05,
    home_timeout_s=30.0,
    skip_homing_check=True,
    save_file=None,
    progress_callback=None,
    stop_event=None,
):
    """
    Fine-alignment single-line scan: step X across ``x_span`` in
    ``jog_spacing`` increments at the fixed Y position ``y0``, sampling the
    detector at each stopped point.

    Each X jog finishes before its position is read and the detector is
    sampled -- the ADC bytes collected during ``sample_duration_s`` are
    averaged into one value at that explicit measured X position. Intended
    for fine-tuning around a line already located with a coarser raster
    scan, not for surveying a 2D area.
    """
    if jog_spacing <= 0:
        raise ValueError("jog_spacing must be positive")
    if sample_duration_s <= 0:
        raise ValueError("sample_duration_s must be positive")

    x_points = build_axis_points(x0 - x_span / 2, x_span, jog_spacing)
    scan_rows = []

    motorx = None
    motory = None
    ser = None
    scan_failed = True

    try:
        motorx = ThorlabsModularStepperController(serial=serial, channel=1, poll_ms=1)
        motory = ThorlabsModularStepperController(serial=serial, channel=2, poll_ms=1)
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

        for motor in (motorx, motory):
            motor.set_velocity_params(
                acceleration=acceleration,
                max_velocity=max_velocity,
                real_unit=True,
            )
        motorx.set_jog_mode(continuous=False, profiled_stop=True)
        motorx.set_jog_velocity_params(
            acceleration=acceleration,
            max_velocity=max_velocity,
            real_unit=True,
        )

        motorx.move_absolute(x_points[0], wait=True, real_unit=True)
        motory.move_absolute(y0, wait=True, real_unit=True)
        ser = open_arduino(port=arduino_port, baud=arduino_baud)

        actual_y = motory.get_position(real_unit=True)
        print(
            f"Jog line scanning {len(x_points)} point(s) at y={actual_y:.6f}; "
            f"X spacing={jog_spacing:.6g} mm, sample={sample_duration_s:.3f}s/point."
        )

        row_x_positions = []
        row_samples = []

        for point_index, target in enumerate(x_points):
            if stop_event is not None and stop_event.is_set():
                print("Jog scan stopped at a completed sample point (requested).")
                break

            if point_index > 0:
                step = target - x_points[point_index - 1]
                motorx.set_jog_step_size(abs(step), real_unit=True)
                if step >= 0:
                    motorx.jog_forward(wait=True)
                else:
                    motorx.jog_backward(wait=True)

            actual_x = motorx.get_position(real_unit=True)
            raw_samples = burst_read_binary(
                ser=ser,
                duration=sample_duration_s,
                reset_buffer=True,
            )
            value = float(np.mean(raw_samples)) if raw_samples else float("nan")
            row_x_positions.append(actual_x)
            row_samples.append(value)

            row = {
                "scan_mode": "jog_line",
                "row": 0,
                "y": actual_y,
                "x_start": row_x_positions[0],
                "x_end": row_x_positions[-1],
                "x_displacement": row_x_positions[-1] - row_x_positions[0],
                "x_positions": list(row_x_positions),
                "jog_spacing": jog_spacing,
                "sample_duration_s": sample_duration_s,
                "acceleration": acceleration,
                "max_velocity": max_velocity,
                "samples": list(row_samples),
            }
            scan_rows = [row]
            print(
                f"Jog point {point_index + 1}/{len(x_points)}: "
                f"x={actual_x:.6f}, value={value:.6g}"
            )
            if progress_callback is not None:
                progress_callback(row, 0, 1)

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

    if save_file and scan_rows:
        np.savez(save_file, scan_rows=np.array(scan_rows, dtype=object))
        print(f"Saved jog line ({len(scan_rows[0]['samples'])} point(s)) to {save_file}")

    return scan_rows


if __name__ == "__main__":
    scan_rows = run_scan(
        serial="50865380",
        arduino_port="COM3",
        arduino_baud=230400,
        x0=1.0,
        y0=1.0,
        x_span=1.0,
        y_span=0.2,
        line_spacing=0.1,
        acceleration=4.0,
        max_velocity=2.0,
        save_file="scan_last.npz",
    )
    plot_scan(scan_rows)
