"""
Map one Arduino burst onto X position using the motor motion profile.

The Arduino signal is treated as time-defined. The calibration output is the
time-to-position mapping caused by:

- trapezoidal stage motion,
- measured delay between move command and clean burst-read start,
- the spatial prefix lost during that delay.

Run from anywhere (adds the repo root to sys.path so it can reach the
drivers/engine one level up):

    python diagnostics/calibrate_line.py
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ArduinoSampler import burst_read_binary, close_arduino, open_arduino
from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController
from motion_timing import (
    expected_move_time,
    make_mapping,
    sample_times,
    sampling_duration_for_move,
)


# Hardware
SERIAL = "50865380"
ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 230400

# Scan geometry, mm
x0 = 1.0
x_span = 1.0
y0 = 1.0  # reserved for consistency with scan_engine.py

# Motion, real units
default_acceleration = 4.0  # mm/s^2
default_max_velocity = 1.0  # mm/s

# Acquisition
read_safety_factor = 1.10
read_overhead_s = 0.05
extra_pre_burst_delay_s = 0.0  # set to e.g. 0.3 to intentionally lose more X

# Safety / setup
home_timeout_s = 30.0
skip_homing_check = True  # only skip if X is already homed
move_tolerance_mm = 0.05

# Output
UNIFORM_GRID_POINTS = None  # None = same number of points as retained samples
SAVE_FILE = str(Path(__file__).resolve().parent / "calibrate_line_last.npz")
REPLAY_FILE = None

# Offline self-test: time-defined signal, like a function generator.
SIMULATE = False
SIMULATED_WAVEFORM = "sine"  # sine, square, triangle, pulse
SIMULATED_SIGNAL_HZ = 8.0
SIMULATED_DEAD_TIME_S = 0.08


def report_mapping(t, samples, meta, mapping):
    t = np.asarray(t, dtype=float)
    n = len(samples)
    duration = t[-1] - t[0] if n > 1 else 0.0
    rate = n / duration if duration > 0 else 0.0

    distance = abs(float(meta["distance"]))
    x_start = float(meta["x_start"])
    x_samples = mapping["x_samples"]
    profile = mapping["profile"]
    elapsed = mapping["elapsed_motion_s"]
    spacing = mapping["sample_spacing"]
    moving_spacing = spacing[spacing > 1e-12]

    ramp_t = profile["ramp_time"]
    cruise_end = ramp_t + profile["cruise_time"]
    moving = elapsed <= profile["total_time"]
    ramp = moving & ((elapsed <= ramp_t) | (elapsed >= cruise_end))
    cruise = moving & (elapsed > ramp_t) & (elapsed < cruise_end)
    post = elapsed > profile["total_time"]

    def pct(count):
        return 100.0 * count / n if n else 0.0

    lost_um = mapping["lost_distance"] * 1000.0
    lost_pct = 100.0 * mapping["lost_distance"] / distance if distance else 0.0

    print("=" * 64)
    print("MOTION-BASED SAMPLE MAPPING")
    print("=" * 64)
    print(f"samples              : {n} over {duration*1000:.1f} ms ({rate:.0f} S/s)")
    print(f"commanded X range    : {x_start:.6f} -> {mapping['x_commanded_end']:.6f} mm")
    print(f"mapped distance      : {distance:.6f} mm")
    print()
    print("Timing / lost prefix")
    print(f"  command call        : {meta.get('move_command_call_s', 0.0)*1000:8.2f} ms")
    print(f"  command -> burst    : {mapping['dead_time_s']*1000:8.2f} ms")
    print(f"  unsampled distance  : {lost_um:8.2f} um ({lost_pct:.2f}%)")
    print(f"  lost grid points    : {mapping['lost_grid_points']} / {len(mapping['x_uniform_full'])}")
    if len(x_samples):
        print(f"  first sampled X     : {x_samples[0]:8.6f} mm")
        print(f"  last sampled X      : {x_samples[-1]:8.6f} mm")
    print()
    print("Trapezoid")
    print(f"  acceleration        : {mapping['acceleration']:8.3f} mm/s^2")
    print(f"  max velocity        : {mapping['max_velocity']:8.3f} mm/s")
    print(f"  peak velocity       : {profile['peak_velocity']:8.3f} mm/s")
    print(f"  ramp time           : {profile['ramp_time']*1000:8.2f} ms")
    print(f"  cruise time         : {profile['cruise_time']*1000:8.2f} ms")
    print(f"  total move time     : {profile['total_time']*1000:8.2f} ms")
    print()
    print("Retained sample distribution")
    print(f"  ramp samples        : {int(np.sum(ramp)):6d} ({pct(np.sum(ramp)):5.1f}%)")
    print(f"  cruise samples      : {int(np.sum(cruise)):6d} ({pct(np.sum(cruise)):5.1f}%)")
    print(f"  post-motion samples : {int(np.sum(post)):6d} ({pct(np.sum(post)):5.1f}%)")
    if len(moving_spacing):
        print(
            "  moving spatial step : "
            f"min {np.min(moving_spacing)*1000:.3f} um, "
            f"median {np.median(moving_spacing)*1000:.3f} um, "
            f"max {np.max(moving_spacing)*1000:.3f} um"
        )
    print("=" * 64)


def plot_mapping(t, samples, mapping):
    import matplotlib.pyplot as plt

    x_samples = mapping["x_samples"]

    fig, axes = plt.subplots(4, 1, figsize=(9, 10))

    axes[0].plot(np.asarray(t) * 1000, samples, lw=0.8)
    axes[0].set(xlabel="time since burst start (ms)", ylabel="ADC")
    axes[0].set_title("Raw burst: uniform in time")
    axes[0].grid(True)

    axes[1].plot(np.arange(len(x_samples)), x_samples, lw=0.9)
    axes[1].set(xlabel="sample index", ylabel="X position (mm)")
    axes[1].set_title("Motor-profile mapping: sample index -> X")
    axes[1].grid(True)

    axes[2].plot(x_samples, samples, ".", ms=2)
    axes[2].set(xlabel="X position (mm)", ylabel="ADC")
    axes[2].set_title("Raw samples placed at mapped X positions")
    axes[2].grid(True)

    axes[3].plot(
        mapping["x_uniform_full"],
        mapping["y_uniform_full"],
        lw=0.9,
        label="full commanded grid",
    )
    axes[3].plot(
        mapping["x_uniform_sampled"],
        mapping["y_uniform_sampled"],
        lw=0.9,
        alpha=0.6,
        label="sampled-only grid",
    )
    axes[3].set(xlabel="X position (mm)", ylabel="ADC")
    axes[3].set_title("Uniform X grid; unsampled prefix remains blank")
    axes[3].legend(fontsize="small")
    axes[3].grid(True)

    fig.tight_layout()
    plt.show()


def acquire_line():
    """Move X across one span while acquiring one clean Arduino burst."""
    x_target_start = x0 - x_span / 2.0
    x_displacement = x_span
    motorx = ThorlabsModularStepperController(serial=SERIAL, channel=1, poll_ms=1)
    ser = None
    failed = True

    try:
        motorx.connect()
        if skip_homing_check:
            motorx.request_update()
            if not motorx.get_status_bits() & 0x00000400:
                raise ThorlabsError(
                    "X axis is not homed. Set skip_homing_check=False or home "
                    "the stage before running calibration."
                )
        else:
            print("Homing X...")
            motorx.home(wait=True, timeout_s=home_timeout_s)

        motorx.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )
        motorx.move_absolute(x_target_start, wait=True, real_unit=True)
        x_start = motorx.get_position(real_unit=True)
        print(f"X preload: requested {x_target_start:.6f} mm, actual {x_start:.6f} mm")

        ser = open_arduino(port=ARDUINO_PORT, baud=ARDUINO_BAUD)
        move_time = expected_move_time(x_span, default_max_velocity, default_acceleration)
        read_duration = sampling_duration_for_move(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
            safety_factor=read_safety_factor,
            overhead_s=read_overhead_s,
        )
        print(
            f"Nominal move time {move_time*1000:.1f} ms, "
            f"burst duration {read_duration*1000:.1f} ms."
        )

        print(f"Moving X by {x_displacement:.6f} mm")
        command_issue = time.perf_counter()
        motorx.move_relative(x_displacement, wait=False, real_unit=True)
        command_return = time.perf_counter()

        if extra_pre_burst_delay_s > 0:
            time.sleep(extra_pre_burst_delay_s)

        samples, timing = burst_read_binary(
            ser=ser,
            duration=read_duration,
            reset_buffer=True,
            return_timing=True,
        )
        dead_time_s = timing["start_time"] - command_issue
        read_delay_s = timing["start_time"] - command_return

        motorx.wait_until_stopped(
            timeout_s=max(read_duration + 2.0, move_time * 2.0 + 2.0),
            require_motion_seen=False,
        )
        x_end = motorx.get_position(real_unit=True)
        displacement = x_end - x_start

        print(f"X final: {x_end:.6f} mm, measured displacement {displacement:.6f} mm")
        if abs(displacement - x_displacement) > move_tolerance_mm:
            raise RuntimeError(
                "Calibration move did not reach the requested displacement: "
                f"expected {x_displacement:.6f} mm, got {displacement:.6f} mm."
            )
        print(
            "Timing: "
            f"command->burst {dead_time_s*1000:.2f} ms, "
            f"command call {(command_return - command_issue)*1000:.2f} ms, "
            f"read loop started {read_delay_s*1000:.2f} ms after command return"
        )
        failed = False
    finally:
        if ser is not None:
            close_arduino(ser)
        if failed:
            motorx.safe_shutdown()
        else:
            motorx.disconnect()

    samples = np.asarray(samples, dtype=float)
    return sample_times(timing["duration"], len(samples)), samples, {
        "distance": abs(displacement),
        "direction": 1.0 if displacement >= 0 else -1.0,
        "x_start": x_start,
        "x_end": x_end,
        "sample_span_s": timing["duration"],
        "dead_time_s": dead_time_s,
        "move_command_call_s": command_return - command_issue,
        "read_loop_delay_s": read_delay_s,
    }


def time_waveform(t, waveform, frequency_hz, amplitude, offset, duty=0.1):
    phase = 2 * np.pi * frequency_hz * t
    waveform = waveform.lower()
    if waveform == "sine":
        unit = np.sin(phase)
    elif waveform == "square":
        unit = np.where(np.sin(phase) >= 0.0, 1.0, -1.0)
    elif waveform == "triangle":
        unit = (2.0 / np.pi) * np.arcsin(np.sin(phase))
    elif waveform == "pulse":
        unit = np.where((frequency_hz * t) % 1.0 < duty, 1.0, -1.0)
    else:
        raise ValueError("waveform must be sine, square, triangle, or pulse")
    return offset + amplitude * unit


def simulate_line():
    read_duration = sampling_duration_for_move(
        distance=x_span,
        max_velocity=default_max_velocity,
        acceleration=default_acceleration,
        safety_factor=read_safety_factor,
        overhead_s=read_overhead_s,
    )
    t = sample_times(read_duration, int(read_duration * 4000))
    rng = np.random.default_rng(0)
    samples = time_waveform(
        t,
        SIMULATED_WAVEFORM,
        SIMULATED_SIGNAL_HZ,
        amplitude=90.0,
        offset=128.0,
    )
    samples = np.clip(np.round(samples + rng.normal(0, 2.0, len(t))), 0, 255)
    return t, samples, {
        "distance": x_span,
        "direction": 1.0,
        "x_start": x0 - x_span / 2.0,
        "x_end": x0 + x_span / 2.0,
        "sample_span_s": read_duration,
        "dead_time_s": SIMULATED_DEAD_TIME_S,
        "move_command_call_s": SIMULATED_DEAD_TIME_S,
        "read_loop_delay_s": 0.0,
    }


def load_replay(path):
    data = np.load(path, allow_pickle=True)
    return data["t"], data["y"], data["meta"].item()


def main():
    if SIMULATE:
        print(">>> SIMULATE mode: time-defined signal, no hardware.\n")
        t, samples, meta = simulate_line()
    elif REPLAY_FILE:
        print(f">>> REPLAY mode: {REPLAY_FILE}\n")
        t, samples, meta = load_replay(REPLAY_FILE)
    else:
        t, samples, meta = acquire_line()

    mapping = make_mapping(
        t,
        samples,
        meta,
        acceleration=default_acceleration,
        max_velocity=default_max_velocity,
        grid_points=UNIFORM_GRID_POINTS,
    )

    if SAVE_FILE and not REPLAY_FILE:
        np.savez(SAVE_FILE, t=t, y=samples, meta=meta, mapping=mapping)
        print(f"Saved acquisition and mapping to {SAVE_FILE}")

    report_mapping(t, samples, meta, mapping)

    try:
        plot_mapping(t, samples, mapping)
    except Exception as exc:
        print(f"(plot skipped: {exc})")


if __name__ == "__main__":
    main()
