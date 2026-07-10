"""
Shared motion-timing code: the trapezoidal motion-profile / calibration
projection math (previously split between Main.py and calibrate_line.py), and
the get_position()-based move-start-lag measurement machinery (previously in
measure_move_latency.py).

Kept together because the scan loop in Main.py needs both at once: it
measures the real per-line motion-start lag while a row is being acquired,
then feeds that measured lag into the trapezoidal projection to place that
row's samples on the X axis.
"""

import math
import threading
import time

import numpy as np


# ----------------------------------------------------------------------
# Trapezoidal motion profile
# ----------------------------------------------------------------------

def motion_profile(distance: float, max_velocity: float, acceleration: float):
    distance = abs(float(distance))
    max_velocity = abs(float(max_velocity))
    acceleration = abs(float(acceleration))

    if distance == 0:
        return {
            "distance": 0.0,
            "ramp_time": 0.0,
            "cruise_time": 0.0,
            "ramp_distance": 0.0,
            "cruise_distance": 0.0,
            "peak_velocity": 0.0,
            "total_time": 0.0,
        }

    if max_velocity <= 0:
        raise ValueError("max_velocity must be positive")
    if acceleration <= 0:
        raise ValueError("acceleration must be positive")

    ramp_time = max_velocity / acceleration
    ramp_distance = 0.5 * acceleration * ramp_time**2

    if distance >= 2 * ramp_distance:
        cruise_distance = distance - 2 * ramp_distance
        cruise_time = cruise_distance / max_velocity
        peak_velocity = max_velocity
    else:
        ramp_time = math.sqrt(distance / acceleration)
        ramp_distance = distance / 2
        cruise_distance = 0.0
        cruise_time = 0.0
        peak_velocity = acceleration * ramp_time

    return {
        "distance": distance,
        "ramp_time": ramp_time,
        "cruise_time": cruise_time,
        "ramp_distance": ramp_distance,
        "cruise_distance": cruise_distance,
        "peak_velocity": peak_velocity,
        "total_time": 2 * ramp_time + cruise_time,
    }


def expected_move_time(distance: float, max_velocity: float, acceleration: float):
    return motion_profile(distance, max_velocity, acceleration)["total_time"]


def sampling_duration_for_move(
    distance: float,
    max_velocity: float,
    acceleration: float,
    safety_factor: float = 1.10,
    overhead_s: float = 0.05,
):
    move_time = expected_move_time(distance, max_velocity, acceleration)
    return move_time * safety_factor + overhead_s


def position_after_elapsed(profile, elapsed_s: float):
    """Scalar distance travelled along the profile after elapsed_s (clamped)."""
    distance = profile["distance"]
    if distance == 0:
        return 0.0

    elapsed_s = max(0.0, float(elapsed_s))
    if elapsed_s >= profile["total_time"]:
        return distance

    ramp_time = profile["ramp_time"]
    cruise_time = profile["cruise_time"]
    acceleration = profile["peak_velocity"] / ramp_time

    if elapsed_s <= ramp_time:
        return 0.5 * acceleration * elapsed_s**2

    cruise_start_s = ramp_time
    cruise_end_s = cruise_start_s + cruise_time
    if elapsed_s <= cruise_end_s:
        return profile["ramp_distance"] + profile["peak_velocity"] * (
            elapsed_s - cruise_start_s
        )

    decel_time = elapsed_s - cruise_end_s
    return (
        profile["ramp_distance"]
        + profile["cruise_distance"]
        + profile["peak_velocity"] * decel_time
        - 0.5 * acceleration * decel_time**2
    )


def sample_positions_for_motion(
    x_start: float,
    displacement: float,
    sample_count: int,
    read_duration_s: float,
    max_velocity: float,
    acceleration: float,
    motion_start_offset_s: float = 0.0,
):
    """Scalar/list version used for plotting a handful of scan rows."""
    if sample_count <= 0:
        return []

    profile = motion_profile(displacement, max_velocity, acceleration)
    direction = 1.0 if displacement >= 0 else -1.0

    if sample_count == 1:
        elapsed_times = [read_duration_s / 2]
    else:
        elapsed_times = [
            read_duration_s * i / (sample_count - 1) for i in range(sample_count)
        ]

    return [
        x_start
        + direction
        * position_after_elapsed(profile, elapsed_s - motion_start_offset_s)
        for elapsed_s in elapsed_times
    ]


def profile_positions(profile, elapsed_s):
    """Vectorized (numpy) distance travelled, for calibration/mapping work."""
    elapsed = np.clip(np.asarray(elapsed_s, dtype=float), 0.0, profile["total_time"])

    ramp_t = profile["ramp_time"]
    cruise_t = profile["cruise_time"]
    peak_v = profile["peak_velocity"]
    ramp_d = profile["ramp_distance"]
    cruise_d = profile["cruise_distance"]
    accel = peak_v / ramp_t if ramp_t > 0 else 0.0

    pos = np.empty_like(elapsed)

    ramp = elapsed <= ramp_t
    pos[ramp] = 0.5 * accel * elapsed[ramp] ** 2

    cruise = (elapsed > ramp_t) & (elapsed <= ramp_t + cruise_t)
    pos[cruise] = ramp_d + peak_v * (elapsed[cruise] - ramp_t)

    decel = elapsed > ramp_t + cruise_t
    dt = elapsed[decel] - (ramp_t + cruise_t)
    pos[decel] = ramp_d + cruise_d + peak_v * dt - 0.5 * accel * dt**2

    return pos


def sample_times(duration_s, count):
    if count <= 1:
        return np.zeros(count)
    return np.linspace(0.0, float(duration_s), count)


def interp_nan(x_samples, y_samples, x_grid):
    """Interpolate where samples exist; leave unsampled grid points as NaN."""
    x_samples = np.asarray(x_samples, dtype=float)
    y_samples = np.asarray(y_samples, dtype=float)
    x_grid = np.asarray(x_grid, dtype=float)
    out = np.full(len(x_grid), np.nan)

    if len(x_samples) == 0:
        return out, np.zeros(len(x_grid), dtype=bool)

    order = np.argsort(x_samples)
    xs = x_samples[order]
    ys = y_samples[order]

    if len(xs) == 1 or xs[0] == xs[-1]:
        valid = np.isclose(x_grid, xs[0])
        out[valid] = ys[0]
        return out, valid

    valid = (x_grid >= xs[0]) & (x_grid <= xs[-1])
    out[valid] = np.interp(x_grid[valid], xs, ys)
    return out, valid


def make_mapping(t, samples, meta, acceleration, max_velocity, grid_points=None):
    """Build all X mappings from timing metadata and the trapezoidal profile."""
    t = np.asarray(t, dtype=float)
    samples = np.asarray(samples, dtype=float)

    distance = abs(float(meta["distance"]))
    direction = 1.0 if float(meta.get("direction", 1.0)) >= 0 else -1.0
    x_start = float(meta["x_start"])
    dead_time_s = max(0.0, float(meta.get("dead_time_s", 0.0)))

    profile = motion_profile(distance, max_velocity, acceleration)
    elapsed = t + dead_time_s
    travelled = profile_positions(profile, elapsed)
    x_samples = x_start + direction * travelled
    x_end = x_start + direction * distance
    lost_distance = float(profile_positions(profile, [dead_time_s])[0])

    if grid_points is None:
        grid_points = len(samples)
    grid_points = int(grid_points)

    x_full = np.linspace(x_start, x_end, grid_points)
    y_full, sampled_mask = interp_nan(x_samples, samples, x_full)

    if len(x_samples):
        x_sampled = np.linspace(x_samples[0], x_samples[-1], grid_points)
    else:
        x_sampled = np.array([], dtype=float)
    y_sampled, _ = interp_nan(x_samples, samples, x_sampled)

    if len(x_samples) and direction >= 0:
        lost_mask = x_full < x_samples[0]
        post_mask = x_full > x_samples[-1]
    elif len(x_samples):
        lost_mask = x_full > x_samples[0]
        post_mask = x_full < x_samples[-1]
    else:
        lost_mask = np.ones(len(x_full), dtype=bool)
        post_mask = np.zeros(len(x_full), dtype=bool)

    spacing = np.abs(np.diff(x_samples)) if len(x_samples) > 1 else np.array([])

    return {
        "profile": profile,
        "dead_time_s": dead_time_s,
        "elapsed_motion_s": elapsed,
        "x_samples": x_samples,
        "x_uniform_full": x_full,
        "y_uniform_full": y_full,
        "x_uniform_sampled": x_sampled,
        "y_uniform_sampled": y_sampled,
        "sampled_mask": sampled_mask,
        "lost_prefix_mask": lost_mask,
        "post_motion_mask": post_mask,
        "lost_distance": lost_distance,
        "lost_grid_points": int(np.sum(lost_mask)),
        "post_motion_grid_points": int(np.sum(post_mask)),
        "sample_spacing": spacing,
        "x_commanded_end": x_end,
        "acceleration": float(acceleration),
        "max_velocity": float(max_velocity),
    }


# ----------------------------------------------------------------------
# Move-start lag measurement
# ----------------------------------------------------------------------

# Same "moving" mask as ThorlabsModularStepperController.is_moving()
MOVING_STATUS_MASK = (
    0x00000010  # moving CW
    | 0x00000020  # moving CCW
    | 0x00000040  # jogging CW
    | 0x00000080  # jogging CCW
    | 0x00000200  # homing
)


def measure_query_latency(motor, count, refresh):
    """Round-trip time of `count` back-to-back get_position() calls, motor stationary."""
    durations = []
    for _ in range(count):
        t0 = time.perf_counter()
        motor.get_position(real_unit=True, refresh=refresh)
        t1 = time.perf_counter()
        durations.append(t1 - t0)
    return durations


def summarize_latencies(label, values_s):
    import statistics

    values_ms = [v * 1000.0 for v in values_s]
    return (
        f"{label}\n"
        f"    n={len(values_ms)}  mean={statistics.mean(values_ms):7.3f} ms  "
        f"median={statistics.median(values_ms):7.3f} ms  "
        f"min={min(values_ms):7.3f} ms  max={max(values_ms):7.3f} ms  "
        f"stdev={statistics.pstdev(values_ms):7.3f} ms"
    )


class MoveStartLagMonitor:
    """
    Background-thread watcher that detects, via get_position()/status-bit
    polling, the wall-clock instant a motor actually begins moving after a
    move command is issued elsewhere (e.g. on the main thread, which may be
    busy doing something else like a blocking Arduino burst read).

    Usage:
        pos0 = motor.get_position(real_unit=True, refresh=True)
        monitor = MoveStartLagMonitor(motor, pos0).start()
        command_issue = time.perf_counter()
        motor.move_relative(displacement, wait=False, real_unit=True)
        ... do other work concurrently (e.g. burst_read_binary) ...
        monitor.join(timeout=2.0)
        lag = monitor.result(command_issue)  # None if never detected

    Polling uses get_position(refresh=False) and get_status_bits(), both of
    which just read the DLL's background-polling cache with no extra device
    round trip or artificial sleep, so the loop runs as fast as Python/ctypes
    allow. It stops as soon as it has detected motion, so it only contends
    with the main thread for the GIL for a brief window right after the move
    is issued, not for the whole read.
    """

    def __init__(self, motor, pos0, position_threshold_mm: float = 0.0005, timeout_s: float = 5.0):
        self.motor = motor
        self.pos0 = pos0
        self.position_threshold_mm = position_threshold_mm
        self.timeout_s = timeout_s

        self.last_unchanged_t = None
        self.first_changed_t = None
        self.first_moving_bit_t = None
        self.n_polls = 0

        self._thread = None

    def start(self):
        start_perf = time.perf_counter()
        self.last_unchanged_t = start_perf
        self._thread = threading.Thread(target=self._run, args=(start_perf,), daemon=True)
        self._thread.start()
        return self

    def _run(self, start_perf):
        deadline = start_perf + self.timeout_s
        while time.perf_counter() < deadline:
            t_poll = time.perf_counter()
            pos = self.motor.get_position(real_unit=True, refresh=False)
            moving = bool(self.motor.get_status_bits() & MOVING_STATUS_MASK)
            self.n_polls += 1

            if self.first_moving_bit_t is None and moving:
                self.first_moving_bit_t = t_poll

            if abs(pos - self.pos0) > self.position_threshold_mm:
                self.first_changed_t = t_poll
                return

            self.last_unchanged_t = t_poll

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)
        return self

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def result(self, command_issue_perf: float):
        """
        Lag estimates relative to command_issue_perf, or None if the position
        change was never detected before the monitor's timeout.
        """
        if self.first_changed_t is None:
            return None

        return {
            "lag_lower_bound_s": self.last_unchanged_t - command_issue_perf,
            "lag_upper_bound_s": self.first_changed_t - command_issue_perf,
            "lag_midpoint_s": (
                0.5 * (self.last_unchanged_t + self.first_changed_t) - command_issue_perf
            ),
            "moving_bit_lag_s": (
                None
                if self.first_moving_bit_t is None
                else self.first_moving_bit_t - command_issue_perf
            ),
            "n_polls_before_detection": self.n_polls,
        }


def measure_move_start_lag(motor, displacement_mm, timeout_s: float = 5.0, position_threshold_mm: float = 0.0005):
    """
    Synchronous convenience wrapper for standalone scripts: issues the move
    itself and blocks until the lag is measured (and the move has finished).

    For measuring the lag of a move issued elsewhere while the calling thread
    does other concurrent work (the Main.py scan loop's use case), use
    MoveStartLagMonitor directly instead.
    """
    pos0 = motor.get_position(real_unit=True, refresh=True)

    monitor = MoveStartLagMonitor(motor, pos0, position_threshold_mm, timeout_s).start()
    command_issue = time.perf_counter()
    motor.move_relative(displacement_mm, wait=False, real_unit=True)
    command_return = time.perf_counter()

    monitor.join()
    result = monitor.result(command_issue)
    if result is None:
        raise TimeoutError(
            "Timed out waiting to see the position change after the move command."
        )

    motor.wait_until_stopped(timeout_s=max(timeout_s, 5.0), require_motion_seen=False)
    pos_final = motor.get_position(real_unit=True, refresh=True)

    result["command_call_s"] = command_return - command_issue
    result["pos0"] = pos0
    result["pos_final"] = pos_final
    result["displacement_measured"] = pos_final - pos0
    return result
