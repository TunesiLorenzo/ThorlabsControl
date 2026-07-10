"""
calibrate_line.py

Acquire ONE scan line with the current Main.py settings, then use the fact that
the scanned signal is a spatial sinusoid to calibrate the time -> position
"warping" of the data.

Two effects distort a burst-read line vs. the ideal (uniform-in-X) signal:

  1. Acceleration warping.
     The stage runs a trapezoidal velocity profile (ramp up -> cruise ->
     ramp down) but the Arduino samples uniformly in TIME.  During the ramps
     the beam moves less X per sample, so a spatial sinusoid looks compressed
     at the two ends and stretched in the middle.

  2. Command / read lag.
     There is a delay t0 between the moment the move command is issued (and the
     burst read starts) and the moment the stage actually starts moving.  The
     samples taken during that delay are "dead" (all sitting at x_start).

Because the underlying signal is known to be a sinusoid in X, we can fit the
time -> position model  x(t) = x_start + dir * profile(a, v)(t - t0)  so that the
sampled data collapses onto a clean  A*sin(2*pi*x/L + phi) + C.  The fitted
(t0, a, v) are the calibration; L, A, phi, C fall out for free.

Run with  SIMULATE = True  to self-test the fitter on synthetic warped data with
no hardware attached.
"""

import time
import numpy as np

# Reuse the real motion model + hardware wrapper from Main.py.  Importing Main
# does NOT run its scan loop (that is guarded by __main__) and does NOT load any
# DLLs (that happens in the controller constructor).
from Main import (
    ThorlabsModularStepperController,
    motion_profile,
    expected_move_time,
    sampling_duration_for_move,
)
from ArduinoSampler import open_arduino, close_arduino, burst_read_binary


# ----------------------------------------------------------------------------
# Configuration (mirrors the __main__ block of Main.py)
# ----------------------------------------------------------------------------
SERIAL = "50865380"
ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 230400

x0 = 1.0                     # line centre, mm
x_span = 1.0                 # line length, mm
y0 = 1.0                     # row position, mm

default_acceleration = 4.0   # nominal, mm/s^2  (real units)
default_max_velocity = 4.0   # nominal, mm/s

read_safety_factor = 1.10
read_overhead_s = 0.05
home_timeout_s = 30.0
skip_homing_check = True      # a single calibration line usually skips homing

# Known spatial period of the sinusoid, in mm.  Set to None to let the fit
# estimate it (FFT initial guess).  Provide it if you know the grating/pattern
# pitch -- the fit is far more robust when L is fixed.
SPATIAL_PERIOD_MM = None

# Also fit the acceleration/velocity, or trust the nominal values and fit only
# the lag t0?  Fitting a, v extracts the true ramp shape; fixing them just
# de-warps using the configured profile.
FIT_ACCEL = True

# Offline / self-test.  When True, no hardware is touched: a synthetic warped
# sinusoid with known parameters is generated and fed through the same fit.
SIMULATE = False

# Optionally save / reload the raw acquisition for offline refitting.
SAVE_FILE = "calibrate_line_last.npz"
REPLAY_FILE = None            # set to a .npz path to refit without hardware


# ----------------------------------------------------------------------------
# Motion model (vectorised trapezoidal position, for the fitter's hot loop)
# ----------------------------------------------------------------------------
def positions_from_profile(profile, elapsed):
    """Position (in the move's own frame, 0 -> distance) at each elapsed time.

    Vectorised twin of Main.position_after_elapsed so the optimiser can evaluate
    thousands of samples at once.  elapsed < 0 clamps to 0 (pre-motion), elapsed
    beyond the move clamps to the full distance (post-motion).
    """
    total = profile["total_time"]
    e = np.clip(np.asarray(elapsed, dtype=float), 0.0, total)

    tr = profile["ramp_time"]
    tc = profile["cruise_time"]
    v = profile["peak_velocity"]
    rd = profile["ramp_distance"]
    cd = profile["cruise_distance"]
    a = v / tr if tr > 0 else 0.0

    x = np.empty_like(e)

    m_ramp = e <= tr
    x[m_ramp] = 0.5 * a * e[m_ramp] ** 2

    m_cruise = (e > tr) & (e <= tr + tc)
    x[m_cruise] = rd + v * (e[m_cruise] - tr)

    m_decel = e > tr + tc
    td = e[m_decel] - (tr + tc)
    x[m_decel] = rd + cd + v * td - 0.5 * a * td ** 2

    return x


def model_x(t, t0, accel, vmax, distance, direction, x_start):
    """Absolute X position at sample times t for a given lag/profile."""
    profile = motion_profile(distance, vmax, accel)
    return x_start + direction * positions_from_profile(profile, t - t0)


# ----------------------------------------------------------------------------
# Sinusoid fit (variable projection: linear amp/phase/offset solved exactly at
# every candidate of the nonlinear params t0, a, v, L)
# ----------------------------------------------------------------------------
def _sine_lstsq(theta, y):
    """Best A*sin+B*cos+C for fixed phase argument theta.  Returns (coef, rms)."""
    design = np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)])
    coef, *_ = np.linalg.lstsq(design, y, rcond=None)
    resid = y - design @ coef
    return coef, float(np.sqrt(np.mean(resid ** 2)))


def _coef_to_sine(coef):
    b_sin, b_cos, offset = coef
    amplitude = float(np.hypot(b_sin, b_cos))
    phase = float(np.arctan2(b_cos, b_sin))  # y = amp*sin(theta + phase) + off
    return amplitude, phase, float(offset)


def _pattern_search(func, x0, steps, n_iter=80, shrink=0.6):
    """Tiny derivative-free minimiser (Hooke-Jeeves style).  No scipy needed."""
    x = list(x0)
    best = func(x)
    steps = list(steps)
    for _ in range(n_iter):
        improved = False
        for i in range(len(x)):
            for delta in (steps[i], -steps[i]):
                cand = list(x)
                cand[i] += delta
                val = func(cand)
                if val < best:
                    best, x, improved = val, cand, True
        if not improved:
            steps = [s * shrink for s in steps]
            if max(steps) < 1e-10:
                break
    return x, best


def _period_guess(t, y, mean_speed):
    """FFT-based initial guess of the spatial period (mm).

    The dominant temporal frequency f (cycles/s) of the line maps to a spatial
    period ~ mean_speed / f, where mean_speed is the average stage speed over
    the move (using vmax here overestimates badly for a triangular move).
    """
    y = np.asarray(y, float)
    n = len(y)
    duration = t[-1] - t[0]
    if duration <= 0 or n < 8:
        return 1.0
    window = np.hanning(n)
    spec = np.abs(np.fft.rfft((y - y.mean()) * window))
    freqs = np.fft.rfftfreq(n, d=duration / (n - 1))
    k = int(np.argmax(spec[1:])) + 1  # skip DC
    f = freqs[k]
    if f <= 0:
        return 1.0
    return float(mean_speed / f)


def fit_warp(t, y, distance, direction, x_start,
             nominal_accel, nominal_vmax,
             period=None, fit_accel=True):
    """Fit the time->position warp so y becomes a clean sinusoid in X.

    Nonlinear params (t0, a, v, and optionally L) are optimised; the linear
    amplitude/phase/offset are solved exactly at every step (variable
    projection).  Uses scipy.least_squares when available, else a derivative
    free pattern search.

    Returns a dict with the calibrated parameters and the de-warped signal.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)

    fit_period = period is None
    # Average stage speed for the nominal profile -> better FFT period seed.
    nominal_move_time = motion_profile(distance, nominal_vmax, nominal_accel)["total_time"]
    mean_speed = distance / nominal_move_time if nominal_move_time > 0 else nominal_vmax
    period0 = period if period is not None else _period_guess(t, y, mean_speed)

    # Full nonlinear vector is [t0, accel, vmax, period]; `free` selects which
    # entries the optimiser is allowed to move.
    full0 = [0.0, nominal_accel, nominal_vmax, period0]
    lower = [-0.5, nominal_accel * 0.05, nominal_vmax * 0.05, period0 * 0.2]
    upper = [max(t[-1], 1.0), nominal_accel * 20, nominal_vmax * 20, period0 * 5]
    free = [0]                       # t0 is always free
    if fit_accel:
        free += [1, 2]
    if fit_period:
        free += [3]

    def expand(p_free, base):
        full = list(base)
        for slot, val in zip(free, p_free):
            full[slot] = val
        return full

    def model_from_full(full):
        t0, accel, vmax, per = full
        x = model_x(t, t0, accel, vmax, distance, direction, x_start)
        theta = 2 * np.pi * x / per
        coef, _ = _sine_lstsq(theta, y)
        design = np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)])
        return design @ coef

    def cost(full):
        _, accel, vmax, per = full
        if accel <= 0 or vmax <= 0 or per <= 0:
            return 1e18
        return float(np.sqrt(np.mean((model_from_full(full) - y) ** 2)))

    # Coarse multi-start on the lag t0 (the most non-convex axis) and, when the
    # period is unknown, on L too (its FFT seed can land on a harmonic), then a
    # local refine of all free params from the best start.
    window = t[-1] - t[0]
    period_starts = [period0]
    if fit_period:
        period_starts = [period0 * s for s in (0.5, 1.0, 2.0)]
    accel_starts = [nominal_accel]
    vmax_starts = [nominal_vmax]
    if fit_accel:
        accel_starts = [nominal_accel * s for s in (0.5, 1.0, 2.0, 4.0, 8.0)]
        vmax_starts = [nominal_vmax * s for s in (0.5, 1.0, 2.0)]
    best_full, best_c = None, np.inf
    for t0_start in np.linspace(0.0, min(window * 0.6, 0.4), 13):
        for per_start in period_starts:
            for accel_start in accel_starts:
                for vmax_start in vmax_starts:
                    base = [t0_start, accel_start, vmax_start, per_start]
                    c = cost(base)
                    if c < best_c:
                        best_c, best_full = c, base

    try:
        from scipy.optimize import least_squares

        def resid_free(p_free):
            full = expand(p_free, best_full)
            _, accel, vmax, per = full
            if accel <= 0 or vmax <= 0 or per <= 0:
                return np.full(len(y), 1e6)
            return model_from_full(full) - y

        sol = least_squares(
            resid_free,
            [best_full[i] for i in free],
            bounds=([lower[i] for i in free], [upper[i] for i in free]),
            method="trf",
        )
        best_full = expand(sol.x, best_full)
    except Exception:
        # No scipy: fall back to the pattern search over the free params.
        steps_full = [0.01, nominal_accel * 0.25, nominal_vmax * 0.25, period0 * 0.05]

        def cost_free(p_free):
            return cost(expand(p_free, best_full))

        p_free, _ = _pattern_search(
            cost_free, [best_full[i] for i in free], [steps_full[i] for i in free]
        )
        best_full = expand(p_free, best_full)

    t0, accel, vmax, per = best_full

    # Final linear solve for amplitude / phase / offset at the optimum.
    x_fit = model_x(t, t0, accel, vmax, distance, direction, x_start)
    coef, rms = _sine_lstsq(2 * np.pi * x_fit / per, y)
    amplitude, phase, offset = _coef_to_sine(coef)
    y_model = amplitude * np.sin(2 * np.pi * x_fit / per + phase) + offset

    # Fit quality as fraction of variance explained.
    ss_res = float(np.sum((y - y_model) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2)) or 1.0
    r2 = 1.0 - ss_res / ss_tot

    profile = motion_profile(distance, vmax, accel)
    return {
        "t0": t0,
        "acceleration": accel,
        "max_velocity": vmax,
        "period": per,
        "amplitude": amplitude,
        "phase": phase,
        "offset": offset,
        "rms": rms,
        "r2": r2,
        "profile": profile,
        "x_fit": x_fit,
        "y_model": y_model,
        "fit_period": fit_period,
        "fit_accel": fit_accel,
    }


# ----------------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------------
def report(t, y, fit, nominal_accel, nominal_vmax, distance):
    t = np.asarray(t, float)
    n = len(y)
    duration = t[-1] - t[0]
    t0 = fit["t0"]
    prof = fit["profile"]
    rise = prof["ramp_time"]
    total = prof["total_time"]

    triangular = prof["cruise_time"] <= 1e-9  # move never reaches vmax

    # Partition every sample into exactly one phase relative to fitted motion.
    elapsed = t - t0
    is_pre = elapsed < 0
    is_post = elapsed > total
    is_cruise = (~is_pre) & (~is_post) & (elapsed > rise) & (elapsed < total - rise)
    is_ramp = (~is_pre) & (~is_post) & (~is_cruise)
    n_pre = int(np.sum(is_pre))
    n_post = int(np.sum(is_post))
    n_cruise = int(np.sum(is_cruise))
    n_ramp = int(np.sum(is_ramp))

    def pct(k):
        return 100.0 * k / n if n else 0.0

    print("=" * 64)
    print("CALIBRATION RESULT")
    print("=" * 64)
    print(f"samples            : {n}  over {duration*1000:.1f} ms "
          f"({n/duration:.0f} S/s)" if duration > 0 else f"samples: {n}")
    print(f"fit quality        : R^2 = {fit['r2']:.4f}   RMS = {fit['rms']:.3f} ADC")
    print()
    print("Motion / lag calibration")
    print(f"  motion-start lag t0 : {t0*1000:8.2f} ms   "
          f"(dead samples before motion: {n_pre}, {pct(n_pre):.1f}%)")
    print(f"  acceleration        : {fit['acceleration']:8.3f} mm/s^2  "
          f"(nominal {nominal_accel:.3f})")
    if triangular:
        peak_v = prof["peak_velocity"]
        print(f"  max velocity        : NOT REACHED (triangular move) - "
              f"peaks at {peak_v:.3f} mm/s; vmax not identifiable from this line")
    else:
        print(f"  max velocity        : {fit['max_velocity']:8.3f} mm/s    "
              f"(nominal {nominal_vmax:.3f})")
    print(f"  rise time (ramp)    : {rise*1000:8.2f} ms"
          f"{'  (= half the move; no cruise)' if triangular else ''}")
    print(f"  ramp distance       : {prof['ramp_distance']*1000:8.2f} um  "
          f"(each end; cruise {prof['cruise_distance']:.3f} mm)")
    print(f"  total move time     : {total*1000:8.2f} ms")
    print()
    print("Spatial sinusoid")
    print(f"  period L            : {fit['period']*1000:8.2f} um "
          f"({'fitted' if fit['fit_period'] else 'fixed'})")
    print(f"  amplitude / offset  : {fit['amplitude']:8.2f} / {fit['offset']:.2f} ADC")
    print()
    print("Data warping / loss (fraction of the burst that is compromised)")
    print(f"  pre-motion (dead)   : {n_pre:5d}  {pct(n_pre):5.1f}%  all at x_start")
    print(f"  ramp (warped)       : {n_ramp:5d}  {pct(n_ramp):5.1f}%  spatially compressed")
    print(f"  cruise (clean)      : {n_cruise:5d}  {pct(n_cruise):5.1f}%  uniform in X")
    print(f"  post-motion (dead)  : {n_post:5d}  {pct(n_post):5.1f}%  all at x_end")
    print("=" * 64)


def dewarp_uniform(fit, y, n_out=None):
    """Resample the line onto a uniform X grid (the corrected line).

    x_fit is monotonic for a single-direction move, so np.interp is valid.
    """
    x = np.asarray(fit["x_fit"], float)
    y = np.asarray(y, float)
    order = np.argsort(x)
    xs, ys = x[order], y[order]
    if n_out is None:
        n_out = len(y)
    x_grid = np.linspace(xs[0], xs[-1], n_out)
    y_grid = np.interp(x_grid, xs, ys)
    return x_grid, y_grid


def plot_result(t, y, fit):
    import matplotlib.pyplot as plt

    t = np.asarray(t, float)
    y = np.asarray(y, float)
    x_grid, y_grid = dewarp_uniform(fit, y)

    fig, axes = plt.subplots(3, 1, figsize=(9, 9))

    # Raw: what the burst read gives you (warped, uniform in time).
    axes[0].plot(t * 1000, y, lw=0.8)
    axes[0].axvline(fit["t0"] * 1000, color="r", ls="--", lw=1,
                    label=f"motion start t0={fit['t0']*1000:.1f} ms")
    axes[0].set_xlabel("time (ms)")
    axes[0].set_ylabel("ADC")
    axes[0].set_title("Raw burst (uniform in TIME - warped by acceleration)")
    axes[0].legend(fontsize="small")
    axes[0].grid(True)

    # Data vs fitted position, with the calibrated sinusoid overlaid.
    axes[1].plot(fit["x_fit"], y, ".", ms=2, label="samples @ fitted X")
    axes[1].plot(fit["x_fit"], fit["y_model"], "r", lw=1,
                 label=f"fit sinusoid (R^2={fit['r2']:.3f})")
    axes[1].set_xlabel("X position (mm)")
    axes[1].set_ylabel("ADC")
    axes[1].set_title("De-warped: data mapped to true X")
    axes[1].legend(fontsize="small")
    axes[1].grid(True)

    # Corrected line resampled to uniform X (what you actually want to store).
    axes[2].plot(x_grid, y_grid, lw=0.9)
    axes[2].set_xlabel("X position (mm)")
    axes[2].set_ylabel("ADC")
    axes[2].set_title("Corrected line, uniform in X")
    axes[2].grid(True)

    fig.tight_layout()
    plt.show()


# ----------------------------------------------------------------------------
# Acquisition of a single line
# ----------------------------------------------------------------------------
def acquire_line():
    """Move X across one span (non-blocking) while burst-reading the ADC.

    Returns (t, samples, meta) where t is a uniform time axis (s) from read
    start and meta carries the geometry needed by the fit.
    """
    x_start = x0 - x_span / 2.0
    direction = 1.0
    x_displacement = direction * x_span

    motorx = ThorlabsModularStepperController(serial=SERIAL, channel=1, poll_ms=1)
    ser = None
    failed = True
    try:
        motorx.connect()
        if not skip_homing_check:
            print("Homing X...")
            motorx.home(wait=True, timeout_s=home_timeout_s)

        motorx.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )
        motorx.move_absolute(x_start, wait=True, real_unit=True)

        ser = open_arduino(port=ARDUINO_PORT, baud=ARDUINO_BAUD)

        read_duration = sampling_duration_for_move(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
            safety_factor=read_safety_factor,
            overhead_s=read_overhead_s,
        )
        move_time = expected_move_time(x_span, default_max_velocity, default_acceleration)
        print(f"Nominal move time {move_time*1000:.1f} ms, "
              f"reading for {read_duration*1000:.1f} ms.")

        actual_x_start = motorx.get_position(real_unit=True)
        ser.reset_input_buffer()
        motorx.move_relative(x_displacement, wait=False, real_unit=True)
        samples, timing = burst_read_binary(
            ser=ser, duration=read_duration, reset_buffer=False, return_timing=True,
        )
        motorx.wait_until_stopped(
            timeout_s=max(read_duration + 2.0, move_time * 2.0 + 2.0),
            require_motion_seen=False,
        )
        failed = False
    finally:
        if ser is not None:
            close_arduino(ser)
        if failed:
            motorx.safe_shutdown()
        else:
            motorx.disconnect()

    n = len(samples)
    duration = timing["duration"]
    t = np.linspace(0.0, duration, n) if n > 1 else np.zeros(n)
    meta = {
        "distance": x_span,
        "direction": direction,
        "x_start": actual_x_start,
    }
    return t, np.asarray(samples, float), meta


# ----------------------------------------------------------------------------
# Synthetic self-test (no hardware)
# ----------------------------------------------------------------------------
def simulate_line(true_t0=0.08, accel_scale=0.85, vmax_scale=0.90,
                  period=0.20, amplitude=90.0, offset=128.0,
                  sample_rate=4000.0, noise=2.0, seed=0):
    """Generate a warped sinusoidal line with known ground-truth parameters."""
    rng = np.random.default_rng(seed)
    x_start = x0 - x_span / 2.0
    direction = 1.0

    true_accel = default_acceleration * accel_scale
    true_vmax = default_max_velocity * vmax_scale

    read_duration = sampling_duration_for_move(
        distance=x_span, max_velocity=true_vmax, acceleration=true_accel,
        safety_factor=read_safety_factor, overhead_s=read_overhead_s,
    ) + true_t0
    n = int(read_duration * sample_rate)
    t = np.linspace(0.0, read_duration, n)

    x = model_x(t, true_t0, true_accel, true_vmax, x_span, direction, x_start)
    y = amplitude * np.sin(2 * np.pi * x / period + 0.7) + offset
    y = np.clip(np.round(y + rng.normal(0, noise, n)), 0, 255)

    truth = {"t0": true_t0, "acceleration": true_accel,
             "max_velocity": true_vmax, "period": period}
    meta = {"distance": x_span, "direction": direction, "x_start": x_start}
    return t, y, meta, truth


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    truth = None
    if SIMULATE:
        print(">>> SIMULATE mode: synthetic warped line, no hardware.\n")
        t, y, meta, truth = simulate_line()
    elif REPLAY_FILE:
        print(f">>> REPLAY mode: {REPLAY_FILE}\n")
        data = np.load(REPLAY_FILE, allow_pickle=True)
        t, y = data["t"], data["y"]
        meta = data["meta"].item()
    else:
        t, y, meta = acquire_line()
        if SAVE_FILE:
            np.savez(SAVE_FILE, t=t, y=y, meta=meta)
            print(f"Saved raw acquisition to {SAVE_FILE}")

    if SPATIAL_PERIOD_MM is None:
        print("\n[!] SPATIAL_PERIOD_MM is None: the sinusoid period will be fitted.\n"
              "    This is under-constrained for a short (triangular) move -- if you\n"
              "    know the pattern pitch, set SPATIAL_PERIOD_MM for a robust fit.\n")

    fit = fit_warp(
        t, y,
        distance=meta["distance"],
        direction=meta["direction"],
        x_start=meta["x_start"],
        nominal_accel=default_acceleration,
        nominal_vmax=default_max_velocity,
        period=SPATIAL_PERIOD_MM,
        fit_accel=FIT_ACCEL,
    )

    report(t, y, fit, default_acceleration, default_max_velocity, meta["distance"])

    if truth is not None:
        print("\nSelf-test recovery (fitted vs true):")
        print(f"  t0    : {fit['t0']*1000:7.2f} ms  vs {truth['t0']*1000:7.2f} ms")
        print(f"  accel : {fit['acceleration']:7.3f}    vs {truth['acceleration']:7.3f}")
        print(f"  vmax  : {fit['max_velocity']:7.3f}    vs {truth['max_velocity']:7.3f}")
        print(f"  period: {fit['period']*1000:7.2f} um  vs {truth['period']*1000:7.2f} um")

    try:
        plot_result(t, y, fit)
    except Exception as exc:  # headless / no display
        print(f"(plot skipped: {exc})")


if __name__ == "__main__":
    main()
