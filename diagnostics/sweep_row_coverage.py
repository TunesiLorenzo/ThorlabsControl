"""
Run a single-line scan at each (acceleration, max_velocity) combination in a
sweep, saving each to its own file and reporting the worst per-row X
coverage -- i.e. whether burst_read_binary's stop_condition fix (see
scan_engine.run_scan) holds up as motion gets faster/more aggressive, where
the old fixed safety margin was thinnest relative to real motor-start lag.

Edit the constants below and run:

    python diagnostics/sweep_row_coverage.py

Needs the stage and Arduino connected (same hardware as
scan_engine.py/scan_gui.py).
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scan_engine import _row_positions, run_scan

SWEEP_OUTPUT_DIR = Path(__file__).resolve().parent

SERIAL = "50865380"
ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 230400

X0, Y0 = 1.0, 1.0
X_SPAN = 1.0
Y_SPAN = 1  # single line per combo is enough to characterize coverage
LINE_SPACING = 0.1

ACCELERATIONS = [4]
MAX_VELOCITIES = [1,2,4]

COVERAGE_WARN_THRESHOLD = 0.97


def row_coverage(row):
    samples = row["samples"]
    if not samples:
        return None
    commanded = abs(row["x_end"] - row["x_start"])
    if commanded == 0:
        return 1.0
    xs = _row_positions(row)
    return float(np.max(xs) - np.min(xs)) / commanded


def main():
    results = []
    for acceleration in ACCELERATIONS:
        for max_velocity in MAX_VELOCITIES:
            save_file = str(SWEEP_OUTPUT_DIR / f"scan_sweep_a{acceleration:g}_v{max_velocity:g}.npz")
            print(
                f"\n=== acceleration={acceleration} mm/s^2, "
                f"max_velocity={max_velocity} mm/s ==="
            )
            scan_rows = run_scan(
                serial=SERIAL,
                arduino_port=ARDUINO_PORT,
                arduino_baud=ARDUINO_BAUD,
                x0=X0,
                y0=Y0,
                x_span=X_SPAN,
                y_span=Y_SPAN,
                line_spacing=LINE_SPACING,
                acceleration=acceleration,
                max_velocity=max_velocity,
                save_file=save_file,
            )
            coverages = [c for c in (row_coverage(row) for row in scan_rows) if c is not None]
            worst = min(coverages) if coverages else float("nan")
            results.append((acceleration, max_velocity, worst, save_file))

    print(f"\n{'accel':>8} {'max_vel':>8} {'worst coverage':>15}  file")
    for acceleration, max_velocity, worst, save_file in results:
        flag = "  <-- LOW" if worst < COVERAGE_WARN_THRESHOLD else ""
        print(f"{acceleration:>8.2f} {max_velocity:>8.2f} {worst:>15.3f}{flag}  {save_file}")


if __name__ == "__main__":
    main()
