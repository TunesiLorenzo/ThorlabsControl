# ThorlabsControl

Raster-scan controller for a two-axis Thorlabs Modular Rack stepper stage
with an Arduino-sampled analog detector (photodiode/ADC) on the X axis. The
X axis does a continuous "fly scan" (moves while the Arduino streams
samples) instead of stepping and stopping at every point, so a line scan is
one continuous move + burst read instead of many discrete point moves.

## Hardware

- **Stage**: Thorlabs Modular Rack stepper motor bench, two channels (X =
  channel 1, Y = channel 2), driven via Thorlabs Kinesis.
- **Detector readout**: Arduino running [`ArduinoCode/ArduinoCode.ino`](ArduinoCode/ArduinoCode.ino),
  free-running at 230400 baud, streaming one raw byte per ADC sample
  (10-bit `analogRead` truncated to 8 bits) on pin `A0`.
- **Kinesis DLLs**: `ThorlabsStepper.py` loads Kinesis from
  `C:\Program Files\Thorlabs\Kinesis` by default (see `KINESIS_DIR` in that
  file). The `dll/` folder in this repo holds copies of the same DLLs for
  reference; a real Kinesis install is what actually gets loaded at
  runtime. `MST602 DLL methods.pdf` documents the underlying Kinesis
  C API the driver wraps.

## Software setup

```
pip install -r requirements.txt
```

Needs `matplotlib`, `numpy`, `plotly`, and `pyserial`. Also needs Thorlabs
Kinesis installed (for the stepper DLLs) and the Arduino flashed with
`ArduinoCode/ArduinoCode.ino`.

Hardware addresses (stage serial number, Arduino COM port/baud) are plain
constants near the top of each entry-point script, not a shared config
file — update them per script to match your setup:

- `scan_gui.py`: `SERIAL`, `ARDUINO_PORT`, `ARDUINO_BAUD`
- `diagnostics/*.py`: same constants, near the top of each file

## Repository layout

```
scan_gui.py            GUI entry point (run this)
scan_engine.py          Scan engine: run_scan(), grid/line building, plotting helpers
ThorlabsStepper.py       Stepper motor driver (Kinesis DLL wrapper)
ArduinoSampler.py         Arduino serial open/close + timed burst read
motion_timing.py           Shared motion-profile math + move-start/stop lag measurement
diagnostics/                Hardware bring-up and calibration tools (not used by the GUI)
ArduinoCode/ArduinoCode.ino   Firmware actually running on the Arduino
dll/                         Reference copies of the Thorlabs Kinesis DLLs
requirements.txt
```

### `scan_gui.py` — main entry point

Tkinter GUI for configuring and running a raster scan and watching it come
in live. Run it with:

```
python scan_gui.py
```

Hardware access runs on a background thread so the window stays
responsive; the plot redraws on a timer (not once per row) so a fast scan
can't flood the UI.

Fields:

- **X/Y center, X/Y span, line spacing** (mm) — scan geometry. The stage
  covers `[center - span/2, center + span/2]` on each axis; Y is stepped
  row by row at `line_spacing`, X is fly-scanned continuously per row.
- **Acceleration / max velocity** (mm/s², mm/s) — motion profile used for
  every row.
- **Max grid points/axis** — display-only downsampling cap for the
  heatmap/surface views; the saved data always keeps every raw sample.
- **View**: 2D heatmap, 3D surface, or overlaid per-row line plot.
- **Sampler strategy**: how each ADC sample's X position is reconstructed —
  *measured position trace* (interpolated from a continuously polled real
  position log, the default and generally more accurate) or *modeled
  trapezoidal profile* (computed from the commanded acceleration/velocity
  instead of measured motion).
- **Home Motors** — homes both axes before scanning.
- **Launch Scan / Stop** — Stop finishes the current row, then ends the
  scan early instead of cutting a row off mid-move.
- **Load Saved Scan...** — reload a previously saved `.npz` scan into the
  plot without touching the hardware.

Every completed scan is saved to `scan_last.npz` (see `SAVE_FILE` in
`scan_gui.py`) and can be reloaded via "Load Saved Scan...".

### `scan_engine.py` — scan engine (library + CLI)

The engine `scan_gui.py` is built on. Its public pieces:

- `run_scan(...)` — connects to the stage + Arduino, runs the full raster
  scan (row by row: move Y, fly-scan X while bursting the Arduino, repeat),
  and returns `scan_rows` (also saved to `.npz` if `save_file` is given).
  Accepts a `progress_callback` (per-row) and a `stop_event` for GUI use,
  and per-row records rich timing/lag metadata (see
  `print_scan_timing_stats`) used by the diagnostics tools.
- `build_scan_grid(scan_rows, max_points=200)` — resamples scan rows onto a
  shared rectangular grid for heatmap/surface plotting.
- `build_scan_lines(scan_rows)` — reconstructs each row's raw (x, samples)
  trace for line-overlay plotting.
- `load_scan(path)` / `plot_scan(scan_rows)` — reload a saved scan / plot it
  with plain matplotlib (used by the CLI path, not the GUI).

It also works as a standalone script:

```
python scan_engine.py
```

which runs one scan using the constants in its `if __name__ == "__main__"`
block and plots the result with matplotlib once done — useful for a quick
scan without the GUI.

### Drivers

- **`ThorlabsStepper.py`** — `ThorlabsModularStepperController`: DLL
  bootstrap, connect/disconnect lifecycle, status/position polling,
  velocity params, homing, relative/absolute moves, jog, and stop. Works in
  device units or "real units" (mm, mm/s, mm/s²) via
  `unit_device2real`/`unit_real2device`. No scan logic lives here.
- **`ArduinoSampler.py`** — `open_arduino`/`close_arduino` (serial
  open/close with the board-reset wait) and `burst_read_binary`, which
  reads one timed burst of single-byte samples. `duration` is a minimum,
  not a hard cutoff: pass `stop_condition` (e.g.
  `MoveStartLagMonitor.has_stopped`) to keep reading past `duration` until
  the stage has actually finished moving, up to `hard_timeout` — this is
  what stops a fast row from being truncated if the real move ran longer
  than modeled.
- **`motion_timing.py`** — shared math used by both the scan engine and the
  diagnostics tools: `motion_profile`/`expected_move_time`/
  `sampling_duration_for_move` (trapezoidal velocity profile), functions to
  place ADC samples on the X axis either from the modeled profile or from a
  measured position trace, and `MoveStartLagMonitor` — a background thread
  that polls the stage's cached position/status to measure, in real time,
  when a move actually started and actually stopped (instead of assuming
  command-issue/command-return timing), which the scan engine uses to place
  samples accurately and to know when it's safe to stop reading.

## `diagnostics/` — calibration and hardware bring-up tools

Standalone scripts, not imported by the GUI. Each is configured by editing
constants near the top of the file, then run directly; each adds the repo
root to `sys.path` itself, so run them from anywhere, e.g.:

```
python diagnostics/timing_check.py
```

- **`timing_check.py`** — live timing check of the move → burst-read
  pipeline. Runs several single-line moves with a real Arduino burst
  attached (the same `MoveStartLagMonitor` + `burst_read_binary` machinery
  `run_scan` uses) and reports, per trial: the delay between issuing the
  move command and the burst-read loop actually starting, the real
  measured motor-start lag, the real measured motor-stop time, when the
  burst read actually stopped, and the delta between the two. That delta
  should always be ≥ 0 (the burst is supposed to keep running until the
  motor has actually stopped); a negative delta or an undetected motor-stop
  is flagged as a real timing bug. Use this to check for latency/gain
  issues between the move command and the burst read, and whether
  `burst_read_binary` is stopping at the right time relative to the motor.

- **`calibrate_line.py`** — acquires one line (or simulates/replays one)
  and maps the raw time-ordered ADC burst onto X position using the
  trapezoidal motion profile, reporting/plotting how much of the line was
  lost to the command→burst dead time and how the retained samples
  distribute across ramp/cruise/post-motion. Has a `SIMULATE` mode (no
  hardware, synthetic waveform) and a `REPLAY_FILE` mode (re-analyze a
  previous acquisition) for offline use. Saves to
  `diagnostics/calibrate_line_last.npz`.

- **`sweep_row_coverage.py`** — runs a real single-line scan (via
  `scan_engine.run_scan`) at every combination of `ACCELERATIONS` ×
  `MAX_VELOCITIES`, saving each to its own `.npz` in `diagnostics/` and
  reporting the worst per-row X coverage (captured span ÷ commanded span)
  per combination — i.e. whether the burst-read stop-condition fix holds up
  as motion gets faster/more aggressive.

## Scan data format

`run_scan` / `scan_gui.py` produce `scan_rows`: a list of one dict per
scanned row, saved as `scan_rows=np.array(scan_rows, dtype=object)` in an
`.npz` file. Each row dict has the raw ADC `samples`, the commanded/actual
X and Y positions, the acceleration/velocity used, and a set of timing
fields (`sample_window_start_s`, `sample_span_s`, `motion_start_offset_s`,
`measured_lag_*_s`, etc.) that `scan_engine._row_positions` uses to place
samples on the X axis, and that `scan_engine.print_scan_timing_stats`
summarizes after a scan. `load_scan(path)` reloads this format.
