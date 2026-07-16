"""
Simple Tkinter GUI for configuring and launching a raster scan (see
Main.run_scan) and visualizing the result as it comes in, as either a 2D
heatmap or a rotatable 3D surface (drag to rotate, matplotlib's mplot3d).

Hardware access happens on a background thread so the window stays
responsive; the scan reports progress row-by-row through a queue, and the
plot redraws at a fixed interval (not once per row) so a fast scan can't
flood the UI with redraws. The (potentially not-cheap, and growing as a
scan progresses) grid/line rebuild itself runs on a dedicated single-worker
background thread too -- see _service_render() -- so a slow redraw can't
hold the GIL long enough to starve the scan worker thread's serial reads
mid-row (this used to cause dropped samples / blank spots in a row; see
diagnostics/timing_check.py).
"""

import concurrent.futures
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the '3d' projection)

from scan_engine import (
    build_scan_grid_incremental,
    build_scan_lines_incremental,
    check_connection_and_home,
    find_scan_max,
    load_scan,
    run_jog_scan,
    run_scan,
)
from ThorlabsStepper import ThorlabsModularStepperController
from ThorlabsNanoTrak import ThorlabsModularNanoTrak

SERIAL = "50865380"
NANOTRAK_SERIAL = "52849313"
ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 230400
SAVE_FILE = "scan_last.npz"

ROW_SETTLE_S = 2.0
READ_SAFETY_FACTOR = 1.10
READ_OVERHEAD_S = 0.05
SKIP_HOMING_CHECK = True
HOME_TIMEOUT_S = 30.0

REDRAW_INTERVAL_MS = 250
DEFAULT_MAX_GRID_POINTS = 200
JOG_SAMPLE_DURATION_S = 0.05


class CircularActionButton(tk.Canvas):
    """Compact circular, canvas-backed action button."""

    def __init__(self, master, command, symbol, diameter=30):
        background = ttk.Style().lookup("TFrame", "background") or "#d9d9d9"
        super().__init__(
            master,
            width=diameter,
            height=diameter,
            background=background,
            highlightthickness=0,
            cursor="hand2",
            takefocus=True,
        )
        self.command = command
        self._state = "normal"
        self._circle = self.create_oval(2, 2, diameter - 2, diameter - 2, width=1)
        self._symbol = self.create_text(
            diameter / 2, diameter / 2, text=symbol, font=("TkDefaultFont", 12)
        )
        self.bind("<Button-1>", self._invoke)
        self.bind("<Return>", self._invoke)
        self.bind("<space>", self._invoke)
        self._update_appearance()

    def _invoke(self, _event=None):
        if self._state == "normal":
            self.command()

    def _update_appearance(self):
        disabled = self._state == "disabled"
        self.itemconfigure(self._circle, fill="#d9d9d9" if disabled else "#f4f4f4")
        self.itemconfigure(self._symbol, fill="#888888" if disabled else "#222222")
        self.configure(cursor="" if disabled else "hand2")

    def configure(self, cnf=None, **kwargs):
        if cnf and "state" in cnf:
            kwargs["state"] = cnf["state"]
            cnf = {key: value for key, value in cnf.items() if key != "state"}
        state = kwargs.pop("state", None)
        result = super().configure(cnf, **kwargs)
        if state is not None:
            self._state = str(state)
            self._update_appearance()
        return result

    config = configure


class ManualControlWindow:
    """
    Controls for jogging/positioning the X/Y stage outside of a scan. The UI
    can be hosted in a standalone window or embedded in the main notebook.
    Motors are provided by ScanGUI's shared, persistent connection (see
    ScanGUI._get_shared_motors) rather than owned by this window -- closing
    it just stops referencing them, it does not disconnect.

    All Kinesis calls happen on a single dedicated worker thread (commands
    pushed through self._command_queue, results read back through
    self._result_queue and applied to widgets from the Tk main loop via
    after()-polling) so overlapping button clicks can't call into the DLL
    from two threads at once. STOP is the one exception -- it calls
    stop_profiled() directly from the UI thread so it takes effect
    immediately even while a move is in flight on the worker thread.
    """

    def __init__(self, master, get_motors, defaults, on_close=None, embedded=False):
        self.get_motors = get_motors
        self.on_close = on_close
        self._result_queue = queue.Queue()
        self._command_queue = queue.Queue()
        self.motorx = None
        self.motory = None
        self._closed = False

        self._embedded = embedded
        if embedded:
            self.top = master
        else:
            self.top = tk.Toplevel(master)
            self.top.title("Manual XY Control")
            self.top.protocol("WM_DELETE_WINDOW", self._on_window_close)

        self.status_var = tk.StringVar(value="Connecting...")
        self.x_pos_var = tk.StringVar(value="--")
        self.y_pos_var = tk.StringVar(value="--")

        self.step_var = tk.StringVar(value="0.01")
        self.accel_var = tk.StringVar(value=defaults.get("acceleration", "4.0"))
        self.vel_var = tk.StringVar(value=defaults.get("max_velocity", "4.0"))
        self.abs_x_var = tk.StringVar(value=defaults.get("x0", "0.0"))
        self.abs_y_var = tk.StringVar(value=defaults.get("y0", "0.0"))

        self._build_ui()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._command_queue.put(("connect",))

        self.top.after(150, self._poll_queue)

    # ---------------------------------------------------------------- UI

    def _build_ui(self):
        frame = ttk.Frame(self.top, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        pos_frame = ttk.LabelFrame(frame, text="Current position", padding=8)
        pos_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(pos_frame, text="X (mm):").grid(row=0, column=0, sticky="w")
        ttk.Label(pos_frame, textvariable=self.x_pos_var).grid(
            row=0, column=1, sticky="w", padx=(4, 16)
        )
        ttk.Label(pos_frame, text="Y (mm):").grid(row=0, column=2, sticky="w")
        ttk.Label(pos_frame, textvariable=self.y_pos_var).grid(
            row=0, column=3, sticky="w", padx=(4, 0)
        )
        self.refresh_btn = ttk.Button(pos_frame, text="Refresh", command=self._on_refresh)
        self.refresh_btn.grid(row=0, column=4, padx=(12, 0))

        speed_frame = ttk.LabelFrame(frame, text="Speed", padding=8)
        speed_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(speed_frame, text="Acceleration (mm/s^2)").grid(row=0, column=0, sticky="w")
        ttk.Entry(speed_frame, textvariable=self.accel_var, width=10).grid(
            row=0, column=1, padx=(6, 12)
        )
        ttk.Label(speed_frame, text="Max velocity (mm/s)").grid(row=0, column=2, sticky="w")
        ttk.Entry(speed_frame, textvariable=self.vel_var, width=10).grid(
            row=0, column=3, padx=(6, 12)
        )
        self.apply_speed_btn = ttk.Button(speed_frame, text="Apply", command=self._on_apply_speed)
        self.apply_speed_btn.grid(row=0, column=4)

        abs_frame = ttk.LabelFrame(frame, text="Absolute move", padding=8)
        abs_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(abs_frame, text="X target (mm)").grid(row=0, column=0, sticky="w")
        ttk.Entry(abs_frame, textvariable=self.abs_x_var, width=10).grid(
            row=0, column=1, padx=(6, 12)
        )
        ttk.Label(abs_frame, text="Y target (mm)").grid(row=0, column=2, sticky="w")
        ttk.Entry(abs_frame, textvariable=self.abs_y_var, width=10).grid(
            row=0, column=3, padx=(6, 12)
        )
        self.move_abs_btn = ttk.Button(abs_frame, text="Move", command=self._on_move_absolute)
        self.move_abs_btn.grid(row=0, column=4)

        jog_frame = ttk.LabelFrame(frame, text="Jog", padding=8)
        jog_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(jog_frame, text="Step (mm)").grid(row=0, column=0, sticky="w")
        ttk.Entry(jog_frame, textvariable=self.step_var, width=10).grid(
            row=0, column=1, padx=(6, 0), pady=(0, 8), sticky="w"
        )

        pad_frame = ttk.Frame(jog_frame)
        pad_frame.grid(row=1, column=0, columnspan=4)
        self.jog_buttons = {}
        specs = [
            ("y+", "▲ Y+", 0, 1),
            ("x-", "◀ X-", 1, 0),
            ("x+", "X+ ▶", 1, 2),
            ("y-", "▼ Y-", 2, 1),
        ]
        for key, label, r, c in specs:
            btn = ttk.Button(pad_frame, text=label, width=8, command=lambda k=key: self._on_jog(k))
            btn.grid(row=r, column=c, padx=4, pady=4)
            self.jog_buttons[key] = btn

        self.stop_btn = ttk.Button(frame, text="STOP", command=self._on_stop)
        self.stop_btn.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(4, 8))

        ttk.Label(frame, textvariable=self.status_var, wraplength=280).grid(
            row=5, column=0, columnspan=2, sticky="w"
        )

        self._busy_widgets = [
            self.refresh_btn,
            self.apply_speed_btn,
            self.move_abs_btn,
        ] + list(self.jog_buttons.values())
        self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for widget in self._busy_widgets:
            widget.configure(state=state)

    # ----------------------------------------------------- worker thread

    def _worker_loop(self):
        while True:
            command = self._command_queue.get()
            name = command[0]
            if name == "shutdown":
                return
            try:
                push_status = True
                if name == "connect":
                    self._cmd_connect()
                elif name == "refresh":
                    pass
                elif name == "move_absolute":
                    self._cmd_move_absolute(command[1], command[2])
                elif name == "jog":
                    self._cmd_jog(command[1], command[2])
                elif name == "set_speed":
                    self._cmd_set_speed(command[1], command[2])
                self._push_positions()
                self._result_queue.put(("ready",))
            except Exception as exc:
                self._result_queue.put(("error", str(exc)))

    def _cmd_connect(self):
        self.motorx, self.motory = self.get_motors()
        try:
            accel = float(self.accel_var.get())
            max_vel = float(self.vel_var.get())
            self._cmd_set_speed(accel, max_vel)
        except ValueError:
            pass
        self._result_queue.put(("connected",))

    def _cmd_move_absolute(self, x_target, y_target):
        self.motorx.move_absolute(x_target, wait=True, real_unit=True)
        self.motory.move_absolute(y_target, wait=True, real_unit=True)

    def _cmd_jog(self, axis, delta):
        motor = self.motorx if axis == "x" else self.motory
        motor.move_relative(delta, wait=True, real_unit=True)

    def _cmd_set_speed(self, acceleration, max_velocity):
        self.motorx.set_velocity_params(
            acceleration=acceleration, max_velocity=max_velocity, real_unit=True
        )
        self.motory.set_velocity_params(
            acceleration=acceleration, max_velocity=max_velocity, real_unit=True
        )

    def _push_positions(self):
        x = self.motorx.get_position(real_unit=True)
        y = self.motory.get_position(real_unit=True)
        self._result_queue.put(("position", x, y))

    # --------------------------------------------------- UI command senders

    def _send(self, command):
        self._set_controls_enabled(False)
        self.status_var.set("Working...")
        self._command_queue.put(command)

    def _on_refresh(self):
        self._send(("refresh",))

    def _on_apply_speed(self):
        try:
            accel = float(self.accel_var.get())
            max_vel = float(self.vel_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Acceleration and max velocity must be numbers.")
            return
        self._send(("set_speed", accel, max_vel))

    def _on_move_absolute(self):
        try:
            x_target = float(self.abs_x_var.get())
            y_target = float(self.abs_y_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "X/Y target must be numbers.")
            return
        self._send(("move_absolute", x_target, y_target))

    def _on_jog(self, key):
        try:
            step = float(self.step_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Jog step must be a number.")
            return
        if step <= 0:
            messagebox.showerror("Invalid input", "Jog step must be positive.")
            return
        axis = "x" if key.startswith("x") else "y"
        sign = 1.0 if key.endswith("+") else -1.0
        self._send(("jog", axis, sign * step))

    def _on_stop(self):
        for motor in (self.motorx, self.motory):
            if motor is not None:
                try:
                    motor.stop_profiled()
                except Exception:
                    pass
        self.status_var.set("Stop requested.")

    # -------------------------------------------------------- queue polling

    def _poll_queue(self):
        if self._closed:
            return
        try:
            while True:
                message = self._result_queue.get_nowait()
                kind = message[0]
                if kind == "connected":
                    self.status_var.set("Connected.")
                    self._set_controls_enabled(True)
                elif kind == "position":
                    self.x_pos_var.set(f"{message[1]:.6f}")
                    self.y_pos_var.set(f"{message[2]:.6f}")
                elif kind == "ready":
                    self.status_var.set("Ready.")
                    self._set_controls_enabled(True)
                elif kind == "error":
                    self.status_var.set(f"Error: {message[1]}")
                    messagebox.showerror("Motor error", message[1])
                    self._set_controls_enabled(True)
        except queue.Empty:
            pass
        self.top.after(150, self._poll_queue)

    # ------------------------------------------------------------- closing

    def close(self, wait=False):
        if self._closed:
            return
        self._closed = True
        self._command_queue.put(("shutdown",))
        if wait and threading.current_thread() is not self._worker:
            self._worker.join()
        if not self._embedded:
            try:
                self.top.destroy()
            except tk.TclError:
                pass

    def _on_window_close(self):
        self.close()
        if self.on_close is not None:
            self.on_close()


class NanoTrakControlWindow:
    """Embeddable manual piezo and automatic rack NanoTrak controls."""

    FEEDBACK_LABELS = [
        ThorlabsModularNanoTrak.FEEDBACK_NAMES[key]
        for key in sorted(ThorlabsModularNanoTrak.FEEDBACK_NAMES)
    ]
    FEEDBACK_BY_LABEL = {
        label: key for key, label in ThorlabsModularNanoTrak.FEEDBACK_NAMES.items()
    }

    def __init__(
        self,
        master,
        serial,
        nanotrak_serial,
        defaults,
        on_close=None,
        embedded=False,
        include_motor_controls=True,
    ):
        self.serial = serial
        self.nanotrak_serial = nanotrak_serial
        self.defaults = defaults
        self.on_close = on_close
        self._embedded = embedded
        self.include_motor_controls = include_motor_controls
        self.controller = None
        self.motorx = None
        self.motory = None
        self._closed = False
        self._closing = False
        self._connected = False
        self._trace = []
        self._last_position = None
        self._track_radius_nt = None
        self._manual_active = False
        self._feedback_source = ThorlabsModularNanoTrak.FEEDBACK_TIA
        self._command_queue = queue.Queue()
        self._result_queue = queue.Queue()

        if embedded:
            self.top = master
        else:
            self.top = tk.Toplevel(master)
            self.top.title("Piezo / Auto Track")
            self.top.protocol("WM_DELETE_WINDOW", self.close)

        self.h_var = tk.StringVar(value="--")
        self.v_var = tk.StringVar(value="--")
        self.target_h_var = tk.StringVar(value="50.0")
        self.target_v_var = tk.StringVar(value="50.0")
        self.track_radius_var = tk.StringVar(value="0.5")
        self.track_frequency_var = tk.StringVar(value="17.5")
        self.mode_var = tk.StringVar(value="--")
        self.signal_label_var = tk.StringVar(value="Optical power (PIN/TIA):")
        self.signal_var = tk.StringVar(value="--")
        self.offset_var = tk.StringVar(value="Offset from center: --")
        self.motor_x_var = tk.StringVar(value="--")
        self.motor_y_var = tk.StringVar(value="--")
        self.motor_step_var = tk.StringVar(value="0.01")
        self.feedback_var = tk.StringVar(value=self.FEEDBACK_LABELS[0])
        self.chan_a_var = tk.StringVar(value="1")
        self.chan_b_var = tk.StringVar(value="2")
        self.phase_h_var = tk.StringVar(value="0")
        self.phase_v_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="Connecting...")
        self._build_ui()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._command_queue.put(("connect",))
        self.top.after(150, self._poll_queue)

    def _build_ui(self):
        frame = ttk.Frame(self.top, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        readback = ttk.LabelFrame(frame, text="NanoTrak status", padding=8)
        readback.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(readback, text="Horizontal:").grid(row=0, column=0, sticky="w")
        ttk.Label(readback, textvariable=self.h_var, width=12).grid(row=0, column=1)
        ttk.Label(readback, text="Vertical:").grid(row=0, column=2, sticky="w")
        ttk.Label(readback, textvariable=self.v_var, width=12).grid(row=0, column=3)
        ttk.Label(readback, text="Mode:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Label(readback, textvariable=self.mode_var, width=12).grid(
            row=1, column=1, pady=(5, 0)
        )
        ttk.Label(readback, textvariable=self.signal_label_var).grid(
            row=1, column=2, sticky="w", pady=(5, 0)
        )
        ttk.Label(readback, textvariable=self.signal_var, width=18).grid(
            row=1, column=3, pady=(5, 0)
        )
        self.refresh_btn = ttk.Button(readback, text="Refresh", command=self._on_refresh)
        self.refresh_btn.grid(row=0, column=4, rowspan=2, padx=(10, 0))

        tracking = ttk.LabelFrame(frame, text="Live NanoTrak H/V grid", padding=8)
        tracking.grid(row=1, column=0, rowspan=4, sticky="n", padx=(0, 8), pady=(0, 8))
        self.tracking_canvas = tk.Canvas(
            tracking,
            width=320,
            height=280,
            background="#101820",
            highlightthickness=1,
            highlightbackground="#777777",
        )
        self.tracking_canvas.grid(row=0, column=0, columnspan=2)
        ttk.Label(tracking, textvariable=self.offset_var).grid(
            row=1, column=0, sticky="w", pady=(6, 0)
        )
        self.clear_trace_btn = ttk.Button(
            tracking, text="Clear trace", command=self._clear_tracking_trace
        )
        self.clear_trace_btn.grid(row=1, column=1, sticky="e", pady=(6, 0))
        self._draw_tracking_grid()

        target = ttk.LabelFrame(frame, text="Piezo position (% of range)", padding=8)
        target.grid(row=1, column=1, sticky="ew", pady=(0, 8))
        ttk.Label(target, text="Horizontal").grid(row=0, column=0, sticky="w")
        self.target_h_entry = ttk.Entry(target, textvariable=self.target_h_var, width=10)
        self.target_h_entry.grid(row=0, column=1, padx=(6, 12))
        ttk.Label(target, text="Vertical").grid(row=0, column=2, sticky="w")
        self.target_v_entry = ttk.Entry(target, textvariable=self.target_v_var, width=10)
        self.target_v_entry.grid(row=0, column=3, padx=(6, 12))
        self.set_btn = ttk.Button(target, text="Set", command=self._on_set_position)
        self.set_btn.grid(row=0, column=4)

        modes = ttk.LabelFrame(frame, text="Piezo mode and tracking", padding=8)
        modes.grid(row=2, column=1, sticky="ew", pady=(0, 8))
        self.mode_buttons = []
        for column, (label, mode) in enumerate(
            (
                ("Manual", "manual"),
                ("Latch", ThorlabsModularNanoTrak.MODE_LATCH),
                ("Track", ThorlabsModularNanoTrak.MODE_TRACKING),
            )
        ):
            button = ttk.Button(modes, text=label, command=lambda m=mode: self._on_mode(m))
            button.grid(row=0, column=column, padx=3, sticky="ew")
            modes.columnconfigure(column, weight=1)
            self.mode_buttons.append(button)
        ttk.Label(modes, text="NT track radius (0–5):").grid(
            row=1, column=0, sticky="w", padx=3, pady=(8, 0)
        )
        self.track_radius_entry = ttk.Entry(
            modes, textvariable=self.track_radius_var, width=10
        )
        self.track_radius_entry.grid(row=1, column=1, padx=3, pady=(8, 0))
        self.radius_apply_btn = ttk.Button(
            modes, text="Set radius", command=self._on_set_track_radius
        )
        self.radius_apply_btn.grid(row=1, column=2, padx=3, pady=(8, 0), sticky="ew")
        ttk.Label(modes, text="Track frequency (17.5-87.5 Hz):").grid(
            row=2, column=0, sticky="w", padx=3, pady=(6, 0)
        )
        self.track_frequency_entry = ttk.Entry(
            modes, textvariable=self.track_frequency_var, width=10
        )
        self.track_frequency_entry.grid(
            row=2, column=1, sticky="w", padx=3, pady=(6, 0)
        )
        self.frequency_apply_btn = ttk.Button(
            modes, text="Set frequency", command=self._on_set_track_frequency
        )
        self.frequency_apply_btn.grid(
            row=2, column=2, sticky="ew", padx=3, pady=(6, 0)
        )

        settings = ttk.LabelFrame(frame, text="Piezo / NanoTrak settings", padding=8)
        settings.grid(row=3, column=1, sticky="ew", pady=(0, 8))
        ttk.Label(settings, text="Signal input:").grid(row=0, column=0, sticky="w")
        self.feedback_combo = ttk.Combobox(
            settings,
            textvariable=self.feedback_var,
            values=self.FEEDBACK_LABELS,
            state="readonly",
            width=22,
        )
        self.feedback_combo.grid(row=0, column=1, columnspan=2, padx=(6, 12), sticky="w")
        self.feedback_apply_btn = ttk.Button(
            settings, text="Apply", command=self._on_apply_feedback
        )
        self.feedback_apply_btn.grid(row=0, column=3)

        ttk.Label(settings, text="NT channel A:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.chan_a_entry = ttk.Entry(settings, textvariable=self.chan_a_var, width=6)
        self.chan_a_entry.grid(row=1, column=1, padx=(6, 12), pady=(6, 0), sticky="w")
        ttk.Label(settings, text="Channel B:").grid(row=1, column=2, sticky="w", pady=(6, 0))
        self.chan_b_entry = ttk.Entry(settings, textvariable=self.chan_b_var, width=6)
        self.chan_b_entry.grid(row=1, column=3, padx=(6, 0), pady=(6, 0), sticky="w")
        self.channel_apply_btn = ttk.Button(
            settings, text="Apply", command=self._on_apply_channels
        )
        self.channel_apply_btn.grid(row=1, column=4, pady=(6, 0))

        ttk.Label(settings, text="Phase H:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.phase_h_entry = ttk.Entry(settings, textvariable=self.phase_h_var, width=6)
        self.phase_h_entry.grid(row=2, column=1, padx=(6, 12), pady=(6, 0), sticky="w")
        ttk.Label(settings, text="Phase V:").grid(row=2, column=2, sticky="w", pady=(6, 0))
        self.phase_v_entry = ttk.Entry(settings, textvariable=self.phase_v_var, width=6)
        self.phase_v_entry.grid(row=2, column=3, padx=(6, 0), pady=(6, 0), sticky="w")
        self.phase_apply_btn = ttk.Button(
            settings, text="Apply", command=self._on_apply_phase
        )
        self.phase_apply_btn.grid(row=2, column=4, pady=(6, 0))

        motors = ttk.LabelFrame(frame, text="Stepper trim while observing tracking", padding=8)
        motors.grid(row=4, column=1, sticky="ew", pady=(0, 8))
        ttk.Label(motors, text="X (mm):").grid(row=0, column=0, sticky="w")
        ttk.Label(motors, textvariable=self.motor_x_var, width=11).grid(row=0, column=1)
        ttk.Label(motors, text="Y (mm):").grid(row=0, column=2, sticky="w")
        ttk.Label(motors, textvariable=self.motor_y_var, width=11).grid(row=0, column=3)
        ttk.Label(motors, text="Step (mm):").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(motors, textvariable=self.motor_step_var, width=10).grid(
            row=1, column=1, sticky="w", pady=(6, 0)
        )
        motor_pad = ttk.Frame(motors)
        motor_pad.grid(row=2, column=0, columnspan=4, pady=(6, 0))
        self.motor_jog_buttons = []
        for text, axis, direction, row, column in (
            ("▲ Y+", "y", 1, 0, 1),
            ("◀ X-", "x", -1, 1, 0),
            ("X+ ▶", "x", 1, 1, 2),
            ("▼ Y-", "y", -1, 2, 1),
        ):
            button = ttk.Button(
                motor_pad,
                text=text,
                width=8,
                command=lambda a=axis, d=direction: self._on_motor_jog(a, d),
            )
            button.grid(row=row, column=column, padx=4, pady=3)
            self.motor_jog_buttons.append(button)
        if not self.include_motor_controls:
            motors.grid_remove()

        ttk.Label(frame, textvariable=self.status_var, wraplength=760).grid(
            row=5, column=0, columnspan=2, sticky="w"
        )
        self._busy_widgets = [
            self.refresh_btn,
            self.set_btn,
            self.clear_trace_btn,
            self.feedback_apply_btn,
            self.channel_apply_btn,
            self.phase_apply_btn,
            self.radius_apply_btn,
            self.frequency_apply_btn,
            *self.mode_buttons,
            *(self.motor_jog_buttons if self.include_motor_controls else []),
        ]
        self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for widget in self._busy_widgets:
            widget.configure(state=state)

    def _draw_tracking_grid(self):
        canvas = self.tracking_canvas
        canvas.delete("grid")
        left, top, right, bottom = 36, 12, 308, 250
        canvas.create_rectangle(
            left, top, right, bottom, outline="#8aa0ad", width=1, tags="grid"
        )
        for value in range(0, 101, 10):
            x = left + (right - left) * value / 100.0
            y = bottom - (bottom - top) * value / 100.0
            color = "#657984" if value == 50 else "#2b414d"
            width = 2 if value == 50 else 1
            canvas.create_line(x, top, x, bottom, fill=color, width=width, tags="grid")
            canvas.create_line(left, y, right, y, fill=color, width=width, tags="grid")
            if value % 20 == 0:
                canvas.create_text(
                    x,
                    bottom + 12,
                    text=str(value),
                    fill="#c5d2d9",
                    font=("TkDefaultFont", 7),
                    tags="grid",
                )
                canvas.create_text(
                    left - 15,
                    y,
                    text=str(value),
                    fill="#c5d2d9",
                    font=("TkDefaultFont", 7),
                    tags="grid",
                )
        canvas.create_text(
            (left + right) / 2, 276, text="Horizontal piezo (%)", fill="#e2edf2", tags="grid"
        )
        canvas.create_text(
            9,
            (top + bottom) / 2,
            text="Vertical piezo (%)",
            angle=90,
            fill="#e2edf2",
            tags="grid",
        )

    def _update_tracking_grid(self, horizontal, vertical, signal_good):
        horizontal = max(0.0, min(100.0, horizontal))
        vertical = max(0.0, min(100.0, vertical))
        self._trace.append((horizontal, vertical))
        self._trace = self._trace[-300:]

        left, top, right, bottom = 36, 12, 308, 250
        to_canvas = lambda h, v: (
            left + (right - left) * h / 100.0,
            bottom - (bottom - top) * v / 100.0,
        )
        canvas = self.tracking_canvas
        canvas.delete("tracking")
        canvas.delete("track_radius")
        if len(self._trace) > 1:
            points = []
            for h, v in self._trace:
                points.extend(to_canvas(h, v))
            canvas.create_line(
                *points, fill="#38bdf8", width=2, smooth=False, tags="tracking"
            )
        x, y = to_canvas(horizontal, vertical)
        self._last_position = (horizontal, vertical)
        if self._track_radius_nt is not None:
            # The full H/V range is 10 NT units, so one NT unit is 10% of
            # either plotted piezo range.  Keep separate X/Y pixel radii
            # because the plotting rectangle is not square.
            radius_percent = self._track_radius_nt * 10.0
            radius_x = (right - left) * radius_percent / 100.0
            radius_y = (bottom - top) * radius_percent / 100.0
            canvas.create_oval(
                x - radius_x,
                y - radius_y,
                x + radius_x,
                y + radius_y,
                outline="#fbbf24",
                width=2,
                dash=(5, 3),
                tags="track_radius",
            )
            canvas.create_text(
                right - 4,
                top + 8,
                anchor="ne",
                text=f"Radius {self._track_radius_nt:.3g} NT",
                fill="#fbbf24",
                tags="track_radius",
            )
        if signal_good is None:
            marker = "#fbbf24"
        else:
            marker = "#4ade80" if signal_good else "#fb7185"
        canvas.create_line(x - 8, y, x + 8, y, fill=marker, width=2, tags="tracking")
        canvas.create_line(x, y - 8, x, y + 8, fill=marker, width=2, tags="tracking")
        canvas.create_oval(x - 4, y - 4, x + 4, y + 4, outline=marker, width=2, tags="tracking")
        self.offset_var.set(
            f"Offset from center: H={horizontal - 50.0:+.3f}%, V={vertical - 50.0:+.3f}%"
        )

    def _clear_tracking_trace(self):
        self._trace.clear()
        self.tracking_canvas.delete("tracking")

    def _worker_loop(self):
        while True:
            try:
                command = self._command_queue.get(timeout=0.05)
            except queue.Empty:
                if self._connected and not self._closing:
                    try:
                        self._push_status()
                    except Exception as exc:
                        self._connected = False
                        self._result_queue.put(("error", str(exc)))
                continue
            name = command[0]
            if name == "shutdown":
                for device in (self.motorx, self.motory, self.controller):
                    if device is not None:
                        device.safe_shutdown()
                self._result_queue.put(("closed",))
                return
            try:
                push_status = True
                if name == "connect":
                    self.controller = ThorlabsModularNanoTrak(self.nanotrak_serial, poll_ms=50)
                    self.controller.connect()
                    # Default to Latch so the outputs hold still and tracking
                    # does not start dithering as soon as the device connects.
                    self.controller.set_mode(self.controller.MODE_LATCH)
                    if self.include_motor_controls:
                        self.motorx = ThorlabsModularStepperController(
                            serial=self.serial,
                            channel=1,
                            kinesis_dir=self.controller.kinesis_dir,
                            poll_ms=50,
                        )
                        self.motory = ThorlabsModularStepperController(
                            serial=self.serial,
                            channel=2,
                            kinesis_dir=self.controller.kinesis_dir,
                            poll_ms=50,
                        )
                        self.motorx.connect()
                        self.motory.connect()
                        try:
                            acceleration = float(self.defaults.get("acceleration", "4.0"))
                            max_velocity = float(self.defaults.get("max_velocity", "4.0"))
                            self.motorx.set_velocity_params(
                                acceleration, max_velocity, real_unit=True
                            )
                            self.motory.set_velocity_params(
                                acceleration, max_velocity, real_unit=True
                            )
                        except ValueError:
                            pass
                    self._connected = True
                    self._result_queue.put(("connected",))
                    if self.include_motor_controls:
                        self._push_motor_positions()
                    self._push_settings(refresh=True)
                elif name == "set_position":
                    self.controller.set_position_percent(command[1], command[2])
                    self._result_queue.put(("manual_active", True))
                elif name == "set_mode":
                    if command[1] == "manual":
                        self.controller.set_mode(self.controller.MODE_LATCH)
                        self._result_queue.put(("manual_active", True))
                    else:
                        self.controller.set_mode(command[1])
                        self._result_queue.put(("manual_active", False))
                elif name == "motor_jog":
                    motor = self.motorx if command[1] == "x" else self.motory
                    motor.move_relative(command[2], wait=True, real_unit=True)
                    self._push_motor_positions()
                elif name == "get_settings":
                    self._push_settings(refresh=True)
                elif name == "set_track_radius":
                    radius = self.controller.set_track_radius_nt(command[1])
                    self._result_queue.put(("setting_applied", "radius", radius))
                    push_status = False
                elif name == "set_track_frequency":
                    frequency = self.controller.set_track_frequency_hz(command[1])
                    self._result_queue.put(
                        ("setting_applied", "frequency", frequency)
                    )
                    push_status = False
                elif name == "set_feedback_source":
                    source = self.controller.set_feedback_source(command[1])
                    self._result_queue.put(("setting_applied", "feedback", source))
                    push_status = False
                elif name == "set_nt_channels":
                    channels = self.controller.set_nt_channels(command[1], command[2])
                    self._result_queue.put(
                        ("setting_applied", "channels", *channels)
                    )
                    push_status = False
                elif name == "set_phase_compensation":
                    phase = self.controller.set_phase_compensation(
                        command[1], command[2]
                    )
                    self._result_queue.put(("setting_applied", "phase", *phase))
                    push_status = False
                self._connected = True
                if push_status:
                    self._push_status()
                self._result_queue.put(("ready",))
            except Exception as exc:
                self._result_queue.put(("error", str(exc)))

    def _push_status(self):
        h, v = self.controller.get_position_percent()
        mode = self.controller.get_mode()
        signal_good, reading = self.controller.get_signal()
        feedback_source = self.controller.get_feedback_source()
        self._result_queue.put(
            ("status", h, v, mode, signal_good, reading, feedback_source)
        )

    def _push_motor_positions(self):
        x = self.motorx.get_position(real_unit=True)
        y = self.motory.get_position(real_unit=True)
        self._result_queue.put(("motor_position", x, y))

    def _push_settings(self, refresh=False):
        if refresh:
            self.controller.refresh_settings_cache()
        feedback_source = self.controller.get_feedback_source()
        chan_a, chan_b = self.controller.get_nt_channels()
        phase_h, phase_v = self.controller.get_phase_compensation()
        track_radius = self.controller.get_track_radius_nt()
        track_frequency = self.controller.get_track_frequency_hz()
        self._result_queue.put(
            (
                "settings",
                feedback_source,
                chan_a,
                chan_b,
                phase_h,
                phase_v,
                track_radius,
                track_frequency,
            )
        )

    def _send(self, command):
        if self._closing:
            return
        self._set_controls_enabled(False)
        self.status_var.set("Working...")
        self._command_queue.put(command)

    def _on_refresh(self):
        self._send(("get_settings",))

    def _on_set_position(self):
        try:
            h = float(self.target_h_var.get())
            v = float(self.target_v_var.get())
        except ValueError:
            messagebox.showerror(
                "Invalid input", "Horizontal and vertical targets must be numbers."
            )
            return
        if not (0 <= h <= 100 and 0 <= v <= 100):
            messagebox.showerror("Invalid input", "Piezo targets must be between 0% and 100%.")
            return
        self._send(("set_position", h, v))

    def _on_mode(self, mode):
        self._send(("set_mode", mode))

    def _on_set_track_radius(self):
        try:
            radius = float(self.track_radius_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "NT track radius must be a number.")
            return
        max_radius = ThorlabsModularNanoTrak.TRACK_RADIUS_MAX_NT
        if not 0 <= radius <= max_radius:
            messagebox.showerror(
                "Invalid input",
                f"NT track radius must be between 0 and {max_radius:g} NT units.",
            )
            return
        self._send(("set_track_radius", radius))

    def _on_set_track_frequency(self):
        try:
            frequency = float(self.track_frequency_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Track frequency must be a number.")
            return
        minimum = ThorlabsModularNanoTrak.TRACK_FREQUENCY_MIN_HZ
        maximum = ThorlabsModularNanoTrak.TRACK_FREQUENCY_MAX_HZ
        if not minimum <= frequency <= maximum:
            messagebox.showerror(
                "Invalid input",
                f"Track frequency must be between {minimum:g} and {maximum:g} Hz.",
            )
            return
        self._send(("set_track_frequency", frequency))

    def _on_apply_feedback(self):
        source = self.FEEDBACK_BY_LABEL.get(self.feedback_var.get())
        if source is None:
            return
        self._send(("set_feedback_source", source))

    def _on_apply_channels(self):
        try:
            chan_a = int(self.chan_a_var.get())
            chan_b = int(self.chan_b_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "NT channels A/B must be integers.")
            return
        self._send(("set_nt_channels", chan_a, chan_b))

    def _on_apply_phase(self):
        try:
            phase_h = int(self.phase_h_var.get())
            phase_v = int(self.phase_v_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Phase H/V must be integers.")
            return
        self._send(("set_phase_compensation", phase_h, phase_v))

    def _on_motor_jog(self, axis, direction):
        try:
            step = float(self.motor_step_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Stepper jog step must be a number.")
            return
        if step <= 0:
            messagebox.showerror("Invalid input", "Stepper jog step must be positive.")
            return
        self._send(("motor_jog", axis, direction * step))

    def _poll_queue(self):
        if self._closed:
            return
        try:
            while True:
                message = self._result_queue.get_nowait()
                kind = message[0]
                if kind == "connected":
                    self.status_var.set("Connected to rack NanoTrak.")
                elif kind == "status":
                    _, h, v, mode, signal_good, reading, feedback_source = message
                    self.h_var.set(f"{h:.4f}%")
                    self.v_var.set(f"{v:.4f}%")
                    if self.top.focus_get() not in (self.target_h_entry, self.target_v_entry):
                        self.target_h_var.set(f"{h:.6g}")
                        self.target_v_var.set(f"{v:.6g}")
                    if self._manual_active and mode == ThorlabsModularNanoTrak.MODE_LATCH:
                        self.mode_var.set("Manual (latched)")
                    else:
                        self.mode_var.set(
                            ThorlabsModularNanoTrak.MODE_NAMES.get(
                                mode, f"Unknown ({mode})"
                            )
                        )
                    self._feedback_source = feedback_source
                    state = "good" if signal_good else "bad"
                    if feedback_source == ThorlabsModularNanoTrak.FEEDBACK_TIA:
                        self.signal_label_var.set("Optical power (PIN/TIA):")
                        self.signal_var.set(f"{reading:.6g} ({state})")
                        display_signal_good = signal_good
                    else:
                        self.signal_label_var.set("BNC voltage:")
                        self.signal_var.set(f"{reading:.6g} V")
                        display_signal_good = None
                    self._update_tracking_grid(h, v, display_signal_good)
                elif kind == "manual_active":
                    self._manual_active = message[1]
                elif kind == "setting_applied":
                    setting = message[1]
                    if setting == "radius":
                        self._track_radius_nt = message[2]
                        self.track_radius_var.set(f"{message[2]:.6g}")
                    elif setting == "frequency":
                        self.track_frequency_var.set(f"{message[2]:.6g}")
                    elif setting == "feedback":
                        source = message[2]
                        self._feedback_source = source
                        self.feedback_var.set(
                            ThorlabsModularNanoTrak.FEEDBACK_NAMES[source]
                        )
                        if source == ThorlabsModularNanoTrak.FEEDBACK_TIA:
                            self.signal_label_var.set("Optical power (PIN/TIA):")
                        else:
                            self.signal_label_var.set("BNC voltage:")
                    elif setting == "channels":
                        self.chan_a_var.set(str(message[2]))
                        self.chan_b_var.set(str(message[3]))
                    elif setting == "phase":
                        self.phase_h_var.set(str(message[2]))
                        self.phase_v_var.set(str(message[3]))
                elif kind == "motor_position":
                    self.motor_x_var.set(f"{message[1]:.6f}")
                    self.motor_y_var.set(f"{message[2]:.6f}")
                elif kind == "settings":
                    (
                        _,
                        feedback_source,
                        chan_a,
                        chan_b,
                        phase_h,
                        phase_v,
                        track_radius,
                        track_frequency,
                    ) = message
                    focused = self.top.focus_get()
                    if focused not in (self.feedback_combo,):
                        self.feedback_var.set(
                            ThorlabsModularNanoTrak.FEEDBACK_NAMES.get(
                                feedback_source, f"Unknown ({feedback_source})"
                            )
                        )
                    if focused not in (self.chan_a_entry, self.chan_b_entry):
                        self.chan_a_var.set(str(chan_a))
                        self.chan_b_var.set(str(chan_b))
                    if focused not in (self.phase_h_entry, self.phase_v_entry):
                        self.phase_h_var.set(str(phase_h))
                        self.phase_v_var.set(str(phase_v))
                    self._track_radius_nt = track_radius
                    self._feedback_source = feedback_source
                    if focused is not self.track_radius_entry:
                        self.track_radius_var.set(f"{track_radius:.6g}")
                    if focused is not self.track_frequency_entry:
                        self.track_frequency_var.set(f"{track_frequency:.6g}")
                elif kind == "ready":
                    self.status_var.set("Ready.")
                    self._set_controls_enabled(True)
                elif kind == "error":
                    self.status_var.set(f"Error: {message[1]}")
                    toplevel = self.top.winfo_toplevel()
                    toplevel.lift()
                    toplevel.focus_force()
                    messagebox.showerror("NanoTrak error", message[1], parent=self.top)
                    self._set_controls_enabled(True)
                elif kind == "closed":
                    self._finish_close()
                    return
        except queue.Empty:
            pass
        self.top.after(150, self._poll_queue)

    def close(self, wait=False):
        if self._closing or self._closed:
            return
        self._closing = True
        self.status_var.set("Disconnecting...")
        self._set_controls_enabled(False)
        self._command_queue.put(("shutdown",))
        if wait and threading.current_thread() is not self._worker:
            self._worker.join()
            self._finish_close()

    def _finish_close(self):
        self._closed = True
        if not self._embedded:
            try:
                self.top.destroy()
            except tk.TclError:
                pass
        if self.on_close is not None:
            self.on_close()


class ScanGUI:
    def __init__(self, root):
        self.root = root
        root.title("Thorlabs Scan")

        self.result_queue = queue.Queue()
        self.worker = None
        self.stop_event = None
        self.scan_rows = []
        self.dirty = False
        self.manual_window = None
        self.nanotrak_window = None

        # Shared X/Y stepper connection, connected lazily on first use (see
        # _get_shared_motors) and kept open for the app's lifetime instead
        # of reconnecting for every Manual Control session, scan, or home --
        # each of those used to own a private connection and tear it down
        # afterward, which meant paying the ~1.5s connect handshake
        # constantly. Safe to share across them because the existing
        # worker/tab-switch guards already ensure at most one of Manual
        # Control, a scan, homing, or "center at current position" is ever
        # actively issuing motor commands at a time.
        self.motorx = None
        self.motory = None
        self._motor_connect_lock = threading.Lock()

        # Grid/line rebuild cache (see scan_engine.build_scan_grid_incremental)
        # and the background thread that runs it during a live scan.
        self._grid_cache = {}
        self._render_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="ScanRender"
        )
        self._render_future = None

        self.view_mode = tk.StringVar(value="heatmap")
        self.status_var = tk.StringVar(value="Idle.")

        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.scan_tab = ttk.Frame(self.notebook)
        self.manual_tab = ttk.Frame(self.notebook)
        self.nanotrak_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.scan_tab, text="Scan")
        self.notebook.add(self.manual_tab, text="Manual Control")
        self.notebook.add(self.nanotrak_tab, text="Piezo / Auto Track")

        self._build_controls(self.scan_tab)
        self._build_plot(self.scan_tab)
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(REDRAW_INTERVAL_MS, self._poll_queue)
        self._grow_window_to_fit()

    # ---------------------------------------------------------------- UI

    def _grow_window_to_fit(self):
        """
        Grow the root window if the currently-selected tab needs more room
        than the window currently has. Tabs other than Scan are built
        lazily (see _activate_manual_control/_activate_nanotrak_control),
        so the window's initial size -- based only on the Scan tab -- can
        be too short for a tab with more stacked sections, clipping its
        bottom controls. Never shrinks a window the user has already
        enlarged.
        """
        self.root.update_idletasks()
        required_width = self.notebook.winfo_reqwidth()
        required_height = self.notebook.winfo_reqheight()
        current_width = max(self.root.winfo_width(), 1)
        current_height = max(self.root.winfo_height(), 1)
        new_width = max(current_width, required_width)
        new_height = max(current_height, required_height)
        if new_width != current_width or new_height != current_height:
            self.root.geometry(f"{new_width}x{new_height}")

    def _get_shared_motors(self):
        """
        Return the shared (motorx, motory), connecting them on first call.

        Must be called from a background thread, not the Tk main loop --
        the first call blocks for the connect handshake (~1.5s). Safe to
        call repeatedly/from different worker threads over the app's
        lifetime since only one such thread is ever active at a time (see
        the comment in __init__); the lock here only protects the
        connect-once check itself.
        """
        with self._motor_connect_lock:
            if self.motorx is None:
                motorx = ThorlabsModularStepperController(serial=SERIAL, channel=1, poll_ms=1)
                motory = ThorlabsModularStepperController(serial=SERIAL, channel=2, poll_ms=1)
                motorx.connect()
                motory.connect()
                self.motorx = motorx
                self.motory = motory
        return self.motorx, self.motory

    def _build_controls(self, master):
        panel = ttk.Frame(master, padding=10)
        panel.grid(row=0, column=0, sticky="ns")

        field_defs = [
            ("x0", "X center (mm)", "1.0"),
            ("y0", "Y center (mm)", "1.0"),
            ("x_span", "X span (mm)", "1.0"),
            ("y_span", "Y span (mm)", "1.0"),
            ("line_spacing", "Line spacing (mm)", "0.1"),
            ("jog_spacing", "X jog spacing (mm)", "0.1"),
            ("acceleration", "Acceleration (mm/s^2)", "4.0"),
            ("max_velocity", "Max velocity (mm/s)", "4.0"),
            ("max_grid_points", "Max grid points/axis", str(DEFAULT_MAX_GRID_POINTS)),
        ]
        self.fields = {}
        for row, (key, label, default) in enumerate(field_defs):
            ttk.Label(panel, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=default)
            ttk.Entry(panel, textvariable=var, width=12).grid(
                row=row, column=1, pady=2, padx=(6, 0)
            )
            self.fields[key] = var

        center_actions = ttk.Frame(panel)
        center_actions.grid(row=0, column=2, rowspan=2, padx=(6, 0))
        self.current_position_btn = CircularActionButton(
            center_actions, command=self._on_center_at_current_position, symbol="⊙"
        )
        self.current_position_btn.grid(row=0, column=0)
        self.center_max_btn = CircularActionButton(
            center_actions, command=self._on_center_at_max, symbol="▲"
        )
        self.center_max_btn.grid(row=0, column=1, padx=(4, 0))

        view_row = len(field_defs)
        ttk.Label(panel, text="View").grid(row=view_row, column=0, sticky="w", pady=(12, 2))
        ttk.Radiobutton(
            panel,
            text="2D Heatmap",
            variable=self.view_mode,
            value="heatmap",
            command=self._redraw,
        ).grid(row=view_row + 1, column=0, columnspan=3, sticky="w")
        ttk.Radiobutton(
            panel,
            text="3D Surface",
            variable=self.view_mode,
            value="surface",
            command=self._redraw,
        ).grid(row=view_row + 2, column=0, columnspan=3, sticky="w")
        ttk.Radiobutton(
            panel,
            text="Overlaid Lines",
            variable=self.view_mode,
            value="lines",
            command=self._redraw,
        ).grid(row=view_row + 3, column=0, columnspan=3, sticky="w")

        btn_row = view_row + 4
        launch_frame = ttk.Frame(panel)
        launch_frame.grid(row=btn_row, column=0, columnspan=3, sticky="ew", pady=(12, 2))
        launch_frame.columnconfigure((0, 1, 2), weight=1)
        self.launch_btn = ttk.Button(launch_frame, text="Launch Scan", command=self._on_launch)
        self.launch_btn.grid(row=0, column=0, sticky="ew", padx=(0, 2))
        self.launch_jog_btn = ttk.Button(
            launch_frame, text="Jog X", command=self._on_launch_jog
        )
        self.launch_jog_btn.grid(row=0, column=1, sticky="ew", padx=2)
        self.launch_y_jog_btn = ttk.Button(
            launch_frame, text="Jog Y", command=self._on_launch_y_jog
        )
        self.launch_y_jog_btn.grid(row=0, column=2, sticky="ew", padx=(2, 0))
        self.stop_btn = ttk.Button(panel, text="Stop", command=self._on_stop, state="disabled")
        self.stop_btn.grid(row=btn_row + 1, column=0, columnspan=3, sticky="ew", pady=2)
        ttk.Button(
            panel, text="Load Saved Scan...", command=self._on_load
        ).grid(row=btn_row + 2, column=0, columnspan=3, sticky="ew", pady=2)
        self.home_btn = ttk.Button(panel, text="Home Motors", command=self._on_home)
        self.home_btn.grid(
            row=btn_row + 3, column=0, columnspan=3, sticky="ew", pady=2
        )
        ttk.Label(panel, textvariable=self.status_var, wraplength=180).grid(
            row=btn_row + 4, column=0, columnspan=3, sticky="w", pady=(12, 0)
        )

    def _build_plot(self, master):
        plot_frame = ttk.Frame(master)
        plot_frame.grid(row=0, column=1, sticky="nsew")
        master.columnconfigure(1, weight=1)
        master.rowconfigure(0, weight=1)

        self.figure = Figure(figsize=(6, 5))
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        toolbar.update()
        self._redraw()

    # ------------------------------------------------------------ helpers

    def _read_params(self):
        try:
            return {
                "x0": float(self.fields["x0"].get()),
                "y0": float(self.fields["y0"].get()),
                "x_span": float(self.fields["x_span"].get()),
                "y_span": float(self.fields["y_span"].get()),
                "line_spacing": float(self.fields["line_spacing"].get()),
                "jog_spacing": float(self.fields["jog_spacing"].get()),
                "acceleration": float(self.fields["acceleration"].get()),
                "max_velocity": float(self.fields["max_velocity"].get()),
                "max_grid_points": max(2, int(float(self.fields["max_grid_points"].get()))),
            }
        except ValueError as exc:
            raise ValueError(f"Invalid input: {exc}") from exc

    def _get_max_grid_points(self):
        try:
            return max(2, int(float(self.fields["max_grid_points"].get())))
        except ValueError:
            return DEFAULT_MAX_GRID_POINTS

    def _set_jog_buttons_state(self, state):
        self.launch_jog_btn.configure(state=state)
        self.launch_y_jog_btn.configure(state=state)

    def _set_manual_tab_enabled(self, enabled):
        self.notebook.tab(self.manual_tab, state="normal" if enabled else "disabled")

    # ------------------------------------------------------------ actions

    def _on_launch(self):
        self._start_scan("fly")

    def _on_launch_jog(self):
        self.view_mode.set("lines")
        self._start_scan("jog_x")

    def _on_launch_y_jog(self):
        self.view_mode.set("lines")
        self._start_scan("jog_y")

    def _start_scan(self, scan_mode):
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            params = self._read_params()
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        self.scan_rows = []
        self.dirty = True
        self._grid_cache = {}
        self._render_future = None  # abandon any in-flight render of the old scan
        self.stop_event = threading.Event()
        self.launch_btn.configure(state="disabled")
        self._set_jog_buttons_state("disabled")
        self.home_btn.configure(state="disabled")
        self._set_manual_tab_enabled(False)
        self.current_position_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.status_var.set(
            f"Starting {scan_mode[-1].upper()} jog scan..."
            if scan_mode.startswith("jog_")
            else "Starting scan..."
        )

        def progress_callback(row_dict, row_index, total_rows):
            self.result_queue.put(("row", row_dict, row_index, total_rows))

        def worker_fn():
            try:
                motorx, motory = self._get_shared_motors()
                if scan_mode.startswith("jog_"):
                    jog_axis = scan_mode[-1]
                    scan_rows = run_jog_scan(
                        motorx=motorx,
                        motory=motory,
                        arduino_port=ARDUINO_PORT,
                        arduino_baud=ARDUINO_BAUD,
                        x0=params["x0"],
                        y0=params["y0"],
                        x_span=params["x_span"] if jog_axis == "x" else params["y_span"],
                        jog_spacing=(
                            params["jog_spacing"]
                            if jog_axis == "x"
                            else params["line_spacing"]
                        ),
                        acceleration=params["acceleration"],
                        max_velocity=params["max_velocity"],
                        sample_duration_s=JOG_SAMPLE_DURATION_S,
                        skip_homing_check=SKIP_HOMING_CHECK,
                        save_file=SAVE_FILE,
                        progress_callback=progress_callback,
                        stop_event=self.stop_event,
                        axis=jog_axis,
                    )
                else:
                    scan_rows = run_scan(
                        motorx=motorx,
                        motory=motory,
                        arduino_port=ARDUINO_PORT,
                        arduino_baud=ARDUINO_BAUD,
                        x0=params["x0"],
                        y0=params["y0"],
                        x_span=params["x_span"],
                        y_span=params["y_span"],
                        line_spacing=params["line_spacing"],
                        acceleration=params["acceleration"],
                        max_velocity=params["max_velocity"],
                        row_settle_s=ROW_SETTLE_S,
                        skip_homing_check=SKIP_HOMING_CHECK,
                        save_file=SAVE_FILE,
                        progress_callback=progress_callback,
                        read_safety_factor=READ_SAFETY_FACTOR,
                        read_overhead_s=READ_OVERHEAD_S,
                        stop_event=self.stop_event,
                    )
                self.result_queue.put(("done", scan_rows))
            except Exception as exc:
                self.result_queue.put(("error", str(exc)))

        self.worker = threading.Thread(target=worker_fn, daemon=True)
        self.worker.start()

    def _on_stop(self):
        if self.stop_event is not None:
            self.stop_event.set()
            self.status_var.set("Stopping safely...")

    def _on_center_at_max(self):
        maximum = find_scan_max(self.scan_rows)
        if maximum is None:
            messagebox.showinfo(
                "No sampled point",
                "Run or load a scan containing samples before setting the center.",
            )
            return

        x, y, value = maximum
        self.fields["x0"].set(f"{x:.9g}")
        self.fields["y0"].set(f"{y:.9g}")
        self.status_var.set(
            f"Center set to scan max: X={x:.6g} mm, Y={y:.6g} mm (ADC={value:.6g})."
        )

    def _on_center_at_current_position(self):
        if self.worker is not None and self.worker.is_alive():
            return
        self.launch_btn.configure(state="disabled")
        self._set_jog_buttons_state("disabled")
        self.home_btn.configure(state="disabled")
        self._set_manual_tab_enabled(False)
        self.current_position_btn.configure(state="disabled")
        self.status_var.set("Reading motor positions...")

        def worker_fn():
            try:
                motorx, motory = self._get_shared_motors()
                x = motorx.get_position(real_unit=True)
                y = motory.get_position(real_unit=True)
                self.result_queue.put(("position_done", x, y))
            except Exception as exc:
                self.result_queue.put(("position_error", str(exc)))

        self.worker = threading.Thread(target=worker_fn, daemon=True)
        self.worker.start()

    def _on_home(self):
        if self.worker is not None and self.worker.is_alive():
            return
        self.launch_btn.configure(state="disabled")
        self._set_jog_buttons_state("disabled")
        self.home_btn.configure(state="disabled")
        self._set_manual_tab_enabled(False)
        self.current_position_btn.configure(state="disabled")
        self.status_var.set("Homing motors...")

        def worker_fn():
            try:
                motorx, motory = self._get_shared_motors()
                check_connection_and_home(
                    motorx=motorx, motory=motory, home_timeout_s=HOME_TIMEOUT_S
                )
                self.result_queue.put(("home_done",))
            except Exception as exc:
                self.result_queue.put(("home_error", str(exc)))

        self.worker = threading.Thread(target=worker_fn, daemon=True)
        self.worker.start()

    def _on_tab_changed(self, _event=None):
        selected = self.notebook.select()
        if selected == str(self.scan_tab):
            self._deactivate_manual_control()
        elif selected == str(self.manual_tab) and self.manual_window is None:
            self._activate_manual_control()
        elif selected == str(self.nanotrak_tab) and self.nanotrak_window is None:
            self._activate_nanotrak_control()

    def _activate_manual_control(self):
        if self.worker is not None and self.worker.is_alive():
            self.notebook.select(self.scan_tab)
            return
        defaults = {
            "x0": self.fields["x0"].get(),
            "y0": self.fields["y0"].get(),
            "acceleration": self.fields["acceleration"].get(),
            "max_velocity": self.fields["max_velocity"].get(),
        }
        self.manual_window = ManualControlWindow(
            self.manual_tab,
            get_motors=self._get_shared_motors,
            defaults=defaults,
            embedded=True,
        )
        self._grow_window_to_fit()

    def _deactivate_manual_control(self):
        if self.manual_window is None:
            return
        self.manual_window.close(wait=True)
        self.manual_window = None
        for child in self.manual_tab.winfo_children():
            child.destroy()

    def _activate_nanotrak_control(self):
        self.nanotrak_window = NanoTrakControlWindow(
            self.nanotrak_tab,
            serial=SERIAL,
            nanotrak_serial=NANOTRAK_SERIAL,
            defaults={
                "acceleration": self.fields["acceleration"].get(),
                "max_velocity": self.fields["max_velocity"].get(),
            },
            embedded=True,
            include_motor_controls=False,
        )
        self._grow_window_to_fit()

    def _on_load(self):
        path = filedialog.askopenfilename(
            title="Load saved scan",
            filetypes=[("Scan files", "*.npz"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.scan_rows = load_scan(path)
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return
        self._grid_cache = {}
        self._render_future = None  # abandon any in-flight render of the old scan
        self.dirty = True
        self.status_var.set(f"Loaded {len(self.scan_rows)} row(s) from {path}")

    # -------------------------------------------------------------- queue

    def _poll_queue(self):
        try:
            while True:
                message = self.result_queue.get_nowait()
                kind = message[0]
                if kind == "row":
                    _, row_dict, row_index, total_rows = message
                    if row_index < len(self.scan_rows):
                        # Replacing (not appending): this is a jog line scan's
                        # row growing point-by-point as a new dict each time.
                        # The grid/position cache keys on id(row), and a
                        # discarded row dict's id can be reused by the next
                        # one, so it must be dropped here to avoid a stale
                        # cache hit against a shorter, previously-cached row.
                        self.scan_rows[row_index] = row_dict
                        self._grid_cache = {}
                    else:
                        self.scan_rows.append(row_dict)
                    self.dirty = True
                    self.status_var.set(f"Row {row_index + 1}/{total_rows} complete.")
                elif kind == "done":
                    _, scan_rows = message
                    self.scan_rows = scan_rows
                    self.dirty = True
                    self.status_var.set(f"Scan finished: {len(scan_rows)} row(s).")
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
                    self.stop_btn.configure(state="disabled")
                elif kind == "error":
                    _, error_text = message
                    self.status_var.set(f"Scan failed: {error_text}")
                    messagebox.showerror("Scan failed", error_text)
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
                    self.stop_btn.configure(state="disabled")
                elif kind == "home_done":
                    self.status_var.set("Homing complete.")
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
                elif kind == "home_error":
                    _, error_text = message
                    self.status_var.set(f"Homing failed: {error_text}")
                    messagebox.showerror("Homing failed", error_text)
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
                elif kind == "position_done":
                    _, x, y = message
                    self.fields["x0"].set(f"{x:.9g}")
                    self.fields["y0"].set(f"{y:.9g}")
                    self.status_var.set(
                        f"Center set to current position: X={x:.6g} mm, Y={y:.6g} mm."
                    )
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
                elif kind == "position_error":
                    _, error_text = message
                    self.status_var.set(f"Position read failed: {error_text}")
                    messagebox.showerror("Position read failed", error_text)
                    self.launch_btn.configure(state="normal")
                    self._set_jog_buttons_state("normal")
                    self.home_btn.configure(state="normal")
                    self._set_manual_tab_enabled(True)
                    self.current_position_btn.configure(state="normal")
        except queue.Empty:
            pass

        self._service_render()

        self.root.after(REDRAW_INTERVAL_MS, self._poll_queue)

    # --------------------------------------------------------------- plot

    def _service_render(self):
        """
        Collect a finished background render (if any) and, if new data has
        arrived since the last one was submitted, kick off the next one.
        Only ever one render job in flight; a still-dirty flag after
        collecting a result means fresh data arrived meanwhile, so the next
        call submits a job that picks it up -- no data is skipped, it's
        just never more than one redraw's worth of latency behind.
        """
        if self._render_future is not None and self._render_future.done():
            try:
                payload = self._render_future.result()
            except Exception as exc:
                payload = ("error", str(exc))
            self._render_future = None
            self._apply_render_payload(payload)

        if self.dirty and self._render_future is None:
            self.dirty = False
            view_mode = self.view_mode.get()
            max_points = self._get_max_grid_points()
            scan_rows_snapshot = list(self.scan_rows)
            self._render_future = self._render_executor.submit(
                self._compute_render_payload, scan_rows_snapshot, view_mode, max_points
            )

    def _compute_render_payload(self, scan_rows, view_mode, max_points):
        """
        Runs on the render worker thread: pure numpy/dict work only (see
        scan_engine.build_scan_grid_incremental/build_scan_lines_incremental)
        -- no matplotlib or Tkinter calls, which must stay on the main
        thread. self._grid_cache is only ever touched from this one
        worker thread (ThreadPoolExecutor(max_workers=1) runs jobs
        strictly one at a time), so no lock is needed.
        """
        if not scan_rows:
            return ("empty",)

        if view_mode == "lines":
            if all(row.get("jog_axis") == "y" for row in scan_rows):
                points = [
                    (row["y"], row["samples"][0])
                    for row in scan_rows
                    if row["samples"]
                ]
                positions = np.asarray([point[0] for point in points], dtype=float)
                samples = np.asarray([point[1] for point in points], dtype=float)
                return ("axis_line", "Y", positions, samples)
            lines = build_scan_lines_incremental(scan_rows, self._grid_cache)
            return ("lines", lines)

        x_grid, y_grid, z = build_scan_grid_incremental(
            scan_rows, self._grid_cache, max_points=max_points
        )
        return ("grid", view_mode, x_grid, y_grid, z)

    def _apply_render_payload(self, payload):
        """Runs on the main thread: matplotlib/Tkinter calls only."""
        self.figure.clf()
        kind = payload[0]

        if kind == "error":
            ax = self.figure.add_subplot(111)
            ax.text(0.5, 0.5, f"Render error: {payload[1]}", ha="center", va="center", wrap=True)
            ax.set_axis_off()
            self.canvas.draw_idle()
            return

        if kind == "empty":
            ax = self.figure.add_subplot(111)
            ax.text(0.5, 0.5, "No scan data yet", ha="center", va="center")
            ax.set_axis_off()
            self.canvas.draw_idle()
            return

        if kind == "lines":
            self._plot_lines(payload[1])
            return

        if kind == "axis_line":
            _, axis_name, positions, samples = payload
            ax = self.figure.add_subplot(111)
            ax.plot(positions, samples, linewidth=1.0, marker=".")
            ax.set_xlabel(f"{axis_name} position (mm)")
            ax.set_ylabel("ADC count")
            ax.grid(True)
            self.canvas.draw_idle()
            return

        _, view_mode, x_grid, y_grid, z = payload
        if x_grid.size == 0 or y_grid.size == 0:
            ax = self.figure.add_subplot(111)
            ax.text(0.5, 0.5, "No samples collected yet", ha="center", va="center")
            ax.set_axis_off()
            self.canvas.draw_idle()
            return

        z_masked = np.ma.masked_invalid(z)

        if view_mode == "surface":
            ax = self.figure.add_subplot(111, projection="3d")
            xx, yy = np.meshgrid(x_grid, y_grid)
            ax.plot_surface(xx, yy, z_masked, cmap="viridis", linewidth=0, antialiased=False)
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("ADC")
        else:
            ax = self.figure.add_subplot(111)
            mesh = ax.pcolormesh(x_grid, y_grid, z_masked, shading="auto", cmap="viridis")
            self.figure.colorbar(mesh, ax=ax, label="ADC")
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")

        self.canvas.draw_idle()

    def _plot_lines(self, lines):
        ax = self.figure.add_subplot(111)
        if not lines:
            ax.text(0.5, 0.5, "No samples collected yet", ha="center", va="center")
            ax.set_axis_off()
            self.canvas.draw_idle()
            return

        color_map = matplotlib.colormaps["viridis"]
        color_count = max(1, len(lines) - 1)
        for line_index, (y, row_xs, samples) in enumerate(lines):
            ax.plot(
                row_xs,
                samples,
                linewidth=1.0,
                color=color_map(line_index / color_count),
                label=f"y={y:.3f}",
            )

        ax.set_xlabel("X position (mm)")
        ax.set_ylabel("ADC count")
        ax.grid(True)
        if len(lines) <= 20:
            ax.legend(title="Row", fontsize="small", ncols=2)

        self.canvas.draw_idle()

    def _redraw(self):
        """
        Synchronous compute+draw, used for one-off, user-triggered redraws
        (initial blank plot, view-mode switch) where there's no hardware
        thread running concurrently to protect -- the live-scan path goes
        through _service_render()'s background job instead.
        """
        view_mode = self.view_mode.get()
        max_points = self._get_max_grid_points()
        payload = self._compute_render_payload(list(self.scan_rows), view_mode, max_points)
        self._apply_render_payload(payload)

    def _on_close(self):
        if self.stop_event is not None:
            self.stop_event.set()
        if self.manual_window is not None:
            self.manual_window.close(wait=True)
        if self.nanotrak_window is not None:
            self.nanotrak_window.close(wait=True)
        # The shared motor connection (see _get_shared_motors) is only
        # ever torn down here, on final app close -- every other consumer
        # (Manual Control, scans, homing) just stops referencing it.
        if self.motorx is not None:
            self.motorx.safe_shutdown()
        if self.motory is not None:
            self.motory.safe_shutdown()
        self._render_executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    root = tk.Tk()
    ScanGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
