"""Reusable plotting helpers for scan data.

Use this module from another script like:

    from scan_plotter import ScanPlotter

    plotter = ScanPlotter(origin=(0, 0, 0))
    plotter.add_point((10.0, 5.0, 0.0), 1.23)
    plotter.add_point((11.0, 5.5, 0.0), 1.41)
    plotter.plot(show=True)

The plot uses the position relative to the chosen origin, so you can
reuse it in a separate scanning script without changing the plotting code.
""";

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
from typing import Iterable, List, Optional, Sequence, Tuple
import time
import serial

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - required for 3D projection

from math import ceil

from basic_serial_arduino import get_reading

Position = Tuple[float, float, float]

# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = '/dev/ttyUSB0'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def relative_position(position: Sequence[float], origin: Sequence[float]) -> Position:
    """Return position expressed relative to origin."""
    px, py, pz = position
    ox, oy, oz = origin
    return float(px) - float(ox), float(py) - float(oy), float(pz) - float(oz)


@dataclass
class ScanPlotter:
    """Collect scan points and plot them in 3D.

    The third axis is the measured value. The positions are converted to
    coordinates relative to the origin before plotting.
    """

    origin: Position = (0.0, 0.0, 0.0)
    points: List[Tuple[Position, float]] = field(default_factory=list)

    def set_origin(self, origin: Sequence[float]) -> None:
        self.origin = tuple(float(v) for v in origin)  # type: ignore[assignment]

    def add_point(self, position: Sequence[float], value: float) -> None:
        self.points.append((tuple(float(v) for v in position), float(value)))

    def extend(self, data: Iterable[Tuple[Sequence[float], float]]) -> None:
        for position, value in data:
            self.add_point(position, value)

    def as_arrays(self):
        if not self.points:
            return np.array([]), np.array([]), np.array([])

        rel_positions = [relative_position(position, self.origin) for position, _ in self.points]
        xs = np.array([p[0] for p in rel_positions], dtype=float)
        ys = np.array([p[1] for p in rel_positions], dtype=float)
        values = np.array([value for _, value in self.points], dtype=float)
        return xs, ys, values

    def plot(self, show: bool = True, save_path: Optional[str] = None, title: str = "Scan data"):
        xs, ys, values = self.as_arrays()
        if xs.size == 0:
            raise ValueError("No scan points available to plot")

        figure = plt.figure(figsize=(9, 7))
        axis = figure.add_subplot(111, projection="3d")

        scatter = axis.scatter(xs, ys, values, c=values, cmap="viridis", s=36)
        axis.set_xlabel("X from origin")
        axis.set_ylabel("Y from origin")
        axis.set_zlabel("Reading")
        axis.set_title(title)
        figure.colorbar(scatter, ax=axis, shrink=0.7, label="Reading")
        figure.tight_layout()

        if save_path:
            figure.savefig(save_path, dpi=200)

        if show:
            plt.show()

        return figure, axis

    def plot_interactive(
        self,
        title: str = "Scan data",
        output_path: Optional[str] = None,
        auto_open: bool = True,
    ):
        try:
            go = importlib.import_module("plotly.graph_objects")
        except ImportError as exc:
            raise RuntimeError(
                "plotly is not installed. Install it to use interactive 3D plots."
            ) from exc

        xs, ys, values = self.as_arrays()
        if xs.size == 0:
            raise ValueError("No scan points available to plot")

        figure = go.Figure(
            data=go.Scatter3d(
                x=xs,
                y=ys,
                z=values,
                mode="markers+lines",
                marker=dict(
                    size=5,
                    color=values,
                    colorscale="Viridis",
                    showscale=True,
                    colorbar=dict(title="Reading"),
                ),
                line=dict(color="rgba(80, 80, 80, 0.45)", width=2),
            )
        )
        figure.update_layout(
            title=title,
            scene=dict(
                xaxis_title="X from origin",
                yaxis_title="Y from origin",
                zaxis_title="Reading",
            ),
            margin=dict(l=0, r=0, t=40, b=0),
        )

        if output_path:
            figure.write_html(output_path, auto_open=auto_open)
        elif auto_open:
            figure.show()

        return figure


def plot_scan_data(
    data: Iterable[Tuple[Sequence[float], float]],
    origin: Sequence[float] = (0.0, 0.0, 0.0),
    show: bool = True,
    save_path: Optional[str] = None,
    title: str = "Scan data",):

    """Convenience function for one-shot plotting from another script."""
    plotter = ScanPlotter(origin=tuple(float(v) for v in origin))
    plotter.extend(data)
    return plotter.plot(show=show, save_path=save_path, title=title)


def plot_interactive_scan_data(
    data: Iterable[Tuple[Sequence[float], float]],
    origin: Sequence[float] = (0.0, 0.0, 0.0),
    output_path: Optional[str] = "scan_plot.html",
    auto_open: bool = True,
    title: str = "Scan data",
):
    """Convenience function for one-shot interactive plotting from another script."""
    plotter = ScanPlotter(origin=tuple(float(v) for v in origin))
    plotter.extend(data)
    return plotter.plot_interactive(title=title, output_path=output_path, auto_open=auto_open)


def plot_interactive_at_end(
    plotter: ScanPlotter,
    output_path: Optional[str] = "scan_plot.html",
    auto_open: bool = True,
    title: str = "Scan data",
):
    """Open the interactive plot after your scan loop has finished collecting points."""
    return plotter.plot_interactive(title=title, output_path=output_path, auto_open=auto_open)


def add_sampled_position(
    plotter: ScanPlotter,
    x_position_mm: float,
    y_position_mm: float,
    *,
    x_displacement_mm: float,
    y_step_mm: float,
    sample_delay: float = 0.0,
    include_endpoints: bool = True,
    port: str = SERIAL_PORT,
    rate: int = SERIAL_RATE,
    z_position_mm: float = 0.0,
):
    """Read the Arduino using the sampling rule and append the point to a plotter.

    The X and Y positions come from your motion code. The Arduino reading becomes
    the plotted value for that position.
    """
    if sample_delay > 0:
        time.sleep(sample_delay)
    
    # Create a serial connection and read the value
    ser = serial.Serial(port, rate)
    try:
        reading = get_reading(ser)
        if reading is not None:
            # Try to convert to float; if it fails, keep as string
            try:
                value = float(reading)
            except (ValueError, TypeError):
                value = reading
            plotter.add_point((x_position_mm, y_position_mm, z_position_mm), value)
            return value
        else:
            print(f"Warning: No reading received from {port}")
            return None
    finally:
        ser.close()


def add_motor_sample(
    plotter: ScanPlotter,
    x_motor,
    y_motor,
    *,
    x_displacement_mm: float,
    y_step_mm: float,
    sample_delay: float = 0.0,
    include_endpoints: bool = True,
    port: str = SERIAL_PORT,
    rate: int = SERIAL_RATE,
):
    """Read X/Y directly from two motor objects and append the sampled point.

    Each motor is expected to provide get_position(real_unit=True).
    """
    x_position_mm = float(x_motor.get_position(real_unit=True))
    y_position_mm = float(y_motor.get_position(real_unit=True))
    return add_sampled_position(
        plotter,
        x_position_mm,
        y_position_mm,
        x_displacement_mm=x_displacement_mm,
        y_step_mm=y_step_mm,
        sample_delay=sample_delay,
        include_endpoints=include_endpoints,
        port=port,
        rate=rate,
    )


if __name__ == "__main__":
    import math
    
    # Create plotter with origin at center
    plotter = ScanPlotter(origin=(0, 0, 0))
    
    # Generate spiral points
    num_points = 20
    max_radius = 50.0  # mm
    num_turns = 3
    
    for i in range(num_points):
        t = i / num_points * num_turns * 2 * math.pi  # angle parameter
        r = (i / num_points) * max_radius  # radius increases with each point
        
        x = r * math.cos(t)
        y = r * math.sin(t)
        z = 0.0
        
        print(f"Reading point {i+1}/{num_points} at ({x:.1f}, {y:.1f}, {z:.1f})...", end=" ", flush=True)
        
        # Read from actual Arduino and add to plotter
        value = add_sampled_position(
            plotter,
            x, y,
            x_displacement_mm=0,
            y_step_mm=0,
            sample_delay=0.1,
            z_position_mm=z,
        )
        
        if value is not None:
            print(f"Value: {3.3}")
        else:
            print("No reading")
    
    # Plot the collected data
    print("\nGenerating plot...")
    plotter.plot_interactive(output_path="scan_plot.html", auto_open=True, title="Spiral Scan with Arduino Data")
    print("Saved interactive spiral plot to scan_plot.html")