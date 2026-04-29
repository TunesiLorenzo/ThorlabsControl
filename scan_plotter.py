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

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - required for 3D projection

from math import ceil

from serial_arduino import read_serial_float

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


def x_axis_sample_count(
    x_displacement_mm: float,
    y_step_mm: float,
    include_endpoints: bool = True,
) -> int:
    """Return how many X-axis sample points are needed for a scan.

    The Y value is treated as the step size between X samples.
    If include_endpoints is True, the count includes both the start and end points.
    """
    if y_step_mm == 0:
        raise ValueError("y_step_mm must not be zero")

    intervals = ceil(abs(float(x_displacement_mm)) / abs(float(y_step_mm)))
    return intervals + 1 if include_endpoints else intervals


def generate_motor_position_list(
    x_start_mm: float,
    y_start_mm: float,
    x_span_um: float,
    y_span_um: float,
    x_step_um: float = 0.5,
    y_step_um: float = 0.1,
    serpentine: bool = True,
):
    """Generate a trial list of motor positions for a 2D scan.

    The scan is built from micrometer step sizes, then converted to mm.
    X is the fast axis and Y is the slow axis.
    """
    if x_step_um <= 0 or y_step_um <= 0:
        raise ValueError("x_step_um and y_step_um must be positive")

    x_span_mm = float(x_span_um) / 1000.0
    y_span_mm = float(y_span_um) / 1000.0
    x_step_mm = float(x_step_um) / 1000.0
    y_step_mm = float(y_step_um) / 1000.0

    x_count = x_axis_sample_count(x_span_mm, x_step_mm)
    y_count = x_axis_sample_count(y_span_mm, y_step_mm)

    positions = []
    for y_index in range(y_count):
        y_position_mm = float(y_start_mm) + (y_index * y_step_mm)
        x_indices = range(x_count)
        if serpentine and y_index % 2 == 1:
            x_indices = range(x_count - 1, -1, -1)

        for x_index in x_indices:
            x_position_mm = float(x_start_mm) + (x_index * x_step_mm)
            positions.append((x_position_mm, y_position_mm, 0.0))

    return positions


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
    reading = read_serial_float(
        port=port,
        rate=rate,
        x_displacement_mm=x_displacement_mm,
        y_step_mm=y_step_mm,
        sample_delay=sample_delay,
        include_endpoints=include_endpoints,
    )
    plotter.add_point((x_position_mm, y_position_mm, z_position_mm), reading)
    return reading


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
    
    demo = [
        ((0.0, 0.0, 0.0), 0.2),
        ((1.0, 0.0, 0.0), 0.5),
        ((1.0, 1.0, 0.0), 1.0),
        ((0.0, 1.0, 0.0), 0.7),
    ]
    plot_interactive_scan_data(demo, output_path="scan_plot.html", auto_open=True)
    print("Saved interactive demo plot to scan_plot.html")