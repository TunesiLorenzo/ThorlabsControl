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
from typing import Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - required for 3D projection


Position = Tuple[float, float, float]


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


def plot_scan_data(
    data: Iterable[Tuple[Sequence[float], float]],
    origin: Sequence[float] = (0.0, 0.0, 0.0),
    show: bool = True,
    save_path: Optional[str] = None,
    title: str = "Scan data",
):
    """Convenience function for one-shot plotting from another script."""
    plotter = ScanPlotter(origin=tuple(float(v) for v in origin))
    plotter.extend(data)
    return plotter.plot(show=show, save_path=save_path, title=title)


if __name__ == "__main__":
    demo = [
        ((0.0, 0.0, 0.0), 0.2),
        ((1.0, 0.0, 0.0), 0.5),
        ((1.0, 1.0, 0.0), 1.0),
        ((0.0, 1.0, 0.0), 0.7),
    ]
    plot_scan_data(demo)