from scan_plotter import generate_motor_position_list
from scan_plotter import ScanPlotter, plot_interactive_at_end

positions = generate_motor_position_list(
    x_start_mm=0.0,
    y_start_mm=0.0,
    x_span_um=5.0,
    y_span_um=1.0,
    x_step_um=0.5,
    y_step_um=0.1,
)

for pos in positions:
    print(pos)


plotter = ScanPlotter()

# during scan:
# plotter.add_point((x_mm, y_mm, 0.0), reading)

# after scan:
plot_interactive_at_end(plotter, output_path="scan_plot.html", auto_open=True)