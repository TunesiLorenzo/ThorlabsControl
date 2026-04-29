from serial_arduino import read_serial_float, x_axis_sample_count

x_displacement_mm = 10.0
y_step_mm = 0.5

sample_count = x_axis_sample_count(x_displacement_mm, y_step_mm)
value = read_serial_float(
	x_displacement_mm=x_displacement_mm,
	y_step_mm=y_step_mm,
	sample_delay=0.05,
)
print(f"sample_count={sample_count}, value={value}")