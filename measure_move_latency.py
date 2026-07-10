"""
Measure the real delay between issuing a move command and the stage actually
starting to move, using get_position() polling to detect the first reported
position change.

As a sanity check, this also measures the round-trip time of back-to-back
get_position() calls while the motor is stationary (both a forced device
refresh, and a plain read of the background-polling cache). That tells us the
best possible detection resolution of our polling loop, so the move-start lag
can be judged against it instead of being assumed to be pure motor/firmware
lag: if the measured lag is on the same order as the query latency itself,
we're mostly seeing polling overhead, not real mechanical delay.
"""

from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController
from motion_timing import measure_move_start_lag, measure_query_latency, summarize_latencies

# Hardware
SERIAL = "50865380"
CHANNEL = 1
POLL_MS = 1

# Motion, real units (mm, mm/s, mm/s^2)
default_acceleration = 4.0
default_max_velocity = 2.0
start_position_mm = 1.0
move_distance_mm = 1

# Move-start detection
# 1 device count of distance is ~1/819200 mm (~1.2 nm); the SBC position is a
# commanded step count (no encoder noise), so this margin is only to guard
# against false positives, not sensor jitter.
position_threshold_mm = 0.0000005  # 0.5 nm
num_trials = 20
poll_timeout_s = 5.0

# Query-latency sanity check
latency_query_count = 300

skip_homing_check = True


def require_homed(motor):
    motor.request_update()
    if not motor.get_status_bits() & 0x00000400:
        raise ThorlabsError(
            "Axis is not homed. Set skip_homing_check=False or home the "
            "stage before running this measurement."
        )


def main():
    motor = ThorlabsModularStepperController(serial=SERIAL, channel=CHANNEL, poll_ms=POLL_MS)
    failed = True
    try:
        motor.connect()

        if skip_homing_check:
            require_homed(motor)
        else:
            print("Homing...")
            motor.home(wait=True)

        motor.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )
        motor.move_absolute(start_position_mm, wait=True, real_unit=True)

        actual_poll_ms = motor.get_polling_duration()

        print("=" * 72)
        print("QUERY LATENCY SANITY CHECK (motor stationary)")
        print("=" * 72)
        refreshed = measure_query_latency(motor, latency_query_count, refresh=True)
        print(summarize_latencies("get_position(refresh=True)  [forces a device round trip + settle wait]", refreshed))
        cached = measure_query_latency(motor, latency_query_count, refresh=False)
        print(summarize_latencies("get_position(refresh=False) [reads the background-poll cache only]", cached))
        print(
            f"\nBackground polling thread runs at {actual_poll_ms} ms "
            f"(requested {POLL_MS} ms). The refresh=False numbers above are "
            "the detection floor of the move-start-lag loop below; if the "
            "measured lag is close to them, it is mostly polling overhead, "
            "not real motor lag."
        )
        print()

        print("=" * 72)
        print(f"MOVE-START LAG  ({num_trials} trials, +/-{move_distance_mm} mm)")
        print("=" * 72)
        lags_lower = []
        lags_upper = []
        for i in range(num_trials):
            # motor.move_absolute(start_position_mm, wait=True, real_unit=True)
            direction = 1 if i % 2 == 0 else -1
            result = measure_move_start_lag(
                motor,
                direction * move_distance_mm,
                timeout_s=poll_timeout_s,
                position_threshold_mm=position_threshold_mm,
            )
            lags_lower.append(result["lag_lower_bound_s"])
            lags_upper.append(result["lag_upper_bound_s"])

            moving_bit_str = (
                "n/a"
                if result["moving_bit_lag_s"] is None
                else f"{result['moving_bit_lag_s']*1000:7.3f} ms"
            )
            print(
                f"trial {i+1}: command call {result['command_call_s']*1000:6.3f} ms | "
                f"position-based lag [{result['lag_lower_bound_s']*1000:7.3f}, "
                f"{result['lag_upper_bound_s']*1000:7.3f}] ms | "
                f"moving-bit lag {moving_bit_str} | "
                f"polls before detection {result['n_polls_before_detection']:4d} | "
                f"displacement {result['displacement_measured']:+.6f} mm"
            )

        print()
        print(summarize_latencies("lag lower bound (last poll still at old position)", lags_lower))
        print(summarize_latencies("lag upper bound (first poll at new position)", lags_upper))

        motor.move_absolute(start_position_mm, wait=True, real_unit=True)
        failed = False
    finally:
        if failed:
            motor.safe_shutdown()
        else:
            motor.disconnect()


if __name__ == "__main__":
    main()
