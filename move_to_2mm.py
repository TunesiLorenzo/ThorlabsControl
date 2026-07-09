from Main import ThorlabsModularStepperController


SERIAL = "50865380"
TARGET_MM = 2.0
POLL_MS = 1


def main():
    motorx = ThorlabsModularStepperController(
        serial=SERIAL,
        channel=1,
        poll_ms=POLL_MS,
    )
    motory = ThorlabsModularStepperController(
        serial=SERIAL,
        channel=2,
        poll_ms=POLL_MS,
    )

    move_failed = True
    try:
        motorx.connect()
        motory.connect()

        print(f"Moving X axis to absolute {TARGET_MM} mm...")
        motorx.move_absolute(TARGET_MM, wait=True, real_unit=True)

        print(f"Moving Y axis to absolute {TARGET_MM} mm...")
        motory.move_absolute(TARGET_MM, wait=True, real_unit=True)

        print(f"X position: {motorx.get_position(real_unit=True):.6f} mm")
        print(f"Y position: {motory.get_position(real_unit=True):.6f} mm")
        move_failed = False

    finally:
        if move_failed:
            motorx.safe_shutdown()
            motory.safe_shutdown()
        else:
            motorx.disconnect()
            motory.disconnect()


if __name__ == "__main__":
    main()
