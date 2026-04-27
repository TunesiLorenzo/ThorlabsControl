from __future__ import annotations

from pathlib import Path

from nanomax_control.config import load_config
from nanomax_control.controller import NanoMaxSystem


def main() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config" / "example_config.yaml"
    config = load_config(config_path)
    system = NanoMaxSystem(config)
    system.open()
    try:
        print("Devices:", system.list_devices())
        print("Initial state:", system.show_state())
        print("Move X +1000")
        print(system.coarse_move_by("x", 1000))
        if system.nanotrak is not None:
            print("Start tracking")
            system.start_tracking()
        print("Final state:", system.show_state())
    finally:
        system.close()


if __name__ == "__main__":
    main()
