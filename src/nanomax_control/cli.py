from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_config
from .controller import NanoMaxSystem


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NanoMax control CLI")
    parser.add_argument("--config", required=True, help="Path to YAML config")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-devices")
    sub.add_parser("show-state")
    sub.add_parser("start-tracking")
    sub.add_parser("stop-tracking")

    coarse_move = sub.add_parser("coarse-move")
    coarse_move.add_argument("--axis", required=True, choices=["x", "y", "z"])
    coarse_move.add_argument("--steps", required=True, type=int)

    coarse_to = sub.add_parser("coarse-move-to")
    coarse_to.add_argument("--axis", required=True, choices=["x", "y", "z"])
    coarse_to.add_argument("--position", required=True, type=float)

    recover = sub.add_parser("recover-coupling")
    recover.add_argument("--axis", required=True, choices=["x", "y", "z"])
    recover.add_argument("--sweep-steps", required=True, type=int)
    recover.add_argument("--count", default=5, type=int)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    config = load_config(Path(args.config))
    system = NanoMaxSystem(config)
    system.open()
    try:
        if args.command == "list-devices":
            print(json.dumps(system.list_devices(), indent=2, default=str))
        elif args.command == "show-state":
            print(json.dumps(system.show_state(), indent=2, default=str))
        elif args.command == "coarse-move":
            pos = system.coarse_move_by(args.axis, args.steps)
            print(json.dumps({"axis": args.axis, "position": pos}, indent=2))
        elif args.command == "coarse-move-to":
            pos = system.coarse_move_to(args.axis, args.position)
            print(json.dumps({"axis": args.axis, "position": pos}, indent=2))
        elif args.command == "start-tracking":
            system.start_tracking()
            print(json.dumps({"tracking": "started"}, indent=2))
        elif args.command == "stop-tracking":
            system.stop_tracking()
            print(json.dumps({"tracking": "stopped"}, indent=2))
        elif args.command == "recover-coupling":
            result = system.recover_coupling(args.axis, args.sweep_steps, args.count)
            print(json.dumps(result, indent=2, default=str))
        else:
            parser.error(f"Unknown command: {args.command}")
    finally:
        system.close()


if __name__ == "__main__":
    main()
