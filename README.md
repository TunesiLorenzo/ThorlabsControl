# NanoMax Control Project

Python starter project for controlling a Thorlabs NanoMax-style setup with:

- **Coarse motion** through **DRV208** actuators connected to an **MST602** rack controller
- **Fine closed-loop tracking** through **MAX381** piezos driven by an **MNA601/IR NanoTrak** controller

This project is structured so that:

- the **MST602** side is controlled through **pylablib**
- the **NanoTrak** side is accessed through **pythonnet** and Thorlabs **Kinesis/.NET** assemblies

## Important note

The `pylablib` part should work with the MST602 path once the device is visible through Kinesis/APT and the correct serial number is configured.

The **NanoTrak** wrapper in this project is intentionally conservative: it gives you a clean Python interface and assembly loading hooks, but the exact DLL/class names can vary slightly depending on your installed Kinesis version and whether your controller is exposed as a modular-rack NanoTrak class or a generic NanoTrak class. You may need to adjust the assembly/class imports in `src/nanomax_control/nanotrak.py` to match your installation.

## Project layout

- `src/nanomax_control/config.py` - configuration loading and validation
- `src/nanomax_control/coarse_stage.py` - MST602 / DRV208 control via `pylablib`
- `src/nanomax_control/nanotrak.py` - NanoTrak wrapper via `pythonnet`
- `src/nanomax_control/controller.py` - high-level project API
- `src/nanomax_control/cli.py` - command-line interface
- `src/nanomax_control/sim.py` - simulation backends for offline development
- `config/example_config.yaml` - example hardware config
- `scripts/run_demo.py` - demo script

## Requirements

- Windows with Thorlabs Kinesis installed
- Python 3.10+ recommended
- USB/rack drivers working in Kinesis first

Install dependencies:

```bash
pip install -r requirements.txt
```

## Configure

Copy and edit the example config:

```bash
copy config\example_config.yaml config\local_config.yaml
```

Update:

- `mst602.serial`
- `mst602.channels`
- `nanotrak.serial`
- `nanotrak.detector_threshold`
- optional DLL directory for Kinesis

## CLI examples

List Kinesis devices seen by `pylablib`:

```bash
python -m nanomax_control.cli list-devices --config config/local_config.yaml
```

Read coarse axis positions:

```bash
python -m nanomax_control.cli show-state --config config/local_config.yaml
```

Move coarse X by 2000 controller steps:

```bash
python -m nanomax_control.cli coarse-move --config config/local_config.yaml --axis x --steps 2000
```

Start NanoTrak tracking:

```bash
python -m nanomax_control.cli start-tracking --config config/local_config.yaml
```

Run recovery helper:

```bash
python -m nanomax_control.cli recover-coupling --config config/local_config.yaml --axis x --sweep-steps 3000 --count 5
```

## Development without hardware

Set `simulation.enabled: true` in the config and run the same commands. This uses the simulator implementations in `sim.py`.

## Suggested workflow on the real setup

1. Verify the MST602 is visible in Kinesis.
2. Verify the NanoTrak is visible in Kinesis/APT.
3. Run the project in simulation once.
4. Switch to hardware mode.
5. Use coarse axes to bring the system near alignment.
6. Start NanoTrak tracking for fine optimization.
7. If coupling is lost, use `recover-coupling` to re-enter the NanoTrak capture range.

## Known limitations

- NanoTrak API class names may require small edits for your installed Thorlabs package.
- The project uses controller steps for coarse motion by default; calibration to physical units is left to your setup-specific mapping.
- This is a starter project and not a full production interlock/safety system.
