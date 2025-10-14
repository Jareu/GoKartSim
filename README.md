# GoKartSim

Lightweight Project Chrono based go-kart simulation that exposes a WebSocket API for streaming kart state and supplying driver inputs.

## Installation (Windows)

1. Ensure Python 3.9+ is installed and available on the PATH.
2. Install the official Project Chrono Python bindings (PyChrono) from the
   [Project Chrono downloads](https://projectchrono.org/download/). The PyPI
   package named `pychrono` is unrelated and will not work.
3. From this folder run the helper script:
   ```
   install_requirements.bat
   ```
   Optionally pass a specific interpreter, e.g. `install_requirements.bat C:\Python311\python.exe`.

## Running the server

Start the simulation server with:

```
python gokartsim.py
```

Common flags:

- `--host` listen address (default `127.0.0.1`)
- `--port` WebSocket port (default `8765`)
- `--karts` number of AI-controlled karts to spawn
- `--fps` state broadcast frequency
- `--step` physics step size in seconds

Connect a WebSocket client to `ws://<host>:<port>/ws` to receive `init` and `state` messages or send control commands.

## Visualising with the pygame client

You can drive the first kart and visualise the simulation using the bundled pygame client:

```
python gokart_client.py --host 127.0.0.1 --port 8765
```

Controls:

- Arrow keys: throttle (up), brake (down), steer left/right.
- Buttons in the top-left corner send pause, play, and reset commands.
- Drag with the mouse to pan the camera, use the scroll wheel to zoom in/out.
- By default the client claims the first kart and runs in playable mode. Launch with `--spectate` to observe only.

Ensure the server is running before launching the client. The client falls back to an offline mode if the WebSocket cannot be reached, so verify the console output to confirm a successful connection.

## Running the test suite

Unit tests validate the simulation helpers (kart assembly, tires, track generation, etc.). After installing the project dependencies and the official PyChrono bindings, run:

```
pip install pytest pandas numpy pyyaml
pytest
```

If you use a virtual environment, activate it first. A couple of the tests create Chrono physics objects, so make sure the Chrono shared libraries are discoverable (e.g., `PYTHONPATH` set to the Chrono install when required).

## Automated single-kart scenario

Use the helper script to run the repeatable 10 second single-kart sine-steer scenario, capture telemetry, and analyse against the linear bicycle model:

```
tools/run_single_test.sh
```

Artifacts (telemetry CSV, meta JSON, analysis report) are written to `artifacts/run_<timestamp>/`. `report.json` summarises KPIs, pass/fail checks, and any tuning hints if thresholds are missed.
