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
