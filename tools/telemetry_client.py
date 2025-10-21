#!/usr/bin/env python3
"""Capture telemetry from the GoKartSim websocket API."""

import argparse
import asyncio
import csv
import json
import os
from pathlib import Path

import websockets

try:
    import yaml
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    yaml = None


CSV_FIELDS = [
    "t",
    "px",
    "py",
    "pz",
    "qx",
    "qy",
    "qz",
    "qw",
    "vx",
    "vy",
    "vz",
    "yaw_rate",
    "steer",
    "throttle",
    "brake",
    "speed",
]


async def capture(uri: str, duration: float, outdir: Path, scenario_path: Path | None):
    outdir.mkdir(parents=True, exist_ok=True)
    meta = {"uri": uri, "duration": duration}
    if scenario_path is not None:
        if yaml is None:
            raise RuntimeError("PyYAML is required to load scenario files")
        with scenario_path.open("r", encoding="utf-8") as fh:
            meta["scenario"] = yaml.safe_load(fh)

    csv_path = outdir / "telemetry.csv"
    meta_path = outdir / "meta.json"

    async def _connect():
        attempts = 10
        delay = 0.2
        for attempt in range(attempts):
            try:
                return await websockets.connect(uri)
            except (ConnectionRefusedError, OSError):
                if attempt == attempts - 1:
                    raise
                await asyncio.sleep(delay)
                delay = min(delay * 2, 1.0)

    async with await _connect() as ws:
        init_payload = None
        start_time = None
        collected = False
        rows_written = 0
        expected_rows = None
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()
            dt_tolerance = 0.0

            try:
                while True:
                    raw = await ws.recv()
                    msg = json.loads(raw)
                    msg_type = msg.get("type")

                    if msg_type == "init":
                        init_payload = msg
                        meta["init"] = init_payload
                        meta_path.write_text(json.dumps(meta, indent=2))
                        await ws.send(json.dumps({"type": "play"}))
                        sim_fps = float(init_payload.get("sim", {}).get("fps", 100.0))
                        expected_rows = max(1, int(duration * sim_fps))
                    elif msg_type == "state" and init_payload is not None:
                        sim_time = float(msg.get("t", 0.0))
                        if start_time is None:
                            start_time = sim_time
                        if dt_tolerance == 0.0 and len(msg.get("karts", [])):
                            dt_tolerance = 0.02  # default margin (~2 frames at 100 Hz)
                        if not msg.get("karts"):
                            continue
                        kart = msg["karts"][0]
                        inputs = kart.get("inputs", {}) or {}
                        row = {
                            "t": sim_time,
                            "px": kart.get("x", 0.0),
                            "py": kart.get("y", 0.0),
                            "pz": kart.get("z", 0.0),
                            "qx": kart.get("qx", 0.0),
                            "qy": kart.get("qy", 0.0),
                            "qz": kart.get("qz", 0.0),
                            "qw": kart.get("qw", 1.0),
                            "vx": kart.get("vx", 0.0),
                            "vy": kart.get("vy", 0.0),
                            "vz": kart.get("vz", 0.0),
                            "yaw_rate": kart.get("yaw_rate", 0.0),
                            "steer": inputs.get("steer", 0.0),
                            "throttle": inputs.get("throttle", 0.0),
                            "brake": inputs.get("brake", 0.0),
                            "speed": kart.get("speed", 0.0),
                        }
                        csv_writer.writerow(row)
                        csv_file.flush()
                        rows_written += 1
                        if sim_time - start_time >= max(0.0, duration - dt_tolerance):
                            collected = True
                            break
                    elif msg_type == "error":
                        print("[telemetry] server error:", msg.get("detail"))
            except websockets.exceptions.ConnectionClosedOK:
                if expected_rows is None:
                    expected_rows = max(1, int(duration * 100))
                if not collected and rows_written >= 0.9 * expected_rows:
                    collected = True
                if not collected:
                    raise


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture telemetry from GoKartSim")
    parser.add_argument("--uri", default="ws://127.0.0.1:8765/ws")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--meta-scenario", type=Path, dest="scenario", help="Optional scenario YAML to embed in metadata")
    args = parser.parse_args()

    asyncio.run(capture(args.uri, args.duration, args.outdir, args.scenario))


if __name__ == "__main__":
    main()
