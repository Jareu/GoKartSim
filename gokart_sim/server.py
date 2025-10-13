"""
Async WebSocket server that drives the Chrono simulation.
"""

import asyncio
import json
import math
import time

import websockets
from pychrono import core as chrono
from .config import KartConfig, rad
from .drivers import ConstantDriver
from .track import build_oval_path
from .world import add_track_visual, build_ground, spawn_karts


class SimServer:
    """
    Outbound:
      - init:   sent on connect and after reset
      - state:  streamed at target FPS
    Inbound (JSON objects):
      {"type":"pause"}
      {"type":"play"}
      {"type":"reset"}              # rebuilds world and sends fresh 'init'
      {"type":"input","id":"kart_3","throttle":0.7,"brake":0.1,"steer":-0.25}
        - 'steer' in radians. Optionally send 'steer_deg' instead.
        - All fields optional; absent ones remain unchanged.
    """

    def __init__(self, host="127.0.0.1", port=8765, fps=60, step=1e-3, num_karts=4):
        self.host = host
        self.port = port
        self.target_fps = fps
        self.dt_broadcast = 1.0 / max(1, fps)
        self.step = step
        self.num_karts = num_karts

        # runtime state
        self.paused = False
        self.clients = set()
        self._server = None
        self._stop = asyncio.Event()
        self.offline_mode = False
        self._offline_end_time = None

        # per-kart input overrides (None = use AI)
        self.inputs_override = {}  # id -> {"throttle":..., "brake":..., "steer":...}

        # Build world
        self._rebuild_world()

    # ---------- world lifecycle ----------

    def _rebuild_world(self):
        cfg = KartConfig()
        cfg.step_size = self.step
        self.cfg = cfg
        self.sys = chrono.ChSystemNSC()
        self.sys.SetGravitationalAcceleration(cfg.gravity)
        self.ground = build_ground(self.sys, cfg)
        self.path, self.track_pts = build_oval_path(chrono.ChVector3d(0, 0, 0), straight_len=40.0, radius=12.0)
        add_track_visual(self.ground, self.path)
        self.karts = spawn_karts(self.sys, cfg, n=self.num_karts, spacing=2.8)
        self.drivers = [ConstantDriver(throttle=0.5, steer_deg=-20.0, brake=0.0) for _ in self.karts]
        # reset overrides
        self.inputs_override.clear()

    # ---------- payloads ----------

    def _init_payload(self):
        pts = [[p.x, p.y] for p in self.track_pts]
        init_karts = []
        for kart in self.karts:
            state = kart.get_state()
            init_karts.append(
                {
                    "id": state["id"],
                    "pose": {
                        "x": state["pos"][0],
                        "y": state["pos"][1],
                        "z": state["pos"][2],
                        "yaw": state["yaw"],
                    },
                    "size": {
                        "L": self.cfg.chassis_length,
                        "W": self.cfg.chassis_width,
                        "H": self.cfg.chassis_height,
                    },
                }
            )
        return {
            "type": "init",
            "sim": {"fps": self.target_fps, "step": self.step},
            "track": {"closed": True, "polyline": pts},
            "karts": init_karts,
        }

    def _state_payload(self):
        t = self.sys.GetChTime()
        arr = []
        for kart in self.karts:
            state = kart.get_state()
            arr.append(
                {
                    "id": state["id"],
                    "x": state["pos"][0],
                    "y": state["pos"][1],
                    "z": state["pos"][2],
                    "yaw": state["yaw"],
                    "speed": state["speed"],
                    "axle_omega": state["axle_omega"],
                    "inputs": state["inputs"],
                }
            )
        return {"type": "state", "t": t, "paused": self.paused, "karts": arr}

    # ---------- networking ----------

    async def _send_json(self, ws, obj):
        try:
            await ws.send(json.dumps(obj, separators=(",", ":")))
            return True
        except Exception:
            return False

    async def _broadcast_json(self, obj):
        if not self.clients:
            return
        dead = []
        for ws in list(self.clients):
            ok = await self._send_json(ws, obj)
            if not ok:
                dead.append(ws)
        for ws in dead:
            try:
                await ws.close()
            except Exception:
                pass
            self.clients.discard(ws)

    async def ws_handler(self, ws):
        # register
        self.clients.add(ws)
        await self._send_json(ws, self._init_payload())

        async def reader():
            async for msg in ws:
                try:
                    obj = json.loads(msg)
                except Exception:
                    continue
                await self._handle_inbound(obj)

        try:
            await reader()
        except Exception:
            pass
        finally:
            self.clients.discard(ws)

    async def _handle_inbound(self, obj):
        """
        Inbound schema:
          {"type":"pause"}
          {"type":"play"}
          {"type":"reset"}
          {"type":"input","id":"kart_1","throttle":0.5,"brake":0.1,"steer":-0.2}
          # optional: "steer_deg": -10.0 (overrides 'steer' if both present)
        """
        msg_type = obj.get("type")
        if msg_type == "pause":
            self.paused = True
        elif msg_type == "play":
            self.paused = False
        elif msg_type == "reset":
            self._rebuild_world()
            await self._broadcast_json(self._init_payload())
        elif msg_type == "input":
            kart_id = obj.get("id")
            if not kart_id:
                return
            steer = obj.get("steer", None)
            if "steer_deg" in obj:
                steer = rad(obj["steer_deg"])
            throttle = obj.get("throttle", None)
            brake = obj.get("brake", None)

            def clamp01(value):
                return max(0.0, min(1.0, float(value)))

            override = self.inputs_override.get(kart_id, {"throttle": None, "brake": None, "steer": None})
            if throttle is not None:
                override["throttle"] = clamp01(throttle)
            if brake is not None:
                override["brake"] = clamp01(brake)
            if steer is not None:
                try:
                    steer = float(steer)
                except Exception:
                    steer = None
                override["steer"] = steer
            self.inputs_override[kart_id] = override
        # else: ignore unknown types

    # ---------- sim loop ----------

    async def run(self):
        bind_ports = [self.port]
        if self.port not in (None, 0):
            bind_ports.append(0)

        last_error = None
        for bind_port in bind_ports:
            try:
                self._server = await websockets.serve(
                    self.ws_handler,
                    self.host,
                    bind_port,
                    ping_interval=20,
                    ping_timeout=20,
                    max_queue=None,
                )
                sockets = self._server.sockets or []
                if sockets:
                    self.port = sockets[0].getsockname()[1]
                print(f"WebSocket listening on ws://{self.host}:{self.port}/ws")
                break
            except OSError as err:
                last_error = err
                self._server = None

        if self._server is None:
            self.offline_mode = True
            self._offline_end_time = self.sys.GetChTime() + 2.0
            msg = "Unable to bind WebSocket server; running offline simulation for 2.0 simulated seconds."
            if last_error:
                msg += f" ({last_error})"
            print(msg)
        else:
            self.offline_mode = False
            self._offline_end_time = None

        next_tick = time.perf_counter()
        try:
            while not self._stop.is_set():
                tnow = time.perf_counter()
                if tnow < next_tick:
                    await asyncio.sleep(next_tick - tnow)
                next_tick += self.dt_broadcast

                # Controls & physics
                substeps = max(1, int(math.ceil(self.dt_broadcast / self.step)))
                for _ in range(substeps):
                    sim_t = self.sys.GetChTime()
                    # Controls per kart (AI or override)
                    for kart, driver in zip(self.karts, self.drivers):
                        state = kart.get_state()
                        # default to AI
                        thr, steer, brk = driver.compute_controls(sim_t, state)
                        # override if present
                        override = self.inputs_override.get(state["id"])
                        if override:
                            if override.get("throttle") is not None:
                                thr = override["throttle"]
                            if override.get("brake") is not None:
                                brk = override["brake"]
                            if override.get("steer") is not None:
                                steer = override["steer"]
                        kart.set_controls(thr, steer, brk)
                        kart.update_axle_torques()
                        # apply lateral tires even when paused to keep contact forces ready
                        kart.apply_tire_forces(self.step)

                    # advance physics only if not paused
                    if not self.paused:
                        self.sys.DoStepDynamics(self.step)

                # one broadcast per frame
                await self._broadcast_json(self._state_payload())
                if self.offline_mode and self.sys.GetChTime() >= (self._offline_end_time or 0.0):
                    print("Offline simulation complete; shutting down.")
                    self.stop()
        finally:
            if self._server is not None:
                self._server.close()
                await self._server.wait_closed()

    def stop(self):
        self._stop.set()
