#!/usr/bin/env python3
"""
Simple pygame client for the GoKart simulation server.

Connects to the websocket API, visualises the track/karts, and lets the user
control the first kart using arrow keys:
  - Up:    throttle 100%
  - Down:  brake 100%
  - Left:  steer left (to -30 deg)
  - Right: steer right (to +30 deg)

Buttons in the top-left corner send pause, play, and reset commands.
Click and drag to pan. Use the mouse wheel to zoom in/out.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import queue
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import pygame
import websockets

BG_COLOR = (20, 20, 24)
TRACK_COLOR = (60, 90, 120)
KART_COLOR = (230, 200, 60)
WHEEL_COLOR = (60, 60, 70)
TEXT_COLOR = (220, 220, 220)
BTN_COLOR = (70, 70, 80)
BTN_TEXT = (240, 240, 240)
BTN_HOVER = (100, 100, 120)
WINDOW_SIZE = (1024, 768)
FPS = 60
STEER_DEG = 30.0


@dataclass
class KartState:
    kart_id: str
    pos: Tuple[float, float, float]
    yaw: float
    length: float
    width: float
    steer: float


@dataclass
class ViewState:
    base_scale: float = 1.0
    min_x: float = 0.0
    min_y: float = 0.0
    pad: float = 0.0
    zoom: float = 1.0
    pan_x: float = 0.0
    pan_y: float = 0.0

    @property
    def scale(self) -> float:
        return self.base_scale * self.zoom


class WebSocketClient:
    """Background websocket client with blocking queues for pygame thread."""

    def __init__(self, uri: str, inbound: queue.Queue, outbound: queue.Queue):
        self.uri = uri
        self.inbound = inbound
        self.outbound = outbound
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._stop_evt = threading.Event()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        self.outbound.put({"__shutdown__": True})
        self._thread.join(timeout=2.0)

    def _thread_main(self) -> None:
        asyncio.run(self._run())

    async def _run(self) -> None:
        backoff = 1.0
        while not self._stop_evt.is_set():
            try:
                async with websockets.connect(self.uri) as ws:
                    backoff = 1.0
                    recv_task = asyncio.create_task(self._recv_loop(ws))
                    send_task = asyncio.create_task(self._send_loop(ws))
                    done, pending = await asyncio.wait(
                        {recv_task, send_task}, return_when=asyncio.FIRST_EXCEPTION
                    )
                    for task in pending:
                        task.cancel()
                    for task in done:
                        if task.exception():
                            raise task.exception()
            except Exception as exc:  # pragma: no cover - simple log path
                self.inbound.put({"type": "error", "detail": str(exc)})
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)

    async def _recv_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            self.inbound.put(msg)

    async def _send_loop(self, ws: websockets.WebSocketClientProtocol) -> None:
        while not self._stop_evt.is_set():
            msg = await asyncio.to_thread(self.outbound.get)
            if "__shutdown__" in msg:
                break
            await ws.send(json.dumps(msg))


class Button:
    def __init__(self, rect: pygame.Rect, label: str, command: Dict[str, str]):
        self.rect = rect
        self.label = label
        self.command = command

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, hover: bool) -> None:
        pygame.draw.rect(surface, BTN_HOVER if hover else BTN_COLOR, self.rect)
        text = font.render(self.label, True, BTN_TEXT)
        text_rect = text.get_rect(center=self.rect.center)
        surface.blit(text, text_rect)

    def contains(self, pos: Tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos)


def compute_view(points: List[Tuple[float, float]]) -> ViewState:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad = 40.0
    width, height = WINDOW_SIZE
    range_x = max_x - min_x or 1.0
    range_y = max_y - min_y or 1.0
    scale = min((width - 2 * pad) / range_x, (height - 2 * pad) / range_y)
    return ViewState(base_scale=scale, min_x=min_x, min_y=min_y, pad=pad)


def world_to_screen(x: float, y: float, view: ViewState) -> Tuple[float, float]:
    sx = (x - view.min_x) * view.scale + view.pad + view.pan_x
    sy = WINDOW_SIZE[1] - ((y - view.min_y) * view.scale + view.pad) + view.pan_y
    return sx, sy


def screen_to_world(sx: float, sy: float, view: ViewState) -> Tuple[float, float]:
    scale = view.scale or 1.0
    x = view.min_x + (sx - view.pan_x - view.pad) / scale
    y = view.min_y + (WINDOW_SIZE[1] - (sy - view.pan_y) - view.pad) / scale
    return x, y


def draw_track(surface: pygame.Surface, track_pts: List[Tuple[float, float, float]], view: ViewState) -> None:
    if not track_pts:
        return
    poly = [
        tuple(map(int, world_to_screen(x, y, view)))
        for x, y, _ in track_pts
    ]
    if len(poly) >= 3:
        pygame.draw.polygon(surface, TRACK_COLOR, poly, 0)


def _rectangle_points(cx, cy, length, width, angle, view: ViewState):
    half_l = length / 2.0
    half_w = width / 2.0
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    corners_local = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
    ]
    points = []
    for dx, dy in corners_local:
        wx = cx + cos_a * dx - sin_a * dy
        wy = cy + sin_a * dx + cos_a * dy
        sx, sy = world_to_screen(wx, wy, view)
        points.append((int(sx), int(sy)))
    return points


def draw_kart(surface: pygame.Surface, state: KartState, view: ViewState) -> None:
    x, y, _ = state.pos
    yaw = state.yaw

    body_points = _rectangle_points(x, y, state.length, state.width, yaw, view)
    pygame.draw.polygon(surface, KART_COLOR, body_points, 0)

    wheel_length = state.length * 0.3
    wheel_width = state.width * 0.25
    longitudinal_offset = state.length / 2.0 - wheel_length / 2.0
    lateral_offset = state.width / 2.0 - wheel_width / 2.0

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    def local_to_world(dx, dy):
        wx = x + cos_yaw * dx - sin_yaw * dy
        wy = y + sin_yaw * dx + cos_yaw * dy
        return wx, wy

    front_left = local_to_world(longitudinal_offset, lateral_offset)
    front_right = local_to_world(longitudinal_offset, -lateral_offset)
    rear_left = local_to_world(-longitudinal_offset, lateral_offset)
    rear_right = local_to_world(-longitudinal_offset, -lateral_offset)

    front_angle = yaw + state.steer
    rear_angle = yaw

    for center, angle in (
        (front_left, front_angle),
        (front_right, front_angle),
        (rear_left, rear_angle),
        (rear_right, rear_angle),
    ):
        pts = _rectangle_points(center[0], center[1], wheel_length, wheel_width, angle, view)
        pygame.draw.polygon(surface, WHEEL_COLOR, pts, 0)


def process_controls() -> Tuple[float, float, float]:
    pressed = pygame.key.get_pressed()
    throttle = 1.0 if pressed[pygame.K_UP] else 0.0
    brake = 1.0 if pressed[pygame.K_DOWN] else 0.0
    steer = 0.0
    if pressed[pygame.K_LEFT] and not pressed[pygame.K_RIGHT]:
        steer = -1.0
    elif pressed[pygame.K_RIGHT] and not pressed[pygame.K_LEFT]:
        steer = 1.0
    return throttle, brake, steer


def flatten_metrics(metrics: Dict, prefix: str = "") -> List[str]:
    lines: List[str] = []
    for key in sorted(metrics.keys()):
        label = f"{prefix}{key}"
        value = metrics[key]
        if isinstance(value, dict):
            lines.extend(flatten_metrics(value, f"{label}."))
        else:
            if isinstance(value, float):
                value_str = f"{value:.3f}"
            else:
                value_str = str(value)
            lines.append(f"{label}: {value_str}")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Pygame client for the GoKart websocket server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--spectate",
        action="store_true",
        help="Connect without claiming a kart (view-only mode)",
    )
    args = parser.parse_args()

    inbound: queue.Queue = queue.Queue()
    outbound: queue.Queue = queue.Queue()
    ws_client = WebSocketClient(f"ws://{args.host}:{args.port}/ws", inbound, outbound)
    ws_client.start()

    pygame.init()
    pygame.display.set_caption("GoKart Client")
    screen = pygame.display.set_mode(WINDOW_SIZE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    buttons = [
        Button(pygame.Rect(10, 10, 80, 30), "Pause", {"type": "pause"}),
        Button(pygame.Rect(100, 10, 80, 30), "Play", {"type": "play"}),
        Button(pygame.Rect(190, 10, 80, 30), "Reset", {"type": "reset"}),
    ]

    running = True
    playable_mode = not args.spectate
    track_pts: List[Tuple[float, float, float]] = []
    view = ViewState()
    kart_states: Dict[str, KartState] = {}
    kart_dimensions: Dict[str, Tuple[float, float]] = {}
    controlled_kart: Optional[str] = None
    last_sent_controls = (None, None, None)
    last_send_time = 0.0
    dragging = False
    claim_sent = False
    latest_diag_lines: List[str] = []

    def ensure_claim():
        nonlocal claim_sent
        if playable_mode and controlled_kart and not claim_sent:
            outbound.put({"type": "claim", "id": controlled_kart, "playable": True})
            print(f"[client] Claimed {controlled_kart} for playable control.")
            claim_sent = True

    while running:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    clicked = False
                    for btn in buttons:
                        if btn.contains(event.pos):
                            outbound.put(btn.command)
                            clicked = True
                            break
                    if not clicked:
                        dragging = True
                        pygame.mouse.get_rel()
                elif event.button == 2:
                    dragging = True
                    pygame.mouse.get_rel()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button in (1, 2):
                    dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx, dy = event.rel
                view.pan_x += dx
                view.pan_y += dy
            elif event.type == pygame.MOUSEWHEEL:
                zoom_factor = 1.1 ** event.y
                mouse_pos = pygame.mouse.get_pos()
                wx, wy = screen_to_world(mouse_pos[0], mouse_pos[1], view)
                view.zoom = max(0.2, min(8.0, view.zoom * zoom_factor))
                sx, sy = world_to_screen(wx, wy, view)
                view.pan_x += mouse_pos[0] - sx
                view.pan_y += mouse_pos[1] - sy

        # Drain inbound messages
        try:
            while True:
                msg = inbound.get_nowait()
                msg_type = msg.get("type")
                if msg_type == "init":
                    track = msg.get("track", {})
                    poly = track.get("polyline", [])
                    track_pts = [(float(x), float(y), 0.0) for x, y in poly]
                    if track_pts:
                        view = compute_view([(p[0], p[1]) for p in track_pts])
                    karts = msg.get("karts", [])
                    kart_dimensions.clear()
                    for entry in karts:
                        size = entry.get("size", {})
                        kart_dimensions[entry["id"]] = (
                            float(size.get("L", 1.8)),
                            float(size.get("W", 1.2)),
                        )
                    if karts and controlled_kart is None:
                        controlled_kart = karts[0]["id"]
                    if playable_mode:
                        claim_sent = False
                        ensure_claim()
                elif msg_type == "state":
                    kart_states.clear()
                    for entry in msg.get("karts", []):
                        kart_id = entry["id"]
                        length, width = kart_dimensions.get(kart_id, (1.8, 1.2))
                        inputs = entry.get("inputs", {}) or {}
                        steer = float(inputs.get("steer", 0.0))
                        kart_states[kart_id] = KartState(
                            kart_id=kart_id,
                            pos=(entry["x"], entry["y"], entry["z"]),
                            yaw=entry["yaw"],
                            length=length,
                            width=width,
                            steer=steer,
                        )
                    if controlled_kart is None and kart_states:
                        controlled_kart = sorted(kart_states.keys())[0]
                    if playable_mode:
                        ensure_claim()
                elif msg_type == "diag":
                    kart_info = msg.get("kart", {}) or {}
                    metrics = kart_info.get("metrics") or {}
                    kart_id = kart_info.get("id", "unknown")
                    lines = [f"Diagnostics [{kart_id}]"]
                    lines.extend(flatten_metrics(metrics))
                    latest_diag_lines = lines
                elif msg_type == "error":
                    print("Network error:", msg.get("detail"))
        except queue.Empty:
            pass

        throttle, brake, steer_dir = process_controls()
        steer_deg = steer_dir * STEER_DEG
        now = pygame.time.get_ticks() / 1000.0
        controls = (throttle, brake, steer_deg)
        if controlled_kart and (
            controls != last_sent_controls or (now - last_send_time) > 0.25
        ):
            outbound.put(
                {
                    "type": "input",
                    "id": controlled_kart,
                    "throttle": throttle,
                    "brake": brake,
                    "steer_deg": steer_deg,
                }
            )
            last_sent_controls = controls
            last_send_time = now

        screen.fill(BG_COLOR)
        draw_track(screen, track_pts, view)

        for kart in kart_states.values():
            draw_kart(screen, kart, view)

        mouse_pos = pygame.mouse.get_pos()
        for btn in buttons:
            btn.draw(screen, font, btn.contains(mouse_pos))

        mode_label = "Playable" if playable_mode else "Spectate"
        info_lines = [
            f"Mode: {mode_label}",
            f"Controlled Kart: {controlled_kart or 'n/a'}",
            f"Throttle: {throttle:.1f}  Brake: {brake:.1f}  Steer: {steer_deg:.0f}°",
        ]
        for i, line in enumerate(info_lines):
            text = font.render(line, True, TEXT_COLOR)
            screen.blit(text, (10, 50 + i * 20))
        if latest_diag_lines:
            start_y = 50 + len(info_lines) * 20 + 20
            for j, line in enumerate(latest_diag_lines):
                text = font.render(line, True, TEXT_COLOR)
                screen.blit(text, (10, start_y + j * 20))

        pygame.display.flip()

    if playable_mode and claim_sent and controlled_kart:
        outbound.put({"type": "claim", "id": controlled_kart, "playable": False})
        print(f"[client] Released {controlled_kart} back to AI control.")
    ws_client.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
