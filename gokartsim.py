#!/usr/bin/env python3
"""
GoKart Chrono simulation WebSocket server entrypoint.
"""

import argparse
import asyncio
import signal

from gokart_sim import SimServer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Chrono multi-kart sim with WebSocket API (control + streaming)"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--karts", type=int, default=4)
    parser.add_argument("--fps", type=int, default=60, help="stream / sim frame rate")
    parser.add_argument("--step", type=float, default=1e-3, help="physics step (s)")
    return parser.parse_args()


def main():
    args = parse_args()
    server = SimServer(
        host=args.host,
        port=args.port,
        fps=args.fps,
        step=args.step,
        num_karts=args.karts,
    )

    async def runner():
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, server.stop)
            except NotImplementedError:
                # On platforms without signal support in asyncio loops (e.g. Windows)
                pass
        await server.run()

    asyncio.run(runner())


if __name__ == "__main__":
    main()

