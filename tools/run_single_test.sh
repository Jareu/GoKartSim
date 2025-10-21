#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUTDIR="$ROOT_DIR/artifacts/run_$(date +%s)"
SCENARIO="$ROOT_DIR/tests/scenarios/single_kart_sine.yaml"
URI="ws://127.0.0.1:8765/ws"

mkdir -p "$OUTDIR"

python "$ROOT_DIR/gokartsim.py" --host 127.0.0.1 --port 8765 --karts 1 --fps 100 --step 0.001 --scenario "$SCENARIO" &
SERVER_PID=$!
trap 'kill $SERVER_PID 2>/dev/null || true' EXIT

sleep 0.2

python "$ROOT_DIR/tools/telemetry_client.py" --uri "$URI" --duration 10.0 --outdir "$OUTDIR" --meta-scenario "$SCENARIO"

wait $SERVER_PID || true

python "$ROOT_DIR/tools/analyze_run.py" --csv "$OUTDIR/telemetry.csv" --fps 100 --meta "$OUTDIR/meta.json" --out "$OUTDIR/report.json"

echo "Artifacts: $OUTDIR"
cat "$OUTDIR/report.json"
