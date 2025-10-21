#!/usr/bin/env python3
"""Analyse telemetry against a linear bicycle model."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from gokart_sim.config import KartConfig


def load_meta(meta_path: Path | None):
    if meta_path is None:
        return {}
    if not meta_path.exists():
        raise FileNotFoundError(meta_path)
    return json.loads(meta_path.read_text())


def dominant_frequency(signal: np.ndarray, dt: float, target: float) -> tuple[float, float]:
    N = len(signal)
    if N < 2:
        return 0.0, 0.0
    freqs = np.fft.rfftfreq(N, d=dt)
    fft_vals = np.fft.rfft(signal)
    idx = int(np.argmin(np.abs(freqs - target)))
    freq = freqs[idx]
    mag = np.abs(fft_vals[idx])
    amplitude = 2.0 * mag / N
    return freq, amplitude


def high_frequency_energy(signal: np.ndarray, dt: float, cutoff: float) -> tuple[float, float]:
    N = len(signal)
    if N < 2:
        return 0.0, 0.0
    freqs = np.fft.rfftfreq(N, d=dt)
    fft_vals = np.fft.rfft(signal)
    mask = freqs >= cutoff
    energy_hi = np.sum(np.abs(fft_vals[mask])) * (2.0 / max(N, 1))
    idx_main = int(np.argmin(np.abs(freqs - 0.5)))
    main_mag = 2.0 * np.abs(fft_vals[idx_main]) / N
    return energy_hi, main_mag


def cross_correlation_lag(x: np.ndarray, y: np.ndarray, dt: float) -> float:
    if len(x) == 0 or len(y) == 0:
        return 0.0
    x0 = x - x.mean()
    y0 = y - y.mean()
    corr = np.correlate(x0, y0, mode="full")
    lags = np.arange(-len(x) + 1, len(x))
    lag_idx = lags[np.argmax(corr)]
    return lag_idx * dt


def bicycle_model_response(u: float, freq_hz: float, delta_amp: float):
    params = KartConfig.dynamics_params()
    m = params["m"]
    L = params["L"]
    a = params["a"]
    b = params["b"]
    Iz = params["Iz"]
    Cf = params["Cf"]
    Cr = params["Cr"]
    u = max(u, 0.5)
    omega = 2.0 * math.pi * freq_hz
    A = np.array(
        [
            [-(Cf + Cr) / (m * u), -(u + (a * Cf - b * Cr) / (m * u))],
            [(a * Cf - b * Cr) / (Iz * u), -(a * a * Cf + b * b * Cr) / (Iz * u)],
        ],
        dtype=float,
    )
    B = np.array([[Cf / m], [a * Cf / Iz]], dtype=float)
    C = np.array([[0.0, 1.0]], dtype=float)
    jwI_minus_A = 1j * omega * np.eye(2) - A
    G = C @ np.linalg.inv(jwI_minus_A) @ B
    G = G[0, 0]
    r_amp = abs(G) * delta_amp
    lag = np.angle(G) / omega
    return {
        "u": u,
        "Cf": Cf,
        "Cr": Cr,
        "m": m,
        "L": L,
        "a": a,
        "b": b,
        "Iz": Iz,
        "freq_hz": freq_hz,
        "omega": omega,
        "G_mag": abs(G),
        "G_phase_rad": float(np.angle(G)),
        "r_amp_expected": r_amp,
        "lag_expected_s": lag,
    }


def analyse(csv_path: Path, fps: float, meta_path: Path | None, out_path: Path):
    meta = load_meta(meta_path)
    df = pd.read_csv(csv_path)
    if df.shape[0] < 10:
        raise RuntimeError("Telemetry too short for analysis")

    df = df.sort_values("t")
    mask = df["t"] >= 1.0
    if mask.sum() < 20:
        mask = df["t"] >= 0.0

    t = df.loc[mask, "t"].to_numpy()
    steer = df.loc[mask, "steer"].to_numpy()
    yaw_rate = df.loc[mask, "yaw_rate"].to_numpy()
    speed = df.loc[mask, "speed"].to_numpy()

    dt = float(np.mean(np.diff(t))) if len(t) > 1 else 1.0 / fps

    steer_freq, steer_amp = dominant_frequency(steer, dt, 0.5)
    yaw_freq, yaw_amp = dominant_frequency(yaw_rate, dt, 0.5)
    steer_pp = 2.0 * steer_amp

    lag_measured = cross_correlation_lag(steer, yaw_rate, dt)
    hi_energy, main_mag = high_frequency_energy(yaw_rate, dt, 2.0)

    speed_mean = float(speed.mean())
    speed_max = float(speed.max())
    u_for_model = speed_mean if speed_mean > 1.0 else 2.0

    delta_amp = math.radians(20.0)
    scenario_cfg = meta.get("scenario", {})
    steer_cfg = scenario_cfg.get("kart", {}).get("steer", {}) if scenario_cfg else {}
    if steer_cfg.get("kind") == "sine":
        delta_amp = math.radians(float(steer_cfg.get("amplitude_deg", 20.0)))

    model = bicycle_model_response(u_for_model, 0.5, delta_amp)

    steer_freq_ok = 0.45 <= steer_freq <= 0.55
    steer_amp_ok = 0.628 <= steer_pp <= 0.768
    yaw_amp_ok = abs(yaw_amp - model["r_amp_expected"]) <= 0.25 * max(model["r_amp_expected"], 1e-6)
    lag_ok = abs(lag_measured - model["lag_expected_s"]) <= 0.15
    speed_ok = speed_mean >= 1.0
    hi_freq_ok = hi_energy <= 0.1 * max(main_mag, 1e-6)

    passfail = {
        "steer_freq_ok": bool(steer_freq_ok),
        "steer_amp_ok": bool(steer_amp_ok),
        "yaw_amp_ok": bool(yaw_amp_ok),
        "yaw_lag_ok": bool(lag_ok),
        "speed_ok": bool(speed_ok),
        "hi_freq_ok": bool(hi_freq_ok),
    }
    passfail["overall"] = all(passfail.values())

    kpis = {
        "steer_dom_freq_hz": steer_freq,
        "steer_amp_pp_rad": steer_pp,
        "yaw_dom_freq_hz": yaw_freq,
        "yaw_dom_amp_rad_s": yaw_amp,
        "lag_measured_s": lag_measured,
        "lag_expected_s": model["lag_expected_s"],
        "speed_mean": speed_mean,
        "speed_max": speed_max,
        "hi_freq_energy": hi_energy,
        "hi_freq_main_mag": main_mag,
    }

    hints = []
    if not speed_ok:
        hints.append("Average speed below 1 m/s; increase push-off speed or verify traction settings.")
    if not hi_freq_ok:
        hints.append("High-frequency yaw content detected; consider reducing timestep or increasing tire relaxation length.")
    if not yaw_amp_ok:
        hints.append("Yaw-rate amplitude deviates from linear model; adjust axle cornering stiffness or vehicle mass distribution.")
    if not lag_ok:
        hints.append("Yaw/steer phase lag differs from model; tune tire relaxation or steering filters.")
    if not steer_amp_ok:
        hints.append("Steering amplitude not tracking commanded ±20°; review steering controller or rate limits.")
    if not steer_freq_ok:
        hints.append("Steering dominant frequency not 0.5 Hz; verify driver input scheduling.")

    report = {
        "kpis": kpis,
        "model": model,
        "passfail": passfail,
        "hints": hints,
        "meta": meta,
    }
    out_path.write_text(json.dumps(report, indent=2))
    print(json.dumps({"overall": passfail["overall"], **kpis}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyse GoKartSim telemetry")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=100.0)
    parser.add_argument("--meta", type=Path)
    args = parser.parse_args()

    analyse(args.csv, args.fps, args.meta, args.out)


if __name__ == "__main__":
    main()
