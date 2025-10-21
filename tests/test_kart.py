import math

from pychrono import core as chrono

from gokart_sim.config import KartConfig
from gokart_sim.kart import GoKart


def make_system_and_kart(name="kart_test"):
    sys = chrono.ChSystemNSC()
    kart = GoKart(sys, KartConfig(), name=name)
    return sys, kart


def test_set_controls_clamps_and_records_inputs():
    _, kart = make_system_and_kart()
    cfg = kart.cfg

    kart.set_controls(throttle=0.8, steer_rad=cfg.max_steer_rad * 2, brake=0.4)

    assert math.isclose(kart.last_throttle, 0.8)
    assert math.isclose(kart.last_brake, 0.4)
    assert math.isclose(kart.last_steer, cfg.max_steer_rad)


def test_update_axle_torques_engine_and_brake_setpoints():
    _, kart = make_system_and_kart()

    kart.set_controls(throttle=1.0, steer_rad=0.0, brake=0.0)
    kart.update_axle_torques()
    assert math.isclose(kart.engine_fun.GetSetpoint(), 30.0, rel_tol=1e-6)
    assert math.isclose(kart.brake_fun.GetSetpoint(), 0.0, abs_tol=1e-9)

    kart.set_controls(throttle=0.0, steer_rad=0.0, brake=1.0)
    kart.axle.SetAngVelLocal(chrono.ChVector3d(0.0, 10.0, 0.0))
    kart.update_axle_torques()
    # Engine torque includes drag: 0 (no throttle) - axle_drag_coefficient * omega
    expected_drag_torque = -kart.cfg.axle_drag_coefficient * 10.0
    assert math.isclose(kart.engine_fun.GetSetpoint(), expected_drag_torque, rel_tol=1e-6)
    assert math.isclose(kart.brake_fun.GetSetpoint(), -kart.cfg.max_brake_torque, rel_tol=1e-6)


def test_apply_tire_forces_runs_without_error():
    _, kart = make_system_and_kart()
    kart.set_controls(throttle=0.7, steer_rad=0.1, brake=0.0)
    kart.update_axle_torques()

    # No assertions on internal Chrono state; the call should simply succeed.
    kart.apply_tire_forces(kart.cfg.step_size)


def test_get_state_initial_values():
    _, kart = make_system_and_kart()
    state = kart.get_state()

    assert state["id"] == kart.name
    assert state["pos"] == (0.0, 0.0, 0.25)
    assert state["vel"] == (0.0, 0.0, 0.0)
    assert state["quat"] == (0.0, 0.0, 0.0, 1.0)
    assert math.isclose(state["yaw"], 0.0, abs_tol=1e-9)
    assert math.isclose(state["yaw_rate"], 0.0, abs_tol=1e-9)
    assert math.isclose(state["speed"], 0.0, abs_tol=1e-9)
    assert math.isclose(state["axle_omega"], 0.0, abs_tol=1e-9)
