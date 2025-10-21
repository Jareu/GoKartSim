import math

from gokart_sim.tires import SimpleTireModel


def test_simple_tire_zero_speed_no_force():
    tire = SimpleTireModel()

    Fy, Mz = tire.step(dt=0.01, u_x=0.1, u_y=1.0, Fz=4000.0)

    assert Fy == 0.0
    assert Mz == 0.0
    assert math.isclose(tire.alpha_eff, 0.0, abs_tol=1e-12)


def test_simple_tire_produces_lateral_force():
    tire = SimpleTireModel()

    Fy, Mz = tire.step(dt=0.01, u_x=12.0, u_y=2.0, Fz=3000.0)

    assert Fy < 0.0  # restoring force opposes positive slip angle
    assert Mz > 0.0  # aligning moment opposes yawing direction
    assert abs(Fy) <= tire.mu * 3000.0 + 1e-6
