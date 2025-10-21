import math

from gokart_sim.drivers import (
    ConstantDriver,
    SineWaveDriver,
    NullDriver,
    IdleDriver,
    DriverFactory,
    DriverKind,
)
from gokart_sim.config import rad


def test_constant_driver_returns_configured_controls():
    driver = ConstantDriver(throttle=0.6, steer_deg=10.0, brake=0.2)

    controls = driver.compute_controls(t=1.23, state={})

    assert controls == (0.6, rad(10.0), 0.2)


def test_sine_wave_driver_generates_expected_amplitude():
    driver = SineWaveDriver(amplitude_deg=20.0, frequency_hz=0.5, throttle=0.5, brake=0.0)

    # key points of sine wave: t=0 -> 0, quarter period -> +amp, half -> 0, 3/4 -> -amp
    period = 1.0 / 0.5
    throttle, steer, brake = driver.compute_controls(t=0.0, state={})
    assert math.isclose(throttle, 0.5)
    assert math.isclose(steer, 0.0, abs_tol=1e-9)
    assert math.isclose(brake, 0.0)

    _, steer_peak, _ = driver.compute_controls(t=period / 4, state={})
    assert math.isclose(steer_peak, rad(20.0), rel_tol=1e-6)

    _, steer_neg, _ = driver.compute_controls(t=3 * period / 4, state={})
    assert math.isclose(steer_neg, -rad(20.0), rel_tol=1e-6)


def test_null_driver_returns_zero_controls():
    driver = NullDriver()

    throttle, steer, brake = driver.compute_controls(t=123.4, state={"id": "kart_1"})

    assert throttle == 0.0
    assert steer == 0.0
    assert brake == 0.0


def test_driver_factory_creates_requested_kind():
    sine_driver = DriverFactory.create(
        DriverKind.SINE, amplitude_deg=10.0, frequency_hz=1.0, throttle=0.4, brake=0.1
    )
    assert isinstance(sine_driver, SineWaveDriver)

    idle_driver = DriverFactory.create("idle")
    assert isinstance(idle_driver, IdleDriver)
