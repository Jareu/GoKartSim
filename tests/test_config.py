import math

from gokart_sim import config


def test_plinterp_behaviour():
    points = [(0.0, 0.0), (10.0, 10.0), (20.0, 40.0)]

    assert config.plinterp(-5.0, points) == 0.0
    assert config.plinterp(0.0, points) == 0.0
    assert config.plinterp(5.0, points) == 5.0
    assert config.plinterp(10.0, points) == 10.0
    assert config.plinterp(15.0, points) == 25.0
    assert config.plinterp(25.0, points) == 40.0


def test_make_nsc_material_sets_expected_properties():
    material = config.make_nsc_material(mu=0.42, cr=0.15)

    assert math.isclose(material.GetStaticFriction(), 0.42, rel_tol=1e-6)
    assert math.isclose(material.GetSlidingFriction(), 0.42, rel_tol=1e-6)
    assert math.isclose(material.GetRestitution(), 0.15, rel_tol=1e-6)


def test_angle_helpers_roundtrip():
    degrees = 45.0
    radians = config.rad(degrees)
    assert math.isclose(config.deg(radians), degrees, rel_tol=1e-12)
