"""Phase 2 tests: the state is (x, y, z, genuine 3D direction)."""

import math

import pytest

from native3d.config import RouterConfig
from native3d.search.state import State3D


def test_state_keeps_3d_position_and_unit_3d_direction():
    s = State3D((1.0, 2.0, 3.0), (0.0, 1.0, 1.0))
    assert norm_ok(s.direction)
    assert s.z == 3.0
    assert s.elevation_deg == pytest.approx(45.0)
    assert s.azimuth_deg == pytest.approx(90.0)


def norm_ok(v):
    return abs(math.sqrt(sum(c * c for c in v)) - 1.0) < 1e-12


def test_same_position_different_direction_never_shares_a_dedup_key():
    cfg = RouterConfig()
    a = State3D((10.0, 20.0, 30.0), (1.0, 0.0, 0.0))
    b = State3D((10.0, 20.0, 30.0), (0.0, 1.0, 0.0))
    c = State3D((10.0, 20.0, 30.0), (1.0, 0.0, 3.0))
    ka = a.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
    kb = b.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
    kc = c.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
    assert ka != kb, "direction must be part of the dedup key"
    assert ka != kc, "elevation must be part of the dedup key"
    assert len(ka) == 5


def test_quantisation_resolution_is_configurable_and_exact_pose_is_kept():
    c1 = RouterConfig(pos_resolution_um=1.0, dir_resolution_deg=5.0)
    c2 = RouterConfig(pos_resolution_um=50.0, dir_resolution_deg=45.0)
    s1 = State3D((10.4, 0.0, 0.0), (1.0, 0.0, 0.0))
    s2 = State3D((10.6, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert s1.key(1.0, 5.0) != s2.key(1.0, 5.0)
    assert s1.key(50.0, 45.0) == s2.key(50.0, 45.0)
    # the exact pose is never snapped: the state keeps the real numbers
    assert s1.position == (10.4, 0.0, 0.0)
    assert s1.key(1.0, 5.0)[0] == 10 and s2.key(1.0, 5.0)[0] == 11


def test_state_rejects_degenerate_direction():
    with pytest.raises(ValueError):
        State3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_azimuth_wraps_in_the_key():
    s = State3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    s2 = State3D((0.0, 0.0, 0.0), (math.cos(math.radians(359.0)), math.sin(math.radians(359.0)), 0.0))
    assert s.key(10.0, 15.0) == s2.key(10.0, 15.0)
