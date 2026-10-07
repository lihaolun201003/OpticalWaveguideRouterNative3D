"""v0.1.2 repair A2 regression: the source pose is part of the verdict.

v0.1.1 computed the start position/tangent errors and printed them, but the final
verdict only counted the TARGET errors.  A path that starts 10 um away from the
source with a 90 deg wrong start tangent was reported verdict=pass, while the
same JSON printed start_position_error_um = 10 and start_tangent_error_deg = 90.

Counterexample actually run:

    path   : LineSegment3D((10,0,0), (100,0,0))
    source : (0,0,0), tangent (0,1,0)
    target : (100,0,0), tangent (1,0,0)
    chip   : [-2000,2000]^3, no obstacle, no reference centreline, default config
    v0.1.1 : verdict=pass, while printing a 10 um start error and a 90 deg
             start tangent error

The source has its own explicit tolerances (source_pos_tol_um /
source_ang_tol_deg); the target keeps the goal acceptance ball (eps_pos_um /
eps_ang_deg).  Neither replaces the C1, radius or length checks.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.validation.validate import validate_path_dict


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def scene_dict(lo, hi, source, target, obstacles=(), refs=()):
    return {
        "chip_volume": {"lo": list(lo), "hi": list(hi)},
        "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
        "reference_centerlines": list(refs),
        "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
        "target_pose": {"position": list(target[0]), "tangent": list(target[1])},
    }


def line_path(p0, p1):
    return {"segments": [{"kind": "line", "p0": list(p0), "p1": list(p1)}]}


def arc_seg(center, u, v, R, theta):
    return {"kind": "arc", "center": list(center), "u": list(u), "v": list(v),
            "radius_um": R, "theta_rad": theta}


BIG = ((-2000.0, -2000.0, -2000.0), (2000.0, 2000.0, 2000.0))


def test_the_exact_source_pose_counterexample_is_now_a_fail():
    path = line_path((10, 0, 0), (100, 0, 0))
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (0, 1, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    v = validate_path_dict(path, scene, cfg_dict())
    e = v["endpoint_errors"]
    assert e["start_position_error_um"] == pytest.approx(10.0)
    assert e["start_tangent_error_deg"] == pytest.approx(90.0)
    assert e["end_position_error_um"] == pytest.approx(0.0, abs=1e-9)
    assert v["verdict"] == "fail", "a 10 um / 90 deg source mismatch is not a pass"
    assert v["violation_counts"]["source_position_out_of_tolerance"] == 1
    assert v["violation_counts"]["source_tangent_out_of_tolerance"] == 1


def test_start_position_error_alone_fails():
    scene = scene_dict(BIG[0], BIG[1], source=((10.0, 0.0, 0.0), (1, 0, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    v = validate_path_dict(line_path((10, 0, 0), (100, 0, 0)), scene, cfg_dict())
    assert v["endpoint_errors"]["start_tangent_error_deg"] == pytest.approx(0.0)
    assert v["violation_counts"]["source_position_out_of_tolerance"] == 0
    assert v["violation_counts"]["source_tangent_out_of_tolerance"] == 0
    assert v["verdict"] == "pass"
    # the same path with the source 1 um earlier is now a source violation
    scene2 = scene_dict(BIG[0], BIG[1], source=((9.0, 0.0, 0.0), (1, 0, 0)),
                        target=((100, 0, 0), (1, 0, 0)))
    v2 = validate_path_dict(line_path((10, 0, 0), (100, 0, 0)), scene2, cfg_dict())
    assert v2["violation_counts"]["source_position_out_of_tolerance"] == 1
    assert v2["verdict"] == "fail"


def test_start_tangent_error_alone_fails():
    scene = scene_dict(BIG[0], BIG[1], source=((10.0, 0.0, 0.0), (0, 1, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    v = validate_path_dict(line_path((10, 0, 0), (100, 0, 0)), scene, cfg_dict())
    assert v["violation_counts"]["source_position_out_of_tolerance"] == 0
    assert v["violation_counts"]["source_tangent_out_of_tolerance"] == 1
    assert v["verdict"] == "fail"


def test_the_source_tolerance_is_explicit_and_configurable():
    path = line_path((10, 0, 0), (100, 0, 0))
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (1, 0, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    assert validate_path_dict(path, scene, cfg_dict())["verdict"] == "fail"
    loose = cfg_dict(source_pos_tol_um=20.0)
    assert validate_path_dict(path, scene, loose)["verdict"] == "pass"
    assert validate_path_dict(path, scene, cfg_dict(source_pos_tol_um=5.0)
                              )["verdict"] == "fail"
    # the target acceptance ball must NOT be reused for the source
    loose_target = cfg_dict(eps_pos_um=50.0)
    assert validate_path_dict(path, scene, loose_target)["verdict"] == "fail"


def test_source_and_target_tolerances_are_reported_separately():
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (1, 0, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    tol = validate_path_dict(line_path((0, 0, 0), (100, 0, 0)), scene,
                             cfg_dict())["endpoint_errors"]["tolerances"]
    assert tol["source_pos_tol_um"] == pytest.approx(1e-6)
    assert tol["source_ang_tol_deg"] == pytest.approx(1e-6)
    assert tol["target_eps_pos_um"] == pytest.approx(2.0)
    assert tol["target_eps_ang_deg"] == pytest.approx(3.0)


def test_a_legal_start_and_end_still_passes():
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (1, 0, 0)),
                       target=((100, 0, 0), (1, 0, 0)))
    v = validate_path_dict(line_path((0, 0, 0), (100, 0, 0)), scene, cfg_dict())
    assert v["verdict"] == "pass", v["violation_counts"]


def test_correct_endpoints_do_not_replace_the_c1_check():
    path = {"segments": [{"kind": "line", "p0": [0, 0, 0], "p1": [50, 0, 0]},
                         {"kind": "line", "p0": [50, 0, 0], "p1": [50, 50, 0]}]}
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (1, 0, 0)),
                       target=((50, 50, 0), (0, 1, 0)))
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["max_junction_tangent_error_deg"] == pytest.approx(90.0)
    assert v["violation_counts"]["non_c1_junctions"] >= 1
    assert v["c1_continuous"] is False
    assert v["verdict"] == "fail"


def test_correct_endpoints_do_not_replace_the_radius_or_length_check():
    # R = 5 um is far below Rmin = 50 um, endpoints and tangents are exact
    seg = arc_seg((0.0, 5.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), 5.0,
                  math.pi / 2.0)
    scene = scene_dict(BIG[0], BIG[1], source=((0, 0, 0), (1, 0, 0)),
                       target=((5.0, 5.0, 0.0), (0, 1, 0)))
    v = validate_path_dict({"segments": [seg]}, scene, cfg_dict())
    assert v["radius_violations"], "R < Rmin must still be reported"
    assert v["verdict"] == "fail"
    assert v["total_length_um_analytic"] == pytest.approx(5.0 * math.pi / 2.0)
    assert v["length_relative_error"] < 1e-6
