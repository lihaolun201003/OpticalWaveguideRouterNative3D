"""v0.1.2 repair A3 regression: the independent validator must see reference ARCS.

v0.1.1 Scene supports arc reference centrelines (io/caseio.py rebuilds them) but
the independent validator filtered kind == "line" only, so an arc reference was
silently dropped from the certification.  Two consequences were reproduced:

    path      : LineSegment3D((10,0,0), (100,0,0))
    reference : CircularArc3D((50,50,0), (0,-1,0), (1,0,0), 50, pi)   <- intersects
                LineSegment3D((0,100,0), (100,100,0))                 <- far away
    chip      : [-2000,2000]^3, default config, no obstacle

    v0.1.1 production spacing    : violation  (bound 3.806 um)
    v0.1.1 independent validator : pass, spacing 100 um  (the far LINE only)
    v0.1.1 with the arc alone    : TypeError: "<" not supported between
                                   instances of "NoneType" and "float"

The validator now decides line/line, line/arc, arc/line and arc/arc, subtracts
the geometric approximation error of BOTH sides when both are curves, and
explicitly rejects unsupported or malformed input instead of ignoring it.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.validation.validate import validate_path_dict


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def line_seg(p0, p1):
    return {"kind": "line", "p0": list(p0), "p1": list(p1)}


def arc_seg(center, u, v, R, theta):
    return {"kind": "arc", "center": list(center), "u": list(u), "v": list(v),
            "radius_um": R, "theta_rad": theta}


def scene_dict(refs, source, target, obstacles=(), lo=(-2000.0,) * 3,
               hi=(2000.0,) * 3):
    return {
        "chip_volume": {"lo": list(lo), "hi": list(hi)},
        "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
        "reference_centerlines": list(refs),
        "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
        "target_pose": {"position": list(target[0]), "tangent": list(target[1])},
    }


# The arc below passes exactly through (50, 0, 0), which is ON the path.
CROSSING_ARC = arc_seg((50.0, 50.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0),
                       50.0, math.pi)
FAR_LINE = line_seg((0.0, 100.0, 0.0), (100.0, 100.0, 0.0))
LINE_PATH = {"segments": [line_seg((10.0, 0.0, 0.0), (100.0, 0.0, 0.0))]}
PATH_SCENE_POSES = dict(source=((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                        target=((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))


def test_the_arc_really_intersects_the_path():
    c, u, v = CROSSING_ARC["center"], CROSSING_ARC["u"], CROSSING_ARC["v"]
    R, th = CROSSING_ARC["radius_um"], CROSSING_ARC["theta_rad"]
    def at(phi):
        return (c[0] + R * (math.cos(phi) * u[0] + math.sin(phi) * v[0]),
                c[1] + R * (math.cos(phi) * u[1] + math.sin(phi) * v[1]),
                c[2] + R * (math.cos(phi) * u[2] + math.sin(phi) * v[2]))
    assert at(0.0) == pytest.approx((50.0, 0.0, 0.0), abs=1e-12)
    assert at(math.pi / 2.0) == pytest.approx((100.0, 50.0, 0.0), abs=1e-12)
    assert at(math.pi) == pytest.approx((50.0, 100.0, 0.0), abs=1e-12)
    # the START of the path (10,0,0)->(100,0,0) is not on the arc, but
    # (50,0,0) is: the arc leaves the path at that point
    assert at(0.0)[1] == 0.0 and 10.0 < at(0.0)[0] < 100.0


def test_the_exact_counterexample_is_now_a_fail():
    scene = scene_dict([CROSSING_ARC, FAR_LINE], **PATH_SCENE_POSES)
    v = validate_path_dict(LINE_PATH, scene, cfg_dict())
    sp = v["centerline_spacing"]
    assert sp["status"] == "violation", "the arc reference really is crossed"
    assert sp["min_sampled_um"] < 5.0, "the sampled spacing must see the crossing"
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9
    assert v["verdict"] == "fail", "the v0.1.1 verdict was pass with spacing 100"
    assert v["violation_counts"]["spacing_below_required"] >= 1


def test_a_pure_arc_reference_set_does_not_raise_and_is_not_a_pass():
    scene = scene_dict([CROSSING_ARC], **PATH_SCENE_POSES)
    v = validate_path_dict(LINE_PATH, scene, cfg_dict())
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["verdict"] == "fail"
    assert "arc" in v["centerline_spacing"]["method"]


def test_a_legal_arc_reference_set_still_passes():
    far_arc = arc_seg((0.0, 1500.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
                      math.pi)
    scene = scene_dict([far_arc], **PATH_SCENE_POSES)
    v = validate_path_dict(LINE_PATH, scene, cfg_dict())
    assert v["centerline_spacing"]["status"] == "safe"
    assert v["verdict"] == "pass", v["violation_counts"]


def test_line_versus_arc_and_arc_versus_line_are_both_decided():
    # path LINE crossing a reference ARC
    v1 = validate_path_dict(LINE_PATH,
                            scene_dict([CROSSING_ARC], **PATH_SCENE_POSES),
                            cfg_dict())
    assert v1["verdict"] == "fail"
    # path ARC crossing a reference LINE
    path_arc = arc_seg((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
                       math.pi)
    ref_line = line_seg((0.0, -100.0, 0.0), (0.0, 100.0, 0.0))
    scene2 = scene_dict([ref_line], source=((50.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                        target=((-50.0, 0.0, 0.0), (0.0, -1.0, 0.0)))
    v2 = validate_path_dict({"segments": [path_arc]}, scene2, cfg_dict())
    assert v2["centerline_spacing"]["status"] == "violation"
    assert v2["verdict"] == "fail"
    assert v2["centerline_spacing"]["per_segment"][0]["kind"] == "arc"


def test_arc_versus_arc_subtracts_both_geometric_errors():
    # the upper semicircle lies exactly ON the full circle with the same centre
    path_arc = arc_seg((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
                       math.pi)
    same_circle = arc_seg((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
                          2.0 * math.pi)
    scene = scene_dict([same_circle], source=((50.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                       target=((-50.0, 0.0, 0.0), (0.0, -1.0, 0.0)))
    v = validate_path_dict({"segments": [path_arc]}, scene, cfg_dict())
    sp = v["centerline_spacing"]
    assert sp["status"] == "violation", (sp["status"], sp["min_sampled_um"])
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9
    assert v["verdict"] == "fail"


def test_arc_versus_arc_that_is_really_separated_is_safe():
    path_arc = arc_seg((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
                       math.pi)
    other = arc_seg((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 130.0,
                    math.pi)
    scene = scene_dict([other], source=((50.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                       target=((-50.0, 0.0, 0.0), (0.0, -1.0, 0.0)))
    v = validate_path_dict({"segments": [path_arc]}, scene, cfg_dict())
    sp = v["centerline_spacing"]
    assert sp["status"] == "safe"
    # both curves are concentric quarter-turn-refined arcs 80 um apart radially:
    # the sampled estimate reaches 80 while the certified lower bound is a
    # conservative number below it (it must never exceed it)
    assert sp["min_sampled_um"] == pytest.approx(80.0, abs=0.05)
    assert sp["conservative_lower_bound_um"] <= 80.0 + 1e-9
    assert sp["conservative_lower_bound_um"] >= 5.0
    assert v["verdict"] == "pass", v["violation_counts"]


@pytest.mark.parametrize("bad", [
    {"kind": "spline", "p0": [0, 0, 0], "p1": [1, 1, 1]},
    {"kind": "line", "p0": [0, 0, 0]},
    {"kind": "line", "p0": [0, 0, 0], "p1": [float("nan"), 0, 0]},
    {"kind": "arc", "center": [0, 0, 0], "u": [1, 0, 0], "v": [0, 1, 0],
     "radius_um": -5.0, "theta_rad": 1.0},
    {"kind": "arc", "center": [0, 0, 0], "u": [1, 0, 0], "v": [0, 1, 0],
     "radius_um": 5.0},
    {"p0": [0, 0, 0], "p1": [1, 1, 1]},
])
def test_unsupported_or_malformed_reference_input_is_rejected(bad):
    scene = scene_dict([bad], **PATH_SCENE_POSES)
    with pytest.raises(ValueError):
        validate_path_dict(LINE_PATH, scene, cfg_dict())


def test_unsupported_path_segment_kind_is_rejected():
    scene = scene_dict([FAR_LINE], **PATH_SCENE_POSES)
    bad_path = {"segments": [{"kind": "bezier", "p0": [0, 0, 0], "p1": [1, 0, 0]}]}
    with pytest.raises(ValueError):
        validate_path_dict(bad_path, scene, cfg_dict())


def test_the_reference_kind_census_is_reported():
    scene = scene_dict([CROSSING_ARC, FAR_LINE], **PATH_SCENE_POSES)
    v = validate_path_dict(LINE_PATH, scene, cfg_dict())
    assert v["centerline_spacing"]["n_reference_curves"] == 2
    assert v["centerline_spacing"]["reference_kinds"] == ["arc", "line"]
