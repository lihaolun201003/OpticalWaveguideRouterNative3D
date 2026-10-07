"""v0.1.2 repair A4 regression: the sagitta bound needs |delta_theta| <= pi.

The chord/sagitta error bound R (1 - cos(dphi/2)) is only a bound on the
deviation of a chord from its sub-arc when the central angle dphi of that
sub-arc is inside the proven domain (dphi <= pi).  v0.1.1 started the arc
subdivision at n = 1 or 2 chords whatever the sweep was, and cos() is periodic:
for a sweep of 8*pi and n = 2 the code computed dphi = 4*pi and therefore
sagitta = 50 (1 - cos(2*pi)) = 0, i.e. it claimed the arc equals a degenerate
chord and so "certified a lower bound" of 99.005 um for a curve that passes
through the reference line and through the box.

Counterexample actually run (production AND independent validator):

    arc      : CircularArc3D((0,0,0), (1,0,0), (0,1,0), 50, 8*pi)
    reference: LineSegment3D((-51,-1,0), (-49,1,0))      <- the arc crosses it
    obstacle : Box3D((-51,-1,-1), (-49,1,1))             <- the arc crosses it
    chip     : [-2000,2000]^3, source/target on the arc itself

    v0.1.1 production spacing   : safe, claimed lower bound 99.005050376 um
    v0.1.1 production obstacle  : safe, claimed lower bound 99.0 um
    v0.1.1 independent validator: sampled spacing 0.022088195608 < dmin = 5,
                                  claimed lower bound 99.005050376,
                                  consistency flag False, verdict = pass

v0.1.2 (a) subdivides until every chord is inside the proven domain, (b) lets a
reliable violation witness (a curve point inside the keep-out) override the
"lower bound says safe" branch, and (c) makes a lower bound above the sampled
upper bound change the verdict instead of being reported as a pass.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.constraints.checks import (Box3D, chord_count_for_domain,
                                         primitive_box_clearance,
                                         primitive_spacing, sagitta_bound_um)
from native3d.geometry import CircularArc3D, LineSegment3D


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def scene_dict(refs, source, target, obstacles=()):
    return {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
        "reference_centerlines": list(refs),
        "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
        "target_pose": {"position": list(target[0]), "tangent": list(target[1])},
    }


CU = (0.0, 0.0, 0.0)
UU = (1.0, 0.0, 0.0)
VV = (0.0, 1.0, 0.0)
REF_LINE = LineSegment3D((-51.0, -1.0, 0.0), (-49.0, 1.0, 0.0))
REF_BOX = Box3D((-51.0, -1.0, -1.0), (-49.0, 1.0, 1.0))
BIG_SWEEP = 8.0 * math.pi


def big_arc(theta=BIG_SWEEP, radius=50.0):
    return CircularArc3D(CU, UU, VV, radius, theta)


def arc_scene(arc, refs=(), obstacles=()):
    return scene_dict(list(refs),
                      (arc.start, arc.tangent_at_phi(0.0)),
                      (arc.end, arc.tangent_at_phi(arc.theta)), obstacles)


def test_the_multi_turn_arc_really_crosses_the_reference_and_the_box():
    arc = big_arc()
    assert arc.theta == pytest.approx(BIG_SWEEP)
    # the arc passes through (-50, 0, 0), the midpoint of the reference line
    hit = [arc.point_at(arc.length * i / 40000.0) for i in range(40001)]
    d_line = min(abs(p[0] + 50.0) + abs(p[1]) for p in hit)
    assert d_line == pytest.approx(0.0, abs=1e-12)
    assert REF_BOX.contains_point((-50.0, 0.0, 0.0))


def test_the_exact_counterexample_is_no_longer_reported_safe():
    cfg = RouterConfig()
    arc = big_arc()
    sp = primitive_spacing(arc, [REF_LINE], cfg.dmin_centerline_um, cfg)
    oc = primitive_box_clearance(arc, REF_BOX, cfg.effective_obstacle_clearance_um,
                                 cfg)
    assert sp.status == "violation", (sp.status, sp.bound_um, sp.detail)
    assert oc.status == "violation", (oc.status, oc.bound_um, oc.detail)


def test_the_exact_counterexample_is_no_longer_a_validator_pass():
    from native3d.validation.validate import validate_path_dict
    arc = big_arc()
    scene = arc_scene(arc, refs=[REF_LINE.to_dict()])
    v = validate_path_dict({"segments": [arc.to_dict()]}, scene, cfg_dict())
    sp = v["centerline_spacing"]
    assert sp["status"] == "violation", (sp["status"], sp["min_sampled_um"])
    assert sp["min_sampled_um"] < 5.0
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9
    assert v["verdict"] == "fail", "the v0.1.1 verdict was pass here"
    assert v["certification"]
    ["certified_lower_bounds_are_never_above_the_sampled_upper_bounds"] is True


def test_the_box_counterexample_is_not_a_validator_pass_either():
    from native3d.validation.validate import validate_path_dict
    arc = big_arc()
    scene = arc_scene(arc, obstacles=[(REF_BOX.lo, REF_BOX.hi)])
    v = validate_path_dict({"segments": [arc.to_dict()]}, scene, cfg_dict())
    assert v["obstacle_clearance"]["status"] == "violation"
    assert v["verdict"] == "fail"


# ------------------------------------------------- the applicability domain
@pytest.mark.parametrize("theta", [
    math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0, 2.0 * math.pi,
    5.0 * math.pi / 2.0, 4.0 * math.pi, 8.0 * math.pi, 16.0 * math.pi,
    -math.pi / 2.0, -math.pi, -3.0 * math.pi / 2.0, -2.0 * math.pi,
    -5.0 * math.pi / 2.0, -4.0 * math.pi, -8.0 * math.pi,
])
@pytest.mark.parametrize("requested", [1, 2, 3, 5])
def test_every_chord_is_inside_the_proven_domain(theta, requested):
    n = chord_count_for_domain(theta, requested)
    assert n >= requested
    assert abs(theta) / n <= math.pi + 1e-12, (theta, requested, n)


def test_the_minimum_chord_count_covers_every_partial_turn():
    assert chord_count_for_domain(math.pi / 2.0, 1) == 1
    assert chord_count_for_domain(math.pi, 1) == 1
    assert chord_count_for_domain(math.pi - 1e-9, 1) == 1
    assert chord_count_for_domain(math.pi + 1e-9, 1) == 2
    assert chord_count_for_domain(2.0 * math.pi, 1) == 2
    assert chord_count_for_domain(8.0 * math.pi, 1) == 8
    assert chord_count_for_domain(16.0 * math.pi, 1) == 16
    assert chord_count_for_domain(-8.0 * math.pi, 1) == 8
    assert chord_count_for_domain(2.0 * math.pi, 7) == 7


def test_endpoint_coincidence_never_implies_zero_deviation():
    """A full turn has start == end; that must NOT make the sagitta zero."""
    arc = big_arc(2.0 * math.pi)
    assert arc.start == pytest.approx(arc.end, abs=1e-12)
    # requested=1 is raised into the domain (n = 2, dphi = pi), never evaluated
    # outside it; the v0.1.1 code used n = 1, dphi = 2*pi and cos(pi) = -1 -> 2R,
    # and used n = 2 with dphi = 4*pi for the 8*pi arc -> 0.
    assert sagitta_bound_um(arc, 1) == pytest.approx(50.0)
    assert sagitta_bound_um(arc, 2) == pytest.approx(50.0)
    assert sagitta_bound_um(arc, 4) == pytest.approx(
        50.0 * (1.0 - math.cos(math.pi / 4.0)))
    assert sagitta_bound_um(big_arc(8.0 * math.pi), 2) == pytest.approx(50.0)
    # 8*pi over 16 chords is pi/2 per chord
    assert sagitta_bound_um(big_arc(8.0 * math.pi), 16) == pytest.approx(
        50.0 * (1.0 - math.cos(math.pi / 4.0)))
    line = LineSegment3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert sagitta_bound_um(line, 3) == 0.0


@pytest.mark.parametrize("theta", [
    math.pi, 3.0 * math.pi / 2.0, 2.0 * math.pi, 5.0 * math.pi / 2.0,
    4.0 * math.pi, 8.0 * math.pi, -math.pi, -3.0 * math.pi / 2.0,
    -2.0 * math.pi, -4.0 * math.pi, -8.0 * math.pi,
])
def test_every_multi_turn_arc_that_reaches_the_reference_is_a_violation(theta):
    cfg = RouterConfig()
    arc = big_arc(theta)
    # |theta| >= pi means the sweep reaches phase pi, i.e. the point (-50,0,0)
    assert abs(arc.theta) >= math.pi - 1e-12
    sp = primitive_spacing(arc, [REF_LINE], cfg.dmin_centerline_um, cfg)
    oc = primitive_box_clearance(arc, REF_BOX, cfg.effective_obstacle_clearance_um,
                                 cfg)
    assert sp.status == "violation", (theta, sp.status, sp.bound_um)
    assert oc.status == "violation", (theta, oc.status, oc.bound_um)


def test_a_large_sweep_arc_that_stays_clear_is_still_safe():
    cfg = RouterConfig()
    arc = big_arc()
    far = LineSegment3D((-3000.0, -1.0, 0.0), (-2998.0, 1.0, 0.0))
    far_box = Box3D((-3000.0, -1.0, -1.0), (-2998.0, 1.0, 1.0))
    assert primitive_spacing(arc, [far], 5.0, cfg).status == "safe"
    assert primitive_box_clearance(arc, far_box, 5.0, cfg).status == "safe"


def test_a_lower_bound_above_the_sampled_upper_bound_changes_the_verdict(
        monkeypatch):
    """Self-check: an impossible (lb > sampled) certification can never pass."""
    import native3d.validation.validate as V
    arc = big_arc()
    scene = arc_scene(arc, refs=[REF_LINE.to_dict()])
    path = {"segments": [arc.to_dict()]}
    # reproduce the v0.1.1 defect exactly: no domain rule on the chord count and
    # the periodic sagitta formula (which returns 0 for dphi = 4*pi)
    monkeypatch.setattr(V, "chord_count_for_domain",
                        lambda seg, requested: max(1, int(requested)))
    monkeypatch.setattr(V, "_sagitta", lambda seg, n: 0.0)
    v = V.validate_path_dict(path, scene, cfg_dict())
    assert v["centerline_spacing"]["conservative_lower_bound_um"] > \
        v["centerline_spacing"]["min_sampled_um"] + 1e-9, \
        "the monkeypatched defect must really produce the contradiction"
    assert v["certification"]
    ["certified_lower_bounds_are_never_above_the_sampled_upper_bounds"] is False
    assert v["certification"]["self_check_failed"] is True
    assert v["verdict"] != "pass"
