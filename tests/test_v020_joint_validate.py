"""v0.2 independent joint validator: independence, semantics and its own bugs.

The validator has to be a genuine SECOND implementation, so the first test here
proves it imports nothing from the engine at all (AST check).  The rest pin the
parts that were wrong while it was being written and that a reader must be able to
trust:

* the angle of a continuous junction is measured in the atan2 form, so a junction
  that is continuous to 1e-13 deg is not reported as a C1 corner (the acos form
  reports ~1.2e-6 deg there);
* an arc's AABB uses radius * hypot(u_i, v_i) per axis and EVALUATES the
  stationary phase instead of assuming "centre +/- radius", so the wrapped phase
  2*pi (which is the sweep start) cannot invent an extremum outside the arc;
* a net that the scheduler REJECTED must be reported as ``rejected_ok`` when the
  geometry really is bad, and as ``fail`` when it is not - never as a pass;
* the published-set invariant is checked among the COMMITTED nets, and a rejected
  candidate is only evidence;
* malformed artifacts are rejected (never silently skipped).
"""

from __future__ import annotations

import ast
import copy
import math
import os

import pytest

from native3d.validation import joint_validate as jv
from v020_helpers import (as_payload, arc, cfg, line, net_entry, occupancy_entry,
                          quarter_arc, json_roundtrip)

CHIP = jv.__dict__ and None  # placeholder to keep imports obvious


def _chip():
    from native3d.constraints.checks import Box3D
    return Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))


def _pose(x, y, z, t):
    from native3d.geometry.primitives import Pose3D
    return Pose3D((float(x), float(y), float(z)), tuple(float(c) for c in t))


# --------------------------------------------------------------- independence
def test_the_validator_is_standard_library_only():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "native3d", "validation", "joint_validate.py")
    with open(path, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    assert "native3d" not in imported
    assert imported <= {"json", "math", "sys", "fractions", "typing", "__future__"}


# --------------------------------------------------- the two precision regressions
def test_a_continuous_junction_is_not_reported_as_a_c1_corner():
    """The atan2 form keeps a continuous junction continuous.

    Two MATHEMATICALLY IDENTICAL tangents whose dot product rounds to
    0.99999999999999989 make ``acos(dot)`` report 8.5e-7 deg - a fabricated corner
    - while the atan2 form of |u x v| against dot stays at 1e-15 deg.  The
    production run really hit this: a 1.207e-6 deg "corner" against a 1e-6 deg
    tolerance, on a junction the router itself had accepted.
    """
    phi = 0.1234567
    ta = (math.cos(phi), math.sin(phi), 0.0)
    tb = (math.cos(phi + 1e-17), math.sin(phi + 1e-17), 0.0)   # the same doubles
    dot = sum(ta[i] * tb[i] for i in range(3))
    acos_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    atan_deg = jv._angle_deg(ta, tb)
    assert dot < 1.0
    assert acos_deg > 5e-7                     # the artifact is really there
    assert atan_deg < 1e-12                    # the atan2 form is not fooled

    # check_c1 really uses that form: its reported number IS the atan2 angle
    a = line((100, 150, 0), (500, 150, 0))
    b = line((500, 150, 0), (900, 150, 0))
    pa = jv.parse_primitive(a.to_dict())
    pb = jv.parse_primitive(b.to_dict())
    r = jv.check_c1([pa, pb], 1e-6, 1e-7)
    assert r["ok"] is True
    assert r["max_tangent_error_deg"] == pytest.approx(
        jv._angle_deg(pa.tangent_at(pa.length), pb.tangent_at(0.0)), abs=1e-15)


def test_an_arc_aabb_never_invents_an_extremum_at_a_wrapped_phase():
    """Regression: an x-y quarter arc at (510,170,0), R=100 must not reach x=610.

    phi* + pi = 2*pi wraps onto the sweep start, where the coordinate is the
    MINIMUM, not the maximum; assuming 'centre + radius' there made the AABB too
    large and produced a false boundary violation.
    """
    a = jv.Arc(arc((510.0, 170.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0),
                   100.0, 0.5 * math.pi).to_dict())
    lo, hi = a.aabb()
    assert lo == pytest.approx((410.0, 70.0, 0.0))
    assert hi == pytest.approx((510.0, 170.0, 0.0))


def test_the_arc_aabb_is_exact_on_random_multi_turn_arcs():
    import random
    rnd = random.Random(20250)
    for _ in range(120):
        th1 = rnd.uniform(0.0, math.pi)
        th2 = rnd.uniform(0.0, 2.0 * math.pi)
        u = (math.cos(th1), math.sin(th1), 0.0)
        w = (math.sin(th1) * math.cos(th2), -math.cos(th1) * math.cos(th2),
             math.sin(th2))
        v = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
             u[0] * w[1] - u[1] * w[0])
        c = (rnd.uniform(-100, 100), rnd.uniform(-100, 100), rnd.uniform(-100, 100))
        R = rnd.uniform(1.0, 200.0)
        theta = rnd.uniform(-4.0 * math.pi, 4.0 * math.pi)
        if abs(theta) < 1e-3:
            continue
        d = {"kind": "arc", "center": list(c), "u": list(u), "v": list(v),
             "radius_um": R, "theta_rad": theta}
        a = jv.Arc(d)
        lo, hi = a.aabb()
        pts = [a.point_at(a.length * k / 2000) for k in range(2001)]
        for i in range(3):
            assert lo[i] <= min(p[i] for p in pts) + 1e-6
            assert hi[i] >= max(p[i] for p in pts) - 1e-6


def test_the_exact_segment_box_distance_matches_dense_sampling():
    import random
    rnd = random.Random(4)
    for _ in range(200):
        p0 = (rnd.uniform(-50, 50), rnd.uniform(-50, 50), rnd.uniform(-50, 50))
        p1 = (rnd.uniform(-50, 50), rnd.uniform(-50, 50), rnd.uniform(-50, 50))
        if jv._dist(p0, p1) < 1e-9:
            continue
        lo = (rnd.uniform(-40, 0), rnd.uniform(-40, 0), rnd.uniform(-40, 0))
        hi = (lo[0] + rnd.uniform(1, 40), lo[1] + rnd.uniform(1, 40),
              lo[2] + rnd.uniform(1, 40))
        d = jv.segment_box_distance(p0, p1, lo, hi)
        dense = min(jv.point_box_distance(
            tuple(p0[i] + (p1[i] - p0[i]) * k / 20000 for i in range(3)), lo, hi)
            for k in range(20001))
        assert d <= dense + 1e-6
        assert d >= dense - 1e-3


# ---------------------------------------------------------------- verdict logic
def _payload_with(segments_a, segments_b=None, occ=None):
    """A payload whose net is DECLARED rejected, so the verdict can be compared."""
    cfg_ = cfg()
    nets_a = net_entry("netA", _pose(100, 150, 0, (1, 0, 0)),
                       _pose(500, 150, 0, (1, 0, 0)), segments_a)
    nets = [nets_a]
    if segments_b is not None:
        nets.append(net_entry("netB", _pose(100, 150, 0, (1, 0, 0)),
                              _pose(500, 150, 0, (1, 0, 0)), segments_b))
    p = as_payload(_chip(), [], [], cfg_, nets, "handmade")
    p["nets"][0]["committed_segment_ids"] = []
    p["nets"][0]["committed"] = False
    p["nets"][0]["accepted"] = False
    p["nets"][0]["status"] = "self_spacing_violation"
    return p


def test_a_rejection_that_the_geometry_corroborates_is_not_a_run_failure():
    """A self-approaching candidate: declared rejected, independently rejected."""
    segs = [line((100, 150, 0), (500, 150, 0)).to_dict(),
            line((500, 152, 0), (100, 152, 0)).to_dict()]
    p = _payload_with(segs)
    r = jv.validate_joint_artifact(p)
    assert r["nets"][0]["self_spacing"]["status"] == "violation"
    assert r["nets"][0]["verdict"] == "rejected_ok"
    assert r["nets"][0]["agrees_with_declared_status"] is True
    assert r["overall_verdict"] == "pass"


def test_a_rejection_the_geometry_does_not_corroborate_is_a_contradiction():
    """Declared rejected, but the candidate is perfectly legal -> fail."""
    segs = [line((100, 150, 0), (500, 150, 0)).to_dict()]
    p = _payload_with(segs)
    r = jv.validate_joint_artifact(p)
    assert r["nets"][0]["self_spacing"]["status"] == "safe"
    assert r["nets"][0]["verdict"] == "fail"
    assert r["nets"][0]["agrees_with_declared_status"] is False
    assert r["overall_verdict"] == "fail"


def test_an_accepted_net_with_bad_geometry_is_a_failure():
    cfg_ = cfg()
    segs = [line((100, 150, 0), (500, 150, 0)).to_dict()]
    nets = [net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)),
                      [line((100, 150, 0), (500, 150, 0))])]
    p = as_payload(_chip(), [], [], cfg_, nets, "handmade")
    p["nets"][0]["segments"] = segs
    p["occupancy"] = {"revision": 1, "entries": [
        occupancy_entry("netA", 0, line((100, 150, 0), (500, 150, 0)))]}
    r = jv.validate_joint_artifact(p)
    assert r["nets"][0]["verdict"] == "pass"
    assert r["nets"][0]["ports"]["ok"] is True
    # now move the published occupancy so it is NOT the net's own path
    p2 = copy.deepcopy(p)
    p2["occupancy"]["entries"][0]["geometry"] = line((100, 150, 5), (500, 150, 5)).to_dict()
    r2 = jv.validate_joint_artifact(p2)
    assert r2["nets"][0]["occupancy_is_this_net_path"]["ok"] is False
    assert r2["nets"][0]["verdict"] == "fail"


def test_a_conflicting_committed_pair_is_caught_by_both_routes():
    """Two committed nets 2 um apart: the sequential recheck AND the pairwise
    committed-set check must both refuse this, and the run is a failure.

    ``netA`` came first and is clean; ``netB`` was declared committed even though
    the curve published BEFORE it (netA's) is only 2 um away, which is exactly the
    contradiction the sequential recheck exists to find.  The full-matrix pairwise
    check is the second, independent route to the same invariant.
    """
    a = line((100, 150, 0), (500, 150, 0))
    b = line((100, 152, 0), (500, 152, 0))
    nets = [net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)), [a]),
            net_entry("netB", _pose(100, 152, 0, (1, 0, 0)), _pose(500, 152, 0, (1, 0, 0)),
                      [b])]
    for k, n in enumerate(nets):
        n["occupancy_revision_before"] = k
        n["occupancy_revision_after"] = k + 1
    p = as_payload(_chip(), [], [], cfg(), nets, "handmade")
    p["occupancy"] = {"revision": 2, "entries": [
        occupancy_entry("netA", 0, a, 1), occupancy_entry("netB", 0, b, 2)]}
    r = jv.validate_joint_artifact(p)
    # route 1: the sequential recheck of netB against the earlier occupancy
    assert r["nets"][0]["published_occupancy_spacing"]["status"] == "safe"
    assert r["nets"][1]["published_occupancy_spacing"]["status"] == "violation"
    assert r["nets"][1]["verdict"] == "fail"
    assert r["overall_verdict"] == "fail"
    # route 2: the independent pairwise committed-set check
    pair = jv.check_inter_net([jv.parse_primitive(a.to_dict())], "netA",
                              [jv.parse_primitive(b.to_dict())], "netB", 5.0, 1e-9)
    assert pair["status"] == "violation"
    assert pair["violation"]["kind"] == "line-line"
    assert pair["violation"]["distance_um"] == pytest.approx(2.0, abs=1e-9)


def test_a_rejected_candidate_is_evidence_not_a_run_invariant():
    """The rejected candidate conflicts, but it was never published: still pass."""
    a = line((100, 150, 0), (500, 150, 0))
    b = line((100, 152, 0), (500, 152, 0))
    nets = [net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)), [a]),
            net_entry("netB", _pose(100, 152, 0, (1, 0, 0)), _pose(500, 152, 0, (1, 0, 0)),
                      [b], status="self_spacing_violation", accepted=False,
                      committed=False)]
    # netA was published first (revision 1), so netB's window really contains it
    nets[1]["occupancy_revision_before"] = 1
    nets[1]["occupancy_revision_after"] = 1
    p = as_payload(_chip(), [], [], cfg(), nets, "handmade")
    p["occupancy"] = {"revision": 1, "entries": [occupancy_entry("netA", 0, a, 1)]}
    r = jv.validate_joint_artifact(p)
    assert r["counts"]["n_committed_pairs_checked"] == 0
    assert r["overall_verdict"] == "pass"
    ev = r["rejected_candidate_spacing_evidence"]
    assert len(ev) == 1 and ev[0]["violation"]["distance_um"] == pytest.approx(2.0,
                                                                              abs=1e-9)


# ------------------------------------------------------- the self-spacing check
def test_the_validator_detects_multi_turn_re_entry_and_the_split_form():
    cfg_ = cfg()
    full = arc((0, 0, 0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, 2.0 * math.pi)
    one = jv.self_spacing_global([jv.parse_primitive(full.to_dict())], 5.0, 10.0, 1e-9)
    assert one["status"] == "violation"
    assert one["witness"]["in_check_domain"] is True
    pieces = []
    for k in range(4):
        ang = k * 0.5 * math.pi
        pieces.append(arc((0, 0, 0), (math.cos(ang), math.sin(ang), 0.0),
                          (-math.sin(ang), math.cos(ang), 0.0), 50.0, 0.5 * math.pi))
    split = jv.self_spacing_global([jv.parse_primitive(p.to_dict()) for p in pieces],
                                   5.0, 10.0, 1e-9)
    assert split["status"] == "violation"
    assert split["witness"]["arclength_separation_um"] == pytest.approx(
        2.0 * math.pi * 50.0, rel=1e-9)


def test_the_validator_and_the_production_check_agree_on_a_straight_line():
    from native3d.multinet.self_spacing import path_self_spacing_global
    from native3d.multinet.models import SelfSpacingPolicy
    cfg_ = cfg()
    segs = [line((0, 0, 0), (400, 0, 0))]
    prod = path_self_spacing_global(segs, cfg_,
                                    SelfSpacingPolicy(2.0 * cfg_.dmin_centerline_um))
    val = jv.self_spacing_global([jv.parse_primitive(s.to_dict()) for s in segs],
                                 5.0, 10.0, 1e-9)
    assert prod["status"] == val["status"] == "safe"
    assert prod["certified_lower_bound_um"] == pytest.approx(
        val["certified_lower_bound_um"], abs=1e-6)
    # up to the single documented FP margin the production module subtracts
    assert prod["certified_lower_bound_um"] == pytest.approx(10.0, abs=2e-9)


# ----------------------------------------------------------- contract rejection
def test_a_malformed_artifact_is_rejected_not_skipped():
    cfg_ = cfg()
    good = net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)),
                     [line((100, 150, 0), (500, 150, 0))])
    base = as_payload(_chip(), [], [], cfg_, [good], "handmade")

    p = copy.deepcopy(base)
    p["units"] = "mm"
    with pytest.raises(jv.ContractError):
        jv.validate_joint_artifact(p)

    p = copy.deepcopy(base)
    p["nets"][0]["segments"][0]["kind"] = "spline"
    with pytest.raises(jv.ContractError):
        jv.validate_joint_artifact(p)

    p = copy.deepcopy(base)
    p["nets"][0]["input"]["source_pose"]["tangent"] = [0.0, 0.0, 0.0]
    with pytest.raises(jv.ContractError):
        jv.validate_joint_artifact(p)

    p = copy.deepcopy(base)
    p["nets"][0]["segments"][0]["p1"] = [100.0, 150.0, 0.0]
    with pytest.raises(jv.ContractError):
        jv.validate_joint_artifact(p)

    p = copy.deepcopy(base)
    bad_arc = {"kind": "arc", "center": [0, 0, 0], "u": [1, 0, 0],
               "v": [1, 1, 0], "radius_um": 50.0, "theta_rad": 1.0}
    p["nets"][0]["segments"] = [bad_arc]
    with pytest.raises(jv.ContractError):
        jv.validate_joint_artifact(p)


def test_an_unknown_occupancy_origin_is_refused():
    cfg_ = cfg()
    good = net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)),
                     [line((100, 150, 0), (500, 150, 0))])
    p = as_payload(_chip(), [], [], cfg_, [good], "handmade")
    p["occupancy"] = {"revision": 1, "entries": [
        {"origin": "not_a_thing", "geometry": {}}]}
    r = jv.validate_joint_artifact(p)
    assert r["occupancy_bookkeeping"]["ok"] is False
    assert r["overall_verdict"] == "fail"


def test_the_validator_reads_only_serialised_geometry():
    """A payload round-tripped through JSON gives exactly the same verdict."""
    cfg_ = cfg()
    a = line((100, 150, 0), (500, 150, 0))
    nets = [net_entry("netA", _pose(100, 150, 0, (1, 0, 0)), _pose(500, 150, 0, (1, 0, 0)),
                      [a])]
    p = as_payload(_chip(), [], [], cfg_, nets, "handmade")
    p["occupancy"] = {"revision": 1, "entries": [occupancy_entry("netA", 0, a)]}
    r1 = jv.validate_joint_artifact(p)
    r2 = jv.validate_joint_artifact(json_roundtrip(p))
    assert r1["overall_verdict"] == r2["overall_verdict"] == "pass"
    assert r1["counts"] == r2["counts"]
