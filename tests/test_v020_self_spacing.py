"""v0.2 self-spacing: the GLOBAL-ARCLENGTH check and its minimum regressions.

Prompt section 5 lists the minimum self-spacing regressions and every one of them
has a test here:

* the same C1 straight gives the SAME conclusion split and unsplit;
* a normal spatial bend passes;
* a single multi-turn segment AND the same curve split into segments both detect
  the non-local return;
* a real self-approach fails and the witness lies INSIDE the check domain;
* the preserved cross-wall path stays legal.

The tests also pin the semantics that the old
``constraints.checks.path_self_spacing(exempt_adjacent=True)`` could not express:
the exemption is a LENGTH band over the assembled arclength, not a pair of segment
indices, and the segment is compared with itself.
"""

from __future__ import annotations

import math

import pytest

from native3d.constraints.checks import path_self_spacing
from native3d.geometry.path3d import Path3D
from native3d.multinet.self_spacing import (SELF_SPACING_FP_MARGIN_UM,
                                            path_self_spacing_global)
from v020_helpers import (arc, cfg, full_circle, line, policy, quarter_arc)


def run(segments, cfg_=None, pol=None):
    cfg_ = cfg_ or cfg()
    return path_self_spacing_global(segments, cfg_, pol or policy(cfg_))


# ------------------------------------------------------------------ semantics
def test_ell_local_is_a_length_not_a_segment_index(v020_cfg, v020_policy):
    r = run((line((0, 0, 0), (400, 0, 0)),), v020_cfg, v020_policy)
    assert r["ell_local_um"] == pytest.approx(2.0 * v020_cfg.dmin_centerline_um)
    assert r["ell_local_um"] == pytest.approx(10.0)
    assert r["exemption_is_arclength_based_not_segment_based"] is True
    assert r["check_domain"].startswith("|s - t| >= ell_local_um")
    assert r["fp_margin_um"] == SELF_SPACING_FP_MARGIN_UM


def test_a_path_shorter_than_ell_local_is_vacuously_safe(v020_cfg, v020_policy):
    r = run((line((0, 0, 0), (5, 0, 0)),), v020_cfg, v020_policy)
    assert r["status"] == "safe"
    assert r["certified_lower_bound_um"] is None


# --------------------------------------------- 1. split invariance (C1 straight)
def test_splitting_a_c1_straight_does_not_change_the_conclusion(v020_cfg,
                                                                v020_policy):
    one = run((line((0, 0, 0), (400, 0, 0)),), v020_cfg, v020_policy)
    split = run((line((0, 0, 0), (100, 0, 0)),
                 line((100, 0, 0), (250, 0, 0)),
                 line((250, 0, 0), (400, 0, 0))), v020_cfg, v020_policy)
    assert one["status"] == split["status"] == "safe"
    assert one["certified_lower_bound_um"] == pytest.approx(
        split["certified_lower_bound_um"], abs=1e-9)
    # and the true value really is ell_local on a straight line, up to the single
    # documented FP margin this module subtracts from every certified lower bound
    assert one["certified_lower_bound_um"] == pytest.approx(
        10.0, abs=2.0 * SELF_SPACING_FP_MARGIN_UM)
    assert 10.0 - one["certified_lower_bound_um"] <= 1.5 * SELF_SPACING_FP_MARGIN_UM


# ------------------------------------------------------- 2. a normal bend passes
def test_a_normal_spatial_bend_passes(v020_cfg, v020_policy):
    xz = quarter_arc((200.0, 50.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
    path = (line((0, 0, 0), (200, 0, 0)), xz,
            line((250, 50, 0), (250, 200, 0)))
    r = run(path, v020_cfg, v020_policy)
    assert r["status"] == "safe"
    assert r["certified_lower_bound_um"] >= v020_cfg.dmin_centerline_um
    # a tilted plane must behave the same way
    tilted = quarter_arc((0.0, 0.0, 50.0), (1.0, 0.0, 0.0),
                         (0.0, 0.7071067811865476, -0.7071067811865476))
    assert run((tilted,), v020_cfg, v020_policy)["status"] == "safe"


# ------------------------------------------- 3. multi-turn / non-local re-entry
def test_a_single_full_turn_arc_detects_its_own_re_entry(v020_cfg, v020_policy):
    r = run((full_circle((0, 0, 0), 50.0),), v020_cfg, v020_policy)
    assert r["status"] == "violation"
    w = r["witness"]
    assert w is not None and w["in_check_domain"] is True
    assert w["arclength_separation_um"] == pytest.approx(2.0 * math.pi * 50.0,
                                                         rel=1e-9)
    assert w["distance_sampled_um"] < v020_cfg.dmin_centerline_um


def test_the_same_circle_split_into_arcs_detects_the_same_re_entry(v020_cfg,
                                                                   v020_policy):
    pieces = []
    for k in range(4):
        ang = k * 0.5 * math.pi
        pieces.append(arc((0, 0, 0), (math.cos(ang), math.sin(ang), 0.0),
                          (-math.sin(ang), math.cos(ang), 0.0), 50.0,
                          0.5 * math.pi))
    r = run(tuple(pieces), v020_cfg, v020_policy)
    assert r["status"] == "violation"
    assert r["witness"]["arclength_separation_um"] == pytest.approx(
        2.0 * math.pi * 50.0, rel=1e-9)
    assert r["witness"]["in_check_domain"] is True


def test_a_two_turn_arc_is_detected(v020_cfg, v020_policy):
    two = arc((0, 0, 0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, 4.0 * math.pi)
    assert run((two,), v020_cfg, v020_policy)["status"] == "violation"


def test_the_old_segment_index_exemption_could_not_see_this(v020_cfg):
    """The regression the v0.2 check exists for.

    The old function exempts every pair of ADJACENT segments, so the same circle
    cut into four arcs compares only the (0,2), (0,3), (1,3) pairs - and it reports
    the pair (1, 2) as exempt even though their far ends are 50 um apart in space
    after a full turn.  Whatever it returns, the v0.2 check must decide the
    GEOMETRY, and the two must not be confused.
    """
    pieces = []
    for k in range(4):
        ang = k * 0.5 * math.pi
        pieces.append(arc((0, 0, 0), (math.cos(ang), math.sin(ang), 0.0),
                          (-math.sin(ang), math.cos(ang), 0.0), 50.0,
                          0.5 * math.pi))
    old = path_self_spacing(Path3D(pieces), v020_cfg, exempt_adjacent=True)
    new = run(tuple(pieces), v020_cfg, policy(v020_cfg))
    assert new["status"] == "violation"
    # the old API has no notion of arclength at all
    assert "ell_local_um" not in old.detail
    assert "global_arclength_s_um" not in old.detail


# ---------------------------------------------------- 4. a real self-approach
def test_two_parallel_legs_of_the_same_path_fail_with_an_in_domain_witness(
        v020_cfg, v020_policy):
    r = run((line((0, 0, 0), (100, 0, 0)),
             line((0, 2, 0), (100, 2, 0))), v020_cfg, v020_policy)
    assert r["status"] == "violation"
    w = r["witness"]
    assert w["in_check_domain"] is True
    assert w["arclength_separation_um"] >= r["ell_local_um"]
    assert w["distance_sampled_um"] == pytest.approx(2.0, abs=1e-9)
    assert w["distance_sampled_um"] < v020_cfg.dmin_centerline_um


def test_a_grazing_u_turn_right_at_dmin_is_reported_safe(v020_cfg, v020_policy):
    """legs 6 um apart: above dmin = 5, so the path is legal."""
    r = run((line((0, 0, 0), (100, 0, 0)),
             line((100, 6, 0), (0, 6, 0))), v020_cfg, v020_policy)
    assert r["status"] == "safe"
    assert r["certified_lower_bound_um"] >= 5.0


def test_a_witness_inside_the_exempt_band_is_never_used(v020_cfg, v020_policy):
    """A path that nearly closes a LOOP must not be condemned by its own junction.

    The two legs here are 2 um apart in space but the arclength distance between
    the close points is 100 um, so the check really is inside its domain - and the
    witness records that fact.  The other direction is checked by the 40 um case
    below, whose nearest points are inside the band and which must stay silent.
    """
    ok = run((line((0, 0, 0), (100, 0, 0)),
              line((100, 40, 0), (0, 40, 0))), v020_cfg, v020_policy)
    assert ok["status"] == "safe"
    # the same geometry pulled to 2 um apart must fail, again with an in-domain gap
    bad = run((line((0, 0, 0), (100, 0, 0)),
               line((100, 2, 0), (0, 2, 0))), v020_cfg, v020_policy)
    assert bad["status"] == "violation"
    assert bad["witness"]["in_check_domain"] is True


# ------------------------------------------------------- 5. cross-wall regression
def test_the_preserved_cross_wall_path_stays_legal(v020_cfg, v020_policy):
    """The exact 594.159265 um cross-wall route of the v0.1.x core case.

    It rises from z = 0 to z = 100 over a wall and must not approach itself: no
    two points of it that are more than ell_local apart along the path may be
    closer than dmin.
    """
    from native3d.cases import core_case
    from native3d.search.finite_graph import route_finite_native3d

    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success"
    assert res.length_um == pytest.approx(594.1592653589794, rel=1e-12)
    r = path_self_spacing_global(tuple(res.path.segments), sc.config,
                                policy(sc.config))
    assert r["status"] == "safe"
    assert r["witness"] is None
    assert r["certified_lower_bound_um"] >= sc.config.dmin_centerline_um


def test_a_multi_turn_arc_is_never_faked_into_safety_by_the_sagitta_domain(
        v020_cfg, v020_policy):
    """cos() periodicity must not turn a 4*pi arc into a zero-sagitta chord."""
    four = arc((0, 0, 0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 30.0, 8.0 * math.pi)
    r = run((four,), v020_cfg, v020_policy)
    assert r["status"] == "violation"
    assert r["witness"]["distance_sampled_um"] < v020_cfg.dmin_centerline_um
