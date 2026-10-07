"""v0.1.1 acceptance on the FINITE canonical reference graph.

The production exact-pose search cannot be exhausted on the wall scenes (a
measured property of that graph, see the v0.1.1 report) so the accept/reject
evidence for those scenes is produced on the finite canonical reference graph of
search/reference_graph.py: canonical poses as nodes, exact Dubins connectors as
edges, the same constraint stack, the same astar_search.
"""

import math

import pytest

from native3d.cases import base_config, core_case, sealed_target_case
from native3d.constraints.checks import Box3D, validate_segments
from native3d.geometry.vec import norm, sub
from native3d.validation.validate import point_box_distance as dist_to_box
from native3d.search.reference_graph import (CANONICAL_DIRECTIONS, best_connector_edge,
                                            canonical_poses, reference_search)

SPACING = 60.0


def _pose_key(p):
    return (round(p.position[0], 9), round(p.position[1], 9), round(p.position[2], 9),
            round(p.tangent[0], 9), round(p.tangent[1], 9), round(p.tangent[2], 9))


# ------------------------------------------------------------- the node set
def test_canonical_pose_set_is_finite_fixed_and_lattice_anchored():
    sc = core_case()
    a = canonical_poses(sc, SPACING)
    b = canonical_poses(sc, SPACING)
    assert len(a) == len(b) > 500
    assert {_pose_key(p) for p in a} == {_pose_key(p) for p in b}
    assert len({_pose_key(p) for p in a}) == len(a), "no duplicate canonical pose"
    assert len(CANONICAL_DIRECTIONS) == 26
    src = sc.source.position
    inner = sc.chip.shrink(sc.config.effective_boundary_margin_um)
    for p in a[::37]:
        assert inner.contains_point(p.position, 1e-9)
        for i in range(3):
            k = (p.position[i] - src[i]) / SPACING
            assert abs(k - round(k)) < 1e-9, "positions must sit on the anchored lattice"


def test_reference_graph_finds_a_native3d_route_over_the_wall():
    sc = core_case()
    res = reference_search(sc, "native3d")
    assert res.status == "success", (res.status, res.reason)
    assert res.state_identity == "canonical_reference_graph"
    assert res.approximate_state_identity is False
    assert res.path is not None
    assert res.cost == pytest.approx(res.length_um + res.congestion_cost, rel=1e-12)
    rep = res.path.continuity_report()
    assert rep["c1_continuous"] is True, rep["corners"]
    assert res.path.min_radius_um >= sc.config.rmin_um - sc.config.radius_tol_um
    assert res.goal_pos_error_um <= sc.config.eps_pos_um
    assert res.goal_tangent_error_deg <= sc.config.eps_ang_deg
    # it is a genuinely 3D route: it climbs over the wall, whose keep-out top is
    # at z = 40 + 5 = 45 while both ports sit at z = 0
    zs = [s["z_um"] for s in res.trace]
    assert max(zs) >= 45.0
    assert min(zs) <= 1e-9
    L = res.path.total_length
    n = 4000
    for i in range(n + 1):
        p = res.path.point_at(L * i / n)
        if 285.0 - 1e-6 <= p[0] <= 315.0 + 1e-6:
            assert p[2] >= 45.0 - 1e-6, "the route must clear the wall keep-out"
    out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                            sc.reference_centerlines, sc.config)
    assert out["ok"] is True, out.get("failed")


def test_reference_graph_route_is_optimal_for_its_finite_graph():
    """A* on the finite graph must beat (or equal) a brute force enumeration."""
    sc = core_case()
    cfg = sc.config
    res = reference_search(sc, "native3d")
    assert res.status == "success"
    best = None
    for q in canonical_poses(sc, SPACING):
        e1 = best_connector_edge(sc, sc.source, q, cfg=cfg)
        if e1 is None:
            continue
        e2 = best_connector_edge(sc, q, sc.target, cfg=cfg)
        if e2 is None:
            continue
        total = e1.cost + e2.cost
        if best is None or total < best - 1e-9:
            best = total
    direct = best_connector_edge(sc, sc.source, sc.target, cfg=cfg)
    if direct is not None:
        best = direct.cost if best is None else min(best, direct.cost)
    assert best is not None, "the reference graph must contain at least one route"
    assert res.cost == pytest.approx(best, abs=1e-9)


def test_reference_graph_really_exhausts_in_2d_mode_over_the_wall():
    res = reference_search(core_case(), "2d")
    assert res.status == "search_exhausted", (res.status, res.reason)
    assert res.budget_exhausted is False
    assert res.path is None
    assert "FINITE canonical reference graph is exhausted" in res.reason


def test_production_search_reports_a_budget_stop_never_unreachability():
    """The production exact-pose run on the same scene must not claim unreachable."""
    from native3d.search.router import route
    res = route(core_case(), mode="native3d", max_expansions=1500, max_runtime_s=8.0)
    assert res.status == "budget_exhausted"
    assert res.budget_exhausted is True
    assert "NOT a proof" in res.reason
    assert res.state_identity == "pose_float"


# ---------------------------------------------------- the geometric argument
def test_2d_separation_is_a_geometric_property_not_a_search_result():
    """Independent of any search: the keep-out spans the whole port-plane slice.

    For every point p of the z = 0 slice that respects the boundary margin,
        dist(p, wall) >= clearance  =>  p.x < wall.lo_x - clearance
                                        or p.x > wall.hi_x + clearance,
    because the wall's y range already covers the whole chip y range and its z
    range contains the port plane, so inside the x slab the distance is 0.  The
    free slice is therefore the union of two disjoint open sets, the source lies
    in one and the target in the other, and a continuous legal curve cannot
    connect them.
    """
    sc = core_case()
    cfg = sc.config
    wall, chip = sc.obstacles[0], sc.chip
    req = cfg.effective_obstacle_clearance_um
    m = cfg.effective_boundary_margin_um
    z0 = sc.source.position[2]
    assert wall.lo[1] - req <= chip.lo[1] + m and wall.hi[1] + req >= chip.hi[1] - m
    assert wall.lo[2] <= z0 <= wall.hi[2]
    assert sc.source.position[0] < wall.lo[0] - req
    assert sc.target.position[0] > wall.hi[0] + req
    # exhaustive finite check of the claim on a 1 um grid of the slice: a curve
    # from the source to the target must cross the plane x = wall.lo_x, and every
    # grid point of that plane inside the slice is INSIDE the wall (distance 0),
    # so the crossing point violates the keep-out no matter what the curve does.
    lo_x = int(math.floor(chip.lo[0] + m))
    hi_x = int(math.ceil(chip.hi[0] - m))
    lo_y = int(math.floor(chip.lo[1] + m))
    hi_y = int(math.ceil(chip.hi[1] - m))
    free_left = free_right = on_wall = band = 0
    for x in range(lo_x, hi_x + 1):
        for y in range(lo_y, hi_y + 1):
            p = (float(x), float(y), z0)
            d = dist_to_box(p, wall.lo, wall.hi)
            if wall.lo[0] <= x <= wall.hi[0]:
                assert d == 0.0, "every slice point on the wall is a violation"
                on_wall += 1
            elif wall.lo[0] - req <= x <= wall.hi[0] + req:
                assert 0.0 <= d <= req, "the keep-out band is only 'req' wide"
                band += 1
            elif x < wall.lo[0]:
                assert d > req, "left of the band the wall is farther than req"
                free_left += 1
            else:
                assert d > req
                free_right += 1
    assert on_wall > 5000 and band > 2000 and free_left > 1000 and free_right > 1000
    # 1. the wall's y and z ranges swallow the whole slice, so crossing its
    #    x faces is a violation; 2. the ports straddle it
    assert wall.lo[1] <= chip.lo[1] + m and wall.hi[1] >= chip.hi[1] - m
    assert wall.lo[2] <= z0 <= wall.hi[2]
    assert sc.source.position[0] < wall.lo[0]
    assert sc.target.position[0] > wall.hi[0]


def test_sealed_target_is_unreachable_in_the_reference_graph():
    sc = sealed_target_case()
    res = reference_search(sc, "2d")
    assert res.status == "search_exhausted"
    assert res.budget_exhausted is False