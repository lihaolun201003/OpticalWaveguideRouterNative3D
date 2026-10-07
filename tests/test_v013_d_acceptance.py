"""v0.1.3 PART D: re-acceptance on the frozen evidence.

This file re-runs the acceptance that v0.1.2 already established and adds the
acceptance that v0.1.2 was missing.  It deliberately does NOT weaken anything:
the numbers below are the ones the frozen artifacts/v012 evidence recorded, and
the original cross-wall result must survive the contract repair unchanged.

1. THE ORIGINAL CROSS-WALL CASE IS UNCHANGED
   native3d: success, cost 594.1592653589794 um, 75 expansions
   2d      : a REAL search_exhausted (81 expansions, not a budget stop)
   no reference-graph fallback, no waypoint, the z-rise comes from the search.

2. THE v0.1.2 CONTRACT COUNTEREXAMPLES ARE STILL CORRECT
   the source-pose check, the reference-arc spacing, the 8*pi multi-turn arc, the
   near-parallel certified segment distance and the state-identity split.
   These are re-run here from the v0.1.3 code, not read back from v0.1.2 JSON.

3. NEW: A FULL LEGAL 3D PATH ABOVE A REFERENCE ARC MUST BE ACCEPTED
   v0.1.2 had only point-distance unit tests for this; the full acceptance is the
   legal path (-10,0,50)->(10,0,50) flown over the XY reference circle of radius
   50, whose true minimum spacing is sqrt(40^2+50^2) = 64.031242374328 um.  The
   production stack already accepted it; the independent validator wrongly
   rejected it (sampled minimum 0.0) and that is what v0.1.3 fixes.  A real
   crossing must STILL be rejected by both.

4. TIMING AND MEMORY ACCOUNTING ARE DISCLOSED, NOT LUMPED TOGETHER
   graph build / search / assembled-path check / independent validation are
   reported separately, and the tracemalloc peak is labelled for what it is
   (tracked Python allocations, not process RSS).
"""

from __future__ import annotations

import json
import math
import os
import time
import tracemalloc

import pytest

from native3d.cases import base_config, core_case
from native3d.config import RouterConfig
from native3d.constraints.checks import (Box3D, primitive_spacing,
                                         radius_check,
                                         segment_segment_distance_certified)
from native3d.geometry import CircularArc3D, LineSegment3D, Pose3D
from native3d.geometry.vec import dot, norm, sub
from native3d.io.caseio import Scene
from native3d.search.finite_graph import route_finite_native3d
from native3d.validation.validate import validate_path_dict

V012_ARTIFACTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "artifacts", "v012", "cases")
CHIP_BIG = Box3D((-2000.0, -2000.0, -2000.0), (2000.0, 2000.0, 2000.0))


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


# ===========================================================================
# 1. the original cross-wall case
# ===========================================================================
@pytest.fixture(scope="module")
def cross_wall():
    tracemalloc.start()
    t0 = time.perf_counter()
    res = route_finite_native3d(core_case(), mode="native3d")
    wall_s = time.perf_counter() - t0
    _cur, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return res, wall_s, peak


def test_cross_wall_native3d_result_is_byte_for_byte_what_v012_established(cross_wall):
    """Pinned against the FROZEN v0.1.2 artifact, not against a remembered number.

    artifacts/v012/cases/core_case_finite_native3d_native3d.json records
    cost = length = 594.1592653589794 um, 75 expansions, 8 trace steps and 7 path
    segments (its trace steps are <source>, straight_2, turn_+x->+z, turn_+z->+x,
    straight_3, turn_+x->-z, turn_-z->+x, port_target_straight).  A draft of this
    test asserted 9 trace steps, which contradicts that artifact.
    """
    res, _wall_s, _peak = cross_wall
    assert res.status == "success", (res.status, res.reason)
    assert res.cost == pytest.approx(594.1592653589794, abs=1e-9)
    assert res.length_um == pytest.approx(594.1592653589794, abs=1e-9)
    assert res.congestion_cost == pytest.approx(0.0)
    assert res.expansions == 75, res.expansions
    assert len(res.trace) == 8, len(res.trace)
    assert len(res.path.segments) == 7, len(res.path.segments)
    assert res.n_segments == 7
    assert [s["action"] for s in res.trace] == [
        "<source>", "straight_2", "turn_+1+0+0_to_+0+0+1_R50",
        "turn_+0+0+1_to_+1+0+0_R50", "straight_3",
        "turn_+1+0+0_to_+0+0-1_R50", "turn_+0+0-1_to_+1+0+0_R50",
        "port_target_straight"]
    assert res.state_identity == "finite_lattice_node_id"
    assert res.approximate_state_identity is False
    assert res.reference_graph is None
    # the route really leaves the port plane, climbs to z >= the wall keep-out top
    # and comes back.  z reaches 100 um, not 50: the search uses the free height up
    # to the chip ceiling (wall keep-out top is 45 um).  The frozen artifact's own
    # trace_z_summary records z_min_um = 0.0 and z_max_um = 100.0.
    zs = [s["z_um"] for s in res.trace]
    assert max(zs) == pytest.approx(100.0)
    assert min(zs) == pytest.approx(0.0)
    assert "z_um" in res.trace[0]
    # and it matches the frozen artifact field for field
    frozen = json.load(open(os.path.join(V012_ARTIFACTS,
                                         "core_case_finite_native3d_native3d.json"),
                            encoding="utf-8"))
    assert [s["action"] for s in frozen["search"]["trace"]] == \
        [s["action"] for s in res.trace]
    assert frozen["search"]["expansions"] == res.expansions
    assert frozen["search"]["cost_um_equivalent"] == pytest.approx(res.cost, abs=1e-9)
    assert len(frozen["search"]["path"]["segments"]) == len(res.path.segments)
    assert frozen["trace_z_summary"]["z_max_um"] == pytest.approx(max(zs))
    assert frozen["trace_z_summary"]["z_min_um"] == pytest.approx(min(zs))
    assert frozen["trace_z_summary"]["z_varies"] is True


def test_cross_wall_2d_is_a_real_exhaustion(cross_wall):
    res2d = route_finite_native3d(core_case(), mode="2d")
    assert res2d.status == "search_exhausted"
    assert res2d.budget_exhausted is False
    assert res2d.expansions == 81, res2d.expansions
    assert res2d.path is None
    assert res2d.expansions < res2d.max_expansions


def test_cross_wall_cost_equals_an_independent_dijkstra_over_the_enumerated_graph():
    """An independent shortest-path check rooted at the node the ROUTER starts from.

    The enumerated graph's edges originate at the virtual port nodes ("source" /
    "target") unless those ports ARE canonical lattice nodes.  Rooting a Dijkstra at
    `graph.source_identity_node` unconditionally is a trap: for the core case that is
    (0,0,0,+x) while the enumerated edges start at the virtual ("source",) node, so
    the search reaches nothing and returns inf.  The correct root is exactly the one
    route_finite_native3d uses.
    """
    import heapq

    from native3d.search.finite_graph import (SOURCE_NODE, TARGET_NODE,
                                              enumerate_graph_edges)

    scene = core_case()
    res = route_finite_native3d(core_case(), mode="native3d")
    graph, edges = enumerate_graph_edges(scene, mode="native3d")
    start = (graph.source_identity_node if graph.source_identity_node is not None
             else SOURCE_NODE)
    goal = (graph.target_identity_node if graph.target_identity_node is not None
            else TARGET_NODE)
    assert start == SOURCE_NODE or start in set(graph.all_canonical_nodes())
    assert any(e.from_node == start for e in edges), \
        "the enumerated graph must contain the start node the router used"
    assert any(e.to_node == goal for e in edges), \
        "the enumerated graph must contain the goal node the router used"

    adj = {}
    for e in edges:
        adj.setdefault(e.from_node, []).append((e.to_node, e.cost))
    dist = {start: 0.0}
    heap = [(0.0, 0, start)]
    counter = 0
    done = set()
    best = math.inf
    while heap:
        d, _c, u = heapq.heappop(heap)
        if u in done:
            continue
        done.add(u)
        if u == goal:
            best = d
            break
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, math.inf) - 1e-15:
                dist[v] = nd
                counter += 1
                heapq.heappush(heap, (nd, counter, v))
    assert best == pytest.approx(594.1592653589794, abs=1e-9), best
    assert best == pytest.approx(res.cost)
    assert len(done) == 678, len(done)
    assert len(edges) == 7867, len(edges)


def test_cross_wall_stays_inside_the_frozen_end_to_end_budget(cross_wall):
    res, wall_s, _peak = cross_wall
    cfg = core_case().config
    assert cfg.finite_max_runtime_s == 60.0
    assert cfg.finite_max_expansions == 200_000
    assert res.timings["end_to_end_s"] <= cfg.finite_max_runtime_s
    assert res.timings["end_to_end_s"] <= wall_s + 1e-6
    assert wall_s < cfg.finite_max_runtime_s


def test_cross_wall_timings_are_broken_down_not_lumped(cross_wall):
    res, _wall_s, _peak = cross_wall
    t = res.timings
    for key in ("graph_build_s", "search_s", "end_to_end_s"):
        assert key in t, key
        assert t[key] >= 0.0
    assert t["end_to_end_s"] >= t["graph_build_s"] + t["search_s"] - 1e-9
    fg = res.finite_graph
    # v0.1.3 splits the per-edge continuous checks by phase instead of claiming
    # that the construction-phase checks are part of search_s
    assert "edge_check_s_graph_build" in fg
    assert "edge_check_s_search" in fg
    assert fg["edge_check_s_search"] <= t["search_s"] + 1e-9
    assert fg["edge_check_s_graph_build"] <= t["graph_build_s"] + 1e-9
    assert fg["edge_check_s"] == pytest.approx(
        fg["edge_check_s_graph_build"] + fg["edge_check_s_search"], abs=1e-9)
    assert "assembled_path_check_s" in t
    assert t["assembled_path_check_s"] >= 0.0
    assert set(t) >= {"graph_build_s", "search_s", "assembled_path_check_s",
                      "end_to_end_s"}


def test_cross_wall_memory_number_is_labelled_for_what_it_is(cross_wall):
    _res, _wall_s, peak = cross_wall
    assert peak > 0
    # the disclosure lives in the evidence producer; assert it is present and honest
    probe = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "artifacts", "v013", "probe", "v013_acceptance_probe.py")
    if os.path.exists(probe):
        text = open(probe, encoding="utf-8").read()
        assert "tracemalloc" in text
        assert "tracked" in text.lower()
        assert "not process" in text.lower() or "not the process" in text.lower()


# ===========================================================================
# 2. the v0.1.2 counterexamples are still correct
# ===========================================================================
def test_v012_partA2_source_pose_counterexample_is_still_a_violation():
    path = {"segments": [{"kind": "line", "p0": [10.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [0.0, 1.0, 0.0]},
             "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}}
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["verdict"] == "fail"
    assert v["violation_counts"]["source_position_out_of_tolerance"] == 1
    assert v["violation_counts"]["source_tangent_out_of_tolerance"] == 1


def test_v012_partA3_reference_arc_counterexample_is_still_a_violation():
    arc = CircularArc3D((50.0, 50.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0),
                        50.0, math.pi)
    path = {"segments": [{"kind": "line", "p0": [10.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [arc.to_dict()],
             "source_pose": {"position": [10.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}}
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["verdict"] == "fail"
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(0.0, abs=1e-9)


def test_v012_partA4_eight_pi_sweep_counterexample_is_still_a_violation():
    cfg = RouterConfig()
    big = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                        50.0, 8.0 * math.pi)
    ref = LineSegment3D((-51.0, -1.0, 0.0), (-49.0, 1.0, 0.0))
    sp = primitive_spacing(big, [ref], cfg.dmin_centerline_um, cfg)
    assert sp.status == "violation"
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [ref.to_dict()],
             "source_pose": {"position": list(big.start),
                             "tangent": list(big.tangent_at_phi(0.0))},
             "target_pose": {"position": list(big.end),
                             "tangent": list(big.tangent_at_phi(big.theta))}}
    v = validate_path_dict({"segments": [big.to_dict()]}, scene, cfg_dict())
    assert v["verdict"] == "fail"
    assert v["certification"]["self_check_failed"] is False


def test_v012_partA5_near_parallel_counterexample_is_still_certified():
    """The v0.1.2 A5 counterexample: two nearly parallel segments that CROSS.

    A = (0,0,0)->(1000,0,0) and B = (0,-5e-6,0)->(1000,+5e-6,0) is not a
    "5e-6 apart" pair: B's y goes from -5e-6 to +5e-6, so the two segments meet at
    (500,0,0) and the TRUE distance is exactly 0.  That is precisely the case the
    classic a*e - b*b denominator got wrong (it produced 9.04e-7); the certified
    decision now returns 0 through the exact-rational fallback.  The frozen v0.1.2
    regression tests/test_v012_a5_near_parallel.py asserts the same thing.
    """
    A0, A1 = (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)
    B0, B1 = (0.0, -5e-6, 0.0), (1000.0, 5e-6, 0.0)
    st, lb, ub, method = segment_segment_distance_certified(A0, A1, B0, B1, 5e-7, 0.0)
    assert st == "violation"
    assert method == "exact_rational"
    assert lb == pytest.approx(0.0, abs=1e-15)
    assert ub <= 1e-15
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [],
             "reference_centerlines": [{"kind": "line", "p0": list(B0), "p1": list(B1)}],
             "source_pose": {"position": list(A0), "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": list(A1), "tangent": [1.0, 0.0, 0.0]}}
    v = validate_path_dict({"segments": [{"kind": "line", "p0": list(A0),
                                          "p1": list(A1)}]},
                           scene, cfg_dict(dmin_centerline_um=5e-7))
    assert v["verdict"] == "fail"
    assert v["centerline_spacing"]["analytic_min_um"] == pytest.approx(0.0, abs=1e-15)
    assert v["centerline_spacing"]["status"] == "violation"


def test_a_genuinely_parallel_pair_5e_6_apart_is_still_certified_safe():
    """The complement of A5: same ordering, but no crossing, so no false violation.

    A = (0,0,0)->(1000,0,0) and B = (0,-5e-6,0)->(1000,-5e-6,0) are exactly
    parallel and exactly 5e-6 um apart.  Asked for 5e-7 um the answer must be SAFE
    (5e-6 > 5e-7) and the certified interval must bracket 5e-6.
    """
    A0, A1 = (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)
    C0, C1 = (0.0, -5e-6, 0.0), (1000.0, -5e-6, 0.0)
    st, lb, ub, _method = segment_segment_distance_certified(A0, A1, C0, C1, 5e-7, 0.0)
    assert st == "safe"
    assert lb <= 5e-6 <= ub
    assert lb == pytest.approx(5e-6, rel=1e-3)
    st2, _lb2, _ub2, _m2 = segment_segment_distance_certified(A0, A1, C0, C1, 1e-8, 0.0)
    assert st2 == "safe", "5e-6 um apart is NOT a violation of an 1e-8 um requirement"


def test_v012_state_identity_counterexample_is_still_split():
    from native3d.search.finite_graph import route_finite_native3d as route_fin
    from native3d.search.router import route
    c = route(core_case(), mode="native3d", strategy="continuous_pose",
              max_expansions=10, max_runtime_s=2.0)
    assert c.state_identity == "pose_float"
    assert c.approximate_state_identity is False
    f = route_fin(core_case(), mode="native3d")
    assert f.state_identity == "finite_lattice_node_id"
    assert f.approximate_state_identity is False


def test_v012_core_acceptance_summary_is_untouched():
    """The frozen v0.1.2 evidence must still be readable and unchanged."""
    p = os.path.join(V012_ARTIFACTS, "v012_core_acceptance_summary.json")
    art = json.load(open(p, encoding="utf-8"))
    c1 = art["results"]["C1"]
    assert c1["native3d"]["cost_um"] == pytest.approx(594.1592653589794, abs=1e-9)
    assert c1["native3d"]["expansions"] == 75
    assert c1["2d"]["status"] == "search_exhausted"
    assert c1["2d"]["expansions"] == 81
    assert c1["independent_dijkstra"]["difference_um"] == pytest.approx(0.0, abs=1e-12)


# ===========================================================================
# 3. NEW: a full legal 3D path above a reference arc
# ===========================================================================
# The reference is the XY full circle, centre (0,0,0), u = +x, v = +y, R = 50,
# theta = 2*pi, i.e. the set {(50 cos p, 50 sin p, 0)}.
XY_FULL_CIRCLE = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                               50.0, 2.0 * math.pi)
# The "wrong plane" control: the XZ circle {(50 cos p, 0, 50 sin p)}.
XZ_FULL_CIRCLE = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                               50.0, 2.0 * math.pi)


def flight_path_distance_um(x0, y0, z0, R=50.0):
    """Closed form dist(circle in the XY plane, the x-axis segment x in [-x0, x0]).

    A point of the circle is Q(p) = R(cos p, sin p, 0).  The squared distance from
    P = (x, y0, z0) is
        (x - R cos p)^2 + (y0 - R sin p)^2 + z0^2
        = x^2 + y0^2 + z0^2 + R^2 - 2R (x cos p + y0 sin p),
    whose stationary condition is the projection condition cos p = x/rho,
    sin p = y0/rho with rho = hypot(x, y0), i.e. the nearest curve point is the
    in-plane projection of (x, y0) scaled by R/rho.  That phase is reached by the
    full sweep, so the critical value is
        d^2 = (rho - R)^2 + z0^2      with rho = hypot(x, y0),
    and the out-of-plane offset z0 enters QUADRATICALLY.  For 0 < x <= R the
    distance grows with |y0| and the minimum over the segment is at x = x0.
    Verified independently by brute force (see the test below).
    """
    rho = math.hypot(x0, y0)
    return math.sqrt((rho - R) ** 2 + z0 ** 2)


def _flyover_scene(z=50.0):
    return {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
            "obstacles": [],
            "reference_centerlines": [XY_FULL_CIRCLE.to_dict()],
            "source_pose": {"position": [-10.0, 0.0, z], "tangent": [1.0, 0.0, 0.0]},
            "target_pose": {"position": [10.0, 0.0, z], "tangent": [1.0, 0.0, 0.0]}}


def _flyover_path(z=50.0):
    return {"segments": [{"kind": "line", "p0": [-10.0, 0.0, z],
                          "p1": [10.0, 0.0, z]}]}


def test_the_closed_form_used_by_these_tests_matches_the_reference_geometry():
    """Independent numeric check of `flight_path_distance_um` itself."""
    def brute(x0, y0, z0, R=50.0, n=200000):
        best = float("inf")
        for i in range(n + 1):
            p = 2.0 * math.pi * i / n
            q = (R * math.cos(p), R * math.sin(p), 0.0)
            d = math.dist((x0, y0, z0), q)
            best = min(best, d)
        return best
    for x0, y0, z0 in ((10.0, 0.0, 50.0), (10.0, 0.0, 0.0), (10.0, 0.0, 10.0),
                       (10.0, 0.0, 90.0), (10.0, 48.0, 0.0), (10.0, 2.0, 0.0),
                       (10.0, 0.0, -30.0), (3.0, 4.0, 20.0)):
        assert flight_path_distance_um(x0, y0, z0) == pytest.approx(
            brute(x0, y0, z0), abs=1e-6), (x0, y0, z0)


def test_a_full_legal_3d_path_above_a_reference_arc_is_accepted():
    """The full-path acceptance v0.1.2 was missing (it had only point tests)."""
    z = 50.0
    v = validate_path_dict(_flyover_path(z), _flyover_scene(z), cfg_dict())
    true_min = flight_path_distance_um(10.0, 0.0, z)
    assert true_min == pytest.approx(64.031242374328, abs=1e-9)
    assert v["verdict"] == "pass", (v["verdict"], v["centerline_spacing"]["status"])
    sp = v["centerline_spacing"]
    assert sp["status"] == "safe"
    assert sp["min_sampled_um"] == pytest.approx(true_min, rel=1e-3)
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9
    assert v["certification"]["self_check_failed"] is False
    assert v["obstacle_clearance"]["status"] == "safe"


def test_the_same_flyover_path_is_accepted_by_the_production_stack():
    """The production stack must say SAFE, and its number must be a valid LOWER bound.

    It reports 50.0 um here, not 64.031242: the production segment/arc spacing check
    is CONSERVATIVE (a chord subdivision with the sagitta subtracted), so it returns
    a certified lower bound, which is exactly what a "safe" decision needs.  The
    independent validator is the one that reports the achieved 64.031242.  Requiring
    the production bound to be the exact distance would be a false requirement.
    """
    z = 50.0
    true_min = flight_path_distance_um(10.0, 0.0, z)
    seg = LineSegment3D((-10.0, 0.0, z), (10.0, 0.0, z))
    cfg = RouterConfig()
    prod = primitive_spacing(seg, [XY_FULL_CIRCLE], cfg.dmin_centerline_um, cfg)
    assert prod.status == "safe"
    assert 0.0 < prod.bound_um <= true_min + 1e-9, prod.bound_um
    # and the validator's sampled estimate is the true distance
    v = validate_path_dict(_flyover_path(z), _flyover_scene(z), cfg_dict())
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(true_min, rel=1e-3)


@pytest.mark.parametrize("z", [10.0, 25.0, 50.0, 90.0])
def test_the_flyover_height_and_the_reported_spacing_agree(z):
    v = validate_path_dict(_flyover_path(z), _flyover_scene(z), cfg_dict())
    true_min = flight_path_distance_um(10.0, 0.0, z)
    assert v["verdict"] == "pass"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(true_min, rel=2e-3)


def test_a_real_crossing_of_the_reference_circle_still_fails():
    """Acceptance must not be achieved by making every failure a pass.

    The path must really reach the curve's plane AND the curve: the segment
    y = z = 0 from x = -100 to x = +100 passes straight through the XY circle and
    through its interior (true minimum distance 0.0).
    """
    scene = _flyover_scene(0.0)
    scene["source_pose"] = {"position": [-100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}
    scene["target_pose"] = {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}
    path = {"segments": [{"kind": "line", "p0": [-100.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["verdict"] == "fail"
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(0.0, abs=1e-9)

    prod = primitive_spacing(LineSegment3D((-100.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
                             [XY_FULL_CIRCLE], RouterConfig().dmin_centerline_um,
                             RouterConfig())
    assert prod.status == "violation"


def test_a_path_that_stays_inside_the_circle_is_legal_not_a_crossing():
    """Control: (-10,0,0)->(10,0,0) is INSIDE the circle (40 um away), so it passes.

    This is the case that must NOT be turned into a failure: it is exactly the kind
    of legal in-plane detour that v0.1.2's wrong point-to-arc distance rejected.
    """
    scene = _flyover_scene(0.0)
    v = validate_path_dict(_flyover_path(0.0), scene, cfg_dict())
    assert flight_path_distance_um(10.0, 0.0, 0.0) == pytest.approx(40.0, abs=1e-12)
    assert v["verdict"] == "pass"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(40.0, rel=1e-9)


def test_a_lowered_flyover_that_comes_too_close_is_rejected():
    """A reference circle small enough that the same path really is too close.

    With R = 12 the true minimum distance of (-10,0,0)->(10,0,0) to the reference
    circle is 12 - 10 = 2 um, below dmin = 5 um, so both the production stack and
    the independent validator must reject it.  This keeps the "violation" branch of
    the spacing check alive for a genuinely close 3D configuration.
    """
    small = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                          12.0, 2.0 * math.pi)
    scene = _flyover_scene(0.0)
    scene["reference_centerlines"] = [small.to_dict()]
    v = validate_path_dict(_flyover_path(0.0), scene, cfg_dict())
    assert v["verdict"] == "fail"
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(2.0, rel=1e-6)
    prod = primitive_spacing(LineSegment3D((-10.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
                             [small], RouterConfig().dmin_centerline_um, RouterConfig())
    assert prod.status == "violation"


# ===========================================================================
# 4. new-contract consistency on a real searched path
# ===========================================================================
def test_a_searched_success_publishes_verifiable_endpoint_data():
    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success"
    assert res.endpoint_errors is not None
    ep = res.endpoint_errors
    assert ep["start_position_error_um"] <= sc.config.source_pos_tol_um
    assert ep["start_tangent_error_deg"] <= sc.config.source_ang_tol_deg
    assert ep["end_position_error_um"] <= sc.config.eps_pos_um
    assert ep["end_tangent_error_deg"] <= sc.config.eps_ang_deg
    assert res.port_assurance["verified_against_the_caller_pose"] is True
    assert res.port_assurance["assembled_path_check"]["ok"] is True
    assert res.zero_length_solution is False
    assert res.n_segments == len(res.path.segments)


def test_the_searched_path_really_starts_and_ends_on_the_caller_poses():
    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    start = res.path.segments[0].start_pose
    end = res.path.segments[-1].end_pose
    assert norm(sub(start.position, sc.source.position)) <= sc.config.source_pos_tol_um
    assert norm(sub(end.position, sc.target.position)) <= sc.config.eps_pos_um
    assert dot(start.tangent, sc.source.tangent) == pytest.approx(1.0, abs=1e-12)
    assert dot(end.tangent, sc.target.tangent) == pytest.approx(1.0, abs=1e-12)


def test_the_artifact_of_a_searched_success_validates_end_to_end():
    from native3d.io.artifacts import build_artifact, finalise_artifact
    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    assert art["independent_validation"] is not None
    assert art["independent_validation"]["verdict"] in ("pass", "uncertified")
    assert art["search"]["status"] == "success"
    assert art["self_spacing_check"].startswith("NOT IMPLEMENTED")
    # the artifact records the cold/warm distinction of the graph it used
    assert "graph_reuse" in art["search"]["finite_graph"]
    assert art["search"]["finite_graph"]["graph_reuse"]["accepted"] is True


def test_a_zero_length_artifact_validates_end_to_end():
    from native3d.io.artifacts import build_artifact, finalise_artifact
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = Scene("v013_d_zero", Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)),
               [], [], pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               RouterConfig())
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success"
    assert res.zero_length_solution is True
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    assert art["independent_validation"] is not None, \
        "a zero-length success must still be independently validated"
    assert art["independent_validation"]["verdict"] == "pass"
    assert art["independent_validation"]["zero_length_solution"] is True
    assert art["search"]["path"] is None
    assert art["search"]["zero_length_solution"] is True
    assert art["search"]["n_segments"] == 0


def test_the_independent_validator_never_imports_the_router():
    import ast
    root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "native3d", "validation", "validate.py")
    tree = ast.parse(open(root, encoding="utf-8").read())
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = ["." * (node.level or 0) + (node.module or "")]
        for mod in names:
            assert not mod.startswith(("native3d", ".", "..")), mod
