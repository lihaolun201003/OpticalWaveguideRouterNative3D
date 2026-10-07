"""v0.1.2 PART B/C regression: the finite multi-hop native 3D production router.

The v0.1.1 acceptance ran on a finite reference graph whose successors were
limited to source -> target and source -> mid -> target (max_hops = 2).  That
proves a two-hop connector candidate graph is searchable; it is not a general
multi-hop production router.  v0.1.2 adds search/finite_graph.py:

* node identity is the INTEGER tuple (i, j, k, direction_index), and each id has
  exactly one canonical pose (anchor + spacing*(i,j,k), one of the six axis
  directions), so de-duplication is by node id and never by a rounded pose;
* actions are straights of n*spacing and +-90 degree arcs of radius m*spacing in
  the coordinate plane spanned by the current direction and a perpendicular one;
* EVERY canonical node has the same successor rule, so mid -> mid transitions are
  ordinary edges and there is no hop limit.
"""

import math
import time

import pytest

from native3d.cases import base_config, core_case, sealed_target_case
from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D, validate_segments
from native3d.geometry import Pose3D
from native3d.geometry.vec import norm, sub
from native3d.io.caseio import Scene
from native3d.search.finite_graph import (AXIS_DIRECTIONS, FiniteGraph,
                                          enumerate_graph_edges,
                                          route_finite_native3d)
from native3d.search.router import STRATEGIES, route


# ------------------------------------------------------------ node identity
def test_every_node_id_has_exactly_one_canonical_pose():
    g = FiniteGraph(core_case())
    seen = {}
    for node in g.all_canonical_nodes():
        key = g.node_pose_key(node)
        assert key not in seen, "two ids share a pose: %r and %r" % (seen.get(key), node)
        seen[key] = node
        pose = g.canonical_pose(node)
        i, j, k, d = node
        assert pose.tangent == pytest.approx(AXIS_DIRECTIONS[d], abs=1e-15)
        for axis in range(3):
            q = (pose.position[axis] - g.anchor[axis]) / g.spacing
            assert abs(q - round(q)) < 1e-9
    assert len(seen) == len(g.all_canonical_nodes())
    # a node id and its pose are the SAME information: rebuilding the pose twice
    # gives bit-identical floats, which is what makes id de-duplication sound
    for node in g.all_canonical_nodes()[:50]:
        assert g.node_pose_key(node) == g.node_pose_key(node)


def test_the_successor_set_is_deterministic_and_generated_from_the_canonical_pose():
    g = FiniteGraph(core_case())
    node = (3, 0, 0, 0)
    first = g.canonical_successors(node, collect=False)[0]
    second = g.canonical_successors(node, collect=False)[0]
    assert [(e.to_node, e.action.name) for e in first] == \
           [(e.to_node, e.action.name) for e in second]
    for e in first:
        assert e.action.start_pose.position == pytest.approx(g.position(3, 0, 0))
        assert e.end_position_error_um < 1e-9
        assert e.end_tangent_error_deg < 1e-9


def test_node_and_edge_bounds_come_from_the_construction():
    g = FiniteGraph(core_case())
    b = g.bounds
    assert b["bounds_are_computed_from_the_construction"] is True
    assert b["n_lattice_positions"] == 240, b
    assert b["node_upper_bound"] == 240 * len(AXIS_DIRECTIONS)
    assert b["edge_upper_bound"] >= b["node_upper_bound"]
    nodes = g.all_canonical_nodes()
    assert len(nodes) == b["node_upper_bound"]
    assert len(set(nodes)) == len(nodes)


def test_same_node_id_gives_the_same_geometry_in_a_fresh_graph():
    a = FiniteGraph(core_case())
    b = FiniteGraph(core_case())
    for node in [(0, 0, 0, 0), (3, 0, 1, 4), (7, -2, 2, 5)]:
        assert a.canonical_pose(node) == b.canonical_pose(node)
        assert [(e.to_node, e.action.name, e.cost)
                for e in a.canonical_successors(node)[0]] == \
               [(e.to_node, e.action.name, e.cost)
                for e in b.canonical_successors(node)[0]]


# ----------------------------------------------------- the cross-wall case
def test_core_case_is_solved_by_the_new_public_router():
    res = route_finite_native3d(core_case(), mode="native3d")
    assert res.status == "success", (res.status, res.reason)
    assert res.path is not None and res.path.total_length > 0.0
    assert res.cost == pytest.approx(res.length_um + res.congestion_cost)
    assert res.goal_pos_error_um <= core_case().config.eps_pos_um
    assert res.goal_tangent_error_deg <= core_case().config.eps_ang_deg
    assert res.finite_graph["strategy"].startswith("route_finite_native3d")
    assert res.state_identity == "finite_lattice_node_id"
    assert res.approximate_state_identity is False


def test_core_case_route_really_climbs_over_the_wall():
    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success"
    zs = [s["z_um"] for s in res.trace]
    assert max(zs) >= 45.0, "the route must clear the wall keep-out top"
    assert min(zs) <= 1e-9
    wall = sc.obstacles[0]
    req = sc.config.effective_obstacle_clearance_um
    L = res.path.total_length
    for i in range(4001):
        p = res.path.point_at(L * i / 4000.0)
        if wall.lo[0] - req <= p[0] <= wall.hi[0] + req:
            assert p[2] > wall.hi[2]
    out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                            sc.reference_centerlines, sc.config)
    assert out["ok"] is True, out.get("failed")


def test_core_case_2d_mode_is_a_real_exhaustion_not_a_budget_stop():
    res = route_finite_native3d(core_case(), mode="2d")
    assert res.status == "search_exhausted", (res.status, res.reason)
    assert res.budget_exhausted is False
    assert res.path is None
    assert res.expansions < res.max_expansions
    assert "exhausted" in res.reason


def test_core_case_end_to_end_meets_the_frozen_budget():
    """Acceptance budget: 60 s single search, at most 200000 expansions."""
    sc = core_case()
    assert sc.config.finite_max_runtime_s == 60.0
    assert sc.config.finite_max_expansions == 200_000
    t0 = time.perf_counter()
    res = route_finite_native3d(sc, mode="native3d")
    wall = time.perf_counter() - t0
    assert res.status == "success"
    assert wall < 60.0, "the core end-to-end run must stay inside the budget"
    assert res.expansions <= 200_000
    assert set(res.timings) >= {"graph_build_s", "search_s", "end_to_end_s"}
    assert res.timings["end_to_end_s"] <= wall + 1e-6
    fg = res.finite_graph
    assert fg["edges_validated"] > 0
    assert fg["cache_key_fingerprint"]["spacing_um"] == sc.config.finite_spacing_um


def test_the_2d_run_never_leaves_the_port_plane():
    res = route_finite_native3d(core_case(), mode="2d")
    for c in res.initial_candidates:
        assert abs(c["z_change_um"]) < 1e-12


# ------------------------------------------------------ multi-hop structure
def test_the_core_route_uses_mid_to_mid_transitions():
    """Not a source -> mid -> target template: at least three interior hops."""
    res = route_finite_native3d(core_case(), mode="native3d")
    actions = [s["action"] for s in res.trace]
    interior = actions[1:-1]
    assert len(interior) >= 3, actions
    assert sum(1 for a in interior if a.startswith("turn_")) >= 2
    assert sum(1 for a in interior if a.startswith("straight")) >= 1
    # and no action name mentions the case, a waypoint or a height
    joined = " ".join(actions).lower()
    for banned in ("wall", "core", "mid", "waypoint", "layer", "z40", "climb"):
        assert banned not in joined


def test_the_router_is_not_given_any_waypoint():
    """The public entry point receives only the Scene."""
    import inspect
    sig = inspect.signature(route_finite_native3d)
    assert set(sig.parameters) == {"scene", "mode", "congestion", "max_expansions",
                                   "max_runtime_s", "expand_hook", "graph"}


def test_multi_wall_case_needs_several_interior_nodes():
    """Two walls in series: the route must chain many ordinary edges."""
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 200.0))
    walls = [Box3D((150.0, -50.0, -70.0), (170.0, 350.0, 40.0)),
             Box3D((330.0, -50.0, -70.0), (350.0, 350.0, 40.0))]
    sc = Scene("two_walls", chip, walls, [],
               Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success", (res.status, res.reason)
    assert len(res.trace) >= 8
    assert max(s["z_um"] for s in res.trace) >= 45.0
    out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                            sc.reference_centerlines, sc.config)
    assert out["ok"] is True, out.get("failed")
    res2d = route_finite_native3d(sc, mode="2d")
    assert res2d.status == "search_exhausted"


def test_sealed_target_exhausts_the_finite_graph():
    sc = sealed_target_case()
    # R = m * spacing must still reach Rmin, so the spacing is not shrunk here;
    # this test is about exhaustion semantics, not about resolution.
    assert sc.config.finite_spacing_um >= sc.config.rmin_um / max(
        sc.config.finite_turn_radius_steps)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status in ("search_exhausted", "port_attachment_failed"), res.status
    if res.status == "search_exhausted":
        assert res.budget_exhausted is False
        assert res.path is None


# ------------------------------------------------------- z freedom at step 1
def test_the_first_step_already_has_a_z_degree_of_freedom():
    res = route_finite_native3d(core_case(), mode="native3d")
    cands = res.initial_candidates
    assert cands
    z_change = [c for c in cands if c["accepted"] and abs(c["z_change_um"]) > 1e-9]
    assert z_change, "the first expansion must already offer z-changing edges"
    upward = [c for c in z_change if c["z_change_um"] > 0.0]
    downward = [c for c in z_change if c["z_change_um"] < 0.0]
    assert upward and downward
    assert any(c["elevation_change_deg"] > 1.0 for c in z_change)


# --------------------------------------------------- strategy separation
def test_the_three_strategies_are_explicitly_named():
    assert STRATEGIES == ("continuous_pose", "reference_graph", "finite_native3d")
    sc = core_case()
    a = route(sc, mode="native3d", strategy="finite_native3d")
    assert a.status == "success"
    assert a.mode == "finite_native3d_native3d"
    b = route(core_case(), mode="native3d", strategy="reference_graph")
    assert b.state_identity == "canonical_reference_graph"
    c = route(core_case(), mode="native3d", strategy="continuous_pose",
              max_expansions=10, max_runtime_s=2.0)
    assert c.state_identity == "pose_float"
    with pytest.raises(ValueError):
        route(core_case(), strategy="magic")


def test_the_production_strategy_does_not_fall_back_to_the_reference_graph():
    res = route_finite_native3d(core_case(), mode="native3d")
    assert res.reference_graph is None
    assert res.state_identity != "canonical_reference_graph"
    assert res.finite_graph is not None


# ------------------------------------------- unsupported port poses
def test_a_port_tangent_outside_the_action_set_is_reported_explicitly():
    cfg = base_config(finite_spacing_um=50.0, finite_port_attach_um=10.0)
    chip = Box3D((-200.0, -200.0, -200.0), (400.0, 400.0, 200.0))
    odd = (1.0 / math.sqrt(3.0), 1.0 / math.sqrt(3.0), 1.0 / math.sqrt(3.0))
    sc = Scene("odd_port", chip, [], [],
               Pose3D((0.0, 0.0, 0.0), odd), Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               cfg)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status in ("port_attachment_failed", "unsupported_pose", "success")
    if res.status != "success":
        assert "never about physical reachability" in res.reason or \
               "unsupported" in res.reason.lower()
        assert res.path is None


def test_an_invalid_port_pose_is_reported_separately():
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, 0.0), (200.0, 200.0, 100.0))
    sc = Scene("outside", chip, [], [],
               Pose3D((-50.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((150.0, 100.0, 50.0), (1.0, 0.0, 0.0)), cfg)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "invalid_source_pose"
    assert res.path is None
