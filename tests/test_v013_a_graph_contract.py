"""v0.1.3 PART A1: the public finite router's graph/scene consistency contract.

THE DEFECT (v0.1.2, reproduced for real in this file before the repair)

``route_finite_native3d(scene, graph=prebuilt)`` accepted any ``FiniteGraph``.
Nothing compared the graph with the scene that was actually being routed, so:

* the GOAL node id was taken from the OLD graph's pose->node table
  (``finite_graph.py`` line ~631 ``target_key = graph.node_pose_key(goal_node)``),
  i.e. from the old scene's target pose;
* the success condition was ``state.pose_key() == target_key``, so the search
  stopped at the OLD target and returned ``success``;
* ``goal_pos_error_um`` was then computed against the NEW scene target and simply
  recorded the mismatch without failing;
* the lattice anchor and the boundary were the old scene's, and every edge had
  been validated against the OLD scene's chip / obstacles / reference centrelines;
* ``graph.fingerprint`` existed (16 keys) but was read by nothing.

Real v0.1.2 pre-fix run on the frozen snapshot, geometry of THIS file
(chip [-150,150]^3, spacing 50 um, source (0,0,0)+x), measured with
artifacts/v013/probe/v013_prefix_replay.py (which loads the frozen snapshot as
package ``native3d_prefix``):

    n_positions = 125, 750 canonical nodes
    source identity node = (0,0,0,0), target identity node = (2,0,0,0)
    pose_key_to_node[node_pose_key(("target",))] == (2,0,0,0) is the lattice
    node at (100,0,0), whose pose key the (virtual) target node of scene B
    would also compute

    graph = FiniteGraph(scene A: target (100,0,0))
    route_finite_native3d(scene B: target (50,0,0), graph=graph)
      -> status = success, cost = 100.0, endpoint = (100,0,0),
         goal_pos_error_um = 50.0            <- the OLD target, reported as success
    route_finite_native3d(scene B, mode="native3d")   (fresh)
      -> status = success, cost = 50.0, endpoint = (50,0,0)

After the v0.1.3 repair the same call returns ``graph_scene_mismatch`` with
``path=None``, ``cost=inf`` and ``goal_pos_error_um=None``; the fresh scene B
route is unchanged.
"""

from __future__ import annotations

import math

import pytest

from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.geometry import CircularArc3D, LineSegment3D, Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import (FiniteGraph, graph_config_fingerprint,
                                          route_finite_native3d)

# Chip of the counterexample: spacing 50 um, boundary margin 2 um, so the lattice
# holds 5 x 5 x 5 = 125 positions / 750 canonical nodes.  A [-2000,2000]^3 chip
# would make every test in this file take ~30 s of graph construction for no
# extra evidence; this one is exact and fast, and the source (0,0,0) is INSIDE
# the shrunken volume (|0| < 148), so the search really runs.
CHIP = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))
SRC = (0.0, 0.0, 0.0)
TGT_A = (100.0, 0.0, 0.0)
TGT_B = (50.0, 0.0, 0.0)


def _scene(name, source, target, chip=CHIP, obstacles=(), refs=(), **cfg_over):
    return Scene(name=name, chip=chip, obstacles=list(obstacles),
                 reference_centerlines=list(refs),
                 source=Pose3D(source[0], source[1]),
                 target=Pose3D(target[0], target[1]),
                 config=RouterConfig(**cfg_over))


def scene_a():
    return _scene("v013_a_scene_A", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_A, (1.0, 0.0, 0.0)))


def scene_b():
    return _scene("v013_a_scene_B", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_B, (1.0, 0.0, 0.0)))


def graph_of(scene, mode="native3d"):
    return FiniteGraph(scene, mode=mode, cfg=scene.config)


# ---------------------------------------------------------------------------
# 0. the counterexample is real: a FRESH route of scene B is correct
# ---------------------------------------------------------------------------
def test_fresh_scene_b_route_is_correct():
    res = route_finite_native3d(scene_b(), mode="native3d")
    assert res.status == "success"
    assert res.path is not None
    assert res.path.end_pose.position == pytest.approx(TGT_B, abs=1e-9)
    assert res.cost == pytest.approx(50.0)
    assert res.goal_pos_error_um <= scene_b().config.eps_pos_um


# ---------------------------------------------------------------------------
# 1. a mismatched prebuilt graph must be refused BEFORE the search
# ---------------------------------------------------------------------------
def test_changed_target_with_a_prebuilt_graph_is_refused_not_success():
    res = route_finite_native3d(scene_b(), graph=graph_of(scene_a()))
    assert res.status not in ("success", "search_exhausted"), res.status
    assert res.status == "graph_scene_mismatch"
    assert res.path is None
    assert res.cost == math.inf
    assert res.length_um == math.inf
    assert res.goal_pos_error_um is None
    assert "target" in res.reason.lower()


def test_changed_source_with_a_prebuilt_graph_is_refused():
    sc = _scene("v013_a_changed_source", ((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                (TGT_A, (1.0, 0.0, 0.0)))
    res = route_finite_native3d(sc, graph=graph_of(scene_a()))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None
    assert "source" in res.reason.lower()


def test_changed_source_tangent_with_a_prebuilt_graph_is_refused():
    sc = _scene("v013_a_changed_source_tangent",
                (SRC, (0.0, 1.0, 0.0)), (TGT_A, (1.0, 0.0, 0.0)))
    res = route_finite_native3d(sc, graph=graph_of(scene_a()))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_changed_target_tangent_with_a_prebuilt_graph_is_refused():
    sc = _scene("v013_a_changed_target_tangent",
                (SRC, (1.0, 0.0, 0.0)), (TGT_A, (0.0, 1.0, 0.0)))
    res = route_finite_native3d(sc, graph=graph_of(scene_a()))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_mode_mismatch_with_a_prebuilt_graph_is_refused():
    """A 2D graph carries the 2D z-plane restriction, so it cannot serve native3d."""
    res = route_finite_native3d(scene_a(), mode="native3d",
                                graph=graph_of(scene_a(), mode="2d"))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None
    res2 = route_finite_native3d(scene_a(), mode="2d",
                                 graph=graph_of(scene_a(), mode="native3d"))
    assert res2.status == "graph_scene_mismatch"
    assert res2.path is None


# ---------------------------------------------------------------------------
# 2. same COUNT but different CONTENT must also be refused
# ---------------------------------------------------------------------------
def test_reference_centrelines_with_the_same_count_but_different_geometry():
    ref_a = LineSegment3D((-1000.0, 500.0, 0.0), (1000.0, 500.0, 0.0))
    ref_b = LineSegment3D((-1000.0, -500.0, 0.0), (1000.0, -500.0, 0.0))
    sc_a = _scene("v013_a_ref_a", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_A, (1.0, 0.0, 0.0)), refs=[ref_a])
    sc_b = _scene("v013_a_ref_b", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_A, (1.0, 0.0, 0.0)), refs=[ref_b])
    assert len(sc_a.reference_centerlines) == len(sc_b.reference_centerlines) == 1
    res = route_finite_native3d(sc_b, graph=graph_of(sc_a))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_reference_arc_with_the_same_count_but_a_different_sweep():
    arc_a = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                          300.0, math.pi)
    arc_b = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                          300.0, -math.pi)
    sc_a = _scene("v013_a_arc_a", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_A, (1.0, 0.0, 0.0)), refs=[arc_a])
    sc_b = _scene("v013_a_arc_b", (SRC, (1.0, 0.0, 0.0)),
                  (TGT_A, (1.0, 0.0, 0.0)), refs=[arc_b])
    res = route_finite_native3d(sc_b, graph=graph_of(sc_a))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_obstacle_added_after_the_graph_was_built_is_refused():
    sc_a = scene_a()
    sc_b = scene_a()
    sc_b.obstacles = [Box3D((-100.0, -100.0, -100.0), (100.0, 100.0, 100.0))]
    res = route_finite_native3d(sc_b, graph=graph_of(sc_a))
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_obstacle_occupancy_changed_in_place_after_the_graph_was_built():
    """Scene is MUTABLE, so the check cannot be an object-identity comparison."""
    sc = scene_a()
    graph = graph_of(sc)
    sc.obstacles = [Box3D((0.0, -10.0, -10.0), (10.0, 10.0, 10.0))]
    res = route_finite_native3d(sc, graph=graph)
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_chip_volume_changed_after_the_graph_was_built_is_refused():
    sc = scene_a()
    graph = graph_of(sc)
    small = Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0))
    sc.chip = small
    res = route_finite_native3d(sc, graph=graph)
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


def test_changed_action_configuration_is_refused():
    sc_a = scene_a()
    graph = graph_of(sc_a)
    for over in ({"finite_spacing_um": 25.0},
                 {"finite_straight_steps": (1, 2)},
                 {"finite_turn_radius_steps": (1,)},
                 # rmin_um cannot be raised ALONE: RouterConfig.__post_init__
                 # rejects action radii below rmin_um, so the action radii that
                 # the new rmin implies have to be given with it (the draft of
                 # this file raised ValueError before any routing happened).
                 {"rmin_um": 100.0, "turn_radii_um": (100.0, 150.0),
                  "connector_radii_um": (100.0, 150.0)},
                 {"radius_tol_um": 1e-3},
                 {"dmin_centerline_um": 7.0},
                 {"obstacle_clearance_um": 9.0},
                 {"obstacle_clearance_includes_envelope": True},
                 {"wg_envelope_radius_um": 4.0},
                 {"boundary_margin_um": 12.0},
                 {"clearance_decision_tol_um": 1e-6},
                 {"max_subdiv_level": 5},
                 {"finite_port_attach_um": 40.0},
                 {"finite_port_attach_max_nodes": 8}):
        sc_b = _scene("v013_a_cfg", (SRC, (1.0, 0.0, 0.0)),
                      (TGT_A, (1.0, 0.0, 0.0)), **over)
        res = route_finite_native3d(sc_b, graph=graph)
        assert res.status == "graph_scene_mismatch", (over, res.status)


def test_changed_congestion_field_is_refused_instead_of_reusing_stale_costs():
    from native3d.search.congestion import RegionCongestion

    sc = scene_a()
    graph = graph_of(sc)
    region = Box3D((60.0, -20.0, -20.0), (90.0, 20.0, 20.0))
    res = route_finite_native3d(sc, congestion=RegionCongestion([(region, 2.0)]),
                                graph=graph)
    assert res.status == "graph_scene_mismatch"
    assert res.path is None


# ---------------------------------------------------------------------------
# 3. a genuinely matching prebuilt graph is a legal, documented reuse
# ---------------------------------------------------------------------------
def test_a_matching_prebuilt_graph_is_still_usable_and_is_the_same_graph():
    sc = scene_a()
    graph = graph_of(sc)
    a = route_finite_native3d(sc, graph=graph)
    assert a.status == "success", (a.status, a.reason)
    assert a.path is not None
    assert a.path.end_pose.position == pytest.approx(TGT_A, abs=1e-9)
    fg = a.finite_graph
    assert fg["graph_reuse"]["accepted"] is True
    assert fg["graph_reuse"]["mismatches"] == []
    assert fg["graph_reuse"]["graph_cache_state"] in ("cold", "warm")
    assert fg["graph_reuse"]["graph_build_count"] >= 1
    assert a.finite_graph["bounds"] == graph.bounds


def test_reusing_the_same_graph_twice_is_reported_as_a_warm_reuse():
    sc = scene_a()
    graph = graph_of(sc)
    first = route_finite_native3d(sc, graph=graph)
    second = route_finite_native3d(sc, graph=graph)
    assert first.status == second.status == "success"
    assert first.cost == pytest.approx(second.cost)
    assert second.finite_graph["graph_reuse"]["graph_cache_state"] == "warm"
    assert second.finite_graph["graph_reuse"]["successor_cache_hits"] == graph.cache_hits
    assert second.finite_graph["graph_reuse"]["successor_cache_hits"] > 0


def test_the_mismatch_is_reported_as_a_structured_diff():
    res = route_finite_native3d(scene_b(), graph=graph_of(scene_a()))
    rep = res.finite_graph["graph_reuse"]
    assert rep["accepted"] is False
    assert "target" in rep["mismatches"]
    assert rep["mismatch_detail"]["target"]["expected"] == pytest.approx(list(TGT_B))
    assert rep["mismatch_detail"]["target"]["graph"] == pytest.approx(list(TGT_A))
    assert "scene_geometry_fingerprint" in rep
    assert "graph_config_fingerprint" in rep


# ---------------------------------------------------------------------------
# 4. the fingerprint itself must cover the content, not the count
# ---------------------------------------------------------------------------
def test_fingerprint_covers_full_reference_geometry_not_the_count():
    ref_a = LineSegment3D((-1000.0, 500.0, 0.0), (1000.0, 500.0, 0.0))
    ref_b = LineSegment3D((-1000.0, -500.0, 0.0), (1000.0, -500.0, 0.0))
    sc_a = _scene("fp_a", (SRC, (1.0, 0.0, 0.0)), (TGT_A, (1.0, 0.0, 0.0)),
                  refs=[ref_a])
    sc_b = _scene("fp_b", (SRC, (1.0, 0.0, 0.0)), (TGT_A, (1.0, 0.0, 0.0)),
                  refs=[ref_b])
    fa = graph_config_fingerprint(sc_a.config, sc_a)
    fb = graph_config_fingerprint(sc_b.config, sc_b)
    assert fa != fb, "the fingerprint must change when the reference geometry changes"
    assert fa["reference_centrelines"] != fb["reference_centrelines"]


def test_fingerprint_covers_the_obstacle_geometry():
    sc_a = scene_a()
    sc_b = scene_a()
    sc_b.obstacles = [Box3D((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))]
    assert (graph_config_fingerprint(sc_a.config, sc_a)
            != graph_config_fingerprint(sc_b.config, sc_b))


def test_fingerprint_covers_the_source_and_target_poses_and_the_mode():
    sc_a = scene_a()
    sc_b = scene_b()
    assert (graph_config_fingerprint(sc_a.config, sc_a, mode="native3d")
            != graph_config_fingerprint(sc_b.config, sc_b, mode="native3d"))
    assert (graph_config_fingerprint(sc_a.config, sc_a, mode="native3d")
            != graph_config_fingerprint(sc_a.config, sc_a, mode="2d"))


# ---------------------------------------------------------------------------
# 5. the bijection guard must be able to fail
# ---------------------------------------------------------------------------
def test_pose_to_node_mapping_is_a_real_bijection():
    """A collision must be DETECTED, not silently overwritten.

    v0.1.2 built ``pose_key_to_node`` with a plain dict assignment.  A grid node
    whose pose key collides with the (virtual) target node silently overwrote the
    target entry, and the guard in route_finite_native3d could not see it because
    it only compared dict SIZES.
    """
    sc = scene_b()          # target (50,0,0) is also the lattice node (1,0,0,+x)
    graph = graph_of(sc)
    assert graph.pose_key_collisions == {}, graph.pose_key_collisions
    # the grid node and the virtual target node must not silently share one key
    assert graph.target_identity_node is not None
    assert graph.pose_key_to_node[graph.node_pose_key(("target",))] == \
        graph.target_identity_node
