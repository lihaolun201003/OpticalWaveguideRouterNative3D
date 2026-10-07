"""Phase 6 CORE acceptance case: 2D mode unreachable, Native3D mode reachable.

v0.1.1 shape of the evidence (the session fixture runs all of it):

* the PRODUCTION search (safe exact-pose identity, correct A* termination) is run
  with a disclosed small budget and must report budget_exhausted - a budget stop
  is never unreachability;
* the finite canonical reference graph (search/reference_graph.py) is run to
  completion: Native3D mode succeeds with a fully constrained C1 route, 2D mode
  really exhausts (search_exhausted, not a budget stop);
* the 2D unreachability is additionally carried by the geometric separation
  argument, which is independent of any search.

The production exact-pose graph is not exhaustible on this scene: measured, the
A* frontier moves from f = 486.4 to f = 491.0 while expanding 20 000 states, so
the number of states below any interesting optimum grows super-exponentially.
"""

import json
import math
import os

import pytest

from native3d.validation.validate import validate_artifact


def test_core_case_inputs_are_identical_for_both_modes(core_case_run):
    a = core_case_run["scene_2d"].to_dict()
    b = core_case_run["scene_3d"].to_dict()
    assert a == b, "the two modes must receive exactly the same input geometry"


def test_core_case_geometry_makes_the_z0_slice_disconnected(core_case_run):
    sc = core_case_run["scene_3d"]
    cfg = sc.config
    wall = sc.obstacles[0]
    chip = sc.chip
    # 1. the wall leaves no side gap: its y range contains the whole chip y range
    assert wall.lo[1] <= chip.lo[1] and wall.hi[1] >= chip.hi[1]
    # 2. the wall blocks the whole port plane and the volume below it
    assert wall.lo[2] <= sc.source.position[2] <= wall.hi[2]
    assert wall.lo[2] <= chip.lo[2]
    # 3. source and target are on opposite sides, each clear of the keep-out
    req = cfg.effective_obstacle_clearance_um
    assert sc.source.position[0] + req <= wall.lo[0]
    assert sc.target.position[0] - req >= wall.hi[0]
    # 4. both ports are at z = 0 exactly
    assert sc.source.position[2] == 0.0 and sc.target.position[2] == 0.0
    # 5. above the wall there is both height and length for a 3D path
    free_height = chip.hi[2] - cfg.effective_boundary_margin_um - (wall.hi[2] + req)
    assert free_height >= cfg.rmin_um
    assert (wall.lo[0] - req) - sc.source.position[0] >= 2.0 * cfg.rmin_um
    assert sc.target.position[0] - (wall.hi[0] + req) >= 2.0 * cfg.rmin_um


def test_core_case_production_2d_run_is_a_budget_stop_not_unreachability(core_case_run):
    res = core_case_run["res_2d"]
    assert res.status == "budget_exhausted", res.status
    assert res.budget_exhausted is True
    assert "NOT a proof" in res.reason
    assert res.path is None
    assert res.state_identity == "pose_float"
    assert res.expansions < core_case_run["production_budget"]["max_expansions"] + 1


def test_core_case_production_native3d_run_is_a_budget_stop_not_unreachability(core_case_run):
    res = core_case_run["res_3d"]
    assert res.status == "budget_exhausted", res.status
    assert res.budget_exhausted is True
    assert "NOT a proof" in res.reason


def test_core_case_2d_mode_exhausts_on_the_finite_reference_graph(core_case_run):
    res = core_case_run["ref_2d"]
    assert res.status == "search_exhausted", (res.status, res.reason)
    assert res.budget_exhausted is False, "a real exhaustion, not a budget stop"
    assert res.path is None
    assert res.state_identity == "canonical_reference_graph"
    assert "FINITE canonical reference graph is exhausted" in res.reason


def test_core_case_2d_mode_never_leaves_the_plane(core_case_run):
    probe = core_case_run["probe"]["continuous_2d_mode"]
    assert probe["n_accepted_z_changing"] == 0
    cfg = core_case_run["scene_2d"].config
    for c in probe["candidates"]:
        assert abs(c["z_change_um"]) < 1e-12


def test_core_case_native3d_mode_is_reachable(core_case_run):
    res = core_case_run["ref_3d"]
    cfg = core_case_run["scene_3d"].config
    assert res.status == "success", (res.status, res.reason)
    assert res.path is not None
    assert res.goal_pos_error_um <= cfg.eps_pos_um
    assert res.goal_tangent_error_deg <= cfg.eps_ang_deg
    assert res.length_um > 0.0
    assert res.cost == pytest.approx(res.length_um + res.congestion_cost)


def test_core_case_native3d_z_varies_in_the_search_trace(core_case_run):
    res = core_case_run["ref_3d"]
    zs = [s["z_um"] for s in res.trace]
    assert len(res.trace) >= 3
    assert max(zs) - min(zs) > 10.0, "z must genuinely vary inside the search"
    # and the path leaves the port plane in the middle
    z_extents = [abs(s.aabb()[1][2] - s.aabb()[0][2]) for s in res.path.segments]
    assert any(e > 1.0 for e in z_extents)
    # the route really clears the wall keep-out instead of sneaking around it
    wall = core_case_run["scene_3d"].obstacles[0]
    cfg = core_case_run["scene_3d"].config
    L = res.path.total_length
    for i in range(2001):
        p = res.path.point_at(L * i / 2000.0)
        if wall.lo[0] - cfg.effective_obstacle_clearance_um <= p[0] <= \
                wall.hi[0] + cfg.effective_obstacle_clearance_um:
            assert p[2] > wall.hi[2]


def test_core_case_initial_state_already_offers_feasible_z_changing_actions(core_case_run):
    probe = core_case_run["probe"]["continuous_native3d_mode"]
    assert probe["n_accepted_z_changing"] >= 1
    accepted = [c for c in probe["candidates"]
                if c["accepted"] and abs(c["z_change_um"]) > 1e-9]
    assert accepted, "the initial state must already have feasible z/pitch candidates"
    # these are generated from the initial pose, BEFORE any search expansion
    assert any(c["elevation_change_deg"] > 1.0 for c in accepted)


def test_core_case_path_is_the_raw_searched_path(core_case_run):
    res = core_case_run["ref_3d"]
    prov = core_case_run["artifact_ref_3d"]["segment_provenance"]
    assert len(prov) == len(res.path.segments), "every segment must come from a searched action"
    for p in prov:
        assert p["action"] != "<source>"
        assert p["action"].startswith("ref_connector")
    # the segments that leave z = 0 come from pitch/skew/connector primitives
    z0 = core_case_run["scene_3d"].source.position[2]
    for p in prov:
        seg = res.path.segments[p["segment_index"]]
        lo, hi = seg.aabb()
        if abs(lo[2] - z0) > 1e-9 or abs(hi[2] - z0) > 1e-9:
            assert p["action"].startswith(("pitch", "skew", "connector", "ref_connector",
                                           "straight")), p


def test_core_case_native3d_full_constraint_evidence(core_case_run):
    art = core_case_run["artifact_ref_3d"]
    v = art["independent_validation"]
    cfg = core_case_run["scene_3d"].config
    assert v["verdict"] == "pass"
    assert v["min_radius_um_refit"] >= cfg.rmin_um - cfg.radius_tol_um
    assert v["radius_refit_max_rel_error"] < 1e-9
    assert v["c1_continuous"] is True
    assert v["violation_counts"]["boundary_violation_samples"] == 0
    assert v["violation_counts"]["obstacle_points_inside"] == 0
    assert v["obstacle_clearance"]["min_sampled_clearance_um"] >= cfg.obstacle_clearance_um - 1e-6
    assert v["obstacle_clearance"]["conservative_clearance_lower_bound_um"] <= \
        v["obstacle_clearance"]["min_sampled_clearance_um"] + 1e-9
    assert v["centerline_spacing"]["min_sampled_um"] >= cfg.dmin_centerline_um
    assert v["centerline_spacing"]["conservative_lower_bound_um"] >= cfg.dmin_centerline_um - 1e-6
    assert v["endpoint_errors"]["end_position_error_um"] <= cfg.eps_pos_um
    assert v["endpoint_errors"]["end_tangent_error_deg"] <= cfg.eps_ang_deg
    assert v["endpoint_errors"]["start_position_error_um"] <= cfg.eps_pos_um
    assert v["endpoint_errors"]["start_tangent_error_deg"] <= cfg.eps_ang_deg
    assert v["boundary"]["certified_contained"] is True
    assert v["certification"]["all_checks_certified"] is True


def test_core_case_artifacts_are_on_disk_and_revalidatable(core_case_run):
    d = core_case_run["artifact_dir"]
    paths = {k: os.path.join(d, k) for k in (
        "core_case_2d_mode.json", "core_case_native3d_mode.json",
        "core_case_initial_action_probe.json",
        "core_case_reference_graph_2d.json", "core_case_reference_graph_native3d.json",
        "core_case_finite_native3d_2d.json",
        "core_case_finite_native3d_native3d.json")}
    for p in paths.values():
        assert os.path.isfile(p) and os.path.getsize(p) > 500
    art3 = json.load(open(paths["core_case_reference_graph_native3d.json"],
                          encoding="utf-8"))
    assert art3["mode"] == "native3d_reference_graph"
    assert art3["search"]["status"] == "success"
    assert art3["search"]["path"]["segments"], "the path geometry is stored"
    assert art3["trace_z_summary"]["z_varies"] is True
    assert art3["trace_z_summary"]["actions_changing_z"] >= 1
    again = validate_artifact(art3)
    assert again["verdict"] == "pass"
    assert again["min_radius_um_refit"] == pytest.approx(
        art3["independent_validation"]["min_radius_um_refit"], rel=1e-12)
    art2 = json.load(open(paths["core_case_2d_mode.json"], encoding="utf-8"))
    assert art2["search"]["status"] == "budget_exhausted"
    assert art2["search"].get("path") is None
    assert art2["search"]["budget_exhausted"] is True
    assert art2["independent_validation"] is None
    artr2 = json.load(open(paths["core_case_reference_graph_2d.json"], encoding="utf-8"))
    assert artr2["search"]["status"] == "search_exhausted"
    assert artr2["search"]["budget_exhausted"] is False
    assert artr2["search"].get("path") is None
    assert artr2["search"]["reference_graph"]["n_canonical_mids"] > 500

    # ---- v0.1.2: the FINITE multi-hop production router on the same geometry
    artf3 = json.load(open(paths["core_case_finite_native3d_native3d.json"],
                           encoding="utf-8"))
    assert artf3["strategy"].startswith("route_finite_native3d")
    assert artf3["search"]["status"] == "success"
    assert artf3["search"]["state_identity"] == "finite_lattice_node_id"
    assert artf3["search"]["finite_graph"]["bounds"]["node_upper_bound"] > 0
    assert artf3["search"]["timings"]["end_to_end_s"] < 60.0
    assert artf3["search"]["finite_graph"]["assembled_path_constraint_check"]["ok"] is True
    assert artf3["independent_validation"]["verdict"] == "pass"
    assert artf3["independent_validation"]["certification"]["self_check_failed"] is False
    assert artf3["independent_validation"]["source_pose_check"]["status"] == "safe"
    assert artf3["independent_validation"]["target_pose_check"]["status"] == "safe"
    assert "NOT IMPLEMENTED" in artf3["self_spacing_check"]
    artf2 = json.load(open(paths["core_case_finite_native3d_2d.json"], encoding="utf-8"))
    assert artf2["search"]["status"] == "search_exhausted"
    assert artf2["search"]["budget_exhausted"] is False
    assert artf2["search"]["finite_graph"]["bounds"]["node_upper_bound"] > 0
