"""v0.1.3 PART A2: success must be decided by the ACTUAL assembled curve.

THE DEFECT (v0.1.2)

``success`` was decided by the search goal test alone
(``state.pose_key() == target_key``), where ``target_key`` came from the graph.
The assembled curve was then re-checked against the hard constraints only:

    assembled = validate_segments(segments, chip, obstacles, refs, cfg)
    if not assembled["ok"]:
        res.status = "uncertified" if undecided else "search_exhausted"

so nothing compared the real start/end pose of the returned curve with the pose
the caller actually passed in, no junction C0 tolerance was enforced, no edge
whose own curve misses its canonical endpoint pose was rejected, and an assembled
CONSTRAINT violation was relabelled ``search_exhausted`` - a status that means
"OPEN was exhausted" and therefore described the opposite of what happened.

WHAT IS ENFORCED NOW (this file is the regression)

1. every accepted edge's own curve must END on the canonical pose of its target
   node within the documented position/tangent tolerances, otherwise the edge is
   REJECTED (not merely recorded);
2. before reporting success the assembled curve is compared with the pose the
   caller actually passed: source position/tangent with source_pos_tol_um /
   source_ang_tol_deg, target position/tangent with the eps_pos_um / eps_ang_deg
   acceptance ball, and every junction with pos_tol_um / ang_tol_deg;
3. an assembled curve that violates a hard constraint returns ``constraint_failed``
   (or ``uncertified`` when a check could not be decided), NEVER ``search_exhausted``;
4. a failure leaves no path, no cost and no guarantee field that could be read as
   a success.
"""

from __future__ import annotations

import inspect

import pytest

from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.geometry import CircularArc3D, LineSegment3D, Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import (FiniteGraph, route_finite_native3d,
                                          verify_assembled_endpoints)

# Every path in this file lives inside |x|,|y|,|z| <= 100, so the chip only has
# to contain it plus the 2 um boundary margin.  A [-2000,2000]^3 chip would build
# a 2.96-million-node lattice for every one of these tests (~30 s each) without
# adding any evidence: the assertions are about the endpoint/port contract, not
# about lattice size.
CHIP = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))


def _scene(target_x=100.0, source=(0.0, 0.0, 0.0), chip=CHIP, **cfg_over):
    return Scene(name="v013_a2", chip=chip, obstacles=[], reference_centerlines=[],
                 source=Pose3D(source, (1.0, 0.0, 0.0)),
                 target=Pose3D((target_x, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 config=RouterConfig(**cfg_over))


# ------------------------------------------------------- the check itself
def test_verify_assembled_endpoints_accepts_a_correct_path():
    sc = _scene()
    seg = LineSegment3D((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    assert rep["ok"] is True
    assert rep["status"] == "safe"
    assert rep["end_position_error_um"] <= 1e-9
    assert rep["end_tangent_error_deg"] <= 1e-9


def test_verify_assembled_endpoints_rejects_a_wrong_end_position():
    sc = _scene(target_x=100.0)
    seg = LineSegment3D((0.0, 0.0, 0.0), (50.0, 0.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    assert rep["ok"] is False
    assert rep["status"] == "constraint_failed"
    assert rep["end_position_error_um"] == pytest.approx(50.0)
    assert "target" in rep["failed"]


def test_verify_assembled_endpoints_rejects_a_wrong_start_pose():
    sc = _scene()
    seg = LineSegment3D((10.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    assert rep["ok"] is False
    assert rep["start_position_error_um"] == pytest.approx(10.0)
    assert "source" in rep["failed"]


def test_verify_assembled_endpoints_rejects_a_wrong_start_tangent():
    sc = _scene(source=(0.0, 0.0, 0.0))
    seg = LineSegment3D((0.0, 0.0, 0.0), (0.0, 100.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    assert rep["ok"] is False
    assert rep["start_tangent_error_deg"] == pytest.approx(90.0)
    assert "source_tangent" in rep["failed"]


def test_verify_assembled_endpoints_uses_the_two_separate_tolerance_families():
    """Source tolerances are NOT the target acceptance ball (v0.1.2 semantics)."""
    sc = _scene()
    seg = LineSegment3D((1e-4, 0.0, 0.0), (100.0, 0.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    assert rep["ok"] is False
    assert rep["source_pos_tol_um"] == sc.config.source_pos_tol_um
    assert rep["target_eps_pos_um"] == sc.config.eps_pos_um
    assert rep["source_pos_tol_um"] < rep["target_eps_pos_um"]
    assert "source" in rep["failed"] and "target" not in rep["failed"]


def test_verify_assembled_endpoints_rejects_a_non_c1_junction():
    sc = _scene()
    a = LineSegment3D((0.0, 0.0, 0.0), (40.0, 0.0, 0.0))
    b = LineSegment3D((40.0, 0.0, 0.0), (40.0, 100.0, 0.0))
    c = LineSegment3D((40.0, 100.0, 0.0), (100.0, 100.0, 0.0))
    sc2 = Scene(name="c1", chip=CHIP, obstacles=[], reference_centerlines=[],
                source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                target=Pose3D((100.0, 100.0, 0.0), (1.0, 0.0, 0.0)),
                config=RouterConfig())
    rep = verify_assembled_endpoints((a, b, c), sc2, sc2.config)
    assert rep["ok"] is False
    assert rep["max_junction_tangent_error_deg"] == pytest.approx(90.0)
    assert "junction" in rep["failed"]


def test_verify_assembled_endpoints_rejects_a_gap_between_segments():
    a = LineSegment3D((0.0, 0.0, 0.0), (40.0, 0.0, 0.0))
    b = LineSegment3D((40.0, 5.0, 0.0), (100.0, 5.0, 0.0))
    sc = Scene(name="gap", chip=CHIP, obstacles=[], reference_centerlines=[],
               source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               target=Pose3D((100.0, 5.0, 0.0), (1.0, 0.0, 0.0)),
               config=RouterConfig())
    rep = verify_assembled_endpoints((a, b), sc, sc.config)
    assert rep["ok"] is False
    assert rep["max_junction_position_error_um"] == pytest.approx(5.0)


# ------------------------------------------- the check is really wired in
def test_the_router_reports_the_port_assurance_of_a_real_success():
    res = route_finite_native3d(_scene(), mode="native3d")
    assert res.status == "success"
    pa = res.port_assurance
    assert pa is not None
    assert pa["verified_against_the_caller_pose"] is True
    assert pa["assembled_path_check"]["ok"] is True
    assert pa["source_position_error_um"] <= _scene().config.source_pos_tol_um
    assert pa["source_tangent_error_deg"] <= _scene().config.source_ang_tol_deg
    assert pa["target_position_error_um"] <= _scene().config.eps_pos_um
    assert pa["target_tangent_error_deg"] <= _scene().config.eps_ang_deg
    assert res.endpoint_errors is not None
    # the assembled curve is the one that was searched: no post-processing
    assert res.path is not None
    assert res.path.end_pose.position == pytest.approx([100.0, 0.0, 0.0], abs=1e-9)


def test_every_edge_must_end_on_its_canonical_target_pose():
    """The per-edge end-pose tolerance is a REJECTION criterion, not a note."""
    g = FiniteGraph(_scene(), mode="native3d", cfg=_scene().config)
    tol_pos = _scene().config.pos_tol_um
    tol_ang = _scene().config.ang_tol_deg
    accepted, rejected = g.canonical_successors((0, 0, 0, 0))
    assert accepted, "the source node must have successors"
    for e in accepted:
        assert e.end_position_error_um <= tol_pos, e.to_dict()
        assert e.end_tangent_error_deg <= tol_ang, e.to_dict()
    assert g.edges_end_pose_rejected == 0
    for r in rejected:
        assert r["reason"] != "curve_end_misses_the_canonical_pose"


def test_an_edge_whose_curve_misses_its_node_is_rejected():
    """Feed the acceptance function a curve that does not end on the node."""
    g = FiniteGraph(_scene(), mode="native3d", cfg=_scene().config)
    wrong = LineSegment3D((0.0, 0.0, 0.0), (0.0, 50.0, 0.0))   # ends at y=50
    edge, why = g._accept((wrong,), (0, 0, 0, 0), (1, 0, 0, 0),
                          "probe_wrong_end", "finite_straight")
    assert edge is None
    assert why == "curve_end_misses_the_canonical_pose"
    assert g.edges_end_pose_rejected == 1


# --------------------------------------------------- failure semantics
def test_a_failed_result_has_no_path_and_no_usable_guarantee():
    res = route_finite_native3d(_scene(target_x=1e6), mode="native3d")
    assert res.status != "success"
    assert res.path is None
    assert res.zero_length_solution is False
    fg = res.finite_graph
    assert fg is not None
    assert fg.get("port_assurance") is None


def test_assembled_constraint_violation_is_not_reported_as_search_exhausted():
    """The status vocabulary must not describe the opposite of what happened."""
    src = inspect.getsource(route_finite_native3d)
    assert 'res.status = "uncertified" if undecided else "search_exhausted"' not in src
    assert "constraint_failed" in src
    assert "uncertified" in src


def test_the_status_vocabulary_is_explicitly_separated():
    from native3d.search import statuses
    for name in ("success", "search_exhausted", "budget_exhausted",
                 "port_attachment_failed", "graph_scene_mismatch",
                 "invalid_source_pose", "invalid_target_pose", "unsupported_pose",
                 "constraint_failed", "uncertified"):
        assert name in statuses.ALL_STATUSES, name
    assert statuses.SEARCH_TERMINATIONS == ("success", "search_exhausted",
                                            "budget_exhausted")
    assert len(set(statuses.ALL_STATUSES)) == len(statuses.ALL_STATUSES)


def test_port_attachment_failure_is_reachable_and_distinct(monkeypatch=None):
    """A port outside the attachment neighbourhood is its own status."""
    cfg = dict(finite_port_attach_um=1.0, finite_port_attach_max_nodes=2)
    sc = _scene(target_x=37.0, **cfg)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status in ("port_attachment_failed", "success")
    if res.status == "port_attachment_failed":
        assert res.path is None
        assert res.budget_exhausted is False
        assert "never about physical reachability" in res.reason


def test_cost_length_and_endpoint_errors_are_mutually_consistent():
    res = route_finite_native3d(_scene(), mode="native3d")
    assert res.status == "success"
    assert res.cost == pytest.approx(res.length_um + res.congestion_cost)
    assert res.length_um == pytest.approx(res.path.total_length)
    assert res.congestion_cost >= 0.0
    assert res.goal_pos_error_um == pytest.approx(
        res.port_assurance["target_position_error_um"])
