"""v0.1.3 PART A3: the zero-length solution contract.

THE DEFECT (v0.1.2, reproduced for real in this file before the repair)

``astar_search`` returns ``SearchResult(..., cost=0.0, length_um=0.0, path=None)``
when the start state already satisfies the goal test, precisely because a
zero-length solution has no primitive to concatenate.  ``route_finite_native3d``
did not know that contract and read ``res.path.segments`` unconditionally:

    source = target = Pose3D((0,0,0), (1,0,0)); chip = [-60,60]^3; default config
    -> AttributeError: 'NoneType' object has no attribute 'segments'

Real v0.1.2 pre-fix run: artifacts/v013/logs/00_prefix_v013_defect_replay.log
section A3.

THE CONTRACT AFTER THE REPAIR (one meaning, everywhere)

A zero-length success is a FIRST-CLASS success whose path is EMPTY:

    status                      == "success"
    zero_length_solution        is True
    cost_um_equivalent          == 0.0
    length_um                   == 0.0
    congestion_cost             == 0.0
    path                        is None          (no fabricated micro-segment)
    n_segments                  == 0
    expansions                  == 0
    endpoint_errors.start_*     verifiable against the SOURCE pose
    endpoint_errors.end_*       verifiable against the TARGET acceptance ball

A FAILURE also has ``path is None``; ``zero_length_solution`` is what separates
them, and the two are checked here for BOTH cases.

The same contract is honoured by the JSON serialisation (io/artifacts.py), the
independent validator (validation/validate.py) and the CLI.
"""

from __future__ import annotations

import json

import pytest

from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.geometry import LineSegment3D, Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import route_finite_native3d

CHIP_60 = Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0))


def _scene(source, target, chip=CHIP_60, obstacles=(), refs=()):
    return Scene(name="v013_a3", chip=chip, obstacles=list(obstacles),
                 reference_centerlines=list(refs), source=source, target=target,
                 config=RouterConfig())


def _zero_scene():
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    return _scene(pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)))


# --------------------------------------------------------------- the contract
def test_source_equals_target_returns_a_defined_zero_length_solution():
    res = route_finite_native3d(_zero_scene(), mode="native3d")
    assert res.status == "success", (res.status, res.reason)
    assert res.zero_length_solution is True
    assert res.cost == pytest.approx(0.0)
    assert res.length_um == pytest.approx(0.0)
    assert res.congestion_cost == pytest.approx(0.0)
    assert res.path is None, "a zero-length solution must not fabricate a segment"
    assert res.n_segments == 0
    assert res.expansions == 0
    assert res.trace and len(res.trace) == 1


def test_zero_length_solution_records_verifiable_endpoint_errors():
    res = route_finite_native3d(_zero_scene(), mode="native3d")
    ep = res.endpoint_errors
    assert ep is not None
    assert ep["start_position_error_um"] <= 1e-9
    assert ep["start_tangent_error_deg"] <= 1e-9
    assert ep["end_position_error_um"] <= 1e-9
    assert ep["end_tangent_error_deg"] <= 1e-9
    assert ep["source_pos_tol_um"] == _zero_scene().config.source_pos_tol_um
    assert ep["target_eps_pos_um"] == _zero_scene().config.eps_pos_um
    assert ep["start_ok"] and ep["end_ok"]
    assert res.port_assurance["zero_length_solution"] is True


def test_zero_length_solution_survives_the_2d_mode_too():
    res = route_finite_native3d(_zero_scene(), mode="2d")
    assert res.status == "success"
    assert res.zero_length_solution is True
    assert res.path is None
    assert res.mode == "finite_native3d_2d"


def test_zero_length_solution_serialises_and_revalidates():
    """Router, JSON and the independent validator must agree on the contract."""
    from native3d.io.artifacts import build_artifact, finalise_artifact
    from native3d.validation.validate import validate_artifact

    sc = _zero_scene()
    res = route_finite_native3d(sc, mode="native3d")
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    assert art["independent_validation"] is not None
    assert art["independent_validation"]["verdict"] == "pass"
    assert art["independent_validation"]["zero_length_solution"] is True
    text = json.dumps(art)
    again = validate_artifact(json.loads(text))
    assert again["verdict"] == "pass"
    assert art["search"]["zero_length_solution"] is True
    assert art["search"]["path"] is None
    assert art["search"]["n_segments"] == 0


def test_zero_length_solution_never_produces_a_degenerate_segment():
    res = route_finite_native3d(_zero_scene(), mode="native3d")
    assert res.path is None
    for step in res.trace:
        assert step["n_segments"] == 0


# --------------------------------------------- not every equal position is a win
def test_same_position_with_the_wrong_target_tangent_is_not_a_zero_length_win():
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = _scene(pose, Pose3D((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status != "success", res.status
    assert res.zero_length_solution is False
    assert res.path is None


def test_nearly_equal_position_outside_the_acceptance_ball_is_not_a_zero_win():
    small = Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0))
    sc = _scene(Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                Pose3D((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)), chip=small)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.zero_length_solution is False
    assert res.status in ("success", "search_exhausted", "port_attachment_failed",
                          "budget_exhausted")
    if res.status == "success":
        assert res.length_um > 0.0
        assert res.path is not None


# ------------------------------------------- input validation is NOT bypassed
def test_source_equal_to_target_still_validates_the_source_pose():
    wall = Box3D((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0))
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = _scene(pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), obstacles=[wall])
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "invalid_source_pose"
    assert res.zero_length_solution is False
    assert res.path is None


def test_source_equal_to_target_outside_the_chip_is_invalid_not_a_zero_win():
    pose = Pose3D((-100.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = _scene(pose, Pose3D((-100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "invalid_source_pose"
    assert res.path is None
    assert res.zero_length_solution is False


def test_source_on_a_fixed_reference_centreline_is_invalid_not_a_zero_win():
    ref = LineSegment3D((0.0, 0.0, 0.0), (0.0, 0.0, 60.0))
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = _scene(pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), refs=[ref])
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "invalid_source_pose"
    assert res.path is None


def test_source_tangent_below_the_target_tolerance_is_refused():
    """A 10 deg misalignment is inside no acceptance ball of the default config."""
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    import math
    th = math.radians(10.0)
    sc = _scene(pose, Pose3D((0.0, 0.0, 0.0), (math.cos(th), math.sin(th), 0.0)))
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status != "success"
    assert res.zero_length_solution is False


def test_zero_length_is_reported_consistently_by_the_route_dispatcher():
    from native3d.search.router import route
    res = route(_zero_scene(), mode="native3d", strategy="finite_native3d")
    assert res.status == "success"
    assert res.zero_length_solution is True
    assert res.path is None
