"""v0.1.3 PART B2 regression: the JSON source/target pose contract.

THE DEFECT (v0.1.2, reproduced for real before the repair)

``validate_path_dict`` fed ``scene["source_pose"]["tangent"]`` and
``scene["target_pose"]["tangent"]`` straight into

    ang(u, v) = degrees(acos(clamp(dot(u, v))))

with no normalisation and only a clamp.  A target tangent of [1, 1, 0] against a
path whose end tangent is +x gives dot = 1, i.e. 0.0 deg and the verdict "pass",
while the real angle is 45 deg - far outside the default 3 deg target acceptance
ball.  The clamp also turned NaN/Infinity into a pass, a 2-component vector into
an IndexError and a zero direction into an arbitrary angle.  Production
``Pose3D.from_dict`` normalises, so the two implementations disagreed about the
same JSON.

THE CONTRACT AFTER THE REPAIR

Both poses are read by ``read_pose_dict`` (published as
``out["pose_contract"]["contract"]`` == POSE_CONTRACT):

    "normalise a legal non-zero 3D direction, reject otherwise"

* exactly 3 finite JSON numbers for position, exactly 3 for the tangent;
* the tangent must be a non-zero direction whose norm exceeds the shared 1e-12
  threshold of production geometry.vec.unit;
* the tangent that enters every dot product is the NORMALISED one;
* zero direction, NaN, +-Infinity, a missing field, a 2- or 4-component vector or
  a non-numeric component raises ValueError from the validator and can never
  produce a verdict of "pass".

The red baseline is executable: every counterexample below is ALSO evaluated
through the frozen v0.1.2 snapshot (artifacts/v013/prefix_src/ -> native3d_prefix
via tests/v013_helpers.py) and the old wrong value is asserted next to the fixed
one.
"""

from __future__ import annotations

import math

import pytest

from native3d.config import RouterConfig
from native3d.geometry import Pose3D
from native3d.validation.validate import (POSE_CONTRACT, read_pose_dict,
                                          validate_artifact,
                                          validate_path_dict,
                                          validate_zero_length_artifact)

from v013_helpers import prefix_module

CHIP = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [], "reference_centerlines": []}
PATH_X = {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                        "p1": [100.0, 0.0, 0.0]}]}


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def scene_dict(source_pos, source_tan, target_pos, target_tan):
    sc = dict(CHIP)
    sc["source_pose"] = {"position": list(source_pos), "tangent": list(source_tan)}
    sc["target_pose"] = {"position": list(target_pos), "tangent": list(target_tan)}
    return sc


def x_scene(target_tan=(1.0, 0.0, 0.0)):
    return scene_dict((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                      target_tan)


# ===========================================================================
# the 45 degree counterexample
# ===========================================================================
def test_the_45_degree_counterexample_is_really_45_degrees_now():
    v = validate_path_dict(PATH_X, x_scene((1.0, 1.0, 0.0)), cfg_dict())
    ep = v["endpoint_errors"]
    assert ep["end_tangent_error_deg"] == pytest.approx(45.0, abs=1e-9)
    assert v["target_pose_check"]["tangent_ok"] is False
    assert v["target_pose_check"]["status"] == "violation"
    assert v["verdict"] == "fail"
    assert v["violation_counts"]["endpoint_tangent_out_of_tolerance"] == 1


def test_the_frozen_v012_snapshot_said_zero_degrees_and_pass():
    """Red -> green from the frozen v0.1.2 source, in one test."""
    frozen = prefix_module("validation.validate")
    sc = x_scene((1.0, 1.0, 0.0))
    v0 = frozen.validate_path_dict(PATH_X, sc, cfg_dict())
    assert v0["endpoint_errors"]["end_tangent_error_deg"] == 0.0
    assert v0["target_pose_check"]["status"] == "safe"
    assert v0["verdict"] == "pass"
    v1 = validate_path_dict(PATH_X, sc, cfg_dict())
    assert v1["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(45.0)
    assert v1["verdict"] == "fail"


def test_a_direction_reversal_is_180_degrees_and_fails():
    v = validate_path_dict(PATH_X, x_scene((-1.0, 0.0, 0.0)), cfg_dict())
    assert v["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(180.0,
                                                                         abs=1e-9)
    assert v["verdict"] == "fail"


def test_a_reversed_SOURCE_tangent_is_180_degrees_and_fails():
    sc = scene_dict((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                    (1.0, 0.0, 0.0))
    v = validate_path_dict(PATH_X, sc, cfg_dict())
    assert v["endpoint_errors"]["start_tangent_error_deg"] == pytest.approx(180.0,
                                                                           abs=1e-9)
    assert v["source_pose_check"]["status"] == "violation"
    assert v["verdict"] == "fail"


def test_an_oblique_SCALED_source_tangent_is_normalised_too():
    """A source tangent of [5, 5, 0] is 45 deg away from a +x path start."""
    sc = scene_dict((0.0, 0.0, 0.0), (5.0, 5.0, 0.0), (100.0, 0.0, 0.0),
                    (1.0, 0.0, 0.0))
    v = validate_path_dict(PATH_X, sc, cfg_dict())
    assert v["endpoint_errors"]["start_tangent_error_deg"] == pytest.approx(45.0,
                                                                           abs=1e-9)
    assert v["verdict"] == "fail"


# ===========================================================================
# legal but non-unit tangents are ACCEPTED and normalised
# ===========================================================================
def test_a_scaled_tangent_normalises_exactly_like_production():
    v = validate_path_dict(PATH_X, x_scene((1000.0, 0.0, 0.0)), cfg_dict())
    pc = v["pose_contract"]
    assert pc["contract"] == POSE_CONTRACT
    assert pc["target"]["tangent_input_norm"] == pytest.approx(1000.0)
    assert pc["target"]["tangent"] == pytest.approx((1.0, 0.0, 0.0), abs=1e-15)
    assert v["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(0.0,
                                                                        abs=1e-12)
    assert v["verdict"] == "pass"
    prod = Pose3D.from_dict({"position": [100.0, 0.0, 0.0],
                             "tangent": [1000.0, 0.0, 0.0]})
    assert pc["target"]["tangent"] == pytest.approx(tuple(prod.tangent), abs=1e-15)


def test_a_non_unit_oblique_tangent_normalises_exactly_like_production():
    """Path tangent (3,0,4)/5 and JSON tangent [3,0,4] are the SAME direction."""
    path = {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                          "p1": [60.0, 0.0, 80.0]}]}
    sc = scene_dict((0.0, 0.0, 0.0), (3.0, 0.0, 4.0), (60.0, 0.0, 80.0),
                    (30.0, 0.0, 40.0))
    v = validate_path_dict(path, sc, cfg_dict())
    pc = v["pose_contract"]
    assert pc["target"]["tangent"] == pytest.approx((0.6, 0.0, 0.8), abs=1e-15)
    assert pc["target"]["tangent_input_norm"] == pytest.approx(50.0)
    assert pc["source"]["tangent_input_norm"] == pytest.approx(5.0)
    prod = Pose3D.from_dict({"position": [60.0, 0.0, 80.0],
                             "tangent": [3.0, 0.0, 4.0]})
    assert pc["target"]["tangent"] == pytest.approx(tuple(prod.tangent), abs=1e-15)
    assert v["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(0.0,
                                                                        abs=1e-12)
    assert v["verdict"] == "pass"


def test_the_pose_contract_is_published_in_the_evidence():
    v = validate_path_dict(PATH_X, x_scene(), cfg_dict())
    pc = v["pose_contract"]
    assert pc["contract"] == POSE_CONTRACT
    assert "normalise a legal non-zero 3D direction" in pc["contract"]
    assert "reject otherwise" in pc["contract"]
    assert "Pose3D.from_dict" in pc["production_semantics"]
    assert pc["source"]["position"] == (0.0, 0.0, 0.0)
    assert pc["target"]["position"] == (100.0, 0.0, 0.0)
    assert pc["source"]["tangent_input_norm"] == pytest.approx(1.0)
    assert pc["target"]["tangent_input_norm"] == pytest.approx(1.0)
    assert v["source_pose_check"]["pose_contract"] == POSE_CONTRACT
    assert v["target_pose_check"]["pose_contract"] == POSE_CONTRACT
    assert "normalised" in v["endpoint_errors"]["tangent_sources"]["note"].lower()


# ===========================================================================
# malformed poses are rejected and can never pass
# ===========================================================================
BAD_TANGENTS = {
    "zero direction": [0.0, 0.0, 0.0],
    "degenerate 1e-13 direction": [1e-13, 0.0, 0.0],
    "nan direction": [float("nan"), 0.0, 0.0],
    "inf direction": [float("inf"), 0.0, 0.0],
    "-inf direction": [float("-inf"), 0.0, 0.0],
    "two components": [1.0, 0.0],
    "four components": [1.0, 0.0, 0.0, 0.0],
    "one component": [1.0],
    "non-numeric string": ["1.0", "0.0", "0.0"],
    "none component": [None, 0.0, 0.0],
    "bool component": [True, 0.0, 0.0],
    "not a vector": 1.0,
}

BAD_POSITIONS = {
    "nan position": [float("nan"), 0.0, 0.0],
    "inf position": [0.0, float("inf"), 0.0],
    "two components": [0.0, 0.0],
    "four components": [0.0, 0.0, 0.0, 0.0],
    "non-numeric": ["0", 0.0, 0.0],
    "not a vector": {"x": 0.0},
}


@pytest.mark.parametrize("label", sorted(BAD_TANGENTS))
def test_every_malformed_TARGET_tangent_is_rejected(label):
    sc = x_scene()
    sc["target_pose"] = {"position": [100.0, 0.0, 0.0],
                         "tangent": BAD_TANGENTS[label]}
    with pytest.raises(ValueError):
        validate_path_dict(PATH_X, sc, cfg_dict())


@pytest.mark.parametrize("label", sorted(BAD_TANGENTS))
def test_every_malformed_SOURCE_tangent_is_rejected(label):
    sc = x_scene()
    sc["source_pose"] = {"position": [0.0, 0.0, 0.0],
                         "tangent": BAD_TANGENTS[label]}
    with pytest.raises(ValueError):
        validate_path_dict(PATH_X, sc, cfg_dict())


@pytest.mark.parametrize("label", sorted(BAD_POSITIONS))
def test_every_malformed_position_is_rejected(label):
    for slot in ("source_pose", "target_pose"):
        sc = x_scene()
        sc[slot] = {"position": BAD_POSITIONS[label], "tangent": [1.0, 0.0, 0.0]}
        with pytest.raises(ValueError):
            validate_path_dict(PATH_X, sc, cfg_dict())


@pytest.mark.parametrize("slot", ["source_pose", "target_pose"])
def test_missing_and_non_object_poses_are_rejected(slot):
    for bad in ({}, None, [], "pose", 3.0,
                {"position": [0.0, 0.0, 0.0]},          # tangent missing
                {"tangent": [1.0, 0.0, 0.0]}):          # position missing
        sc = x_scene()
        sc[slot] = bad
        with pytest.raises(ValueError):
            validate_path_dict(PATH_X, sc, cfg_dict())


def test_a_malformed_pose_never_produces_a_pass():
    """The failure mode must be an exception, never a dict with verdict pass."""
    sc = x_scene((float("nan"), 0.0, 0.0))
    with pytest.raises(ValueError) as exc:
        validate_path_dict(PATH_X, sc, cfg_dict())
    assert "scene.target_pose" in str(exc.value)
    sc2 = x_scene()
    sc2["target_pose"] = {"position": [100.0, 0.0, 0.0]}
    with pytest.raises(ValueError) as exc2:
        validate_path_dict(PATH_X, sc2, cfg_dict())
    assert "tangent" in str(exc2.value)


def test_the_frozen_v012_snapshot_let_several_malformed_poses_through():
    """The pre-fix behaviour of the malformed inputs, from the frozen snapshot."""
    frozen = prefix_module("validation.validate")
    scene_ok = x_scene()
    # NaN direction -> v0.1.2 produced verdict "pass"
    sc = dict(scene_ok)
    sc["source_pose"] = {"position": [0.0, 0.0, 0.0],
                         "tangent": [float("nan"), 0.0, 0.0]}
    v0 = frozen.validate_path_dict(PATH_X, sc, cfg_dict())
    assert v0["verdict"] == "pass"
    # scaled tangent was compared UN-normalised
    sc2 = dict(scene_ok)
    sc2["source_pose"] = {"position": [0.0, 0.0, 0.0], "tangent": [3.0, 0.0, 0.0]}
    v0b = frozen.validate_path_dict(PATH_X, sc2, cfg_dict())
    assert v0b["verdict"] == "pass"          # correct answer here, wrong reason
    # a 2-component tangent raised IndexError instead of a clear ValueError
    sc3 = dict(scene_ok)
    sc3["target_pose"] = {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0]}
    with pytest.raises(IndexError):
        frozen.validate_path_dict(PATH_X, sc3, cfg_dict())


# ===========================================================================
# the reader itself
# ===========================================================================
def test_read_pose_dict_normalises_and_reports_what_it_read():
    got = read_pose_dict({"position": [1, 2, 3], "tangent": [0.0, 0.0, 7.0]},
                         "scene.source_pose")
    assert got["position"] == (1.0, 2.0, 3.0)
    assert got["tangent"] == pytest.approx((0.0, 0.0, 1.0), abs=1e-15)
    assert got["tangent_input"] == (0.0, 0.0, 7.0)
    assert got["tangent_input_norm"] == pytest.approx(7.0)
    assert got["contract"] == POSE_CONTRACT


def test_the_true_angle_is_used_for_both_poses_at_once():
    """A 45 deg target and a 45 deg source error are both counted, not clamped."""
    path = {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    sc = scene_dict((0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (100.0, 0.0, 0.0),
                    (1.0, -1.0, 0.0))
    v = validate_path_dict(path, sc, cfg_dict())
    assert v["endpoint_errors"]["start_tangent_error_deg"] == pytest.approx(45.0,
                                                                           abs=1e-9)
    assert v["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(45.0,
                                                                         abs=1e-9)
    assert v["violation_counts"]["source_tangent_out_of_tolerance"] == 1
    assert v["violation_counts"]["endpoint_tangent_out_of_tolerance"] == 1
    assert v["verdict"] == "fail"


def test_a_legal_pose_pair_still_passes_after_the_contract():
    v = validate_path_dict(PATH_X, x_scene(), cfg_dict())
    assert v["verdict"] == "pass"
    assert v["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(0.0,
                                                                         abs=1e-12)
    assert v["pose_contract"]["source"]["normalised"] is False
    assert v["pose_contract"]["target"]["normalised"] is False


def test_the_tolerance_boundary_is_still_decided_by_the_target_ball():
    """45 deg fails the default 3 deg ball but passes a 45.001 deg ball."""
    v_bad = validate_path_dict(PATH_X, x_scene((1.0, 1.0, 0.0)), cfg_dict())
    assert v_bad["verdict"] == "fail"
    v_ok = validate_path_dict(PATH_X, x_scene((1.0, 1.0, 0.0)),
                              cfg_dict(eps_ang_deg=45.001))
    assert v_ok["verdict"] == "pass"
    assert v_ok["endpoint_errors"]["end_tangent_error_deg"] == pytest.approx(
        45.0, abs=1e-6)


# ===========================================================================
# the zero-length dispatch uses the SAME strict pose reader
# ===========================================================================
def zero_artifact(source=(0.0, 0.0, 0.0), target=(0.0, 0.0, 0.0),
                  source_tan=(1.0, 0.0, 0.0), target_tan=(1.0, 0.0, 0.0)):
    sc = dict(CHIP)
    sc["name"] = "b2_zero"
    sc["config"] = cfg_dict()
    sc["source_pose"] = {"position": list(source), "tangent": list(source_tan)}
    sc["target_pose"] = {"position": list(target), "tangent": list(target_tan)}
    return {"input": sc,
            "search": {"status": "success", "path": None,
                       "zero_length_solution": True, "n_segments": 0,
                       "cost": 0.0, "length_um": 0.0, "congestion_cost": 0.0}}


def test_a_legal_zero_length_artifact_is_validated_and_passes():
    res = validate_artifact(zero_artifact())
    assert res["verdict"] == "pass"
    assert res["zero_length_solution"] is True
    assert res["n_segments"] == 0
    assert res["certification"]["all_checks_certified"] is True
    assert res["pose_contract"]["contract"] == POSE_CONTRACT
    assert res["source_pose_check"]["status"] == "safe"
    assert res["target_pose_check"]["status"] == "safe"
    assert res["zero_length_checks"]["position_mismatch_um"] == 0.0


def test_a_zero_length_claim_whose_poses_do_not_coincide_fails():
    res = validate_zero_length_artifact(zero_artifact(target=(10.0, 0.0, 0.0)))
    assert res["verdict"] == "fail"
    assert res["zero_length_checks"]["position_mismatch_um"] == pytest.approx(10.0)
    assert any("acceptance ball" in v for v in res["violations"])


def test_a_zero_length_artifact_with_a_malformed_pose_is_rejected():
    art = zero_artifact()
    art["input"]["target_pose"] = {"position": [0.0, 0.0, 0.0],
                                   "tangent": [0.0, 0.0, 0.0]}
    with pytest.raises(ValueError):
        validate_artifact(art)
    art2 = zero_artifact()
    art2["input"]["source_pose"] = {"position": [float("nan"), 0.0, 0.0],
                                    "tangent": [1.0, 0.0, 0.0]}
    with pytest.raises(ValueError):
        validate_artifact(art2)


def test_a_zero_length_artifact_whose_router_claim_disagrees_fails():
    art = zero_artifact()
    art["search"]["endpoint_errors"] = {"end_position_error_um": 5.0}
    res = validate_artifact(art)
    assert res["verdict"] == "fail"
    assert res["router_claims_cross_check"]["disagreements"]


def test_a_failure_artifact_is_never_reported_as_a_validation_pass():
    art = zero_artifact()
    art["search"] = {"status": "search_exhausted", "path": None,
                     "zero_length_solution": False, "n_segments": 0}
    with pytest.raises(ValueError):
        validate_artifact(art)


def test_an_artifact_with_a_real_path_still_uses_the_path_validator():
    art = {"input": dict(CHIP, config=cfg_dict(), source_pose={
        "position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
        target_pose={"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}),
        "search": {"status": "success", "path": PATH_X, "n_segments": 1}}
    res = validate_artifact(art)
    assert res["verdict"] == "pass"
    assert "zero_length_solution" not in res
