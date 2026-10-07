"""Independent validation tests (JSON-only second implementation)."""

import json
import os
import re

import pytest

from native3d.cases import base_config, core_case, sealed_target_case
from native3d.constraints.checks import Box3D
from native3d.geometry import Pose3D
from native3d.io.caseio import Scene
from native3d.io.artifacts import build_artifact, finalise_artifact
from native3d.search.router import route
from native3d.validation.validate import validate_artifact, validate_path_dict


@pytest.fixture(scope="module")
def small_run():
    cfg = base_config(max_expansions=5000, max_runtime_s=60.0,
                      pos_resolution_um=20.0, dir_resolution_deg=30.0,
                      straight_lengths_um=(20.0, 40.0), turn_angles_deg=(45.0, 90.0),
                      turn_radii_um=(50.0,), skew_axis_tilt_deg=(45.0,),
                      connector_radii_um=(50.0,))
    chip = Box3D((-100.0, -100.0, -100.0), (300.0, 300.0, 200.0))
    sc = Scene("validator_demo", chip, [Box3D((60.0, -100.0, -100.0), (70.0, 100.0, 20.0))],
               [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = route(sc, mode="native3d")
    assert res.status == "success"
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    return sc, res, art


def test_validator_accepts_a_real_searched_path(small_run):
    _sc, _res, art = small_run
    v = validate_artifact(art)
    assert v["verdict"] == "pass"
    assert v["c1_continuous"] is True
    assert v["radius_violations"] == []
    assert v["length_relative_error"] < 1e-9
    assert v["z_profile"]["z_varies"] is True
    assert v["endpoint_errors"]["end_position_error_um"] <= art["input"]["config"]["eps_pos_um"]


def test_validator_rejects_a_tampered_radius(small_run):
    _sc, _res, art = small_run
    art2 = json.loads(json.dumps(art))
    for seg in art2["search"]["path"]["segments"]:
        if seg["kind"] == "arc":
            seg["radius_um"] = 5.0       # far below Rmin and inconsistent with theta
            break
    v = validate_path_dict(art2["search"]["path"], art2["input"], art2["input"]["config"])
    assert v["verdict"] == "fail"
    assert v["violation_counts"]["radius_violations"] >= 1


def test_validator_rejects_a_translated_path(small_run):
    """A path moved out of the chip volume must fail the independent check."""
    _sc, _res, art = small_run
    art2 = json.loads(json.dumps(art))
    for seg in art2["search"]["path"]["segments"]:
        if seg["kind"] == "line":
            seg["p0"] = [c + 500.0 for c in seg["p0"]]
            seg["p1"] = [c + 500.0 for c in seg["p1"]]
        else:
            seg["center"] = [c + 500.0 for c in seg["center"]]
    v = validate_path_dict(art2["search"]["path"], art2["input"], art2["input"]["config"])
    assert v["verdict"] == "fail"


def test_validator_is_an_independent_implementation():
    root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "native3d", "validation", "validate.py")
    import ast
    src = open(root, encoding="utf-8").read()
    tree = ast.parse(src)
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            imported.append(("%s%s" % ("." * (node.level or 0), node.module or "")))
    assert imported, "the module should import at least the standard library"
    for mod in imported:
        assert not mod.startswith(("native3d", "..", ".")), \
            "validator must not reuse router code, found import %r" % mod
        assert mod.split(".")[0] in ("__future__", "json", "math", "sys", "os",
                                     "typing", "fractions"), mod
    # and it does implement its own curve evaluation and distance queries
    for needed in ("def eval_seg", "def point_box_distance", "def seg_seg_distance",
                   "def point_seg_distance", "def seg_seg_distance_certified",
                   "def _exact_sq_distance", "def chord_count_for_domain"):
        assert needed in src


def test_artifact_round_trip_through_json_reproduces_the_validation(small_run):
    _sc, _res, art = small_run
    text = json.dumps(art)
    art2 = json.loads(text)
    a = validate_artifact(art)
    b = validate_artifact(art2)
    assert a["verdict"] == b["verdict"] == "pass"
    assert a["min_radius_um_refit"] == pytest.approx(b["min_radius_um_refit"], rel=1e-15)
    assert a["total_length_um_analytic"] == pytest.approx(b["total_length_um_analytic"], rel=1e-15)
