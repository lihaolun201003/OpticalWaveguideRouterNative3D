"""v0.2 occupancy: what is published, when, and the immutability rules.

Prompt section 3.2: occupancy is the COMPLETE ACTUAL CURVES of a verified net, it
carries net_id / segment_id / occupancy_revision / geometry, a failed or
unverified path may never be published, and nothing may modify an already
published curve.
"""

from __future__ import annotations

import copy
import math

import pytest

from native3d.constraints.checks import Box3D
from native3d.geometry.primitives import LineSegment3D, Pose3D
from native3d.io.caseio import Scene
from native3d.multinet.models import MultiNetCase, NetSpec
from native3d.multinet.occupancy import (PUBLISHED_ORIGIN, STATIC_ORIGIN,
                                         Occupancy, OccupancyError,
                                         geometry_fingerprint, scene_for_net)
from native3d.search.finite_graph import route_finite_native3d
from v020_helpers import cfg, line


def occ_fixture():
    static = [line((150, 120, 0), (450, 120, 0))]
    return Occupancy(static, dmin_centerline_um=5.0)


def test_static_reference_centrelines_are_occupied_but_own_no_net():
    occ = occ_fixture()
    assert len(occ.static_entries()) == 1
    assert occ.static_entries()[0].origin == STATIC_ORIGIN
    assert occ.static_entries()[0].net_id is None
    assert occ.published_entries() == []
    assert occ.revision == 0


def test_publishing_stores_the_complete_curves_with_their_provenance():
    occ = occ_fixture()
    segs = (line((0, 0, 0), (100, 0, 0)), line((100, 0, 0), (200, 40, 0)))
    rev = occ.publish("netA", segs, verified=True)
    assert rev == 1 and occ.revision == 1
    entries = occ.published_entries()
    assert [e.segment_id for e in entries] == [0, 1]
    assert {e.net_id for e in entries} == {"netA"}
    assert {e.occupancy_revision for e in entries} == {1}
    assert all(e.origin == PUBLISHED_ORIGIN for e in entries)
    # the stored geometry is the curve itself, not a node / control point / AABB
    assert entries[0].geometry == segs[0].to_dict()
    assert entries[1].geometry == segs[1].to_dict()
    assert entries[0].fingerprint == geometry_fingerprint(segs[0])
    # |(100,40,0) - (100,0,0)| = sqrt(100^2 + 40^2)
    assert entries[1].length_um == pytest.approx(math.sqrt(100.0 ** 2 + 40.0 ** 2))


def test_a_failed_or_unverified_path_can_never_be_published():
    occ = occ_fixture()
    with pytest.raises(OccupancyError):
        occ.publish("netA", (line((0, 0, 0), (100, 0, 0)),), verified=False)
    assert occ.revision == 0 and occ.published_entries() == []


def test_a_net_is_never_published_twice():
    occ = occ_fixture()
    occ.publish("netA", (line((0, 0, 0), (100, 0, 0)),), verified=True)
    with pytest.raises(OccupancyError):
        occ.publish("netA", (line((0, 0, 0), (100, 0, 0)),), verified=True)
    assert occ.revision == 1


def test_an_empty_path_is_refused():
    occ = occ_fixture()
    with pytest.raises(OccupancyError):
        occ.publish("netA", (), verified=True)


def test_publication_is_monotone_and_immutable():
    occ = occ_fixture()
    occ.publish("netA", (line((0, 0, 0), (100, 0, 0)),), verified=True)
    occ.publish("netB", (line((0, 50, 0), (100, 50, 0)),), verified=True)
    assert occ.revision == 2
    assert occ.verify_immutability()["ok"] is True
    assert occ.counts()["per_net_published_curve_counts"] == {"netA": 1, "netB": 1}
    # the curves handed out are COPIES, so touching them cannot change the store
    handed_out = occ.curves()
    assert handed_out[0] is not occ._curves[0]
    object.__setattr__(handed_out[0], "p0", (999.0, 999.0, 999.0))
    assert handed_out[0].p0 == (999.0, 999.0, 999.0)
    assert occ._curves[0].p0 == (150.0, 120.0, 0.0)
    assert occ.verify_immutability()["ok"] is True
    assert occ.entries[0].geometry["p0"] == [150.0, 120.0, 0.0]


def test_a_modified_stored_curve_is_detected():
    occ = occ_fixture()
    occ.publish("netA", (line((0, 0, 0), (100, 0, 0)),), verified=True)
    # deliberate corruption of the frozen store (the dataclasses are frozen, so
    # this needs object.__setattr__ - which is exactly why the fingerprint exists)
    object.__setattr__(occ._curves[1], "p1", (100.0, 0.5, 0.0))
    rep = occ.verify_immutability()
    assert rep["ok"] is False
    assert rep["n_changed"] == 1
    assert rep["changed"][0]["net_id"] == "netA"


def _case():
    return MultiNetCase(
        name="occ_scene", chip=Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0)),
        obstacles=[], reference_centerlines=[line((150, 20, 0), (450, 20, 0))],
        config=cfg(), nets=[
            NetSpec("netA", Pose3D((60, 150, 0), (1, 0, 0)),
                    Pose3D((540, 150, 0), (1, 0, 0))),
            NetSpec("netB", Pose3D((60, 200, 0), (1, 0, 0)),
                    Pose3D((540, 200, 0), (1, 0, 0)))])


def test_scene_for_net_is_a_fresh_copy_carrying_static_plus_published():
    case = _case()
    occ = Occupancy(case.reference_centerlines, dmin_centerline_um=5.0)
    segs = (line((300, 2, 0), (300, 298, 0)),)
    occ.publish("netA", segs, verified=True)
    sc = scene_for_net(case, case.nets[1], occ)
    assert sc is not case
    assert len(sc.reference_centerlines) == 2
    assert sc.name.endswith("::netB")
    assert sc.metadata["n_static_reference_centerlines"] == 1
    assert sc.metadata["n_published_curves"] == 1
    assert sc.metadata["occupancy_revision"] == 1
    assert sc.metadata["static_reference_centerlines_are_not_a_net_path"] is True
    # deep copies: no shared mutable object with the case or the occupancy
    assert sc.chip is not case.chip
    assert sc.config is not case.config
    assert sc.reference_centerlines[0] is not case.reference_centerlines[0]
    assert sc.reference_centerlines[1] is not occ._curves[1]
    assert sc.source is not case.nets[1].source
    # and the copy really is the same geometry
    assert sc.reference_centerlines[1].to_dict() == segs[0].to_dict()


def test_the_router_really_avoids_a_published_straight_bar():
    """The hard-occupancy mechanism itself: no congestion penalty involved."""
    case = _case()
    occ = Occupancy(case.reference_centerlines, dmin_centerline_um=5.0)
    bar = (line((300.0, 2.0, 0.0), (300.0, 298.0, 0.0)),)
    occ.publish("netA", bar, verified=True)
    sc = scene_for_net(case, case.nets[1], occ)
    res = route_finite_native3d(sc, mode="native3d")
    assert res.status == "success"
    # the only way past the bar in this chip is a 3-D detour
    zs = [s.aabb()[1][2] for s in res.path.segments]
    assert max(zs) > 40.0
    assert res.congestion_cost == 0.0
