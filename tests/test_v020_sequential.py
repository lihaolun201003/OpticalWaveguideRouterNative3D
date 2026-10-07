"""v0.2 sequential scheduler: the main flow, the counts and the failure semantics.

The five minimum acceptance cases of prompt section 5 are exercised here on the
REAL cases (the same objects examples/run_multichannel_baseline.py runs), plus the
structural claims of prompt sections 2.1 and 3.2:

* every net is searched with its OWN freshly constructed finite graph, and the
  ``graph=`` argument is never passed;
* the congestion field is zero and the fixed occupancy is a HARD constraint;
* a post-check failure is never relabelled ``search_exhausted``;
* a failed net is never published, and the already published nets are untouched;
* ``routed`` / ``verified`` / ``committed`` are three different counts.
"""

from __future__ import annotations

import pytest

from native3d.multinet.cases_v020 import (case_barrier_3d_avoidance,
                                          case_no_solution_partial_success,
                                          case_reversed_order,
                                          case_two_parallel_straights,
                                          with_nets)
from native3d.multinet.models import (STATUS_COMMITTED,
                                      STATUS_VERIFIED_NOT_COMMITTED,
                                      MultiNetCase, NetSpec)
from native3d.multinet.sequential import joint_validation_payload, route_multinet


@pytest.fixture(scope="module")
def two_channel_runs():
    out = {}
    for c in (case_two_parallel_straights(), case_barrier_3d_avoidance(),
              case_no_solution_partial_success(), case_reversed_order()):
        out[c.name] = (c, route_multinet(c))
    return out


def _m(runs, name):
    return runs[name]


# ------------------------------------------------------------------ case (1)
def test_two_independent_straights_are_both_committed(two_channel_runs):
    case, r = _m(two_channel_runs, "two_channel_01_parallel_straights")
    c = r.counts()
    assert (c["routed_count"], c["verified_count"], c["committed_count"]) == (2, 2, 2)
    assert c["all_verified"] is True and c["partial_success"] is False
    assert r.occupancy_revision == 2
    assert r.occupancy["n_published_curves"] == 6      # 3 primitives per net
    for o in r.outcomes:
        assert o.status == STATUS_COMMITTED
        assert o.length_um == pytest.approx(450.0)
        assert o.trace_z["z_varies"] is False


# ------------------------------------------------------------------ case (2)
def test_the_barrier_forces_a_native_3d_detour(two_channel_runs):
    case, r = _m(two_channel_runs, "two_channel_02_barrier_3d_avoidance")
    a, b = r.outcomes
    assert a.status == STATUS_COMMITTED and b.status == STATUS_COMMITTED
    assert a.length_um == pytest.approx(296.0)
    assert a.trace_z["z_varies"] is False
    # the prompt geometry again: 594.159265 um, z from 0 up to 100
    assert b.length_um == pytest.approx(594.1592653589794, rel=1e-12)
    assert b.trace_z["z_varies"] is True
    assert b.trace_z["z_min_um"] == pytest.approx(0.0)
    assert b.trace_z["z_max_um"] == pytest.approx(100.0)
    assert b.self_spacing["status"] == "safe"
    assert b.inter_net_spacing["status"] == "safe"
    assert a.inter_net_spacing["n_pairs"] == 0        # first net: nothing published


def test_the_control_run_without_the_barrier_is_a_plain_straight(two_channel_runs):
    case, _ = _m(two_channel_runs, "two_channel_02_barrier_3d_avoidance")
    ctl = route_multinet(with_nets(case, ["netB"]))
    assert ctl.counts()["committed_count"] == 1
    assert ctl.outcomes[0].length_um == pytest.approx(480.0)
    assert ctl.outcomes[0].trace_z["z_varies"] is False


# ------------------------------------------------------------------ case (4)
def test_no_solution_under_the_published_occupancy_is_a_partial_success(
        two_channel_runs):
    case, r = _m(two_channel_runs, "two_channel_04_no_solution_under_occupancy")
    a, b = r.outcomes
    assert a.status == STATUS_COMMITTED
    assert b.status == "search_exhausted"
    assert b.routed_candidate is False
    assert b.failure_category == "search_exhausted"
    c = r.counts()
    assert (c["routed_count"], c["verified_count"], c["committed_count"]) == (1, 1, 1)
    assert c["all_verified"] is False and c["partial_success"] is True
    assert c["failure_categories"] == {"search_exhausted": 1}
    # A is untouched: its published geometry is exactly its own returned path
    pub = [e for e in r.occupancy["entries"] if e["net_id"] == "netA"]
    assert len(pub) == len(a.candidate_segments) == 2
    assert [e["geometry"] for e in pub] == \
        [s.to_dict() for s in a.candidate_segments]
    assert r.occupancy["immutability"]["ok"] is True
    # and B alone IS routable in the same chip: the occupancy is the cause
    ctl = route_multinet(with_nets(case, ["netB"]))
    assert ctl.counts()["committed_count"] == 1


# ------------------------------------------------------------------ case (5)
def test_reversing_the_input_order_changes_the_result(two_channel_runs):
    case2, r2 = _m(two_channel_runs, "two_channel_02_barrier_3d_avoidance")
    case5, r5 = _m(two_channel_runs, "two_channel_05_reversed_order")
    assert r2.order != r5.order
    a2, b2 = r2.outcomes
    b5, a5 = r5.outcomes
    # whoever goes SECOND pays for the detour; the numbers are simply different
    assert b2.trace_z["z_varies"] is True
    assert a5.trace_z["z_varies"] is True
    assert a2.length_um == pytest.approx(296.0)
    assert b5.length_um == pytest.approx(480.0)
    assert a5.length_um == pytest.approx(410.1592653589793, rel=1e-9)
    assert a5.length_um != pytest.approx(a2.length_um, rel=1e-6)
    assert (r2.counts()["committed_count"], r5.counts()["committed_count"]) == (2, 2)


# ------------------------------------------------------- structural guarantees
def test_every_net_uses_a_freshly_built_graph_and_never_a_prebuilt_one(
        two_channel_runs):
    for _case, r in two_channel_runs.values():
        assert r.metadata["per_net_fresh_graph"] is True
        assert r.metadata["graph_argument_never_passed"] is True
        for o in r.outcomes:
            assert o.graph_fresh is True
            assert o.graph_branch == "fresh_cold_graph"
            reuse = o.production_acceptance["graph_reuse"]
            assert reuse["caller_supplied_graph"] is False
            assert reuse["accepted"] is True
            assert reuse["graph_cache_state"] == "cold"
    assert "warm" in r.metadata["warm_graph_reuse_scope"]


def test_the_congestion_penalty_is_zero_and_the_occupancy_is_a_hard_constraint(
        two_channel_runs):
    r = two_channel_runs["two_channel_02_barrier_3d_avoidance"][1]
    assert r.metadata["congestion_field"].startswith("ZeroCongestion")
    assert r.metadata["occupancy_is_a_hard_constraint"] is True
    for o in r.outcomes:
        assert o.cost_um_equivalent == pytest.approx(o.length_um)
    b = r.outcomes[1]
    assert b.inter_net_spacing["min_certified_lower_bound_um"] >= 5.0


def test_a_post_check_failure_is_never_relabelled_as_a_search_failure(two_channel_runs):
    for _case, r in two_channel_runs.values():
        for o in r.outcomes:
            if o.routed_candidate and not o.accepted:
                assert o.search_status == "success"
                assert o.status != "search_exhausted"
                assert len(o.candidate_segments) > 0
            if o.status == "search_exhausted":
                assert o.routed_candidate is False
                assert o.candidate_segments == []
                assert o.length_um is None


def test_a_net_that_is_not_accepted_publishes_nothing(two_channel_runs):
    for _case, r in two_channel_runs.values():
        published_ids = set(r.occupancy["per_net_published_curve_counts"])
        for o in r.outcomes:
            if not o.accepted:
                assert o.net_id not in published_ids
                assert o.occupancy_revision_after == o.occupancy_revision_before
                assert o.committed_segment_ids == []
            else:
                assert o.net_id in published_ids
                assert o.occupancy_revision_after == o.occupancy_revision_before + 1


def test_verification_only_runs_commit_nothing():
    case = case_two_parallel_straights()
    r = route_multinet(case, publish_on_verify=False)
    assert r.counts()["verified_count"] == 2
    assert r.counts()["committed_count"] == 0
    assert all(o.status == STATUS_VERIFIED_NOT_COMMITTED for o in r.outcomes)
    assert r.occupancy_revision == 0
    assert r.occupancy["n_published_curves"] == 0


def test_continue_after_failure_can_be_switched_off():
    """With the flag off, a failing net ends the run; with it on, later nets still run."""
    base = case_no_solution_partial_success()
    three = MultiNetCase(
        name="three_net_stop_test", chip=base.chip, obstacles=list(base.obstacles),
        reference_centerlines=list(base.reference_centerlines), config=base.config,
        nets=list(base.nets) + [NetSpec("netC", base.nets[1].source,
                                       base.nets[1].target)])
    run_all = route_multinet(three, continue_after_failure=True)
    assert len(run_all.outcomes) == 3
    assert [o.net_id for o in run_all.outcomes] == ["netA", "netB", "netC"]
    assert run_all.outcomes[1].status == "search_exhausted"
    assert run_all.outcomes[2].status == "search_exhausted"
    assert run_all.totals["stopped_after_failure"] is False

    stopped = route_multinet(three, continue_after_failure=False)
    assert [o.net_id for o in stopped.outcomes] == ["netA", "netB"]
    assert stopped.totals["stopped_after_failure"] is True
    assert stopped.counts()["committed_count"] == 1


def test_the_joint_validation_payload_is_the_file_contract(two_channel_runs):
    case, r = two_channel_runs["two_channel_02_barrier_3d_avoidance"]
    p = joint_validation_payload(case, r)
    assert p["units"] == "um"
    assert p["channel_count"] == 2
    assert p["order"] == ["netA", "netB"]
    assert len(p["nets"]) == 2
    assert len(p["nets"][1]["segments"]) == len(r.outcomes[1].candidate_segments)
    assert p["no_optical_loss_model"].startswith("geometry only")
    # every occupation entry can be attributed
    for e in p["occupancy"]["entries"]:
        assert e["origin"] in ("static_reference_centerline", "committed_net_path")
        if e["origin"] == "committed_net_path":
            assert e["net_id"] in p["order"]
            assert e["segment_id"] is not None
            assert e["occupancy_revision"] >= 1
