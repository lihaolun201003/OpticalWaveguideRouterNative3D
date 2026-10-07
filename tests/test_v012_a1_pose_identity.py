"""v0.1.2 repair A1 regression: the default state identity must be the stored pose.

v0.1.1 kept State3D.key_exact(), which rounds the whole pose (position AND
direction) to 9 decimals.  Two poses 2e-10 um apart therefore share a key, the
search prunes the more expensive one, and if only the pruned pose had a viable
successor the graph is declared exhausted.  The tolerance is 10 orders of
magnitude below Rmin, which says nothing about the successors of two poses in
one bucket: the successor set is a function of the exact pose.

Counterexample actually run (real astar_search, Euclidean heuristic, explicit
finite graph):

    S = (0,0,0)   A = (10,0,0)   B = (10.0000000002,0,0)   G = (20,0,0)
    direction = +x everywhere
    S -> A : 10                A has no successor
    S -> B : 10.0000000002     B -> G : 9.9999999998

    v0.1.1 key_exact()          -> search_exhausted, cost = inf, expansions = 2
    stored position+direction   -> success,         cost = 20,  expansions = 3

The v0.1.1 identity is replaced by State3D.pose_key(): the actual stored float
triple pair, with no rounding anywhere.  The old 10 / 10.4 counterexample of
v0.1.1 is kept as well.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.geometry import LineSegment3D
from native3d.geometry.vec import norm, sub
from native3d.search import astar as astar_mod
from native3d.search import state as state_mod
from native3d.search.astar import Candidate, astar_search
from native3d.search.state import State3D

S, A, B, G = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0000000002, 0.0, 0.0), (20.0, 0.0, 0.0)
X = (1.0, 0.0, 0.0)
MICRO_EDGES = {S: [(A, 10.0), (B, 10.0000000002)], B: [(G, 9.9999999998)]}
MICRO_NODES = {S: S, A: A, B: B, G: G}

# the v0.1.1 counterexample, kept verbatim
S2, A2, B2, G2 = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.4, 0.0, 0.0), (20.0, 0.0, 0.0)
OLD_EDGES = {S2: [(A2, 10.0), (B2, 10.4)], B2: [(G2, 9.6)]}
OLD_NODES = {S2: S2, A2: A2, B2: B2, G2: G2}


class GraphAction:
    """Duck-typed action of an explicit finite graph edge."""

    def __init__(self, a, b, cost):
        self.name = "edge"
        self.kind = "graph_edge"
        self.segments = (LineSegment3D(a, b),)
        self._cost = float(cost)

    @property
    def length(self):
        return self.segments[0].length

    @property
    def z_change_um(self):
        return 0.0

    @property
    def elevation_change_deg(self):
        return 0.0


def run_graph(edges, nodes, start, goal, key_fn, max_expansions=10000):
    def generate(state, collect):
        acc, rej = [], []
        for nxt, w in edges.get(state.position, []):
            acc.append(Candidate(GraphAction(nodes[state.position], nodes[nxt], w),
                                 State3D(nodes[nxt], X), w, True, ""))
        return acc, rej

    def goal_test(state):
        d = norm(sub(state.position, nodes[goal]))
        return d <= 1e-9, d, 0.0

    def h(state):
        return norm(sub(state.position, nodes[goal]))

    return astar_search(State3D(nodes[start], X), generate, goal_test, h, key_fn,
                        RouterConfig(), mode="graph", heuristic_name="euclidean",
                        max_expansions=max_expansions, max_runtime_s=30.0)


def stored_pose_key(state):
    return state.pose_key()


# --------------------------------------------------------------- the defect
def test_rounding_to_nine_decimals_really_merges_the_micro_difference_poses():
    """The counterexample is only meaningful if the v0.1.1 identity merges them."""
    assert round(10.0000000002, 9) == round(10.0, 9) == 10.0
    assert norm(sub(A, B)) > 1e-10, "the two poses really are different poses"


def test_default_identity_is_not_a_rounded_pose():
    cfg = RouterConfig()
    assert cfg.state_identity == "pose_float", (
        "the default identity must be named for what it is (the stored float "
        "pose), not exact with a 1e-9 rounding inside")
    assert "approximate_quantised" in state_mod.STATE_IDENTITIES
    ka = State3D(A, X).pose_key()
    kb = State3D(B, X).pose_key()
    assert ka != kb, "two different stored poses must never share the identity"


def test_pose_key_is_exactly_the_stored_pose():
    st = State3D((1.0, 2.0, 3.0), (1.0, 1.0, 0.0))
    assert tuple(st.pose_key()[0]) == st.position
    assert tuple(st.pose_key()[1]) == st.direction


def test_micro_difference_counterexample_returns_cost_20():
    res = run_graph(MICRO_EDGES, MICRO_NODES, S, G, stored_pose_key)
    assert res.status == "success", (res.status, res.reason)
    assert res.cost == pytest.approx(20.0, abs=1e-9)
    assert res.expansions == 3
    assert [round(s["x_um"], 10) for s in res.trace] == [0.0, 10.0000000002, 20.0]


def test_the_original_10_versus_10_4_counterexample_is_kept():
    res = run_graph(OLD_EDGES, OLD_NODES, S2, G2, stored_pose_key)
    assert res.status == "success"
    assert res.cost == pytest.approx(20.0, abs=1e-9)
    assert [round(s["x_um"], 6) for s in res.trace] == [0.0, 10.4, 20.0]


def test_no_rounding_helper_survives_under_an_exact_sounding_name():
    assert not hasattr(State3D((0.0, 0.0, 0.0), X), "key_exact"), \
        "the rounded identity was removed, not renamed"
    assert not hasattr(state_mod, "EXACT_KEY_DECIMALS")


def test_the_retracted_claims_are_gone_from_the_sources():
    import os
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    banned = ("Every distinct real pose", "pruning on this key never merges",
              "SAFE identity")
    for name in ("src/native3d/search/state.py", "src/native3d/config.py",
                 "src/native3d/search/router.py", "README.md"):
        text = open(os.path.join(root, name), encoding="utf-8").read()
        for phrase in banned:
            assert phrase not in text, "%s still claims %r" % (name, phrase)


# ------------------------------------------------- the numeric comparison rule
def test_cost_comparison_tolerance_is_documented_not_implicit():
    assert astar_mod.G_COMPARISON_ABS_TOL_UM > 0.0
    assert astar_mod.G_COMPARISON_REL_TOL > 0.0
    doc = astar_mod.__doc__ or ""
    assert "tolerance" in doc.lower()
    assert astar_mod.g_improves(1.0, 1.0) is False
    assert astar_mod.g_improves(1.0 - 1e-3, 1.0) is True
    # a difference far below the documented tolerance is NOT an improvement
    assert astar_mod.g_improves(1.0 - 1e-15, 1.0) is False


def test_pose_identity_is_not_sold_as_continuous_space_completeness():
    doc = state_mod.__doc__ or ""
    low = doc.lower()
    assert "complete" in low and "not" in low
    assert "infinite" in low, \
        "the enumerated pose graph is generally infinite and must say so"
