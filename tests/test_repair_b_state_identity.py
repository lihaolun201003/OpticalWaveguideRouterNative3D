"""Repair B regression: the dedup key must not merge poses with different successors.

v0.1.2 kept this file and updated it to the renamed identity: the v0.1.1
key_exact() (rounded to 1e-9 um) is gone, the default is pose_key() (the stored
pose itself, no rounding), and the config value "exact" is only a read alias.
See tests/test_v012_a1_pose_identity.py for the residual defect that forced it.

v0.1 used a quantised (position, direction) key for duplicate detection while the
successors, the collision checks and the terminal connector were still computed
from the REAL pose.  Two poses in one cell are not the same state.

Counterexample (real astar_search, explicit finite graph):

    S = (0,0,0)   A = (10,0,0)   B = (10.4,0,0)   G = (20,0,0)
    direction = +x everywhere, position key resolution 10 um, direction 15 deg
    S -> A : 10      S -> B : 10.4      B -> G : 9.6      A has no successor

A and B share the v0.1 key, so the cheaper-but-dead-end A suppresses B and the
search reports 'search_exhausted'; the true optimum is 20 through B.
"""

import math

import pytest

from native3d.cases import base_config
from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.geometry import LineSegment3D, Pose3D
from native3d.geometry.vec import norm, sub
from native3d.io.caseio import Scene
from native3d.search import state as state_mod
from native3d.search.astar import Candidate, astar_search
from native3d.search.router import route
from native3d.search.state import State3D

S, A, B, G = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.4, 0.0, 0.0), (20.0, 0.0, 0.0)
NODES = {S: S, A: A, B: B, G: G}
EDGES = [(S, A, 10.0), (S, B, 10.4), (B, G, 9.6)]
V01_POS_RES, V01_DIR_RES = 10.0, 15.0


class GraphAction:
    """Duck-typed action of an explicit finite graph edge."""

    def __init__(self, name, a, b, cost):
        self.name = name
        self.kind = "graph_edge"
        self.segments = (LineSegment3D(a, b),)
        self._cost = float(cost)

    @property
    def length(self):
        return self.segments[0].length

    @property
    def z_change_um(self):
        return self.segments[0].p1[2] - self.segments[0].p0[2]

    @property
    def elevation_change_deg(self):
        return 0.0


def v01_key(s):
    """Exactly the v0.1 duplicate-detection key."""
    return s.key(V01_POS_RES, V01_DIR_RES)


def safe_key(s):
    """The v0.1.2 default state identity (v0.1.1 called it key_exact()).

    v0.1.2 renamed it to pose_key() and removed the 1e-9 rounding it used to
    contain: the identity is now the stored pose itself.  See
    tests/test_v012_a1_pose_identity.py for the residual defect that forced the
    rename (two poses 2e-10 um apart shared the rounded key).
    """
    return s.pose_key()


def run_graph(key_fn, edges=EDGES, start=S, goal=G, reject=(), max_expansions=10000):
    adj = {}
    for u, v, w in edges:
        adj.setdefault(u, []).append((v, w))
    goal_p = NODES[goal]

    def generate(state, collect):
        accepted, rejected = [], []
        for nxt, w in adj.get(state.position, []):
            a, b = NODES[state.position], NODES[nxt]
            act = GraphAction("edge_%s_%s" % (state.position, nxt), a, b, w)
            new = State3D(b, (1.0, 0.0, 0.0))
            if (state.position, nxt) in reject:
                rejected.append(Candidate(act, new, math.inf, False,
                                          "obstacle_clearance"))
            else:
                accepted.append(Candidate(act, new, w, True, ""))
        return accepted, rejected

    def goal_test(state):
        d = norm(sub(state.position, goal_p))
        return d <= 1e-6, d, 0.0

    def h(state):
        return max(0.0, norm(sub(state.position, goal_p)))

    return astar_search(State3D(NODES[start], (1.0, 0.0, 0.0)), generate, goal_test,
                        h, key_fn, RouterConfig(), mode="graph",
                        heuristic_name="euclidean", max_expansions=max_expansions,
                        max_runtime_s=30.0)


# ------------------------------------------------------------------- the defect
def test_v01_key_really_merges_the_two_counterexample_poses():
    ka = State3D(A, (1.0, 0.0, 0.0)).key(V01_POS_RES, V01_DIR_RES)
    kb = State3D(B, (1.0, 0.0, 0.0)).key(V01_POS_RES, V01_DIR_RES)
    assert ka == kb, "the counterexample is only meaningful if the v0.1 key merges them"
    # ... and the same cell also swallows a direction difference
    k0 = State3D((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)).key(V01_POS_RES, V01_DIR_RES)
    k5 = State3D((10.0, 0.0, 0.0), (math.cos(math.radians(5.0)),
                                    math.sin(math.radians(5.0)), 0.0)
                 ).key(V01_POS_RES, V01_DIR_RES)
    assert k0 == k5, "5 deg apart is inside one 15 deg direction cell"


def test_default_identity_separates_every_pose_the_v01_key_would_merge():
    poses = [State3D(A, (1.0, 0.0, 0.0)), State3D(B, (1.0, 0.0, 0.0)),
             State3D((10.0000000002, 0.0, 0.0), (1.0, 0.0, 0.0)),
             State3D((10.0, 0.0, 0.0), (math.cos(math.radians(5.0)),
                                       math.sin(math.radians(5.0)), 0.0)),
             State3D((10.0, 0.0, 0.0), (math.cos(math.radians(29.0)),
                                       math.sin(math.radians(29.0)), 0.0)),
             State3D((10.0, 0.0, 0.0), (1.0, 0.0, 0.2))]
    keys = [p.pose_key() for p in poses]
    assert len(set(keys)) == len(poses)
    # there is no merge tolerance any more: identical keys mean identical poses
    assert not hasattr(state_mod, "EXACT_KEY_POSITION_TOLERANCE_UM")
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            assert keys[i] != keys[j]
            assert norm(sub(poses[i].position, poses[j].position)) >= 0.0


def test_counterexample_returns_cost_20_in_the_safe_mode():
    res = run_graph(safe_key)
    assert res.status == "success", (res.status, res.reason)
    assert res.cost == pytest.approx(20.0, abs=1e-9)
    assert [round(s["x_um"], 6) for s in res.trace] == [0.0, 10.4, 20.0]


def test_v01_key_cannot_solve_the_counterexample_and_says_so():
    res = run_graph(v01_key)
    assert res.status == "search_exhausted"
    assert res.budget_exhausted is False
    assert res.path is None


def test_same_bucket_different_collision_feasibility():
    """A is in a bucket whose other pose is blocked; B is free.  Both must live."""
    res = run_graph(safe_key, reject={(A, G)})
    assert res.status == "success"
    assert res.cost == pytest.approx(20.0, abs=1e-9)


def test_safe_mode_is_deterministic_and_order_independent():
    for edges in (EDGES, [EDGES[1], EDGES[0], EDGES[2]], [EDGES[2], EDGES[1], EDGES[0]]):
        res = run_graph(safe_key, edges=edges)
        assert res.status == "success"
        assert res.cost == pytest.approx(20.0, abs=1e-9)


# ------------------------------------------------------- the real router level
def _small_wall_scene(**over):
    """A wall scene the PRODUCTION exact-pose search can still finish.

    The action set is the reduced one (this is a search parameter, not a
    constraint), so the correct A* - which must expand every state below the
    optimum - stays affordable: 102 expansions, all constraints unchanged.
    """
    kw = dict(max_expansions=20000, max_runtime_s=30.0,
              straight_lengths_um=(20.0, 40.0), turn_angles_deg=(45.0, 90.0),
              turn_radii_um=(50.0,), skew_axis_tilt_deg=(45.0,),
              connector_radii_um=(50.0,))
    kw.update(over)
    cfg = base_config(**kw)
    chip = Box3D((-50.0, -50.0, -50.0), (250.0, 150.0, 150.0))
    wall = Box3D((90.0, -60.0, -60.0), (110.0, 160.0, 30.0))
    return Scene("small_wall", chip, [wall], [],
                 Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)


def test_full_router_defaults_to_the_safe_identity():
    cfg = RouterConfig()
    assert cfg.state_identity == "pose_float"
    sc = _small_wall_scene(state_identity="pose_float")
    res = route(sc, mode="native3d")
    assert res.status == "success"
    assert res.state_identity == "pose_float"
    assert res.approximate_state_identity is False
    # the v0.1.1 spelling "exact" is only a READ alias: it resolves to the same
    # identity instead of silently selecting the old rounded mode
    assert RouterConfig(state_identity="exact").state_identity == "pose_float"


def test_full_router_keeps_poses_that_the_v01_key_would_discard():
    """Router-level evidence: the safe run expands many distinct bucket-mates."""
    sc = _small_wall_scene()
    cfg = sc.config
    seen = []
    res = route(sc, mode="native3d",
                expand_hook=lambda node, f, h: seen.append((node.state, node.g)))
    assert res.status == "success"
    assert len(seen) == res.expansions
    bucket_min_g = {}
    would_drop = 0
    for st, g in seen:
        b = st.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
        if b in bucket_min_g and g >= bucket_min_g[b] - 1e-12:
            would_drop += 1          # the v0.1 key discards exactly these
        bucket_min_g[b] = min(bucket_min_g.get(b, math.inf), g)
    assert len(bucket_min_g) < len(seen), "the v0.1 key really is lossy here"
    assert would_drop >= 5, (
        "the safe identity must keep materially more states than the v0.1 key; "
        "dropped would be %d of %d expansions" % (would_drop, len(seen)))


def test_approximate_mode_is_explicit_and_its_exhaustion_is_not_unreachability():
    sc = _small_wall_scene(state_identity="approximate_quantised", max_runtime_s=5.0)
    res = route(sc, mode="native3d")
    assert res.state_identity == "approximate_quantised"
    assert res.approximate_state_identity is True
    d = res.to_dict()
    assert d["state_identity"] == "approximate_quantised"
    assert "not a proof" in d["reason"].lower() or res.status != "search_exhausted"


def test_state_identity_config_is_validated():
    with pytest.raises(ValueError):
        RouterConfig(state_identity="whatever")
    with pytest.raises(ValueError):
        RouterConfig(state_identity="rounded_exact")
    assert RouterConfig.from_dict({"state_identity": "approximate_quantised"}
                                  ).state_identity == "approximate_quantised"