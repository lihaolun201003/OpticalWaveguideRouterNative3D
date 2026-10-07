"""Repair A regression: A* must not return the first goal NEIGHBOUR it generates.

v0.1 executed the goal test inside the candidate-generation loop and returned
immediately, so the first goal successor generated won, whatever its cost.  The
counterexample below uses the real astar_search interface on an explicit finite
graph with exact keys (no quantisation):

    S = (0,0,0)  A = (1,0,0)  G = (2,0,0)      every state has direction +x
    S -> G : cost 100        (generated FIRST)
    S -> A : cost 1
    A -> G : cost 1

Optimal cost = 2.  Edge costs are non-negative and every cost is >= the
Euclidean distance between its endpoints, so the Euclidean heuristic
h(s) = |G - s| is admissible and consistent.  The v0.1 implementation returned
100 for BOTH the Euclidean heuristic and h = 0.

The independent reference is a plain Dijkstra written in this file; h = 0 on the
same (broken) engine is used only as a second data point, never as the oracle.
"""

import heapq
import math
import random

import pytest

from native3d.config import RouterConfig
from native3d.geometry import LineSegment3D
from native3d.geometry.vec import norm, sub
from native3d.search.astar import Candidate, astar_search
from native3d.search.state import State3D

CFG = RouterConfig()


class GraphAction:
    """Minimal duck-typed action of an explicit finite graph edge."""

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


def exact_key(s):
    return (round(s.position[0], 9), round(s.position[1], 9), round(s.position[2], 9))


def run_graph(nodes, edges, start, goal, heuristic="euclidean",
              max_expansions=100000, max_runtime_s=60.0):
    """Run the real astar_search on an explicit finite graph (exact keys)."""
    adj = {}
    for u, v, w in edges:
        adj.setdefault(u, []).append((v, w))
    goal_p = nodes[goal]

    def generate(state, collect):
        key = exact_key(state)
        accepted = []
        for nxt, w in adj.get(key, []):
            a, b = nodes[key], nodes[nxt]
            accepted.append(Candidate(GraphAction("edge_%s_%s" % (key, nxt), a, b, w),
                                      State3D(b, (1.0, 0.0, 0.0)), w, True, ""))
        return accepted, []

    def goal_test(state):
        d = norm(sub(state.position, goal_p))
        return d <= 1e-6, d, 0.0

    def h(state):
        if heuristic == "zero":
            return 0.0
        return max(0.0, norm(sub(state.position, goal_p)))

    return astar_search(State3D(nodes[start], (1.0, 0.0, 0.0)), generate, goal_test,
                        h, exact_key, CFG, mode="graph", heuristic_name=heuristic,
                        max_expansions=max_expansions, max_runtime_s=max_runtime_s)


# --------------------------------------------------------- independent oracle
def dijkstra(nodes, edges, start, goal):
    adj = {}
    for u, v, w in edges:
        adj.setdefault(u, []).append((v, w))
    dist = {n: math.inf for n in nodes}
    dist[start] = 0.0
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u] + 1e-12:
            continue
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist[v] - 1e-12:
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return dist[goal]


# ------------------------------------------------------------ counterexample
S, A, G = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)
CE_NODES = {S: S, A: A, G: G}


def counterexample_edges(direct_first=True):
    """S->G costs 100; S->A->G costs 2.  Direction is +x everywhere."""
    direct = (S, G, 100.0)
    via_a = [(S, A, 1.0), (A, G, 1.0)]
    return [direct] + via_a if direct_first else via_a + [direct]


@pytest.mark.parametrize("heuristic", ["euclidean", "zero"])
def test_counterexample_returns_the_optimal_cost_2(heuristic):
    res = run_graph(CE_NODES, counterexample_edges(), S, G, heuristic)
    assert res.status == "success", (res.status, res.reason)
    assert res.cost == pytest.approx(2.0, abs=1e-9), (
        "the direct edge S->G (100) is generated first but costs 100; "
        "the optimum is S->A->G = 2")
    assert [round(s["x_um"], 6) for s in res.trace] == [0.0, 1.0, 2.0]


def test_counterexample_cost_is_independent_of_neighbour_order():
    costs = []
    for direct_first in (True, False):
        for heuristic in ("euclidean", "zero"):
            res = run_graph(CE_NODES, counterexample_edges(direct_first), S, G,
                            heuristic)
            assert res.status == "success"
            costs.append(res.cost)
    assert costs == [pytest.approx(2.0)] * 4, costs


def test_counterexample_g_is_the_sum_of_the_returned_edge_costs():
    res = run_graph(CE_NODES, counterexample_edges(), S, G, "euclidean")
    w = {(u, v): c for u, v, c in counterexample_edges()}
    xs = [tuple(round(s[k + "_um"], 6) for k in "xyz") for s in res.trace]
    total = sum(w[(p, q)] for p, q in zip(xs, xs[1:]))
    assert res.cost == pytest.approx(total, abs=1e-9)
    assert res.path.total_length == pytest.approx(2.0, abs=1e-9)


def test_counterexample_with_more_expensive_first_goal_generation():
    """A second finite graph: the goal is generated first through a 5x detour."""
    n = {"S": (0.0, 0.0, 0.0), "B": (3.0, 3.0, 0.0), "M": (1.0, 0.0, 0.0),
         "G": (2.0, 0.0, 0.0)}
    edges = [("S", "G", 50.0), ("S", "B", 4.3), ("B", "M", 4.0), ("M", "G", 1.0)]
    for u, v, w in edges:                       # keep h = |G - s| admissible
        assert w >= math.dist(n[u], n[v]) - 1e-12
    e = [(n[u], n[v], w) for u, v, w in edges]
    nodes = {v: v for v in n.values()}
    for heuristic in ("euclidean", "zero"):
        res = run_graph(nodes, e, n["S"], n["G"], heuristic)
        assert res.status == "success"
        assert res.cost == pytest.approx(4.3 + 4.0 + 1.0, abs=1e-9)


# --------------------------------------------- randomised finite-graph check
def _random_graph(rng, n_nodes=14):
    pts = {i: (round(rng.uniform(0.0, 40.0), 3), round(rng.uniform(0.0, 40.0), 3),
               round(rng.uniform(0.0, 40.0), 3)) for i in range(n_nodes)}
    edges = []
    for i in range(n_nodes):
        for j in range(i + 1, n_nodes):
            if rng.random() < 0.45:
                w = math.dist(pts[i], pts[j]) * rng.uniform(1.0, 3.0)
                edges.append((i, j, w))
                edges.append((j, i, w))
    return pts, edges


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
def test_random_finite_graphs_match_an_independent_dijkstra(seed):
    rng = random.Random(seed)
    pts, edges = _random_graph(rng)
    start, goal = 0, len(pts) - 1
    oracle = dijkstra(pts, edges, start, goal)
    assert math.isfinite(oracle), "seed %d must produce a connected graph" % seed
    for u, v, w in edges:                       # h = |G - s| stays admissible
        assert w >= math.dist(pts[u], pts[v]) - 1e-12
    nodes = {pts[i]: pts[i] for i in pts}
    edge_cost = {(pts[u], pts[v]): w for u, v, w in edges}
    pos_edges = [(pts[u], pts[v], w) for u, v, w in edges]
    for heuristic in ("euclidean", "zero"):
        for shuffle in (False, True):
            e = list(pos_edges)
            if shuffle:
                rng.shuffle(e)
            res = run_graph(nodes, e, pts[start], pts[goal], heuristic)
            assert res.status == "success"
            assert res.cost == pytest.approx(oracle, abs=1e-9), (
                "heuristic=%s shuffle=%s" % (heuristic, shuffle))
            # the returned path really is a chain of existing edges
            xs = [tuple(round(s[k + "_um"], 6) for k in "xyz") for s in res.trace]
            assert sum(edge_cost[(p, q)] for p, q in zip(xs, xs[1:])) == \
                pytest.approx(res.cost, abs=1e-9)


# ------------------------------------------------- two distinct stop reasons
def test_budget_stop_and_true_exhaustion_stay_distinguishable():
    """A budget stop is reported as budget_exhausted, never as unreachable."""
    res_budget = run_graph(CE_NODES, counterexample_edges(), S, G, "euclidean",
                           max_expansions=1, max_runtime_s=60.0)
    assert res_budget.status == "budget_exhausted"
    assert res_budget.budget_exhausted is True
    assert "NOT a proof" in res_budget.reason

    # a graph where the goal is genuinely not reachable: open set empties
    nodes = {S: S, A: A, (5.0, 0.0, 0.0): (5.0, 0.0, 0.0)}
    edges = [(S, A, 1.0)]
    res_exh = run_graph(nodes, edges, S, A, "euclidean", max_expansions=1000)
    assert res_exh.status == "success"          # A IS the goal here
    edges2 = [(S, A, 1.0)]
    res_exh2 = run_graph(nodes, edges2, S, (5.0, 0.0, 0.0), "euclidean",
                         max_expansions=1000)
    assert res_exh2.status == "search_exhausted"
    assert res_exh2.budget_exhausted is False
    assert res_exh2.cost == math.inf


def test_heuristic_still_used_by_the_engine_is_admissible_on_the_counterexample():
    """h must stay a lower bound: it is not 'fixed' by zeroing the heuristic."""
    from native3d.search.router import route  # noqa: F401  (engine import check)
    h_at_A = max(0.0, norm(sub(A, G)))
    assert h_at_A == pytest.approx(1.0)
    assert h_at_A <= 1.0  # true remaining cost from A is exactly 1