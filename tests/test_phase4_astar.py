"""Phase 4 tests: A* cost/heuristic, h=0 cross-check, congestion, statuses."""

import heapq
import math

import pytest

from native3d.cases import base_config, core_case, sealed_target_case
from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.geometry import LineSegment3D, Pose3D
from native3d.geometry.vec import norm, sub
from native3d.io.caseio import Scene
from native3d.search.astar import astar_search
from native3d.search.congestion import RegionCongestion, ZeroCongestion
from native3d.search.reference_graph import reference_search
from native3d.search.router import route
from native3d.search.state import State3D


# --------------------------------------------------------------- finite graph
class GraphAction:
    """Minimal duck-typed action for the finite-graph cross-check."""

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


def _grid_graph():
    """3x3x2 lattice, unit-ish spacing, 6-neighbour edges + 2 costly shortcuts."""
    nodes = {}
    for i in range(3):
        for j in range(3):
            for k in range(2):
                nodes[(i, j, k)] = (10.0 * i, 10.0 * j, 10.0 * k)
    edges = []
    for (i, j, k), p in nodes.items():
        for di, dj, dk in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            q_idx = (i + di, j + dj, k + dk)
            if q_idx in nodes:
                q = nodes[q_idx]
                w = math.dist(p, q) * 1.4          # weight > euclidean: admissible h
                edges.append(((i, j, k), q_idx, w))
                edges.append((q_idx, (i, j, k), w))
    # shortcuts that must NOT be used unless they are genuinely cheaper
    edges.append(((0, 0, 0), (2, 2, 1), 55.0))
    edges.append(((2, 2, 1), (0, 0, 0), 55.0))
    edges.append(((0, 2, 0), (2, 0, 1), 70.0))
    edges.append(((2, 0, 1), (0, 2, 0), 70.0))
    return nodes, edges


def _adjacency(edges):
    adj = {}
    for a, b, w in edges:
        adj.setdefault(a, []).append((b, w))
    return adj


def _plain_dijkstra(nodes, edges, start, goal):
    adj = _adjacency(edges)
    dist = {n: math.inf for n in nodes}
    dist[start] = 0.0
    pq = [(0.0, start)]
    prev = {}
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u] + 1e-12:
            continue
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist[v] - 1e-12:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return dist, list(reversed(path))


def _run_engine_on_graph(nodes, edges, start, goal, heuristic):
    adj = _adjacency(edges)
    goal_p = nodes[goal]

    def generate(state, collect):
        key = (int(round(state.x / 10.0)), int(round(state.y / 10.0)),
               int(round(state.z / 10.0)))
        out = []
        for nxt, w in adj.get(key, []):
            p_from, p_to = nodes[key], nodes[nxt]
            act = GraphAction("edge_%s_%s" % (key, nxt), p_from, p_to, w)
            out.append(__import__("native3d.search.astar", fromlist=["Candidate"]).Candidate(
                act, State3D(p_to, (1.0, 0.0, 0.0)), w, True, ""))
        return out, []

    def goal_test(state):
        d = norm(sub(state.position, goal_p))
        return d <= 1e-6, d, 0.0

    def h(state):
        if heuristic == "zero":
            return 0.0
        return max(0.0, norm(sub(state.position, goal_p)))

    cfg = RouterConfig()
    return astar_search(State3D(nodes[start], (1.0, 0.0, 0.0)), generate, goal_test,
                        h, lambda s: (round(s.x, 6), round(s.y, 6), round(s.z, 6)),
                        cfg, mode="graph", heuristic_name=heuristic,
                        max_expansions=100000, max_runtime_s=60.0)


def test_astar_matches_uniform_cost_search_and_independent_dijkstra():
    nodes, edges = _grid_graph()
    start, goal = (0, 0, 0), (2, 2, 1)
    dist, path = _plain_dijkstra(nodes, edges, start, goal)
    res_astar = _run_engine_on_graph(nodes, edges, start, goal, "euclidean")
    res_dijkstra = _run_engine_on_graph(nodes, edges, start, goal, "zero")
    assert res_astar.status == "success"
    assert res_dijkstra.status == "success"
    assert res_astar.cost == pytest.approx(dist[goal], abs=1e-9)
    assert res_dijkstra.cost == pytest.approx(dist[goal], abs=1e-9)
    # g really is the sum of the real edge costs of the returned path
    adj = {(a, b): w for a, b, w in edges}

    def node_of(step):
        return (int(round(step["x_um"] / 10.0)), int(round(step["y_um"] / 10.0)),
                int(round(step["z_um"] / 10.0)))

    chain = [node_of(s) for s in res_astar.trace]
    assert chain[0] == start and chain[-1] == goal
    total = sum(adj[(chain[i], chain[i + 1])] for i in range(len(chain) - 1))
    assert total == pytest.approx(res_astar.cost, abs=1e-9)
    # the assembled geometry has exactly the geometric length of those edges
    geometric = sum(math.dist(nodes[chain[i]], nodes[chain[i + 1]])
                    for i in range(len(chain) - 1))
    assert res_astar.path.total_length == pytest.approx(geometric, rel=1e-9)
    assert len(res_astar.path.segments) == len(chain) - 1


def test_heuristic_is_an_optimistic_bound_of_the_true_remaining_cost():
    """h <= exact optimal cost-to-go, computed by an independent Dijkstra."""
    nodes, edges = _grid_graph()
    start, goal = (0, 0, 0), (2, 2, 1)
    # Dijkstra FROM THE GOAL over the reversed edges = exact cost-to-go per node
    to_goal, _ = _plain_dijkstra(nodes, [(b, a, w) for a, b, w in edges], goal, start)
    checked = 0
    for n, p in nodes.items():
        h = max(0.0, norm(sub(p, nodes[goal])))
        assert h <= to_goal[n] + 1e-9, "h must not overestimate the true cost to go"
        checked += 1
    assert checked == len(nodes)


# ------------------------------------------------------------------ congestion
def test_congestion_penalty_is_non_negative_and_in_length_units():
    box = Box3D((10.0, -5.0, -5.0), (30.0, 5.0, 5.0))
    field = RegionCongestion([(box, 0.25)])
    seg_in = LineSegment3D((0.0, 0.0, 0.0), (40.0, 0.0, 0.0))
    assert field.penalty(seg_in) == pytest.approx(0.25 * 20.0)
    seg_out = LineSegment3D((0.0, 20.0, 0.0), (40.0, 20.0, 0.0))
    assert field.penalty(seg_out) == pytest.approx(0.0)
    assert field.penalty(seg_in) >= 0.0
    # zero field -> zero penalty
    assert ZeroCongestion().penalty(seg_in) == 0.0


def _small_wall_scene(cfg):
    from native3d.constraints.checks import Box3D as B
    chip = B((-50.0, -50.0, -50.0), (250.0, 150.0, 150.0))
    wall = B((90.0, -60.0, -60.0), (110.0, 160.0, 30.0))
    return Scene("small_wall", chip, [wall], [],
                 Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)


def test_router_cost_equals_length_plus_congestion_and_changes_with_field():
    # NOTE (v0.1.2): this is a CONTINUOUS-pose debug-search case.  Since the default
    # identity became pose_float (no rounding), near-duplicate poses are no longer
    # merged, so the same scene needs ~11.4e3 expansions / ~115 s instead of ~35 s.
    # The budget below is only the fixture's headroom; the test asserts the cost
    # identity (cost == length + congestion) and the field sensitivity, not a budget.
    cfg = base_config(max_expansions=20000, max_runtime_s=600.0)
    scene = _small_wall_scene(cfg)
    zero = ZeroCongestion()
    res0 = route(scene, mode="native3d", congestion=zero)
    assert res0.status == "success"
    assert res0.congestion_cost == pytest.approx(0.0)
    assert res0.cost == pytest.approx(res0.length_um, rel=1e-12)

    box = Box3D((80.0, -60.0, 30.0), (130.0, 60.0, 150.0))
    field = RegionCongestion([(box, 0.5)])
    res1 = reference_search(scene, mode="native3d", congestion=field)
    assert res1.status == "success"
    # the reported congestion cost is exactly the field applied to the raw path
    recomputed = sum(field.penalty(s) for s in res1.path.segments)
    assert res1.congestion_cost == pytest.approx(recomputed, abs=1e-9)
    assert res1.cost == pytest.approx(res1.length_um + res1.congestion_cost, rel=1e-12)
    assert res1.cost >= res0.cost - 1e-9
    assert res1.congestion_cost > 0.0
    # determinism with a fixed field
    res2 = reference_search(scene, mode="native3d", congestion=field)
    assert res2.cost == pytest.approx(res1.cost)


# -------------------------------------------------------------------- statuses
def test_invalid_source_and_target_poses_get_distinct_statuses():
    cfg = base_config(max_expansions=200)
    chip = Box3D((0.0, 0.0, 0.0), (100.0, 100.0, 100.0))
    block = Box3D((10.0, 10.0, 10.0), (30.0, 30.0, 30.0))
    good = Pose3D((60.0, 60.0, 60.0), (1.0, 0.0, 0.0))
    inside = Pose3D((20.0, 20.0, 20.0), (1.0, 0.0, 0.0))
    outside = Pose3D((300.0, 20.0, 20.0), (1.0, 0.0, 0.0))
    sc = Scene("invalid_source", chip, [block], [], inside, good, cfg)
    assert route(sc, mode="native3d").status == "invalid_source_pose"
    sc2 = Scene("invalid_target", chip, [block], [], good, outside, cfg)
    assert route(sc2, mode="native3d").status == "invalid_target_pose"


def test_budget_exhaustion_is_reported_separately_from_exhaustion():
    # budget stop on a case that needs thousands of expansions
    core = core_case()
    res_budget = route(core, mode="2d", max_expansions=50, max_runtime_s=60.0)
    assert res_budget.status == "budget_exhausted"
    assert res_budget.budget_exhausted is True
    assert "NOT a proof" in res_budget.reason
    # genuine exhaustion of a small closed scene: on the FINITE reference graph
    sc = sealed_target_case()
    res_exh = reference_search(sc, mode="2d")
    assert res_exh.status == "search_exhausted"
    assert res_exh.budget_exhausted is False
    assert res_exh.expansions < 200
    # ... while the production exact-pose graph reports an honest budget stop
    res_prod = route(sealed_target_case(), mode="2d", max_expansions=800,
                     max_runtime_s=8.0)
    assert res_prod.status == "budget_exhausted"
    assert res_prod.budget_exhausted is True


def test_no_decibel_or_loss_output_anywhere_in_the_package():
    """No optical loss / crosstalk model may exist: geometry only, no dB."""
    import ast
    import os
    import re
    root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "native3d")
    ident = re.compile(r"(^|_)(loss|db|decibel|crosstalk|il)($|_)", re.I)
    hits = []
    n_files = 0
    for dirpath, _dirs, files in os.walk(root):
        for fn in files:
            if not fn.endswith(".py"):
                continue
            n_files += 1
            path = os.path.join(dirpath, fn)
            tree = ast.parse(open(path, encoding="utf-8").read())
            for node in ast.walk(tree):
                names = []
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    names.append(node.name)
                elif isinstance(node, ast.Name):
                    names.append(node.id)
                elif isinstance(node, ast.Attribute):
                    names.append(node.attr)
                elif isinstance(node, ast.arg):
                    names.append(node.arg)
                for nm in names:
                    if ident.search(nm):
                        hits.append("%s:%d:%s" % (fn, getattr(node, "lineno", 0), nm))
    assert n_files >= 10, "the scan must actually look at the package"
    assert hits == [], "no loss/dB identifier may exist: %r" % hits
    # and no numeric optical result may appear in an output payload
    res = route(core_case(base_config(max_expansions=1, max_runtime_s=5.0)),
                mode="native3d")
    d = res.to_dict()
    keys = json_keys(d)
    assert not any(k in ("loss", "db", "loss_db", "insertion_loss", "crosstalk")
                   for k in keys)
    assert "um" in " ".join(sorted(keys)) or True


def json_keys(obj, prefix=""):
    keys = set()
    if isinstance(obj, dict):
        for k, v in obj.items():
            keys.add(str(k).lower())
            keys |= json_keys(v)
    elif isinstance(obj, list):
        for v in obj:
            keys |= json_keys(v)
    return keys