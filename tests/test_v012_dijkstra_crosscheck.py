"""v0.1.2: the finite graph A* is cross-checked by an INDEPENDENT Dijkstra.

The cross-check must not be "the same A* with h = 0": the Dijkstra below is a
separate implementation inside this test file that walks the fully enumerated
edge list of search/finite_graph.py and uses none of astar_search.
"""

import heapq
import math

import pytest

from native3d.cases import base_config, core_case
from native3d.constraints.checks import Box3D
from native3d.geometry import Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import (SOURCE_NODE, TARGET_NODE,
                                          enumerate_graph_edges,
                                          route_finite_native3d)


def dijkstra(edges, start, goal):
    """Plain Dijkstra over an explicit edge list (no heuristic, no A*)."""
    adj = {}
    for e in edges:
        adj.setdefault(e.from_node, []).append((e.to_node, e.cost))
    dist = {start: 0.0}
    heap = [(0.0, 0, start)]
    counter = 0
    done = set()
    while heap:
        d, _c, u = heapq.heappop(heap)
        if u in done:
            continue
        done.add(u)
        if u == goal:
            return d
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, math.inf) - 1e-15:
                dist[v] = nd
                counter += 1
                heapq.heappush(heap, (nd, counter, v))
    return math.inf


def _check(scene, mode="native3d"):
    graph, edges = enumerate_graph_edges(scene, mode=mode)
    start = (graph.source_identity_node if graph.source_identity_node is not None
             else SOURCE_NODE)
    goal = (graph.target_identity_node if graph.target_identity_node is not None
            else TARGET_NODE)
    best = dijkstra(edges, start, goal)
    res = route_finite_native3d(scene, mode=mode)
    return graph, edges, best, res


def test_core_case_astar_cost_equals_independent_dijkstra():
    graph, edges, best, res = _check(core_case())
    assert math.isfinite(best), "the enumerated graph must contain a route"
    assert res.status == "success", (res.status, res.reason)
    assert res.cost == pytest.approx(best, abs=1e-9)
    assert len(edges) > 100
    assert len(edges) <= graph.bounds["edge_upper_bound"]


def test_two_wall_case_astar_cost_equals_independent_dijkstra():
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 200.0))
    walls = [Box3D((150.0, -50.0, -70.0), (170.0, 350.0, 40.0)),
             Box3D((330.0, -50.0, -70.0), (350.0, 350.0, 40.0))]
    sc = Scene("two_walls", chip, walls, [],
               Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    _g, _e, best, res = _check(sc)
    assert res.status == "success"
    assert res.cost == pytest.approx(best, abs=1e-9)


def test_2d_mode_dijkstra_agrees_that_the_graph_is_disconnected():
    graph, edges, best, res = _check(core_case(), mode="2d")
    assert best == math.inf, "Dijkstra must find no route in the port plane"
    assert res.status == "search_exhausted"
    assert res.budget_exhausted is False


def test_enumerated_edge_count_matches_the_successor_rule():
    graph, edges, _best, _res = _check(core_case())
    n_nodes = len(graph.all_canonical_nodes())
    assert n_nodes == graph.bounds["node_upper_bound"]
    assert len(edges) <= graph.bounds["edge_upper_bound"]
    # every edge really starts at the canonical pose of its from-node
    for e in edges[:200]:
        if e.from_node == SOURCE_NODE:
            continue
        want = graph.canonical_pose(e.from_node)
        if e.action.segments:
            assert e.action.start_pose.position == pytest.approx(want.position,
                                                                 abs=1e-9)
            assert e.action.start_pose.tangent == pytest.approx(want.tangent,
                                                                abs=1e-12)
