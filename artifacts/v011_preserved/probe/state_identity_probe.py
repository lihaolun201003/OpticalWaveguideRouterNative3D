"""Probe (throwaway): how expensive is the SAFE exact-pose state identity?

Runs the core acceptance case with three state-identity strategies and a
*corrected* A* termination (goal on pop, stale skip, reopen) so the measurement
is not an artefact of the v0.1 early-return bug.
"""
import heapq, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.cases import core_case
from native3d.constraints.checks import validate_segments
from native3d.geometry.vec import angle_between, norm, sub
from native3d.search.actions import generate_actions, primitive_is_in_plane
from native3d.search.astar import Candidate, Node, _build_path, _cand_dict
from native3d.search.connector import connector_actions
from native3d.search.congestion import ZeroCongestion
from native3d.search.router import pose_invalid_reason
from native3d.search.state import State3D


def run(scene, mode="native3d", key_fn=None, max_exp=200000, max_rt=120.0):
    cfg = scene.config
    allow = (mode == "native3d")
    field = ZeroCongestion()
    z0 = scene.source.position[2]

    def generate(state, collect):
        pose = state.pose
        actions = list(generate_actions(pose, cfg, allow))
        actions.extend(connector_actions(pose, scene.target, cfg, allow))
        accepted, rejected = [], []
        for action in actions:
            if not allow and not primitive_is_in_plane(action, z0, 1e-9):
                continue
            val = validate_segments(action.segments, scene.chip, scene.obstacles,
                                    scene.reference_centerlines, cfg, detailed=False)
            new_state = State3D(action.end_pose.position, action.end_pose.tangent)
            if val["ok"]:
                accepted.append(Candidate(action, new_state,
                                          action.length + field.penalty_of_segments(action.segments),
                                          True, ""))
        return accepted, []

    def goal_test(state):
        pe = norm(sub(state.position, scene.target.position))
        ae = math.degrees(angle_between(state.direction, scene.target.tangent))
        return (pe <= cfg.eps_pos_um and ae <= cfg.eps_ang_deg), pe, ae

    def h_fn(state):
        return max(0.0, norm(sub(state.position, scene.target.position)) - cfg.eps_pos_um)

    start = State3D(scene.source.position, scene.source.tangent)
    t0 = time.perf_counter()
    nodes = [Node(start, 0.0, -1, "<source>", "source", 0)]
    heap = [(h_fn(start), 0.0, 0, 0)]
    best_g = {key_fn(start): 0.0}
    closed = set()
    counter = 0
    expansions = 0
    ok0, pe0, ae0 = goal_test(start)
    if ok0:
        return dict(status="success", cost=0.0, expansions=0, runtime_s=0.0, nodes=1)
    while heap:
        f, g, _, idx = heapq.heappop(heap)
        node = nodes[idx]
        k = key_fn(node.state)
        if g > best_g.get(k, math.inf) + 1e-12:
            continue
        if k in closed:
            continue
        closed.add(k)
        if goal_test(node.state)[0]:
            path = _build_path(nodes, idx)
            return dict(status="success", cost=node.g, expansions=expansions,
                        runtime_s=time.perf_counter() - t0, nodes=len(nodes),
                        length=path.total_length, depth=node.depth)
        if expansions >= max_exp or (time.perf_counter() - t0) > max_rt:
            return dict(status="budget_exhausted", cost=None, expansions=expansions,
                        runtime_s=time.perf_counter() - t0, nodes=len(nodes),
                        best_f=f)
        accepted, _ = generate(node.state, False)
        expansions += 1
        for c in accepted:
            ng = node.g + c.cost
            k2 = key_fn(c.new_state)
            prev = best_g.get(k2)
            if prev is not None and ng >= prev - 1e-12:
                continue
            best_g[k2] = ng
            closed.discard(k2)
            nodes.append(Node(c.new_state, ng, idx, c.action.name,
                              getattr(c.action, "kind", "motion"), node.depth + 1,
                              tuple(c.action.segments)))
            counter += 1
            heapq.heappush(heap, (ng + h_fn(c.new_state), ng, counter, len(nodes) - 1))
    return dict(status="search_exhausted", expansions=expansions,
                runtime_s=time.perf_counter() - t0, nodes=len(nodes))


def key_exact(s):
    return (round(s.position[0], 9), round(s.position[1], 9), round(s.position[2], 9),
            round(s.direction[0], 9), round(s.direction[1], 9), round(s.direction[2], 9))


def key_quant(cfg):
    def f(s):
        return s.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
    return f


if __name__ == "__main__":
    import json
    sc = core_case()
    cfg = sc.config
    print("config: pos_res=%g dir_res=%g max_exp=%d 2d?" % (
        cfg.pos_resolution_um, cfg.dir_resolution_deg, cfg.max_expansions))
    for label, kf, mx in (("exact-key native3d", key_exact, 200000),
                          ("quant-key native3d", key_quant(cfg), 200000)):
        r = run(sc, "native3d", kf, max_exp=mx, max_rt=120.0)
        print("%-22s %s" % (label, json.dumps(r)))
    r = run(core_case(), "2d", key_exact, max_exp=200000, max_rt=120.0)
    print("%-22s %s" % ("exact-key 2d", json.dumps(r)))
