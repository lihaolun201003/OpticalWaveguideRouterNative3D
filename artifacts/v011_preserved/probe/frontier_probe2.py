"""Probe: frontier growth with the admissible obstacle-crossing heuristic."""
import heapq, json, math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from crossing_bound import crossing_bound
import state_identity_probe as P
from native3d.cases import core_case


def make_h(scene, use_crossing=True):
    cfg = scene.config
    boxes = [(b.lo, b.hi) for b in scene.obstacles]
    clearance = cfg.effective_obstacle_clearance_um
    eps = cfg.eps_pos_um
    tgt = scene.target.position

    class B:
        pass

    wrapped = []
    for lo, hi in boxes:
        b = B(); b.lo = lo; b.hi = hi
        wrapped.append(b)

    def h(state):
        p = state.position
        base = max(0.0, math.dist(p, tgt) - eps)
        if not use_crossing or not wrapped:
            return base
        return max(base, crossing_bound(p, tgt, wrapped, clearance, eps))
    return h


def run_progress(scene, mode, key_fn, h_fn, max_rt=1200.0, report_every=2000):
    from native3d.constraints.checks import validate_segments
    from native3d.geometry.vec import angle_between, norm, sub
    from native3d.search.actions import generate_actions, primitive_is_in_plane
    from native3d.search.astar import Candidate, Node
    from native3d.search.connector import connector_actions
    from native3d.search.congestion import ZeroCongestion
    from native3d.search.state import State3D
    cfg = scene.config
    allow = (mode == "native3d")
    z0 = scene.source.position[2]

    def generate(state, collect):
        pose = state.pose
        actions = list(generate_actions(pose, cfg, allow))
        actions.extend(connector_actions(pose, scene.target, cfg, allow))
        accepted = []
        for action in actions:
            if not allow and not primitive_is_in_plane(action, z0, 1e-9):
                continue
            val = validate_segments(action.segments, scene.chip, scene.obstacles,
                                    scene.reference_centerlines, cfg, detailed=False)
            if val["ok"]:
                accepted.append(Candidate(action, State3D(action.end_pose.position,
                                                          action.end_pose.tangent),
                                          action.length, True, ""))
        return accepted, []

    def goal_test(state):
        pe = norm(sub(state.position, scene.target.position))
        ae = math.degrees(angle_between(state.direction, scene.target.tangent))
        return (pe <= cfg.eps_pos_um and ae <= cfg.eps_ang_deg), pe, ae

    start = State3D(scene.source.position, scene.source.tangent)
    t0 = time.perf_counter()
    nodes = [Node(start, 0.0, -1, "<source>", "source", 0)]
    heap = [(h_fn(start), 0.0, 0, 0)]
    best_g = {key_fn(start): 0.0}
    closed, counter, expansions = set(), 0, 0
    while heap:
        f, g, _, idx = heapq.heappop(heap)
        node = nodes[idx]
        k = key_fn(node.state)
        if g > best_g.get(k, math.inf) + 1e-12 or k in closed:
            continue
        closed.add(k)
        if goal_test(node.state)[0]:
            print("SUCCESS cost=%.6f expansions=%d nodes=%d t=%.1fs" %
                  (node.g, expansions, len(nodes), time.perf_counter() - t0), flush=True)
            return True
        if (time.perf_counter() - t0) > max_rt:
            print("STOP budget expansions=%d best_f=%.3f t=%.1fs" %
                  (expansions, f, time.perf_counter() - t0), flush=True)
            return False
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
        if expansions % report_every == 0:
            print("expansions=%6d best_f=%.3f h0=%.3f nodes=%7d t=%.1fs" %
                  (expansions, f, h_fn(start), len(nodes), time.perf_counter() - t0),
                  flush=True)
    print("EXHAUSTED expansions=%d" % expansions, flush=True)
    return False


if __name__ == "__main__":
    h = make_h(core_case(), True)
    print("h(source) with crossing bound = %.6f" % h(type("S", (), {"position": (60.0, 150.0, 0.0)})()), flush=True)
    run_progress(core_case(), "native3d", P.key_exact, h, max_rt=float(sys.argv[1]) if len(sys.argv) > 1 else 1200.0)
