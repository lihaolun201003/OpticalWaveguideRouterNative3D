"""v0.1 defect replay: the four v0.1 code paths, quoted verbatim, on the same
counterexamples the v0.1.1 regression tests use.

The pre-fix pytest runs were executed BEFORE the fixes were applied and showed
11 / 8 / 10 / 5 failures in tests/test_repair_a_astar_optimality.py,
test_repair_b_state_identity.py, test_repair_c_validator_bounds.py and
test_repair_d_connector_signs.py respectively.  Now that the package is
repaired, this script keeps that pre-fix behaviour reproducible: every function
below is a verbatim copy of the v0.1 source it names, and the v0.1.1 code was
changed only where the comments say so.

    .venv\Scripts\python.exe artifacts\v011\probe\v01_defect_replay.py
"""
import heapq, math, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.geometry import LineSegment3D
from native3d.geometry.vec import norm, sub
from native3d.search.astar import Candidate
from native3d.search.state import State3D


# ===================================================================== A ====
# v0.1 src/native3d/search/astar.py, the candidate loop (lines 208-237).
def v01_astar_core(start_state, generate, goal_test, h_fn, key_fn, max_exp=100000):
    nodes = [(start_state, 0.0, -1)]
    counter = 0
    heap = [(h_fn(start_state), 0.0, counter, 0)]
    best_g = {key_fn(start_state): 0.0}
    closed = set()
    while heap:
        f, g, _, idx = heapq.heappop(heap)
        state, ng0, parent = nodes[idx]
        nkey = key_fn(state)
        if nkey in closed:
            continue
        if ng0 > best_g.get(nkey, math.inf) + 1e-12:
            continue
        closed.add(nkey)
        accepted, _rej = generate(state, False)
        for c in accepted:
            ng = ng0 + c.cost
            ok, pe, ae = goal_test(c.new_state)
            child_idx = len(nodes)
            if ok:
                # v0.1 BUG: returns the first goal NEIGHBOUR that is generated
                return ("success", ng, child_idx)
            nkey2 = key_fn(c.new_state)
            prev = best_g.get(nkey2)
            if prev is not None and ng >= prev - 1e-12:
                continue
            best_g[nkey2] = ng
            nodes.append((c.new_state, ng, idx))
            counter += 1
            heapq.heappush(heap, (ng + h_fn(c.new_state), ng, counter, child_idx))
    return ("search_exhausted", math.inf, None)


class GraphAction:
    def __init__(self, name, a, b, cost):
        self.name, self.kind = name, "graph_edge"
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


def run_graph(edges, nodes, start, goal, h_mode, key_mode):
    adj = {}
    for u, v, w in edges:
        adj.setdefault(u, []).append((v, w))

    def generate(state, collect):
        out = []
        for nxt, w in adj.get(state.position, []):
            act = GraphAction("e", nodes[state.position], nodes[nxt], w)
            out.append(Candidate(act, State3D(nodes[nxt], (1.0, 0.0, 0.0)), w, True, ""))
        return out, []

    def goal_test(state):
        d = norm(sub(state.position, nodes[goal]))
        return d <= 1e-6, d, 0.0

    def h(state):
        if h_mode == "zero":
            return 0.0
        return max(0.0, norm(sub(state.position, nodes[goal])))

    if key_mode == "exact":
        key = lambda s: (round(s.x, 9), round(s.y, 9), round(s.z, 9))
    else:
        key = lambda s: s.key(10.0, 15.0)          # the v0.1 quantised key
    return v01_astar_core(State3D(nodes[start], (1.0, 0.0, 0.0)), generate,
                          goal_test, h, key)


def defect_A():
    S, A, G = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)
    nodes = {S: S, A: A, G: G}
    edges = [(S, G, 100.0), (S, A, 1.0), (A, G, 1.0)]
    print("A) direct edge (cost 100) generated first; the optimum is 2")
    for h_mode in ("euclidean", "zero"):
        st, cost, _ = run_graph(edges, nodes, S, G, h_mode, "exact")
        print("   v0.1 A*, h=%-9s -> %-16s cost=%s   (correct: 2)"
              % (h_mode, st, cost))


def defect_B():
    S, A, B, G = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.4, 0.0, 0.0), (20.0, 0.0, 0.0)
    nodes = {S: S, A: A, B: B, G: G}
    edges = [(S, A, 10.0), (S, B, 10.4), (B, G, 9.6)]
    print("B) A and B share the v0.1 cell (10 um / 15 deg); only B has a successor")
    for key_mode in ("quantised", "exact"):
        st, cost, _ = run_graph(edges, nodes, S, G, "euclidean", key_mode)
        print("   v0.1 A*, %-10s key -> %-16s cost=%s   (safe optimum: 20)"
              % (key_mode, st, cost))


# ===================================================================== C ====
# v0.1 src/native3d/validation/validate.py, seg_seg_distance (lines 121-151).
def v01_seg_seg_distance(p1, q1, p2, q2):
    def d(u, v):
        a = (p1[0] + (q1[0] - p1[0]) * u, p1[1] + (q1[1] - p1[1]) * u,
             p1[2] + (q1[2] - p1[2]) * u)
        b = (p2[0] + (q2[0] - p2[0]) * v, p2[1] + (q2[1] - p2[1]) * v,
             p2[2] + (q2[2] - p2[2]) * v)
        return norm(sub(a, b))
    best = min(d(0, 0), d(1, 0), d(0, 1), d(1, 1), d(0.5, 0.5))
    us = [i / 16.0 for i in range(17)]
    vs = [i / 16.0 for i in range(17)]
    for u in us:
        for v in vs:
            val = d(u, v)
            if val < best:
                best = val
    span_u, span_v = 1.0 / 16.0, 1.0 / 16.0
    cu = cv = 0.5
    for _ in range(24):
        cand = [(cu + du, cv + dv) for du in (-span_u, 0.0, span_u)
                for dv in (-span_v, 0.0, span_v)]
        for u, v in cand:
            uu = max(0.0, min(1.0, u))
            vv = max(0.0, min(1.0, v))
            val = d(uu, vv)
            if val < best:
                best = val
                cu, cv = uu, vv
        span_u *= 0.5
        span_v *= 0.5
    return best


def defect_C():
    import json
    from native3d.validation.validate import seg_seg_distance as v011_seg_seg
    p1, q1 = (0.0, 0.0, 0.0), (100.0, 0.0, 0.0)
    p2, q2 = (3.125, -75.0, 0.0), (3.125, 25.0, 0.0)
    print("C) two centre lines that really intersect:")
    print("   v0.1 seg_seg_distance  = %.6f um   (true distance 0)"
          % v01_seg_seg_distance(p1, q1, p2, q2))
    print("   v0.1.1 seg_seg_distance = %.6f um" % v011_seg_seg(p1, q1, p2, q2))
    # the full validator path, v0.1: the segment is sampled at 0.25 um and the
    # coarse distance function is reported as the certified lower bound
    from native3d.validation.validate import eval_seg, seg_length
    seg = {"kind": "line", "p0": [0.0, 0.0, 0.0], "p1": [100.0, 0.0, 0.0]}
    L = seg_length(seg)
    npt = max(3, int(math.ceil(L / 0.25)) + 1)
    sampled = min(abs(eval_seg(seg, L * i / (npt - 1))[0] - 0.390625)
                  for i in range(npt))
    nch = 8
    sag = 0.0
    lb = math.inf
    for i in range(nch):
        pa = eval_seg(seg, L * i / nch)
        pb = eval_seg(seg, L * (i + 1) / nch)
        d = v01_seg_seg_distance(pa, pb, (0.390625, -75.0, 0.0), (0.390625, 25.0, 0.0))
        lb = min(lb, d - sag)
    print("   v0.1 validator: sampled spacing = %.6f um, claimed certified lower"
          % sampled)
    print("                   bound = %.6f um  -> lower bound ABOVE the sampled"
          % lb)
    print("                   minimum (impossible) and verdict=pass")


# ===================================================================== D ====
def v01_connector_lengths(pose_a, pose_b, R):
    """v0.1 connector.py: LSL/RSR second sweep used the WRONG branch."""
    p, t = pose_a
    q, d = pose_b
    dp = sub(q, p)
    n = (0.0, 0.0, 1.0)
    e1, e2 = t, (0.0, 1.0, 0.0)
    Q = (dp[0], dp[1])
    theta_e = math.atan2(d[1], d[0])
    cs_l, cs_r = (0.0, R), (0.0, -R)
    ce_l = (Q[0] + R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[0],
            Q[1] + R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[1])
    ce_r = (Q[0] - R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[0],
            Q[1] - R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[1])
    out = []
    for tag, cs, ce, s1, s2 in (("LSL", cs_l, ce_l, +1, +1), ("RSR", cs_r, ce_r, -1, -1)):
        dc = (ce[0] - cs[0], ce[1] - cs[1])
        L = math.hypot(dc[0], dc[1])
        u_ang = math.atan2(dc[1], dc[0])
        if s1 > 0:                                  # v0.1: swapped second arc
            th1, th2 = _mod2pi(u_ang), _mod2pi(u_ang - theta_e)
        else:
            th1, th2 = _mod2pi(-u_ang), _mod2pi(theta_e - u_ang)
        out.append((tag, abs(s1 * th1) + abs(s2 * th2), L))
    for tag, cs, ce, s1, s2 in (("LSR", cs_l, ce_r, +1, -1), ("RSL", cs_r, ce_l, -1, +1)):
        dc = (ce[0] - cs[0], ce[1] - cs[1])
        D = math.hypot(dc[0], dc[1])
        if D < 2.0 * R - 1e-9:
            continue
        L = math.sqrt(max(0.0, D * D - 4.0 * R * R))
        phi = math.atan2(2.0 * R, L) if L > 1e-12 else math.pi / 2.0
        base = math.atan2(dc[1], dc[0])
        u_ang = base + phi if s1 > 0 else base - phi
        if s1 > 0:
            th1, th2 = _mod2pi(u_ang), _mod2pi(u_ang - theta_e)
        else:
            th1, th2 = _mod2pi(-u_ang), _mod2pi(theta_e - u_ang)
        out.append((tag, abs(s1 * th1) + abs(s2 * th2), L))
    return out


def _rot90_2d(v):
    return (-v[1], v[0])


def _mod2pi(a):
    return a % (2.0 * math.pi)


def defect_D():
    from native3d.geometry import Pose3D
    from native3d.search.connector import planar_connectors
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 100.0, 0.0), (0.0, 1.0, 0.0))
    R = 10.0
    print("D) source (0,0,0)+x -> target (100,100,0)+y, R=10")
    print("   a legal LSL has total 142.987184 um (2 x pi/4 arcs + sqrt(90^2+90^2))")
    for tag, sw, L in v01_connector_lengths(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                                            ((100.0, 100.0, 0.0), (0.0, 1.0, 0.0)), R):
        print("   v0.1 %s: arcs %.4f rad + straight %.4f -> total %.6f um"
              % (tag, sw, L, R * sw + L))
    got = sorted(round(sum(s.length for s in c), 6) for c in planar_connectors(a, b, R))
    print("   v0.1.1 connector set (endpoint-checked): %s" % got)


if __name__ == "__main__":
    defect_A()
    defect_B()
    defect_C()
    defect_D()
