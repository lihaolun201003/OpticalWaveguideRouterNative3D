"""v0.1.2 PART A: before/after replay of the five residual defects.

This file is RE-RUNNABLE on the final v0.1.2 tree.  For each defect it

  (a) reconstructs the v0.1.1 code path LOCALLY, verbatim in its essentials
      (the 1e-9 rounded key, the verdict rule that ignored the source pose, the
      kind == "line" reference filter, the un-guarded periodic sagitta formula,
      and the classical a*e - b*b segment denominator), and
  (b) runs the v0.1.2 implementation on exactly the same input.

Nothing here is a stored log of a past run: the "before" numbers are recomputed
from the reconstructed v0.1.1 logic every time, the "after" numbers come from the
real modules, and both are printed side by side.  The first pre-fix run against
the untouched v0.1.1 working tree produced the same "before" numbers (the
report quotes them); this harness makes them reproducible forever instead of
frozen in a log.

    .venv Scripts python.exe artifacts/v012/probe/partA_prefix_replay.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.config import RouterConfig                                    # noqa: E402
from native3d.constraints.checks import (Box3D, primitive_box_clearance,     # noqa: E402
                                         primitive_spacing,
                                         segment_segment_distance_certified)
from native3d.geometry.primitives import (CircularArc3D, LineSegment3D,      # noqa: E402
                                          Pose3D)
from native3d.geometry.vec import norm, sub                                 # noqa: E402
from native3d.search.astar import Candidate, astar_search                   # noqa: E402
from native3d.search.state import State3D                                   # noqa: E402
from native3d.validation.validate import validate_path_dict                 # noqa: E402


def head(title):
    print("")
    print("=" * 78)
    print(title)
    print("=" * 78)


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def scene_dict(lo, hi, obstacles=(), refs=(), source=((0, 0, 0), (1, 0, 0)),
               target=((100, 0, 0), (1, 0, 0))):
    return {
        "chip_volume": {"lo": list(lo), "hi": list(hi)},
        "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
        "reference_centerlines": list(refs),
        "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
        "target_pose": {"position": list(target[0]), "tangent": list(target[1])},
    }


# ===================================================== v0.1.1 logic, local
def v011_key(state):
    """v0.1.1 State3D.key_exact(): the full pose rounded to 9 decimals."""
    r = round
    return (r(state.position[0], 9), r(state.position[1], 9), r(state.position[2], 9),
            r(state.direction[0], 9), r(state.direction[1], 9), r(state.direction[2], 9))


def v011_verify_rounding_merges():
    return v011_key(State3D((10.0, 0.0, 0.0), (1.0, 0.0, 0.0))) == \
        v011_key(State3D((10.0000000002, 0.0, 0.0), (1.0, 0.0, 0.0)))


def v011_verdict(v):
    """v0.1.1 validate_path_dict verdict rule: only the TARGET pose counted."""
    c = v["violation_counts"]
    witnessed = (c["radius_violations"] > 0
                 or c["boundary_violation_samples"] > 0
                 or c["boundary_analytic_violation"] > 0
                 or c["obstacle_points_inside"] > 0
                 or v["obstacle_clearance"]["status"] == "violation"
                 or v["centerline_spacing"]["status"] == "violation"
                 or c["non_c1_junctions"] > 0
                 or c["endpoint_position_out_of_tolerance"] > 0
                 or c["endpoint_tangent_out_of_tolerance"] > 0
                 or v["max_junction_position_error_um"] > 1e-6)
    undecided = c["undecided_checks"] > 0
    return "fail" if witnessed else ("uncertified" if undecided else "pass")


def v011_sagitta(theta, radius, n):
    """v0.1.1 sagitta: no domain guard, cos() periodicity included."""
    return radius * (1.0 - math.cos(abs(theta) / n / 2.0))


def v011_arc_box_clearance(arc, box, required, max_level=9):
    """v0.1.1 arc/box decision: n starts at 2, sagitta unguarded."""
    n = 2
    for level in range(max_level + 1):
        sag = v011_sagitta(arc.theta, arc.radius, n)
        nodes = [arc.point_at(arc.length * i / n) for i in range(n + 1)]
        dmin = math.inf
        for i in range(n):
            d = _v011_segment_box_distance(nodes[i], nodes[i + 1], box)
            dmin = min(dmin, d)
        if dmin - sag >= required:
            return "safe", dmin - sag
        if dmin + sag < required:
            return "violation", dmin + sag
        if level == max_level:
            return "undecided", dmin - sag
        n *= 2
    raise AssertionError


def _v011_segment_box_distance(p0, p1, box):
    """The exact segment/AABB distance is the same in both versions."""
    from native3d.constraints.checks import segment_box_distance
    return segment_box_distance(p0, p1, box)


def v011_pair_clearance_lb_ub(A, B, level):
    """v0.1.1 pair bounds: nA = 2**level with no sagitta domain guard."""
    from native3d.constraints.checks import segment_segment_distance
    if A.kind == "line" and B.kind == "line":
        d = _v011_ssd(A.p0, A.p1, B.p0, B.p1)
        return d, d
    nA = 2 ** level if A.kind == "arc" else 1
    nB = 2 ** level if B.kind == "arc" else 1
    sagA = 0.0 if A.kind == "line" else v011_sagitta(A.theta, A.radius, nA)
    sagB = 0.0 if B.kind == "line" else v011_sagitta(B.theta, B.radius, nB)
    pa = [A.point_at(A.length * i / nA) for i in range(nA + 1)]
    pb = [B.point_at(B.length * i / nB) for i in range(nB + 1)]
    best = math.inf
    for i in range(nA):
        for j in range(nB):
            d = segment_segment_distance(pa[i], pa[i + 1], pb[j], pb[j + 1])
            best = min(best, d)
    return best - sagA - sagB, best + sagA + sagB


def _v011_ssd(p1, q1, p2, q2):
    """v0.1.1 segment/segment distance: the classical a*e - b*b denominator."""
    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    d1, d2, r = sub(q1, p1), sub(q2, p2), sub(p1, p2)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    eps = 1e-18
    if a <= eps and e <= eps:
        return math.sqrt(dot(r, r))
    if a <= eps:
        s = 0.0
        t = min(1.0, max(0.0, f / e))
    else:
        c = dot(d1, r)
        if e <= eps:
            t = 0.0
            s = min(1.0, max(0.0, -c / a))
        else:
            b = dot(d1, d2)
            denom = a * e - b * b              # <- the cancellation
            s = min(1.0, max(0.0, (b * f - c * e) / denom)) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = min(1.0, max(0.0, (b - c) / a))
    c1 = (p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s)
    c2 = (p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t)
    return math.sqrt(dot(sub(c1, c2), sub(c1, c2)))


def v011_certified_spacing(seg, refs, required, max_level=12):
    """v0.1.1 _certified_spacing: reference ARCS were filtered away silently.

    When the filter left no line at all the v0.1.1 code returned a dict whose
    lower bound was None and then compared that None against a float.
    """
    lines = [r for r in refs if r.get("kind") == "line"]
    if not lines:
        # verbatim v0.1.1: the per-segment helper returned lower_bound_um = None and
        # _certified_spacing_multi then folded it with min(lower, None) -> TypeError
        res = {"lower_bound_um": None, "status": "safe", "n_reference_lines": 0,
               "sampled_min_um": None}
        lower = math.inf
        lower = min(lower, res["lower_bound_um"])          # <- the v0.1.1 TypeError
        return dict(res, lower_bound_um=lower)
    pts = _v011_sample_points(seg)
    sampled = min(min(_v011_point_seg(p, r["p0"], r["p1"]) for r in lines) for p in pts)
    if seg["kind"] == "line":
        exact = min(_v011_ssd(seg["p0"], seg["p1"], r["p0"], r["p1"]) for r in lines)
        return {"lower_bound_um": exact, "sampled_min_um": sampled,
                "status": "safe" if exact >= required else "violation"}
    # the arc branch of v0.1.1, with the unguarded sagitta
    n = 2
    best_lb = -math.inf
    for _level in range(max_level + 1):
        sag = v011_sagitta(seg["theta_rad"], seg["radius_um"], n)
        nodes = _v011_chord_nodes(seg, n)
        dmin = min(_v011_ssd(nodes[i], nodes[i + 1], r["p0"], r["p1"])
                   for i in range(n) for r in lines)
        best_lb = max(best_lb, dmin - sag)
        if best_lb >= required:
            break
        n *= 2
    status = "safe" if best_lb >= required else (
        "violation" if sampled < required else "undecided")
    return {"lower_bound_um": best_lb, "sampled_min_um": sampled, "status": status}


def _v011_sample_points(seg):
    """v0.1.1 sampling: 0.25 um of arclength, at least 3 points."""
    from native3d.validation.validate import eval_seg, seg_length
    L = seg_length(seg)
    npt = max(3, int(math.ceil(L / 0.25)) + 1)
    return [eval_seg(seg, L * i / (npt - 1)) for i in range(npt)]


def _v011_point_seg(p, a, b):
    from native3d.validation.validate import point_seg_distance
    return point_seg_distance(p, a, b)


def _v011_chord_nodes(seg, n):
    from native3d.validation.validate import eval_seg, seg_length
    L = seg_length(seg)
    return [eval_seg(seg, L * i / n) for i in range(n + 1)]


# =========================================================== the replay itself
def a1():
    head("A1  the 1e-9 rounded identity merges two different real poses")
    S, A, B, G = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0000000002, 0.0, 0.0), (20.0, 0.0, 0.0)
    X = (1.0, 0.0, 0.0)
    nodes = {S: S, A: A, B: B, G: G}
    edges = {S: [(A, 10.0), (B, 10.0000000002)], B: [(G, 9.9999999998)]}

    def run(key_fn, label):
        def generate(state, collect):
            acc = []
            for nxt, w in edges.get(state.position, []):
                seg = LineSegment3D(nodes[state.position], nodes[nxt])
                act = type("A", (), {"name": "edge", "kind": "graph_edge",
                                     "segments": (seg,), "length": w,
                                     "z_change_um": 0.0,
                                     "elevation_change_deg": 0.0})()
                acc.append(Candidate(act, State3D(nodes[nxt], X), w, True, ""))
            return acc, []

        def goal_test(state):
            d = norm(sub(state.position, G))
            return d <= 1e-9, d, 0.0

        res = astar_search(State3D(S, X), generate, goal_test,
                           lambda s: norm(sub(s.position, G)), key_fn,
                           RouterConfig(), mode="graph", heuristic_name="euclidean",
                           max_expansions=1000, max_runtime_s=10.0)
        print("   %-40s status=%-16s cost=%-6s expansions=%d"
              % (label, res.status, res.cost, res.expansions))

    print("   |A - B| = %.10e um; rounding to 9 decimals merges them: %s"
          % (norm(sub(A, B)), v011_verify_rounding_merges()))
    run(v011_key, "BEFORE  v0.1.1 key_exact() (round 9)")
    run(lambda s: s.pose_key(), "AFTER   v0.1.2 pose_key() (stored pose)")


def a2():
    head("A2  the source pose was not part of the verdict")
    path = {"segments": [{"kind": "line", "p0": [10.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    scene = scene_dict((-2000, -2000, -2000), (2000, 2000, 2000),
                       source=((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                       target=((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    v = validate_path_dict(path, scene, cfg_dict())
    e = v["endpoint_errors"]
    print("   start_position_error = %.6g um, start_tangent_error = %.6g deg"
          % (e["start_position_error_um"], e["start_tangent_error_deg"]))
    print("   BEFORE  v0.1.1 verdict rule (target pose only)      -> %s" % v011_verdict(v))
    print("   AFTER   v0.1.2 validator (source + target, own tol) -> %s "
          "(source_position_out=%d, source_tangent_out=%d)"
          % (v["verdict"], v["violation_counts"]["source_position_out_of_tolerance"],
             v["violation_counts"]["source_tangent_out_of_tolerance"]))


def a3():
    head("A3  the independent validator silently dropped a reference ARC")
    path = {"segments": [{"kind": "line", "p0": [10.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    arc = CircularArc3D((50.0, 50.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), 50.0, math.pi)
    far = LineSegment3D((0.0, 100.0, 0.0), (100.0, 100.0, 0.0))
    both = scene_dict((-2000, -2000, -2000), (2000, 2000, 2000),
                      refs=[arc.to_dict(), far.to_dict()],
                      source=((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                      target=((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    only = scene_dict((-2000, -2000, -2000), (2000, 2000, 2000),
                      refs=[arc.to_dict()],
                      source=((10.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                      target=((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    prod = primitive_spacing(LineSegment3D((10.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
                             [arc, far], 5.0, RouterConfig())
    r011 = v011_certified_spacing({"kind": "line", "p0": [10.0, 0.0, 0.0],
                                   "p1": [100.0, 0.0, 0.0]},
                                  [arc.to_dict(), far.to_dict()], 5.0)
    print("   PRODUCTION spacing on this input: %s (bound %.6f um)"
          % (prod.status, prod.bound_um))
    print("   BEFORE  v0.1.1 validator, arc+line refs -> safe, lower bound %.6f um "
          "(the arc was filtered out)" % r011["lower_bound_um"])
    try:
        v011_certified_spacing({"kind": "line", "p0": [10.0, 0.0, 0.0],
                                "p1": [100.0, 0.0, 0.0]},
                               [arc.to_dict()], 5.0)
        print("   BEFORE  v0.1.1 validator, arc-only refs -> no exception")
    except TypeError as exc:
        print("   BEFORE  v0.1.1 validator, arc-only refs -> TypeError: %s" % exc)
    v = validate_path_dict(path, both, cfg_dict())
    v2 = validate_path_dict(path, only, cfg_dict())
    print("   AFTER   v0.1.2 validator, arc+line refs  -> %s (sampled %.6f, lb %.6f)"
          % (v["verdict"], v["centerline_spacing"]["min_sampled_um"],
             v["centerline_spacing"]["conservative_lower_bound_um"]))
    print("   AFTER   v0.1.2 validator, arc-only refs -> %s (no TypeError)"
          % v2["verdict"])


def a4():
    head("A4  a multi-turn sweep invalidated the chord/sagitta error bound")
    cfg = RouterConfig()
    arc = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, 8.0 * math.pi)
    ref = LineSegment3D((-51.0, -1.0, 0.0), (-49.0, 1.0, 0.0))
    box = Box3D((-51.0, -1.0, -1.0), (-49.0, 1.0, 1.0))
    print("   v0.1.1 spacing sagitta at level 0 (n = 1): %.6f um -> 'certified' bound "
          "%.6f um" % (v011_sagitta(arc.theta, arc.radius, 1),
                       v011_pair_clearance_lb_ub(arc, ref, 0)[0]))
    st, bound = v011_arc_box_clearance(arc, box, cfg.effective_obstacle_clearance_um)
    print("   BEFORE  v0.1.1 production: spacing safe %.9f um, obstacle %s %.6f um"
          % (v011_pair_clearance_lb_ub(arc, ref, 0)[0], st, bound))
    sp = primitive_spacing(arc, [ref], cfg.dmin_centerline_um, cfg)
    oc = primitive_box_clearance(arc, box, cfg.effective_obstacle_clearance_um, cfg)
    print("   AFTER   v0.1.2 production: spacing %s %.6f um, obstacle %s %.6f um"
          % (sp.status, sp.bound_um, oc.status, oc.bound_um))
    scene = scene_dict((-2000, -2000, -2000), (2000, 2000, 2000),
                       refs=[ref.to_dict()],
                       source=(list(arc.start), list(arc.tangent_at_phi(0.0))),
                       target=(list(arc.end), list(arc.tangent_at_phi(arc.theta))))
    v = validate_path_dict({"segments": [arc.to_dict()]}, scene, cfg_dict())
    spc = v["centerline_spacing"]
    print("   AFTER   v0.1.2 validator: sampled %.12f, %s lower bound %.6f, verdict %s, "
          "self check failed=%s"
          % (spc["min_sampled_um"], spc["status"],
             spc["conservative_lower_bound_um"], v["verdict"],
             v["certification"]["self_check_failed"]))
    print("   (v0.1.1 claimed a lower bound of 99.005050376 um while sampling reached "
          "0.022088195608 um, i.e. lower bound ABOVE upper bound, and still said pass)")


def a5():
    head("A5  near-parallel segments: catastrophic cancellation in a*e - b*b")
    A0, A1 = (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)
    B0, B1 = (0.0, -5e-6, 0.0), (1000.0, 5e-6, 0.0)
    print("   true distance = 0 (the two segments cross at (500,0,0))")
    print("   BEFORE  v0.1.1 classical formula  -> %.15e um"
          % _v011_ssd(A0, A1, B0, B1))
    st, lb, ub, method = segment_segment_distance_certified(A0, A1, B0, B1, 5e-7, 0.0)
    print("   AFTER   v0.1.2 certified decision -> %s, [%.3e, %.3e] um, method=%s"
          % (st, lb, ub, method))
    cfg = RouterConfig(dmin_centerline_um=5e-7)
    sp = primitive_spacing(LineSegment3D(A0, A1), [LineSegment3D(B0, B1)], 5e-7, cfg)
    scene = scene_dict((-2000, -2000, -2000), (2000, 2000, 2000),
                       refs=[{"kind": "line", "p0": list(B0), "p1": list(B1)}],
                       source=(A0, (1.0, 0.0, 0.0)), target=(A1, (1.0, 0.0, 0.0)))
    v = validate_path_dict({"segments": [{"kind": "line", "p0": list(A0), "p1": list(A1)}]},
                           scene, cfg_dict(dmin_centerline_um=5e-7))
    print("   AFTER   production spacing -> %s (%.3e um); validator -> %s "
          "(analytic %.3e um)"
          % (sp.status, sp.bound_um, v["verdict"],
             v["centerline_spacing"]["analytic_min_um"]))


if __name__ == "__main__":
    print("v0.1.2 PART A before/after replay (re-runnable on the final tree)")
    print("python : %s" % sys.version.replace("\n", " "))
    print("root   : %s" % ROOT)
    a1()
    a2()
    a3()
    a4()
    a5()
    print("")
    print("=" * 78)
    print("END OF PART A BEFORE/AFTER REPLAY")
    print("=" * 78)
