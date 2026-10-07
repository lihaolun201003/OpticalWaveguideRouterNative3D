"""Probe: is a finite canonical reference graph (source -> mid -> target via
exact Dubins connectors) able to carry the native-3D core case?
"""
import itertools, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import core_case
from native3d.constraints.checks import validate_segments
from native3d.geometry.primitives import Pose3D
from native3d.geometry.vec import norm, sub, unit
from native3d.search.connector import planar_connectors
from native3d.search.router import pose_invalid_reason

DIRS = [t for t in itertools.product((-1, 0, 1), repeat=3) if t != (0, 0, 0)]


def unit_dir(t):
    n = math.sqrt(sum(c * c for c in t))
    return (t[0] / n, t[1] / n, t[2] / n)


def canonical_poses(scene, spacing):
    cfg = scene.config
    inner = scene.chip.shrink(cfg.effective_boundary_margin_um)
    src = scene.source.position
    ijk = []
    for i in range(-4, 25):
        for j in range(-4, 6):
            for k in range(-3, 4):
                p = (src[0] + spacing * i, src[1] + spacing * j, src[2] + spacing * k)
                if not inner.contains_point(p, 1e-9):
                    continue
                ijk.append(p)
    out = []
    for p in ijk:
        for t in DIRS:
            pose = Pose3D(p, unit_dir(t))
            if pose_invalid_reason(scene, pose) is None:
                out.append(pose)
    return out


def legal_connectors(scene, a, b, cfg):
    ok = []
    for R in cfg.connector_radii_um:
        for segs in planar_connectors(a, b, float(R)):
            total = sum(s.length for s in segs)
            if total > cfg.connector_max_length_um:
                continue
            val = validate_segments(segs, scene.chip, scene.obstacles,
                                    scene.reference_centerlines, cfg, detailed=False)
            if val["ok"]:
                ok.append((total, R, segs))
    ok.sort(key=lambda x: x[0])
    return ok


def run(scene, mode, spacing=60.0, max_hops=2, limit_report=6):
    cfg = scene.config
    if mode == "2d":
        # in 2D mode every primitive must stay in the plane z = z(source)
        z0 = scene.source.position[2]
        def in_plane(segs):
            for s in segs:
                lo, hi = s.aabb()
                if abs(lo[2] - z0) > 1e-9 or abs(hi[2] - z0) > 1e-9:
                    return False
            return True
    else:
        def in_plane(segs):
            return True
    poses = canonical_poses(scene, spacing)
    print("canonical mids:", len(poses), flush=True)
    src = scene.source
    tgt = scene.target
    t0 = time.perf_counter()
    best = None
    checked = 0
    for q in poses:
        c1 = [c for c in legal_connectors(scene, src, q, cfg) if in_plane(c[2])]
        if not c1:
            continue
        c2 = [c for c in legal_connectors(scene, q, tgt, cfg) if in_plane(c[2])]
        if not c2:
            continue
        checked += 1
        tot = c1[0][0] + c2[0][0]
        if best is None or tot < best[0]:
            best = (tot, q, c1[0], c2[0])
        if checked >= limit_report and best is not None and checked > 40:
            break
    print("mids with both connectors: %d   (%.1fs)" % (checked, time.perf_counter() - t0))
    if best:
        print("best 2-hop route cost=%.3f um  via %s" % (best[0], best[1].position))
        print("  leg1 R=%g len=%.3f  leg2 R=%g len=%.3f" %
              (best[2][1], best[2][0], best[3][1], best[3][0]))
        zs = []
        for segs in (best[2][2], best[3][2]):
            for s in segs:
                lo, hi = s.aabb()
                zs += [lo[2], hi[2]]
        print("  z range of route: %.3f .. %.3f  (source z=%.1f)" % (min(zs), max(zs), src.position[2]))
    else:
        print("NO 2-hop route in the canonical reference graph")
    return best


if __name__ == "__main__":
    sc = core_case()
    print("=== native3d ===")
    run(sc, "native3d", spacing=float(sys.argv[1]) if len(sys.argv) > 1 else 60.0)
    print("=== 2d control ===")
    run(core_case(), "2d", spacing=float(sys.argv[1]) if len(sys.argv) > 1 else 60.0, limit_report=2)
