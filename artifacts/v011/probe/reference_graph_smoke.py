import os, sys, time, json
ROOT = r"C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D"
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import core_case
from native3d.search.reference_graph import reference_search, canonical_poses
sc = core_case()
t0 = time.perf_counter()
mids = canonical_poses(sc, 60.0)
print("mids", len(mids), "%.2fs" % (time.perf_counter() - t0))
t0 = time.perf_counter()
res = reference_search(sc, mode="native3d")
print("native3d:", res.status, "cost", res.cost, "expansions", res.expansions,
      "runtime %.2fs" % (time.perf_counter() - t0))
if res.path:
    print("  length", res.length_um, "segments", [s.kind for s in res.path.segments])
    print("  trace:", [(t["action"], round(t["x_um"],2), round(t["y_um"],2), round(t["z_um"],2)) for t in res.trace])
    zs=[s.aabb() for s in res.path.segments]
    print("  z range", min(z[0][2] for z in zs), max(z[1][2] for z in zs))
    print("  c1:", res.path.continuity_report()["c1_continuous"], "minR", res.path.min_radius_um)
t0 = time.perf_counter()
res2 = reference_search(core_case(), mode="2d")
print("2d:", res2.status, res2.reason[:120], "expansions", res2.expansions,
      "runtime %.2fs" % (time.perf_counter() - t0))
