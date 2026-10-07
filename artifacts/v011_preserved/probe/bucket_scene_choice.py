import math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import base_config
from native3d.constraints.checks import Box3D
from native3d.geometry import Pose3D
from native3d.io.caseio import Scene
from native3d.search.router import route

def scene(cfg, chip, wall):
    return Scene("probe", chip, [wall], [], Pose3D((0.0,0.0,0.0),(1.0,0.0,0.0)),
                 Pose3D((200.0,0.0,0.0),(1.0,0.0,0.0)), cfg)

cfgs = {
 "small_wall_full_actions": (base_config(max_expansions=20000, max_runtime_s=20.0),
    Box3D((-50.,-50.,-50.),(250.,150.,150.)), Box3D((90.,-60.,-60.),(110.,160.,30.))),
 "small_wall_small_actions": (base_config(max_expansions=20000, max_runtime_s=20.0,
     straight_lengths_um=(20.,40.), turn_angles_deg=(45.,90.), turn_radii_um=(50.,),
     skew_axis_tilt_deg=(45.,), connector_radii_um=(50.,)),
    Box3D((-50.,-50.,-50.),(250.,150.,150.)), Box3D((90.,-60.,-60.),(110.,160.,30.))),
 "validator_demo_small": (base_config(max_expansions=20000, max_runtime_s=20.0,
     straight_lengths_um=(20.,40.), turn_angles_deg=(45.,90.), turn_radii_um=(50.,),
     skew_axis_tilt_deg=(45.,), connector_radii_um=(50.,)),
    Box3D((-100.,-100.,-100.),(300.,300.,200.)), Box3D((60.,-100.,-100.),(70.,100.,20.))),
}
for name,(cfg,chip,wall) in cfgs.items():
    sc = scene(cfg, chip, wall)
    seen=[]
    t0=time.perf_counter()
    res = route(sc, mode="native3d", expand_hook=lambda n,f,h: seen.append((n.state, n.g)))
    bm={}; drop=0
    for st,g in seen:
        b = st.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
        if b in bm and g >= bm[b]-1e-12: drop += 1
        bm[b] = min(bm.get(b, math.inf), g)
    print("%-26s status=%-18s exp=%5d buckets=%5d would_drop=%5d t=%.2fs" %
          (name, res.status, res.expansions, len(bm), drop, time.perf_counter()-t0), flush=True)
