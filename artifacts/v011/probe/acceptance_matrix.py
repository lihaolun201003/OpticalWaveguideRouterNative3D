"""Probe: which acceptance searches still work with the SAFE exact identity and
the CORRECTED A* termination (production router), and which need the finite
canonical reference graph?
"""
import json, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import (base_config, core_case, mid_curve_obstacle_case,
                            sealed_target_case, terminal_tangent_case)
from native3d.constraints.checks import Box3D
from native3d.geometry import LineSegment3D, Pose3D
from native3d.io.caseio import Scene
from native3d.search.router import route
from native3d.search.reference_graph import reference_search


def show(label, res, t0):
    print("%-42s %-18s exp=%6d cost=%s t=%.2fs" % (
        label, res.status, res.expansions,
        ("%.3f" % res.cost) if res.cost != float("inf") else "-",
        time.perf_counter() - t0), flush=True)


def run(label, fn, budget=60.0, max_exp=50000):
    t0 = time.perf_counter()
    res = fn(budget, max_exp)
    show(label, res, t0)
    return res


# 1. pure straight
cfg = base_config(max_expansions=50000)
chip = Box3D((-10.0, -50.0, -50.0), (200.0, 50.0, 50.0))
sc = Scene("straight", chip, [], [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
           Pose3D((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)
run("case1 pure straight (production)", lambda b, m: route(sc, mode="native3d", max_runtime_s=b, max_expansions=m))

# 2. spatial bending
cfg2 = base_config(max_expansions=50000)
chip2 = Box3D((-200.0, -200.0, -200.0), (200.0, 200.0, 200.0))
sc2 = Scene("bend", chip2, [], [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            Pose3D((120.0, 40.0, 25.0), (1.0, 0.0, 0.0)), cfg2)
run("case2 spatial bend (production)", lambda b, m: route(sc2, mode="native3d", max_runtime_s=b, max_expansions=m))

# 3. core case, both modes, production
run("core native3d (production)", lambda b, m: route(core_case(), mode="native3d", max_runtime_s=b, max_expansions=m))
run("core 2d (production)", lambda b, m: route(core_case(), mode="2d", max_runtime_s=b, max_expansions=m))
run("core native3d (reference graph)", lambda b, m: reference_search(core_case(), "native3d", max_runtime_s=b, max_expansions=m))
run("core 2d (reference graph)", lambda b, m: reference_search(core_case(), "2d", max_runtime_s=b, max_expansions=m))

# 4. sealed target
run("sealed 2d (production)", lambda b, m: route(sealed_target_case(), mode="2d", max_runtime_s=b, max_expansions=m))
run("sealed 2d (reference graph)", lambda b, m: reference_search(sealed_target_case(), "2d", max_runtime_s=b, max_expansions=m))
run("sealed native3d (reference graph)", lambda b, m: reference_search(sealed_target_case(), "native3d", max_runtime_s=b, max_expansions=m))

# 5. terminal tangent 2d
run("terminal tangent 2d good (production)", lambda b, m: route(terminal_tangent_case(True), mode="2d", max_runtime_s=b, max_expansions=m))
run("terminal tangent 2d wrong (production)", lambda b, m: route(terminal_tangent_case(False), mode="2d", max_runtime_s=b, max_expansions=m))
run("terminal tangent 2d good (reference)", lambda b, m: reference_search(terminal_tangent_case(True), "2d", max_runtime_s=b, max_expansions=m))
run("terminal tangent 2d wrong (reference)", lambda b, m: reference_search(terminal_tangent_case(False), "2d", max_runtime_s=b, max_expansions=m))

# 6. small wall scene (phase4 congestion test)
cfg6 = base_config(max_expansions=50000, max_runtime_s=120.0)
chip6 = Box3D((-50.0, -50.0, -50.0), (250.0, 150.0, 150.0))
wall6 = Box3D((90.0, -60.0, -60.0), (110.0, 160.0, 30.0))
sc6 = Scene("small_wall", chip6, [wall6], [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg6)
run("small wall (production)", lambda b, m: route(sc6, mode="native3d", max_runtime_s=b, max_expansions=m))
run("small wall (reference graph)", lambda b, m: reference_search(sc6, "native3d", max_runtime_s=b, max_expansions=m))

# 7. validator demo scene (test_independent_validation)
cfg7 = base_config(max_expansions=50000, max_runtime_s=60.0, pos_resolution_um=20.0,
                   dir_resolution_deg=30.0, straight_lengths_um=(20.0, 40.0),
                   turn_angles_deg=(45.0, 90.0), turn_radii_um=(50.0,),
                   skew_axis_tilt_deg=(45.0,), connector_radii_um=(50.0,))
chip7 = Box3D((-100.0, -100.0, -100.0), (300.0, 300.0, 200.0))
sc7 = Scene("validator_demo", chip7, [Box3D((60.0, -100.0, -100.0), (70.0, 100.0, 20.0))],
            [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg7)
run("validator demo (production)", lambda b, m: route(sc7, mode="native3d", max_runtime_s=b, max_expansions=m))
run("validator demo (reference graph)", lambda b, m: reference_search(sc7, "native3d", max_runtime_s=b, max_expansions=m))
