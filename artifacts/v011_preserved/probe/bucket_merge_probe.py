"""Probe: does a real router run in the SAFE identity keep poses that the v0.1
quantised key would have merged away, and does the accepted route depend on one?
"""
import os, sys, json
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import base_config
from native3d.constraints.checks import Box3D
from native3d.geometry import Pose3D
from native3d.io.caseio import Scene
from native3d.search import state as state_mod
from native3d.search.router import route


def exact_key(self, pos_res_um=None, dir_res_deg=None):
    return (round(self.position[0], 9), round(self.position[1], 9), round(self.position[2], 9),
            round(self.direction[0], 9), round(self.direction[1], 9), round(self.direction[2], 9))


def bucket(s, pr=20.0, dr=30.0):
    return (round(s.position[0] / pr), round(s.position[1] / pr), round(s.position[2] / pr),
            round((s.azimuth_deg % 360.0) / dr), round(s.elevation_deg / dr))


cfg = base_config(max_expansions=4000, max_runtime_s=60.0)
chip = Box3D((-50.0, -50.0, -50.0), (250.0, 150.0, 150.0))
wall = Box3D((90.0, -60.0, -60.0), (110.0, 160.0, 30.0))
sc = Scene("small_wall", chip, [wall], [], Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
           Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)

orig = state_mod.State3D.key
state_mod.State3D.key = exact_key
try:
    seen = []
    res = route(sc, mode="native3d", expand_hook=lambda n, f, h: seen.append((n.state, n.g, n.depth)))
finally:
    state_mod.State3D.key = orig

print("status", res.status, "expansions", res.expansions, "cost", res.cost, "len", res.length_um)
print("expanded nodes", len(seen))
first = {}
collisions = []
for i, (s, g, d) in enumerate(seen):
    b = bucket(s)
    if b in first:
        collisions.append((i, first[b][0], g, first[b][1]))
    else:
        first[b] = (i, g)
print("bucket collisions among expanded nodes:", len(collisions))
if collisions:
    print("sample:", collisions[:5])
trace = [tuple(round(v, 6) for v in (t["x_um"], t["y_um"], t["z_um"])) for t in res.trace]
print("trace len", len(trace))
# do two trace poses share a bucket?
tb = {}
shared_trace = []
for k, p in enumerate(trace):
    b = bucket(type("S", (), {"position": p, "azimuth_deg": 0.0, "elevation_deg": 0.0})())
    if b in tb:
        shared_trace.append((k, tb[b]))
    else:
        tb[b] = k
print("trace poses sharing a quantised bucket:", shared_trace)
print("trace:", json.dumps(res.trace and [ (t["action"], round(t["x_um"],3), round(t["y_um"],3), round(t["z_um"],3)) for t in res.trace]))
