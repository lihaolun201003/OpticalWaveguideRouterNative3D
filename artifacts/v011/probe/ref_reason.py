import os, sys
ROOT = r"C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D"
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.cases import core_case
from native3d.search.reference_graph import reference_search
r = reference_search(core_case(), "2d")
print(r.status, "|", r.reason)
r2 = reference_search(core_case(), "native3d")
print(r2.status, "|", r2.reason)
print("to_dict keys ok:", r2.to_dict()["reference_graph"]["n_canonical_mids"])
