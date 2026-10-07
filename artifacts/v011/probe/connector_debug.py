import math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src"))
from native3d.geometry import Pose3D
from native3d.geometry.vec import dot
from native3d.search.connector import planar_connectors, _rot90_2d, _mod2pi

R = 10.0
for label, bpos in (("+y offset 80", (0.0, 80.0, 0.0)), ("-y offset 80", (0.0, -80.0, 0.0))):
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D(bpos, (1.0, 0.0, 0.0))
    print("==", label)
    for segs in planar_connectors(a, b, R):
        tot = sum(s.length for s in segs)
        desc = []
        for s in segs:
            if s.kind == "line":
                desc.append("L=%.4f" % s.length)
            else:
                desc.append("arc R=%.1f th=%.4fdeg n=%s" % (s.radius, math.degrees(s.theta), tuple(round(c,4) for c in s.normal)))
        print("  total=%.6f  %s" % (tot, " | ".join(desc)))
    print("  expected LSR total = %.6f" % (R*2*(math.pi/2+math.atan2(2*R, math.sqrt(80**2-4*R*R))) + math.sqrt(80**2-4*R*R)))
