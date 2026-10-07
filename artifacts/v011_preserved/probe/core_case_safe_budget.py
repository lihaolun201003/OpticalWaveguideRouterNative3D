"""Probe: how long does the corrected A* need on the core case, per state identity?

Runs the core acceptance case in native3d mode with (a) the safe exact-pose key
and (b) the v0.1 quantised key, using a corrected A* termination (goal on pop),
and reports progress every 15 s.  Budget probe only - it decides what budget the
v0.1.1 acceptance runs must disclose.
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "state_identity_probe.py").rsplit(os.sep, 1)[0])
sys.path.insert(0, HERE)
from state_identity_probe import run, key_exact, key_quant  # noqa: E402
from native3d.cases import core_case  # noqa: E402

if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "exact"
    budget = float(sys.argv[2]) if len(sys.argv) > 2 else 900.0
    max_exp = int(sys.argv[3]) if len(sys.argv) > 3 else 2000000
    sc = core_case()
    kf = key_exact if which == "exact" else key_quant(sc.config)
    t0 = time.perf_counter()
    r = run(sc, "native3d", kf, max_exp=max_exp, max_rt=budget)
    print("%s -> %s" % (which, json.dumps(r)))
    print("wall clock %.1f s" % (time.perf_counter() - t0))
