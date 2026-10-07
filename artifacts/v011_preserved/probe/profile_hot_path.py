"""Profile the search hot path (constraint validation per expansion)."""
import cProfile, io, os, pstats, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from state_identity_probe import run, key_exact
from native3d.cases import core_case

if __name__ == "__main__":
    sc = core_case()
    pr = cProfile.Profile()
    pr.enable()
    r = run(sc, "native3d", key_exact, max_exp=150, max_rt=600.0)
    pr.disable()
    print(r)
    s = io.StringIO()
    ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
    ps.print_stats(22)
    print(s.getvalue())
