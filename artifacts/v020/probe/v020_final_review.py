"""v0.2 adversarial final review: re-derive every headline number from the FILES.

This is the Lead's independent check, in the spirit of the v0.1.3
``v013_final_review.py``.  It imports nothing from the router and nothing from the
scheduler; it reads the case JSON artifacts and

1. re-runs the independent joint validator on each file;
2. BRUTE-FORCE re-checks the self-spacing of EVERY net path by dense arclength
   sampling (with the block scheme that only compares point pairs whose arclength
   blocks differ by >= 2, so every compared pair really is inside the check
   domain), and compares the result with the certified lower bounds of BOTH
   implementations - a certified lower bound may never EXCEED the true minimum,
   and the brute-force minimum is an upper bound of it, so
   ``LB <= brute + tol`` must hold;
3. brute-force re-checks the INTER-NET distance of every committed pair with no
   arclength exemption at all;
4. checks the claims of the generated artifacts: the committed set is mutually
   clear, every declared committed net really passes its own checks, no
   uncommitted net appears in the occupancy, and the occupancy is immutable;
5. checks the quoted counts against what the per-net records really say.

Run:

    .venv\\Scripts\\python.exe artifacts\\v020\\probe\\v020_final_review.py
"""

from __future__ import annotations

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))
sys.path.insert(0, os.path.join(_ROOT, "src"))

from native3d.validation import joint_validate as jv            # noqa: E402

CASES = os.path.join(_ROOT, "artifacts", "v020", "cases")
OUT = os.path.join(_ROOT, "artifacts", "v020", "probe",
                   "v020_final_review_result.json")
TOL = 1e-6


def primitive_points(seg, n):
    """Dense points of one primitive from its JSON."""
    if seg["kind"] == "line":
        p0, p1 = seg["p0"], seg["p1"]
        return [tuple(p0[i] + (p1[i] - p0[i]) * k / n for i in range(3))
                for k in range(n + 1)]
    c, u, v = seg["center"], seg["u"], seg["v"]
    R, th = float(seg["radius_um"]), float(seg["theta_rad"])
    out = []
    for k in range(n + 1):
        t = th * k / n
        co, si = math.cos(t), math.sin(t)
        out.append(tuple(c[i] + R * (co * u[i] + si * v[i]) for i in range(3)))
    return out


def densify(segments, per_um=1.0):
    """(arclength, point) samples of the assembled path."""
    samples = []
    off = 0.0
    for seg in segments:
        L = float(seg["length_um"])
        n = max(4, int(math.ceil(L * per_um)))
        pts = primitive_points(seg, n)
        for k, p in enumerate(pts):
            if k == 0 and samples:
                continue
            samples.append((off + L * k / n, p))
        off += L
    return samples


def brute_self_spacing(segments, ell, per_um=1.0):
    """True-ish minimum over |s - t| >= ell, by dense sampling + block pruning."""
    samples = densify(segments, per_um)
    best = math.inf
    best_pair = None
    for x in range(len(samples)):
        sx, px = samples[x]
        bx = math.floor(sx / ell)
        for y in range(x + 1, len(samples)):
            sy, py = samples[y]
            if sy - sx < ell:
                continue
            if math.floor(sy / ell) - bx < 2:
                continue
            d = math.dist(px, py)
            if d < best:
                best = d
                best_pair = (sx, sy)
    return best, best_pair


def brute_pair_distance(seg_a, seg_b, per_um=1.0):
    pa = densify([seg_a], per_um)
    pb = densify([seg_b], per_um)
    best = math.inf
    for _sa, a in pa:
        for _sb, b in pb:
            d = math.dist(a, b)
            if d < best:
                best = d
    return best


def main():
    files = sorted(f for f in os.listdir(CASES)
                   if f.endswith(".validated.json"))
    report = {"artifact": "native3d_v020_adversarial_final_review",
              "n_cases": len(files), "cases": [], "summary": {}}
    failures = []
    for fn in files:
        path = os.path.join(CASES, fn)
        with open(path, "r", encoding="utf-8") as fh:
            payload = json.load(fh)
        case = payload["case"]
        cfg = payload["config"]
        dmin = float(cfg["dmin_centerline_um"])
        ell = float(payload["self_spacing_policy"]["ell_local_um"])
        tol = float(cfg.get("clearance_decision_tol_um", 1e-9))

        # ---- 1. the independent validator on the file --------------------
        joint = jv.validate_joint_artifact(payload)
        rec = {"case": case,
               "channel_count": payload["channel_count"],
               "declared_counts": payload["counts"],
               "joint_verdict": joint["overall_verdict"],
               "joint_counts": joint["counts"],
               "nets": [], "problems": []}

        def problem(msg):
            rec["problems"].append(msg)
            failures.append("%s: %s" % (case, msg))

        # ---- 2/3/4. brute force every net and every committed pair -------
        committed = [n for n in payload["nets"] if n.get("committed")]
        committed_ids = {n["net_id"] for n in committed}
        for n in payload["nets"]:
            segs = n["segments"]
            if not segs:
                continue
            brute_self, pair = brute_self_spacing(segs, ell)
            jn = next(x for x in joint["nets"] if x["net_id"] == n["net_id"])
            lb = jn["self_spacing"].get("certified_lower_bound_um")
            entry = {"net_id": n["net_id"], "status": n["status"],
                     "committed": bool(n.get("committed")),
                     "joint_verdict": jn["verdict"],
                     "brute_self_spacing_um": brute_self,
                     "brute_self_witness_gap_um": (pair[1] - pair[0]) if pair else None,
                     "certified_lower_bound_um": lb,
                     "self_spacing_status": jn["self_spacing"]["status"]}
            if lb is not None and lb > brute_self + 1e-6:
                problem("certified LB %.9f EXCEEDS the brute-force minimum %.9f "
                        "for %s - impossible for a lower bound"
                        % (lb, brute_self, n["net_id"]))
            if n.get("committed"):
                if brute_self < dmin - TOL:
                    problem("COMMITTED net %s has a brute-force self distance "
                            "%.6f < dmin %.3f" % (n["net_id"], brute_self, dmin))
                if jn["verdict"] != "pass":
                    problem("net %s is declared committed but the independent "
                            "validator says %r" % (n["net_id"], jn["verdict"]))
                if n["net_id"] not in payload["occupancy"].get(
                        "per_net_published_curve_counts", {}):
                    problem("committed net %s publishes no occupancy curve"
                            % n["net_id"])
            else:
                if n["net_id"] in payload["occupancy"].get(
                        "per_net_published_curve_counts", {}):
                    problem("net %s is NOT committed but has published curves"
                            % n["net_id"])
            rec["nets"].append(entry)

        # inter-net, no exemption, committed pairs only
        cross = []
        for x in range(len(committed)):
            for y in range(x + 1, len(committed)):
                best = math.inf
                for sa in committed[x]["segments"]:
                    for sb in committed[y]["segments"]:
                        d = brute_pair_distance(sa, sb)
                        best = min(best, d)
                cross.append({"pair": [committed[x]["net_id"], committed[y]["net_id"]],
                              "brute_min_distance_um": best})
                if best < dmin - TOL:
                    problem("committed pair %s/%s is %.6f um apart in the "
                            "brute-force check"
                            % (committed[x]["net_id"], committed[y]["net_id"], best))
        rec["committed_pair_brute_force"] = cross

        # ---- 5. the quoted counts ---------------------------------------
        counts = payload["counts"]
        routed = sum(1 for n in payload["nets"] if n["search_status"] == "success"
                     and n["segments"])
        verified = sum(1 for n in payload["nets"] if n.get("accepted"))
        commit = sum(1 for n in payload["nets"] if n.get("committed"))
        if (counts["routed_count"], counts["verified_count"],
                counts["committed_count"]) != (routed, verified, commit):
            problem("the quoted counts %r do not match the per-net records "
                    "(%d, %d, %d)" % ((counts["routed_count"],
                                       counts["verified_count"],
                                       counts["committed_count"]),
                                      routed, verified, commit))
        if not payload["occupancy"]["immutability"]["ok"]:
            problem("the occupancy reports a modification")
        if joint["overall_verdict"] != "pass":
            problem("the joint validator verdict is %r" % joint["overall_verdict"])
        if joint["counts"]["n_disagreements"]:
            problem("%d disagreements between the declared and the independent "
                    "verdicts" % joint["counts"]["n_disagreements"])
        report["cases"].append(rec)

    report["summary"] = {
        "n_cases": len(files),
        "n_problems": len(failures),
        "problems": failures,
        "verdict": "PASS" if not failures else "FAIL",
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)
    for rec in report["cases"]:
        worst_self = min((n["brute_self_spacing_um"] for n in rec["nets"]
                          if n["brute_self_spacing_um"] is not None), default=None)
        worst_pair = min((c["brute_min_distance_um"]
                          for c in rec["committed_pair_brute_force"]), default=None)
        print("%-42s joint=%-5s nets=%2d  brute self-spacing min=%-10s  "
              "committed-pair min=%-10s  problems=%d"
              % (rec["case"], rec["joint_verdict"], len(rec["nets"]),
                 "%.6f" % worst_self if worst_self is not None else "-",
                 "%.6f" % worst_pair if worst_pair is not None else "-",
                 len(rec["problems"])))
        for p in rec["problems"]:
            print("    PROBLEM: %s" % p)
    print()
    print("adversarial final review verdict: %s (%d problem(s))"
          % (report["summary"]["verdict"], report["summary"]["n_problems"]))
    print("written: %s" % OUT)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
