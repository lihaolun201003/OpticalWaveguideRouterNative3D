"""Post-hoc GLOBAL-ARCLENGTH self-spacing check of one assembled path.

WHY THE EXISTING ``constraints.checks.path_self_spacing`` IS NOT ENOUGH
----------------------------------------------------------------------
That function decides the closest approach of two SEGMENTS and exempts the whole
combination of ADJACENT segments (``j - i == 1``).  Two consequences:

* a path that comes back onto itself WITHIN one segment (a multi-turn arc, or a
  long straight that is traversed twice when the assembly joins two collinear
  pieces) is never compared with itself at all;
* the exemption is expressed in SEGMENT indices, so it has no relation to a
  physical length: it exempts the far end of an adjacent segment as well as the
  junction.

THE SEMANTICS IMPLEMENTED HERE (mandatory, prompt section 4.1)
-------------------------------------------------------------
The assembled path is the curve ``P(s)`` with ``s`` the CUMULATIVE ARCLENGTH of
the whole path, ``s in [0, L]``.  The check domain is

    D = { (s, t) in [0, L]^2 : |s - t| >= ell_local }

and the question is ``min over D of |P(s) - P(t)| >= dmin ?``.  Only the
near-neighbour band ``|s - t| < ell_local`` is exempt, and ``ell_local`` is a
plain LENGTH in um: it is independent of the sampling step, of the chord
subdivision level and of any segment index.  This covers, in particular:

* the segment compared WITH ITSELF (i == j, including one element pair inside one
  segment) - what the old function could not see;
* a multi-turn arc whose far arclengths return to the same place;
* the FAR parts of two adjacent segments (whose junction is exempt only inside
  the band);
* every pair of non-adjacent segments.

HOW IT IS DECIDED (and why every reported number is a bound)
-----------------------------------------------------------
Each segment is split, domain-safely, into chord elements with ``|sweep| <= pi``
per chord, so every point of an element is within the SAGITTA ``delta`` of that
element's chord, and the chord is parametrised by the element's OWN arclength
with the exact chord/arc speed ratio ``|chord| / |arc|``.  For one element pair
the check domain inside the arclength rectangle is the rectangle minus the
diagonal band, i.e. the union of the two CONVEX polygons

    P_left  = rectangle n { t <= s - ell_local }
    P_right = rectangle n { t >= s + ell_local }

and the squared distance ``|Q(s) - Q(t)|^2`` between the two chord points is a
CONVEX quadratic in ``(s, t)``.  The minimum of a convex function over a convex
polygon is attained either at the (unique) unconstrained minimiser, when that
point lies inside the polygon, or on the polygon boundary; the boundary is
minimised in closed form edge by edge.  Hence, for one element pair,

    min_D |P(s) - P(t)|  >=  (min over P_left u P_right of |Q(s) - Q(t)|) - d1 - d2
    min_D |P(s) - P(t)|  <=  (min over P_left u P_right of |Q(s) - Q(t)|) + d1 + d2

The MINUS direction is the certified lower bound, the PLUS direction is the
certified upper bound, and the sign is not cosmetic: a point of the element is at
most ``delta`` away from its chord, so the curve can be ``delta`` CLOSER than the
chord (lower bound) and ``delta`` FARTHER than the chord (upper bound).

A violation is only ever reported through a point pair that is EXPLICITLY
verified to be inside the arclength rectangle and to satisfy
``|s - t| >= ell_local``, i.e. two points of the assembled path that really lie
in the check domain; the nearest point inside the exempt band is never used as a
violation.  ``+ delta`` is applied before the comparison, so a violation is
proved by real path points.

Two conservative guards, both reported in every result:

* ``SELF_SPACING_FP_MARGIN_UM`` (1e-9 um, the error floor of constraints.checks)
  is subtracted from every certified lower bound, so clipping arithmetic can
  never turn a violation into a "safe";
* the certified lower bound of an element pair is raised to at least the
  certified bound of the FULL chords, because the check domain is a subset of the
  rectangle and the rectangle minimum can never exceed the restricted minimum.

When neither "safe" nor "violation" can be decided inside the refinement cap the
check reports ``undecided``, which is NEVER counted as a pass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import (SEGMENT_DISTANCE_ERROR_FLOOR_UM, aabb_gap,
                                  chord_count_for_domain, sagitta_bound_um,
                                  segment_segment_bounds)
from .models import SelfSpacingPolicy

# Documented conservative FP guard of this module (um).  Same value as the
# module-wide numeric error floor of constraints.checks; reported in every result
# so a reader never has to guess it.
SELF_SPACING_FP_MARGIN_UM = SEGMENT_DISTANCE_ERROR_FLOOR_UM

# Refinement caps.  A straight/straight element pair needs no refinement at all
# (delta == 0 and the polygon minimisation is exact), so only pairs that contain
# an arc are refined.
SELF_SPACING_MAX_REFINEMENT_LEVEL = 6
SELF_SPACING_MAX_CHORDS_PER_SEGMENT = 64


@dataclass
class _Element:
    """One chord element of one segment, in GLOBAL arclength coordinates."""

    seg_index: int
    s0: float
    s1: float
    chord_start: Tuple[float, float, float]
    delta_um: float
    # Q(s) = chord_start + (s - s0) * vel, for s in [s0, s1]
    vel: Tuple[float, float, float]


def _elements_of_segment(seg, seg_index: int, s_offset: float,
                         n_chords: int) -> List[_Element]:
    """Domain-safe chord elements of one segment with a finer chord count."""
    L = float(seg.length)
    if seg.kind == "line":
        p0 = tuple(seg.point_at(0.0))
        if L <= 0.0:
            return []
        d = (seg.point_at(L)[0] - p0[0], seg.point_at(L)[1] - p0[1],
             seg.point_at(L)[2] - p0[2])
        nrm = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        if nrm <= 0.0:
            return []
        vel = (d[0] / L, d[1] / L, d[2] / L)
        return [_Element(seg_index, s_offset, s_offset + L, p0, 0.0, vel)]
    n = chord_count_for_domain(float(seg.theta), int(n_chords))
    delta = sagitta_bound_um(seg, n)
    step = L / n
    pts = [tuple(seg.point_at(step * i)) for i in range(n + 1)]
    out: List[_Element] = []
    for i in range(n):
        a, b = pts[i], pts[i + 1]
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        cl = math.sqrt(dx * dx + dy * dy + dz * dz)
        if cl <= 0.0:
            continue
        out.append(_Element(seg_index, s_offset + step * i, s_offset + step * (i + 1),
                            a, delta, (dx / step, dy / step, dz / step)))
    return out


def _clip_polygon(poly: List[Tuple[float, float]], g: Callable[[float, float], float],
                  ) -> List[Tuple[float, float]]:
    """Clip a convex polygon by the half-plane ``g <= 0``."""
    if not poly:
        return []
    out: List[Tuple[float, float]] = []
    n = len(poly)
    for k in range(n):
        cur = poly[k]
        nxt = poly[(k + 1) % n]
        gc = g(cur[0], cur[1])
        gn = g(nxt[0], nxt[1])
        cin = gc <= 0.0
        nin = gn <= 0.0
        if cin:
            out.append(cur)
        if cin != nin:
            den = gc - gn
            tau = 0.5 if abs(den) < 1e-300 else (gc / den)
            out.append((cur[0] + tau * (nxt[0] - cur[0]),
                        cur[1] + tau * (nxt[1] - cur[1])))
    dedup: List[Tuple[float, float]] = []
    for p in out:
        if not dedup or abs(p[0] - dedup[-1][0]) > 1e-12 or \
                abs(p[1] - dedup[-1][1]) > 1e-12:
            dedup.append(p)
    if len(dedup) > 1 and abs(dedup[0][0] - dedup[-1][0]) <= 1e-12 and \
            abs(dedup[0][1] - dedup[-1][1]) <= 1e-12:
        dedup.pop()
    return dedup if len(dedup) >= 2 else []


class _ChordPair:
    """The convex quadratic |Q1(s) - Q2(t)|^2 of one element pair."""

    def __init__(self, A: _Element, B: _Element) -> None:
        self.A = A
        self.B = B
        va, vb = A.vel, B.vel
        self.W = (A.chord_start[0] - B.chord_start[0] - A.s0 * va[0] + B.s0 * vb[0],
                  A.chord_start[1] - B.chord_start[1] - A.s0 * va[1] + B.s0 * vb[1],
                  A.chord_start[2] - B.chord_start[2] - A.s0 * va[2] + B.s0 * vb[2])
        # parallel test on the chord directions actually used
        cx = va[1] * vb[2] - va[2] * vb[1]
        cy = va[2] * vb[0] - va[0] * vb[2]
        cz = va[0] * vb[1] - va[1] * vb[0]
        self.cross_sq = cx * cx + cy * cy + cz * cz
        self.parallel = self.cross_sq <= 1e-18
        a = va[0] * va[0] + va[1] * va[1] + va[2] * va[2]
        b = vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2]
        c = va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2]
        d1 = va[0] * self.W[0] + va[1] * self.W[1] + va[2] * self.W[2]
        d2 = vb[0] * self.W[0] + vb[1] * self.W[1] + vb[2] * self.W[2]
        self.crit: Optional[Tuple[float, float]] = None
        if not self.parallel:
            det = c * c - a * b            # == -cross_sq < 0
            s = (b * d1 - c * d2) / det
            t = (c * d1 - a * d2) / det
            if math.isfinite(s) and math.isfinite(t):
                self.crit = (s, t)

    def f(self, s: float, t: float) -> float:
        va, vb, W = self.A.vel, self.B.vel, self.W
        dx = W[0] + s * va[0] - t * vb[0]
        dy = W[1] + s * va[1] - t * vb[1]
        dz = W[2] + s * va[2] - t * vb[2]
        return dx * dx + dy * dy + dz * dz

    def __call__(self, s: float, t: float) -> float:
        return self.f(s, t)

    def min_over_polygon(self, poly: List[Tuple[float, float]],
                         accept: Callable[[float, float], bool]) -> Tuple[float, float, float]:
        """(min |Q1-Q2|, s, t) over the polygon, boundary and interior.

        ``accept`` is used only to decide whether the interior minimiser may be
        used: the minimum of a convex function over a convex polygon is attained
        at the unconstrained minimiser when that point is inside, and on the
        boundary otherwise.  The boundary is a finite list of straight edges, and
        each edge is minimised in closed form.
        """
        best = math.inf
        bs, bt = poly[0][0], poly[0][1]
        if self.crit is not None and accept(self.crit[0], self.crit[1]):
            val = self.f(self.crit[0], self.crit[1])
            if val < best:
                best, bs, bt = val, self.crit[0], self.crit[1]
        n = len(poly)
        for k in range(n):
            s0, t0 = poly[k]
            s1, t1 = poly[(k + 1) % n]
            ds, dt = s1 - s0, t1 - t0
            if ds == 0.0 and dt == 0.0:
                cand = [(0.0, s0, t0)]
            else:
                a0 = (self.W[0] + s0 * self.A.vel[0] - t0 * self.B.vel[0],
                      self.W[1] + s0 * self.A.vel[1] - t0 * self.B.vel[1],
                      self.W[2] + s0 * self.A.vel[2] - t0 * self.B.vel[2])
                a1 = (ds * self.A.vel[0] - dt * self.B.vel[0],
                      ds * self.A.vel[1] - dt * self.B.vel[1],
                      ds * self.A.vel[2] - dt * self.B.vel[2])
                aa = a1[0] * a1[0] + a1[1] * a1[1] + a1[2] * a1[2]
                ab = a0[0] * a1[0] + a0[1] * a1[1] + a0[2] * a1[2]
                tau = 0.0 if aa <= 0.0 else min(1.0, max(0.0, -ab / aa))
                cand = [(tau, s0 + tau * ds, t0 + tau * dt), (0.0, s0, t0),
                        (1.0, s1, t1)]
            for _tau, sv, tv in cand:
                val = self.f(sv, tv)
                if val < best:
                    best, bs, bt = val, sv, tv
        return math.sqrt(max(0.0, best)), bs, bt


def _element_pair_bounds(A: _Element, B: _Element, ell: float
                         ) -> Optional[Tuple[float, float, Optional[Tuple[float, float]],
                                             bool]]:
    """Certified (lower, upper, witness, upper_is_valid) of one element pair.

    ``lower`` is a certified LOWER bound of ``min over D of |P1 - P2|`` for the
    two elements; ``upper`` is a certified UPPER bound of the same quantity and is
    only returned when a point pair of the elements could be verified to lie in
    the check domain.  ``None`` means the whole rectangle is exempt.
    """
    a0, a1 = A.s0, A.s1
    b0, b1 = B.s0, B.s1
    if max(abs(a1 - b0), abs(a0 - b1)) < ell:
        return None                      # the whole rectangle is inside the band

    rect = [(a0, b0), (a1, b0), (a1, b1), (a0, b1)]
    halfplanes = ((lambda s, t: t - s + ell), (lambda s, t: s - t + ell))

    def in_box(s: float, t: float) -> bool:
        return (a0 - 1e-12 <= s <= a1 + 1e-12) and (b0 - 1e-12 <= t <= b1 + 1e-12)

    def in_domain(s: float, t: float) -> bool:
        return abs(s - t) >= ell

    qp = _ChordPair(A, B)
    delta = A.delta_um + B.delta_um
    chord_min = math.inf            # min over the polygon (chord space)
    chord_ub = math.inf             # valid only through an in-domain point pair
    best_pt: Optional[Tuple[float, float]] = None
    for g in halfplanes:
        poly = _clip_polygon(rect, g)
        if not poly:
            continue
        val, s_star, t_star = qp.min_over_polygon(
            poly, lambda s, t: in_box(s, t) and g(s, t) <= 0.0)
        if val < chord_min:
            chord_min = val
        if in_box(s_star, t_star) and in_domain(s_star, t_star) and val < chord_ub:
            chord_ub = val
            best_pt = (s_star, t_star)
    if math.isinf(chord_min):
        return None

    # the whole rectangle is a superset of the check domain, so its certified
    # minimum is also a valid lower bound of the restricted minimum
    c0 = (A.chord_start[0], A.chord_start[1], A.chord_start[2])
    c1 = (A.chord_start[0] + A.vel[0] * (a1 - a0),
          A.chord_start[1] + A.vel[1] * (a1 - a0),
          A.chord_start[2] + A.vel[2] * (a1 - a0))
    d0 = (B.chord_start[0], B.chord_start[1], B.chord_start[2])
    d1v = (B.chord_start[0] + B.vel[0] * (b1 - b0),
           B.chord_start[1] + B.vel[1] * (b1 - b0),
           B.chord_start[2] + B.vel[2] * (b1 - b0))
    full_lo, _full_hi = segment_segment_bounds(c0, c1, d0, d1v)
    if full_lo > chord_min:
        chord_min = full_lo

    lower = max(0.0, chord_min - delta - SELF_SPACING_FP_MARGIN_UM)
    if math.isinf(chord_ub):
        return lower, math.inf, None, False
    return lower, max(0.0, chord_ub + delta), best_pt, True


def path_self_spacing_global(path_or_segments, cfg: RouterConfig,
                             policy: SelfSpacingPolicy,
                             max_level: int = SELF_SPACING_MAX_REFINEMENT_LEVEL,
                             sampled_estimate: bool = True) -> Dict:
    """The v0.2 post-hoc GLOBAL-ARCLENGTH self-spacing check of ONE assembled path."""
    segments = list(getattr(path_or_segments, "segments", path_or_segments))
    required = float(cfg.dmin_centerline_um)
    tol = float(cfg.clearance_decision_tol_um)
    ell = float(policy.ell_local_um)

    spans: List[Tuple[float, float]] = []
    off = 0.0
    for seg in segments:
        L = float(seg.length)
        spans.append((off, off + L))
        off += L
    total = off

    result: Dict = {
        "name": "self_spacing_global_arclength",
        "policy": policy.to_dict(),
        "check_domain": ("|s - t| >= ell_local_um over the assembled-path cumulative "
                         "arclength s, t in [0, L]; only the near-neighbour band "
                         "|s - t| < ell_local_um is exempt"),
        "required_um": required,
        "ell_local_um": ell,
        "dmin_centerline_um": required,
        "total_length_um": total,
        "n_segments": len(segments),
        "fp_margin_um": SELF_SPACING_FP_MARGIN_UM,
        "max_refinement_level": int(max_level),
        "max_chords_per_segment": SELF_SPACING_MAX_CHORDS_PER_SEGMENT,
        "covers_same_segment_self_intersection": True,
        "covers_multi_turn_reentry": True,
        "covers_adjacent_segment_far_points": True,
        "covers_non_adjacent_segments": True,
        "exemption_is_arclength_based_not_segment_based": True,
        "not_implemented_by_this_module": False,
    }
    if not segments:
        result.update({"status": "safe", "certified_lower_bound_um": None,
                       "certified_upper_bound_um": None, "witness": None,
                       "note": "empty path: nothing to compare"})
        return result

    lb = math.inf
    ub = math.inf
    witness: Optional[dict] = None
    violation = False
    n_pairs = 0
    n_pairs_exempt = 0
    n_pairs_aabb_safe = 0
    n_undecided = 0
    best_skipped_gap = math.inf
    per_pair: List[dict] = []

    for i in range(len(segments)):
        for j in range(i, len(segments)):
            n_pairs += 1
            si, sj = segments[i], segments[j]
            ai0, ai1 = spans[i]
            bj0, bj1 = spans[j]
            if max(abs(ai1 - bj0), abs(ai0 - bj1)) < ell:
                n_pairs_exempt += 1
                continue
            lo_i, hi_i = si.aabb()
            lo_j, hi_j = sj.aabb()
            gap = aabb_gap(tuple(lo_i), tuple(hi_i), tuple(lo_j), tuple(hi_j))
            if gap >= required:
                n_pairs_aabb_safe += 1
                best_skipped_gap = min(best_skipped_gap, gap)
                continue

            level_cap = int(max_level) if (si.kind == "arc" or sj.kind == "arc") else 0
            pair_lo = math.inf
            pair_hi = math.inf
            pair_pt: Optional[Tuple[float, float]] = None
            pair_level = 0
            status = "undecided"
            for level in range(level_cap + 1):
                # every element pair of the level is scanned, so the reported
                # lower bound of the level is complete and therefore certified
                n_i = min(SELF_SPACING_MAX_CHORDS_PER_SEGMENT, 2 ** level)
                els_i = _elements_of_segment(si, i, ai0, n_i)
                els_j = els_i if j == i else _elements_of_segment(sj, j, bj0, n_i)
                pair_lo = math.inf
                pair_hi = math.inf
                pair_pt = None
                for A in els_i:
                    for B in els_j:
                        if j == i and B.s0 < A.s0:
                            continue
                        bounds = _element_pair_bounds(A, B, ell)
                        if bounds is None:
                            continue
                        lo_e, hi_e, pt, hi_valid = bounds
                        if lo_e < pair_lo:
                            pair_lo = lo_e
                        if hi_valid and hi_e < pair_hi:
                            pair_hi = hi_e
                            pair_pt = pt
                pair_level = level
                if math.isfinite(pair_hi) and pair_hi + tol < required:
                    status = "violation"
                    break
                if pair_lo >= required - tol:
                    status = "safe"
                    break
            if pair_lo < lb:
                lb = pair_lo
            if math.isfinite(pair_hi) and pair_hi < ub:
                ub = pair_hi
            if status == "violation":
                violation = True
                s_star, t_star = pair_pt
                p_s = _point_at_global(segments, spans, s_star)
                p_t = _point_at_global(segments, spans, t_star)
                witness = {
                    "segment_pair": [i, j],
                    "global_arclength_s_um": s_star,
                    "global_arclength_t_um": t_star,
                    "arclength_separation_um": abs(s_star - t_star),
                    "in_check_domain": bool(abs(s_star - t_star) >= ell),
                    "distance_sampled_um": _point_distance(p_s, p_t),
                    "distance_upper_bound_um": pair_hi,
                    "point_s_um": list(p_s),
                    "point_t_um": list(p_t),
                    "refinement_level": pair_level,
                }
            elif status == "undecided":
                n_undecided += 1
            per_pair.append({"segment_pair": [i, j], "status": status,
                             "lower_bound_um": pair_lo,
                             "upper_bound_um": (pair_hi if math.isfinite(pair_hi)
                                                else None),
                             "refinement_level": pair_level})

    if math.isfinite(best_skipped_gap):
        lb = min(lb, best_skipped_gap)
    if violation:
        status = "violation"
    elif lb >= required - tol:
        # lb == +inf is the (vacuous but honest) case in which the whole path is
        # shorter than ell_local, so the check domain contains no point pair
        status = "safe"
    else:
        status = "undecided"

    result.update({
        "status": status,
        "certified_lower_bound_um": (lb if math.isfinite(lb) else None),
        "certified_upper_bound_um": (ub if math.isfinite(ub) else None),
        "sampled_estimate_um": (_sampled_min_distance(segments, spans, ell)
                                if sampled_estimate else None),
        "witness": witness,
        "n_segment_pairs": n_pairs,
        "n_pairs_fully_exempt": n_pairs_exempt,
        "n_pairs_certified_safe_by_aabb_gap": n_pairs_aabb_safe,
        "n_pairs_undecided": n_undecided,
        "pairs": per_pair,
        "verdict_rule": ("violation only through a point pair with |s - t| >= "
                         "ell_local whose distance is below dmin; safe only when the "
                         "certified lower bound reaches dmin; undecided is never a "
                         "pass and is never reported as safe"),
    })
    return result


def _point_at_global(segments, spans, s: float) -> Tuple[float, float, float]:
    """The point of the assembled path at GLOBAL arclength ``s``.

    The segment spans are half-open ([a, b)), so a junction arclength belongs to
    the NEXT segment.  For a C1-continuous assembly both neighbours give the same
    point; for a non-continuous one this convention is what makes the witness
    point of the FOLLOWING segment, which is the segment the arclength really
    names.
    """
    if s <= spans[0][0]:
        return tuple(segments[0].point_at(0.0))
    for k, (a, b) in enumerate(spans):
        if a <= s < b:
            return tuple(segments[k].point_at(s - a))
    return tuple(segments[-1].point_at(float(segments[-1].length)))


def _point_distance(p, q) -> float:
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2)


def _sampled_min_distance(segments, spans, ell: float,
                          n_per_segment: int = 24) -> Optional[float]:
    """A SAMPLED estimate; only pairs of arclength blocks differing by >= 2.

    Never a certificate: a sampled minimum can only be too LARGE.  Blocks of
    length ``ell`` guarantee that points in blocks k and m with ``m - k >= 2``
    really satisfy ``|s - t| >= ell``; pairs inside or next to the band are
    excluded from the estimate instead of being reported as a distance.
    """
    samples: List[Tuple[float, Tuple[float, float, float]]] = []
    for seg, (a, b) in zip(segments, spans):
        L = max(0.0, b - a)
        if L <= 0.0:
            continue
        for k in range(n_per_segment + 1):
            samples.append((a + L * k / n_per_segment,
                            tuple(seg.point_at(L * k / n_per_segment))))
    if len(samples) < 2:
        return None
    best = math.inf
    for x in range(len(samples)):
        sx, px = samples[x]
        for y in range(x + 1, len(samples)):
            sy, py = samples[y]
            if abs(sy - sx) < ell:
                continue
            if ell > 0.0 and abs(math.floor(sy / ell) - math.floor(sx / ell)) < 2:
                continue
            d = _point_distance(px, py)
            if d < best:
                best = d
    return None if math.isinf(best) else best
