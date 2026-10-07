"""Prototype: admissible obstacle-crossing lower bound as an A* heuristic.

Soundness of the bound: every legal centreline is a continuous curve that stays
outside the keep-out box B' = B (+ clearance).  If the start and the target lie
on opposite sides of B' along axis a, the curve meets every plane
a = c (c in [lo_a, hi_a]) in a point p outside B'.  For that point,
    curve length >= |start - p| + |p - target|,
so curve length >= min over allowed p in that plane of the same quantity.
The min is computed exactly: the unconstrained minimiser is the intersection of
the start-target segment with the plane; if it is forbidden, the minimum sits on
the rectangle boundary and is found from the four side reflections and the four
corners (f is convex, so no interior point of the complement can beat them).
"""
import math


def _two_dist(a, b, p):
    return math.hypot(a[0] - p[0], a[1] - p[1]) + math.hypot(b[0] - p[0], b[1] - p[1])


def min_two_dist_outside_rect(A, B, lo, hi):
    """Exact min of |A-p| + |B-p| over the 2D complement of int([lo,hi]^2)."""
    # unconstrained minimiser: any point of the segment AB
    # if the segment does not enter the rectangle, the answer is |A-B|
    best = math.hypot(A[0] - B[0], A[1] - B[1])
    def inside(p):
        return lo[0] < p[0] < hi[0] and lo[1] < p[1] < hi[1]
    # does the segment AB meet the open rectangle?
    t0, t1 = 0.0, 1.0
    ok = True
    for i in range(2):
        d = B[i] - A[i]
        if abs(d) < 1e-15:
            if not (lo[i] < A[i] < hi[i]):
                ok = False
                break
        else:
            ta = (lo[i] - A[i]) / d
            tb = (hi[i] - A[i]) / d
            if ta > tb:
                ta, tb = tb, ta
            t0 = max(t0, ta)
            t1 = min(t1, tb)
    if not ok or t0 >= t1:
        return best                      # nothing forbidden on the way
    # minimum over the boundary
    cands = [(lo[0], lo[1]), (lo[0], hi[1]), (hi[0], lo[1]), (hi[0], hi[1])]
    # side reflections: the minimum of |A-p|+|B-p| over a line is the
    # reflection point of A across the line intersected with B
    for axis in (0, 1):
        for val in (lo[axis], hi[axis]):
            A2 = list(A)
            B2 = list(B)
            A2[axis] = 2.0 * val - A[axis]
            denom = (B2[axis] - A2[axis])
            if abs(denom) < 1e-15:
                continue
            s = (val - A2[axis]) / denom
            if not (0.0 <= s <= 1.0):
                continue
            p = (A2[0] + (B2[0] - A2[0]) * s, A2[1] + (B2[1] - A2[1]) * s)
            other = 1 - axis
            if lo[other] - 1e-12 <= p[other] <= hi[other] + 1e-12:
                cands.append(p)
    for p in cands:
        v = _two_dist(A, B, p)
        if v < best:
            best = v
    return best


def crossing_bound(s, t, boxes, clearance, eps_pos):
    best = math.dist(s, t) - eps_pos
    for box in boxes:
        blo = [box.lo[i] - clearance for i in range(3)]
        bhi = [box.hi[i] + clearance for i in range(3)]
        for a in range(3):
            b, c = [i for i in range(3) if i != a]
            if min(s[a], t[a]) >= blo[a] or max(s[a], t[a]) <= bhi[a]:
                continue
            planes = (blo[a], 0.5 * (blo[a] + bhi[a]), bhi[a])
            for plane in planes:
                # 3D: the minimiser over the plane is the intersection of the
                # segment s-t with the plane (they straddle it by construction)
                denom = t[a] - s[a]
                lam = (plane - s[a]) / denom
                A = (s[b], s[c])
                B = (t[b], t[c])
                p3 = [None, None, None]
                p3[a] = plane
                p3[b] = s[b] + (t[b] - s[b]) * lam
                p3[c] = s[c] + (t[c] - s[c]) * lam
                rect_lo = (blo[b], blo[c])
                rect_hi = (bhi[b], bhi[c])
                if not (rect_lo[0] < p3[b] < rect_hi[0] and rect_lo[1] < p3[c] < rect_hi[1]):
                    val = math.dist(s, t)          # straight crossing is allowed
                else:
                    val = math.hypot(s[b] - p3[b], s[c] - p3[c]) + \
                          math.hypot(t[b] - p3[b], t[c] - p3[c])
                    # recomputed below by the exact 2D routine on (b, c)
                    val = None
                if val is None:
                    d2 = min_two_dist_outside_rect(A, B, rect_lo, rect_hi)
                    # add the perpendicular (axis a) components
                    val = None
                    # |s-p| and |t-p| split into the a component and the bc part,
                    # so the exact value needs the 3D distances: reuse the 2D
                    # optimum point set by evaluating in 3D
                    val = _min_3d_in_plane(s, t, a, plane, rect_lo, rect_hi)
                if val - eps_pos > best:
                    best = val - eps_pos
    return best


def _min_3d_in_plane(s, t, a, plane, rect_lo, rect_hi):
    """Exact min over the plane a=plane outside the rectangle of |s-p|+|p-t|."""
    b, c = [i for i in range(3) if i != a]
    denom = t[a] - s[a]
    lam = (plane - s[a]) / denom
    p_hit = [s[b] + (t[b] - s[b]) * lam, s[c] + (t[c] - s[c]) * lam]
    def f2(p):
        db = s[b] - p[0]
        dc = s[c] - p[1]
        eb = t[b] - p[0]
        ec = t[c] - p[1]
        return (math.sqrt(db * db + dc * dc + (s[a] - plane) ** 2) +
                math.sqrt(eb * eb + ec * ec + (t[a] - plane) ** 2))
    if not (rect_lo[0] < p_hit[0] < rect_hi[0] and rect_lo[1] < p_hit[1] < rect_hi[1]):
        return f2(p_hit)
    A = (s[b], s[c])
    B = (t[b], t[c])
    best = math.inf
    cands = [(rect_lo[0], rect_lo[1]), (rect_lo[0], rect_hi[1]),
             (rect_hi[0], rect_lo[1]), (rect_hi[0], rect_hi[1])]
    for axis in (0, 1):
        for val in (rect_lo[axis], rect_hi[axis]):
            A2 = list(A)
            B2 = list(B)
            A2[axis] = 2.0 * val - A[axis]
            den = B2[axis] - A2[axis]
            if abs(den) < 1e-15:
                continue
            sp = (val - A2[axis]) / den
            if not (0.0 <= sp <= 1.0):
                continue
            p = (A2[0] + (B2[0] - A2[0]) * sp, A2[1] + (B2[1] - A2[1]) * sp)
            other = 1 - axis
            if rect_lo[other] - 1e-12 <= p[other] <= rect_hi[other] + 1e-12:
                cands.append(p)
    for p in cands:
        v = f2(p)
        if v < best:
            best = v
    return best


if __name__ == "__main__":
    class Box:
        def __init__(self, lo, hi):
            self.lo, self.hi = lo, hi
    wall = Box((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))
    s = (60.0, 150.0, 0.0)
    t = (540.0, 150.0, 0.0)
    print("euclid      ", math.dist(s, t))
    print("crossing    ", crossing_bound(s, t, [wall], 5.0, 0.0))
    print("expected    ", 2 * math.hypot(240.0, 45.0))
    # sanity: a wall that does NOT separate gives the euclidean bound
    s2 = (60.0, 150.0, 0.0)
    t2 = (200.0, 150.0, 0.0)
    print("no straddle ", crossing_bound(s2, t2, [wall], 5.0, 0.0), math.dist(s2, t2))
    # a box between but easy to pass: bound must stay <= true euclid + detour
    b2 = Box((100.0, 140.0, -10.0), (110.0, 160.0, 10.0))
    print("small box   ", crossing_bound((0.0, 150.0, 0.0), (200.0, 150.0, 0.0), [b2], 5.0, 0.0),
          "expected >= 200")