"""Full-board routing layout figure for the v0.2 multi-channel baseline.

READS THE CASE JSON ARTIFACTS ONLY (the same files the independent joint validator
consumes) - it imports nothing from the search, and it never re-routes anything.
Every drawn curve is one primitive of one committed net exactly as it was
published, plus the wall/obstacle boxes and the chip volume.

    cd "C:\\Users\\lihao\\Desktop\\Graduation Project\\OpticalWaveguideRouterNative3D"
    .venv\\Scripts\\python.exe examples\\plot_board_layout.py [--only NAME]

Writes PNG figures under artifacts/v020/figures/:

    board_layout_<case>.png     the per-case board sheet: 3-D view, top view,
                               side view, and the port/lane detail
    board_layout_all16.png      the consolidated 16-channel sheet with every
                               committed net of the must-pass baseline
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
import numpy as np                                    # noqa: E402
from matplotlib.lines import Line2D                   # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
CASES = os.path.join(_ROOT, "artifacts", "v020", "cases")
FIGS = os.path.join(_ROOT, "artifacts", "v020", "figures")

# Distinct, print-friendly colours; cycled if a case has more nets.
PALETTE = [
    "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b",
    "#17becf", "#e377c2", "#7f7f7f", "#bcbd22", "#393b79", "#637939",
    "#8c6d31", "#843c39", "#7b4173", "#3182bd",
]


def load(case_name):
    """Read a case artifact; prefer the .validated.json (it carries the verdict).

    Both files hold the SAME geometry; the validated one additionally carries the
    result of the independent joint validator, which the sheet reports.
    """
    validated = os.path.join(CASES, "%s.validated.json" % case_name)
    path = validated if os.path.exists(validated) else \
        os.path.join(CASES, "%s.json" % case_name)
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def polyline_of(seg, n=64):
    """Dense polyline of ONE serialised primitive (line or arc)."""
    if seg["kind"] == "line":
        p0, p1 = seg["p0"], seg["p1"]
        return np.array([[p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]]])
    c = seg["center"]
    u = seg["u"]
    v = seg["v"]
    R = float(seg["radius_um"])
    th = float(seg["theta_rad"])
    t = np.linspace(0.0, th, n)
    x = c[0] + R * (np.cos(t) * u[0] + np.sin(t) * v[0])
    y = c[1] + R * (np.cos(t) * u[1] + np.sin(t) * v[1])
    z = c[2] + R * (np.cos(t) * u[2] + np.sin(t) * v[2])
    return np.vstack([x, y, z])


def net_path(net):
    """Concatenated committed path of one net, plus its per-segment pieces."""
    pieces = [polyline_of(s) for s in net["segments"]]
    if not pieces:
        return None, []
    return np.hstack(pieces), pieces


def box_faces(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    c = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
         [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
    idx = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
           (2, 3, 7, 6), (3, 0, 4, 7)]
    return [[c[i] for i in f] for f in idx]


def committed(payload):
    return [n for n in payload["nets"] if n.get("committed") and n.get("segments")]


def draw_3d(ax, payload, nets, colour_of):
    chip = payload["chip_volume"]
    lo, hi = chip["lo"], chip["hi"]
    for face in box_faces(lo, hi):
        ax.add_collection3d(Poly3DCollection(
            [face], facecolors="#eef3fa", edgecolors="#9bb4d0",
            linewidths=0.4, alpha=0.10))
    for ob in payload.get("obstacles", []):
        for face in box_faces(ob["lo"], ob["hi"]):
            ax.add_collection3d(Poly3DCollection(
                [face], facecolors="#c9c9c9", edgecolors="#555555",
                linewidths=0.5, alpha=0.55))
    zmax = max([hi[2]] + [max(net_path(n)[0][2]) for n in nets if net_path(n)[0].size])
    for n in nets:
        p, pieces = net_path(n)
        col = colour_of[n["net_id"]]
        ax.plot(p[0], p[1], p[2], color=col, lw=1.9, solid_capstyle="round",
                label=n["net_id"])
        for pc in pieces:
            if pc.shape[1] >= 3:
                ax.plot(pc[0], pc[1], pc[2], color="#111111", lw=0.55, alpha=0.85)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], zmax + 0.12 * (zmax - lo[2] + 1e-9))
    ax.set_xlabel("x (um)", fontsize=7, labelpad=-2)
    ax.set_ylabel("y (um)", fontsize=7, labelpad=-2)
    ax.set_zlabel("z (um)", fontsize=7, labelpad=-2)
    ax.tick_params(labelsize=6, pad=-1)
    ax.view_init(elev=20, azim=-58)
    ax.set_title("(a) full-board 3-D view", fontsize=9, loc="left")


def draw_top(ax, payload, nets, colour_of):
    for ob in payload.get("obstacles", []):
        lo, hi = ob["lo"], ob["hi"]
        ax.add_patch(plt.Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1],
                                   facecolor="#d8d8d8", edgecolor="#666666",
                                   lw=0.6, zorder=1))
    for n in nets:
        p, _ = net_path(n)
        ax.plot(p[0], p[1], color=colour_of[n["net_id"]], lw=1.7, zorder=3)
        ax.plot(p[0][0], p[1][0], marker="o", ms=3.4, mfc="white",
                mec=colour_of[n["net_id"]], mew=0.8, zorder=4)
        ax.plot(p[0][-1], p[1][-1], marker="s", ms=3.4, mfc=colour_of[n["net_id"]],
                mec="black", mew=0.4, zorder=4)
    chip = payload["chip_volume"]
    ax.set_xlim(chip["lo"][0], chip["hi"][0])
    ax.set_ylim(chip["lo"][1], chip["hi"][1])
    ax.set_aspect("equal")
    ax.set_xlabel("x (um)", fontsize=8)
    ax.set_ylabel("y (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4)
    ax.set_title("(b) top view (x-y): ports 0 and lanes", fontsize=9, loc="left")


def draw_side(ax, payload, nets, colour_of):
    for ob in payload.get("obstacles", []):
        lo, hi = ob["lo"], ob["hi"]
        ax.add_patch(plt.Rectangle((lo[0], lo[2]), hi[0] - lo[0], hi[2] - lo[2],
                                   facecolor="#d8d8d8", edgecolor="#666666",
                                   lw=0.6, zorder=1))
    prof = []
    for n in nets:
        p, _ = net_path(n)
        prof.append(p)
        ax.plot(p[0], p[2], color=colour_of[n["net_id"]], lw=1.7, zorder=3)
    same = len(prof) > 1 and all(
        q.shape == prof[0].shape
        and np.allclose(q[0], prof[0][0], atol=1e-9)
        and np.allclose(q[2], prof[0][2], atol=1e-9) for q in prof[1:])
    chip = payload["chip_volume"]
    ax.set_xlim(chip["lo"][0], chip["hi"][0])
    ax.set_ylim(chip["lo"][2], chip["hi"][2])
    ax.set_xlabel("x (um)", fontsize=8)
    ax.set_ylabel("z (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4)
    ax.axhline(0.0, color="#bbbbbb", lw=0.6, ls=":")
    ttl = "(c) side view (x-z): the 3-D detours over the wall"
    if same:
        ttl += "\n(all %d x-z profiles coincide exactly)" % len(prof)
    ax.set_title(ttl, fontsize=9, loc="left")


def draw_lanes(ax, payload, nets, colour_of):
    """Port/lane cross-section: the y coordinate at the two port planes."""
    for n in nets:
        s = n["input"]["source_pose"]["position"]
        t = n["input"]["target_pose"]["position"]
        col = colour_of[n["net_id"]]
        ax.plot([0.0, float(n["length_um"])], [s[1], t[1]], color=col, lw=1.4,
                marker="o", ms=3.0, mfc="white", mec=col)
        ax.annotate(n["net_id"], (float(n["length_um"]), t[1]), fontsize=5.5,
                    color=col, xytext=(3, 0), textcoords="offset points",
                    va="center")
    ax.set_xlabel("path length (um)", fontsize=8)
    ax.set_ylabel("lane y (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4)
    ax.set_title("(d) lane map: source y -> target y vs path length",
                 fontsize=9, loc="left")


def draw_spacing_certificate(ax, payload, nets, colour_of):
    """The quantitative result: certified spacing per net against the requirement.

    Three series per net, all taken from the artifact:
      * certified LOWER bound of the spacing to the published occupancy
        (the independent validator's number, when the .validated.json was read);
      * certified LOWER bound of the assembled-path self-spacing;
      * the dmin requirement line.
    A net without a committed path is marked as such instead of being drawn as 0.
    """
    joint = payload.get("joint_validation_from_file") or {}
    by_id = {n["net_id"]: n for n in (joint.get("nets") or [])}
    labels, pub, self_, missing = [], [], [], []
    for n in payload["nets"]:
        labels.append(n["net_id"])
        jn = by_id.get(n["net_id"]) or {}
        p = (jn.get("published_occupancy_spacing") or {}).get(
            "min_certified_lower_bound_um")
        s = (jn.get("self_spacing") or {}).get("certified_lower_bound_um")
        pub.append(p)
        self_.append(s)
        if not n.get("committed"):
            missing.append(n["net_id"])
    xs = np.arange(len(labels))
    dmin = payload["config"]["dmin_centerline_um"]

    def series(vals, off, colour, name):
        xv = [x for x, v in zip(xs, vals) if isinstance(v, (int, float))]
        yv = [v for v in vals if isinstance(v, (int, float))]
        if xv:
            ax.bar([x + off for x in xv], yv, width=0.38, color=colour,
                   label=name, zorder=3)

    series(pub, -0.20, "#2b6cb0", "spacing to published occupancy (certified LB)")
    series(self_, 0.20, "#c05621", "assembled-path self-spacing (certified LB)")
    ax.axhline(dmin, color="#b00020", lw=1.2, ls="--", zorder=4,
               label="dmin = %g um (requirement)" % dmin)
    vals = [v for v in list(pub) + list(self_) if isinstance(v, (int, float))]
    top = max(vals + [dmin]) if vals else dmin
    ax.set_ylim(0.0, top * 1.30 + 0.5)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=6, rotation=90)
    ax.set_ylabel("certified lower bound (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4, axis="y")
    ax.legend(fontsize=6.2, loc="upper left", framealpha=0.9)
    note = ""
    if missing:
        note = "\nnot drawn (no committed path): %s" % ", ".join(missing)
    ax.set_title("(e) spacing certificate per net, re-derived from the JSON%s" % note,
                 fontsize=9, loc="left")


def draw_zprofile(ax, payload, nets, colour_of):
    zs = []
    for n in nets:
        p, _ = net_path(n)
        seglen = np.sqrt(np.diff(p[0]) ** 2 + np.diff(p[1]) ** 2
                         + np.diff(p[2]) ** 2)
        s = np.concatenate([[0.0], np.cumsum(seglen)])
        zs.append(p[2])
        ax.plot(s, p[2], color=colour_of[n["net_id"]], lw=1.5)
    same = False
    if len(zs) > 1:
        same = all(len(z) == len(zs[0]) and np.allclose(z, zs[0], atol=1e-9)
                   for z in zs[1:])
    ax.set_xlabel("arclength s of the net (um)", fontsize=8)
    ax.set_ylabel("z (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4)
    ttl = "(e) per-net z(s): every height change is a search action"
    if same:
        ttl += "\n(all %d profiles coincide exactly; the nets differ only in y)" % len(zs)
    ax.set_title(ttl, fontsize=9, loc="left")


def draw_wall_section(ax, payload, nets, colour_of, x_plane=300.0, slab=1.0):
    """Cross-section in the y-z plane at the obstacle: the 16 real lanes."""
    for ob in payload.get("obstacles", []):
        lo, hi = ob["lo"], ob["hi"]
        if lo[0] - 1e-9 <= x_plane <= hi[0] + 1e-9:
            ax.add_patch(plt.Rectangle((lo[1], lo[2]), hi[1] - lo[1], hi[2] - lo[2],
                                       facecolor="#d8d8d8", edgecolor="#666666",
                                       lw=0.7, zorder=1))
    ys = []
    for n in nets:
        p, pieces = net_path(n)
        hits = []
        for pc in pieces:
            for k in range(pc.shape[1]):
                if abs(pc[0][k] - x_plane) <= slab:
                    hits.append((pc[1][k], pc[2][k]))
        if hits:
            hy = float(np.mean([h[0] for h in hits]))
            hz = float(np.mean([h[1] for h in hits]))
            ys.append(hy)
            ax.plot(hz, hy, marker="o", ms=5.0, mfc=colour_of[n["net_id"]],
                    mec="black", mew=0.4, zorder=4)
    ax.set_xlabel("z (um)  <- height of the crossing", fontsize=8)
    ax.set_ylabel("lane y (um)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, lw=0.4)
    note = ""
    if len(ys) >= 2:
        ys_sorted = sorted(ys)
        pitch = min(b - a for a, b in zip(ys_sorted, ys_sorted[1:]))
        dmin = payload["config"]["dmin_centerline_um"]
        note = ("\nminimum lane pitch %.1f um vs dmin %.1f um" % (pitch, dmin))
    ax.set_title("(e) wall-plane cross-section (y-z at x = %g): the routed lanes"
                 "%s" % (x_plane, note), fontsize=9, loc="left")


def fig_for_case(case_name, out_name=None, only_committed=True):
    payload = load(case_name)
    nets = committed(payload) if only_committed else [
        n for n in payload["nets"] if n.get("segments")]
    if not nets:
        print("  %s: no committed net to draw" % case_name)
        return None
    colour_of = {}
    for k, n in enumerate(payload["nets"]):
        colour_of[n["net_id"]] = PALETTE[k % len(PALETTE)]

    fig = plt.figure(figsize=(16.5, 10.6), dpi=150)
    gs = fig.add_gridspec(2, 3, hspace=0.30, wspace=0.24,
                          left=0.045, right=0.985, top=0.93, bottom=0.06)
    ax3d = fig.add_subplot(gs[0, 0], projection="3d")
    draw_3d(ax3d, payload, nets, colour_of)
    draw_top(fig.add_subplot(gs[0, 1]), payload, nets, colour_of)
    draw_side(fig.add_subplot(gs[0, 2]), payload, nets, colour_of)
    draw_lanes(fig.add_subplot(gs[1, 0]), payload, nets, colour_of)
    draw_spacing_certificate(fig.add_subplot(gs[1, 1]), payload, nets, colour_of)

    axt = fig.add_subplot(gs[1, 2])
    axt.axis("off")
    counts = payload["counts"]
    joint = payload.get("joint_validation_from_file") or {}
    occ = payload["occupancy"]
    n_pub = occ.get("n_published_entries")
    if n_pub is None:
        n_pub = sum(1 for e in (occ.get("entries") or [])
                    if e.get("origin") == "committed_net_path")
    immut = occ.get("immutability") or {}
    rows = [
        ["case", payload["case"]],
        ["channels", str(payload["channel_count"])],
        ["order", " -> ".join(payload["order"])],
        ["routed", str(counts["routed_count"])],
        ["verified", str(counts["verified_count"])],
        ["committed", str(counts["committed_count"])],
        ["failed", str(counts["failed_count"])],
        ["failure categories", json.dumps(counts["failure_categories"]) or "{}"],
        ["total length (um)", "%.3f" % counts["total_length_um"]],
        ["total net wall (s)", "%.2f" % payload["totals"]["sum_net_wall_time_s"]],
        ["occupancy revision", str(occ["revision"])],
        ["published curves", str(n_pub)],
        ["occupancy immutable", str(immut.get("ok"))],
        ["ell_local (um)", "%g" % payload["self_spacing_policy"]["ell_local_um"]],
        ["independent verdict", str(joint.get("overall_verdict"))],
        ["independent disagreements", str((joint.get("counts") or {})
                                          .get("n_disagreements"))],
    ]
    body = [[r[0], r[1]] for r in rows]
    tbl = axt.table(cellText=body, colWidths=[0.44, 0.56], loc="upper center",
                    cellLoc="left")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.0)
    tbl.scale(1.0, 1.28)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_linewidth(0.4)
        if c == 0:
            cell.set_text_props(color="#334455")
    axt.set_title("(f) case summary (from the artifact JSON only)",
                  fontsize=9, loc="left")

    handles = [Line2D([0], [0], color=colour_of[n["net_id"]], lw=2.2,
                      label=n["net_id"]) for n in nets]
    if payload.get("obstacles"):
        handles.append(plt.Rectangle((0, 0), 1, 1, facecolor="#c9c9c9",
                                     edgecolor="#555555", label="obstacle"))
    fig.legend(handles=handles, loc="lower center", ncol=min(len(handles), 16),
               fontsize=7.5, frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.suptitle("Native3D v0.2 multi-channel sequential baseline - full-board "
                 "routing layout: %s" % payload["case"],
                 fontsize=12.5, y=0.975)
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, out_name or ("board_layout_%s.png" % case_name))
    fig.savefig(out)
    plt.close(fig)
    print("  wrote %s" % out)
    return out


def fig_all16():
    """The consolidated 16-channel must-pass board sheet."""
    payload = load("crosswall_16_channels")
    nets = committed(payload)
    colour_of = {n["net_id"]: PALETTE[k % len(PALETTE)]
                 for k, n in enumerate(payload["nets"])}
    fig = plt.figure(figsize=(17.0, 11.5), dpi=150)
    gs = fig.add_gridspec(2, 2, hspace=0.24, wspace=0.20, left=0.05, right=0.98,
                          top=0.93, bottom=0.07)
    ax3d = fig.add_subplot(gs[0, 0], projection="3d")
    draw_3d(ax3d, payload, nets, colour_of)
    ax3d.set_title("(a) 16 channels, complete board, 3-D", fontsize=10, loc="left")
    draw_top(fig.add_subplot(gs[0, 1]), payload, nets, colour_of)
    draw_side(fig.add_subplot(gs[1, 0]), payload, nets, colour_of)
    ax = fig.add_subplot(gs[1, 1])
    draw_spacing_certificate(ax, payload, nets, colour_of)
    handles = [Line2D([0], [0], color=colour_of[n["net_id"]], lw=2.2,
                      label=n["net_id"]) for n in nets]
    handles.append(plt.Rectangle((0, 0), 1, 1, facecolor="#c9c9c9",
                                 edgecolor="#555555", label="wall / obstacle"))
    fig.legend(handles=handles, loc="lower center", ncol=8, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    joint = payload.get("joint_validation_from_file") or {}
    fig.suptitle("Native3D v0.2 - 16-channel cross-wall must-pass baseline: "
                 "%d/%d committed, independent joint verdict %s"
                 % (payload["counts"]["committed_count"], payload["channel_count"],
                    joint.get("overall_verdict")),
                 fontsize=13, y=0.972)
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, "board_layout_all16.png")
    fig.savefig(out)
    plt.close(fig)
    print("  wrote %s" % out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="v0.2 board-layout figures")
    ap.add_argument("--only", default=None)
    args = ap.parse_args(argv)
    if not os.path.isdir(CASES):
        print("no artifacts yet: run examples/run_multichannel_baseline.py first")
        return 2
    names = sorted(f[:-5] for f in os.listdir(CASES)
                   if f.endswith(".json") and not f.endswith(".validated.json"))
    if args.only:
        wanted = [s.strip() for s in args.only.split(",")]
        names = [n for n in names if any(w in n for w in wanted)]
    print("board layout figures from the case JSON artifacts:")
    for n in names:
        try:
            fig_for_case(n)
        except Exception as exc:                       # keep the others going
            print("  %s: FAILED to plot (%s: %s)" % (n, type(exc).__name__, exc))
    if "crosswall_16_channels" in names:
        fig_all16()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
