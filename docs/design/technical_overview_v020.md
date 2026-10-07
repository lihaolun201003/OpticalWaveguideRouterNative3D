# OpticalWaveguideRouterNative3D - Native3D v0.2 (multi-channel baseline)

Two layers live in this repository and they are never mixed:

| Layer | Entry point | Scope | Evidence |
|---|---|---|---|
| **v0.1.3 single net** | `route_finite_native3d(scene, mode)` | one net, one finite graph, no occupancy | `artifacts/v013/` |
| **v0.2 multi-net baseline** | `native3d.multinet.route_multinet(case)` | 2 → 4 → 8 → 16 nets, sequential, published fixed occupancy | `artifacts/v020/` |

Single-net **native three-dimensional** optical waveguide router: the path is
produced by a search that runs directly in 3D from its very first expansion
(position **and** a true 3D direction), never by routing in 2D and elevating
afterwards.  Geometry only: **no optical loss model, no dB, no crosstalk.**

All lengths are **micrometres (um)**, all angles inside the geometry are radians
(angles in the configuration object are degrees, as documented there).  Every
number in the test cases is a **synthetic geometry parameter**, not a material
parameter of any paper.

## v0.2 multi-channel sequential baseline (this round)

The v0.2 layer turns the verified single-net router into a **working sequential
multi-net baseline**.  It does NOT touch the geometry kernel or the search
architecture, and it is deliberately small:

| Path | Responsibility |
|---|---|
| src/native3d/multinet/models.py | `NetSpec` / `MultiNetCase` / `NetOutcome` / `MultiNetResult`, the four-value net vocabulary (committed / verified_not_committed / rejected / no path) and `routed_count` vs `verified_count` vs `committed_count`. |
| src/native3d/multinet/occupancy.py | `Occupancy`: publishing the **actual complete straight/arc primitives** of a VERIFIED net together with `net_id`, `segment_id`, `occupancy_revision` and a content fingerprint; a failed or unverified path **cannot** be published, a net is published at most once, and `verify_immutability()` re-reads every stored curve.  `scene_for_net()` assembles the next net's Scene = deep-copied case geometry + static reference centrelines + the published occupancy. |
| src/native3d/multinet/self_spacing.py | `path_self_spacing_global()`: post-hoc **global-arclength** self-spacing over `P(s), s in [0, L]` with the check domain `|s - t| >= ell_local`.  Covers the segment against itself, multi-turn re-entry, the FAR parts of adjacent segments and non-adjacent pairs.  A violation needs a witness whose two arclengths really satisfy the domain condition. |
| src/native3d/multinet/inter_net.py | Inter-net spacing with **no local exemption**: every pair of curves of two different nets, line/line, line/arc, arc/line, arc/arc, reported as sampled estimate / certified lower bound / violation witness. |
| src/native3d/multinet/sequential.py | `route_multinet()`: one net at a time in input order; **a fresh finite graph per net** (`graph=` is never passed), `ZeroCongestion`, the published occupancy as a HARD constraint, the declared acceptance wired into the main flow, publication only for a fully verified path, and partial success recorded as partial success. |
| src/native3d/multinet/cases_v020.py | The v0.2 cases: the five two-channel minimum acceptance cases, the 2/4/8/16 cross-wall series (the first N nets of ONE ordered list) and the 4/8-channel port-permutation pressure cases. |
| src/native3d/validation/joint_validate.py | **Independent joint validator**: reads ONE JSON file, imports **only the standard library** (AST-checked in the tests), and re-derives the pose contract, the ports, C1, Rmin, boundary, obstacle clearance, the static-reference and occupancy spacing, the global-arclength self-spacing and every committed net pair.  Its self-spacing is a SECOND algorithm (rectangle edges + band line instead of polygon clipping). |
| examples/run_multichannel_baseline.py | The runnable entry point.  Writes `artifacts/v020/{cases,logs,metrics}` and feeds the JSON **read back from disk** to the independent validator. |
| examples/plot_board_layout.py | The full-board routing figures (3-D board, top view, side view, lane map, spacing-certificate panel), read from the case JSON only. |

```powershell
.venv\Scripts\python.exe examples\run_multichannel_baseline.py     # all 11 cases
.venv\Scripts\python.exe examples\plot_board_layout.py             # board figures
$env:PYTHONPATH="src"
.venv\Scripts\python.exe -m native3d.validation.joint_validate artifacts\v020\cases\crosswall_16_channels.json
.venv\Scripts\python.exe artifacts\v020\probe\v020_final_review.py # brute-force recheck
.venv\Scripts\python.exe -m pytest
```

Real results of this round (all re-derived by the independent validator, 0
disagreements): the cross-wall series commits **2/2, 4/4, 8/8 and 16/16** nets
(16 channels: 9506.548246 um total, 3.61 s wall, every net 594.159265359 um and
`LAALAAL`, z from 0 to 100); the 4-channel permutation case routes 4 candidates and
commits **0** (all four are correctly rejected by the post-hoc self-spacing check);
the 8-channel permutation case routes 6 and commits **5**, with the three distinct
failure categories `self_spacing_violation`, `search_exhausted` and
`port_attachment_failed`.  Full detail, the disclosed policy and the remaining
limitations: [docs/reports/native3d_v02_multichannel_baseline_report.md](../reports/native3d_v02_multichannel_baseline_report.md).

### v0.2 scope and limitations (do not over-read the numbers)

* **No path history.**  The search state carries no path history, so v0.2 does NOT
  claim completeness or global optimality under the self-spacing constraint.  The
  post-hoc check can REJECT a self-approaching candidate (and it does, in the
  permutation cases); it never repairs one.
* **`ell_local` is a disclosed POLICY, not a paper parameter.**  The baseline sets
  `ell_local = 2 * dmin = 10 um` as its local geometric exemption band.  It is a
  length in um, independent of the sampling step, the chord subdivision level and
  the segment index; the report states its scope and its derivation status
  (none - it is not derived from a curvature bound).
* **No net ordering, no rip-up & reroute, no history congestion.**  Partial failure
  in the pressure cases is the expected result and is reported per net.
* **The warm / prebuilt-graph branch is NOT exercised.**  Every net builds its own
  graph on its own deep-copied input, which is what isolates the known
  mutable-alias hazard of the cache path; the cache path itself is left exactly as
  v0.1.3 left it and remains unverified here.
* **The old segment-index self-spacing API is not used for any verdict.**
  `constraints.checks.path_self_spacing(exempt_adjacent=True)` still exists, but
  neither the v0.2 flow nor either validator derives a verdict from it (it cannot
  see a self-intersection inside one segment).
* Still AABB-only obstacles, still the finite coplanar connector family, still a
  finite lattice, no optical model.  The round stops at 16 channels: no 64/256/1024,
  no ML, no BPM, no GUI/GDS.

## The v0.1.3 single-net layer

v0.1.3 changes **no geometry guarantee**.  It closes the residual defects of the
**public interface and the independent validator**, each with a real pre-fix
replay against a frozen v0.1.2 source snapshot:

| # | v0.1.2 behaviour | v0.1.3 behaviour |
|---|---|---|
| A1 | `route_finite_native3d(scene_B, graph=graph_of(scene_A))` returned `success` with the **OLD** target `(100,0,0)`, cost 100, and `goal_pos_error_um = 50` recorded but not enforced | refused **before the search**: `status = graph_scene_mismatch`, `path = None`, `cost = inf`, structured diff of every contract field |
| A2 | `success` was decided by the search goal test alone; a violation of the assembled curve was relabelled `search_exhausted` | `success` requires the **actual assembled curve** to start/end on the poses the caller passed and every edge to end on its canonical node pose; otherwise `constraint_failed` / `uncertified` |
| A3 | `source == target` raised `AttributeError: 'NoneType' object has no attribute 'segments'` | a documented **zero-length success** (`zero_length_solution = True`, `cost = length = 0`, `path = None`, `n_segments = 0`) |
| B1 | `point_arc_distance` used `abs(|p-C| - R)`, the radial difference to the **sphere**; a legal path 64.031242 um above a reference arc was reported as minimum distance 0 and **rejected** | the true 3D distance to the **planar arc**, with the centre-axis and endpoint branches, witness points and the turn-angle mapping for negative / multi-turn / tilted sweeps |
| B2 | a JSON tangent was fed into a clamped dot product without normalisation: `[1,1,0]` gave `0.0 deg` and `verdict = pass` | a strict pose contract: three finite numbers, non-zero direction, then normalise (identical to production `Pose3D.from_dict`); malformed poses are rejected, never a pass |
| B3 | a basis within 1e-9 of orthonormal was accepted and the "circle" was really an ellipse while `R` stayed the stored radius | the basis is normalised inside a documented `1e-12` tolerance and rejected beyond it, so the stored radius really is the circumradius of the stored basis |

The full before/after evidence, the exact commands, the tolerance argument and
the remaining limitations are in
[docs/reports/native3d_v013_contract_geometry_repair_report.md](../reports/native3d_v013_contract_geometry_repair_report.md).
The v0.1.2 report carries a v0.1.3 errata banner; the v0.1.1 and v0.1 reports
keep their own errata banners.

## The three strategies (never mix them)

| Entry point | state_identity | Graph | Status |
|---|---|---|---|
| route(scene, mode, strategy="continuous_pose") | pose_float | continuous pose debug search; generally **infinite** | usually budget_exhausted, never "unreachable" |
| route(scene, mode, strategy="reference_graph") / reference_search() | canonical_reference_graph | v0.1.1 finite canonical reference graph, **at most two connector hops** | kept for comparison |
| route_finite_native3d(scene, mode) / strategy="finite_native3d" | finite_lattice_node_id | v0.1.2 **finite multi-hop** integer-node lattice graph | the production strategy for this round |

route(..., strategy=...) dispatches to all three and every result records which
one produced it.

## Layout and file responsibilities

| Path | Responsibility |
|---|---|
| src/native3d/identity.py | The two state identity names (pose_float default, approximate_quantised) and the v0.1.1 read alias "exact". |
| src/native3d/config.py | RouterConfig: units, tolerances, the three distance notions (centreline dmin, obstacle keep-out, envelope radius), Rmin, **target acceptance ball** (eps_pos_um / eps_ang_deg), **source pose tolerances** (source_pos_tol_um / source_ang_tol_deg), action parameters, budgets, the finite-graph parameters. |
| src/native3d/geometry/primitives.py | Pose3D, LineSegment3D, CircularArc3D (arbitrary 3D plane) with an exact analytic AABB. **v0.1.3:** the in-plane basis is normalised and then checked against `BASIS_UNIT_DEFECT_TOL` / `BASIS_ORTHOGONALITY_TOL` = 1e-12, so a "circular" arc really is a circle; the radius is **never** rounded or snapped. |
| src/native3d/geometry/path3d.py | Path3D: ordered segments, cumulative length, position/tangent evaluation, C1 report with separate non-C1 corners and curvature jumps. |
| src/native3d/search/state.py | State3D = (x, y, z, direction).  pose_key() is the default identity: **the stored pose itself, no rounding**.  key(pos_res, dir_res) is the v0.1 quantised key and is **approximate exploration only**. |
| src/native3d/search/actions.py | Motion primitives: straight, yaw, pitch, skew.  Also carries the zero-length port attachment pose pair. |
| src/native3d/search/connector.py | Terminal connector: planar Dubins arc-straight-arc (LSL/RSR/LSR/RSL) for coplanar pose pairs. |
| src/native3d/search/congestion.py | Fixed congestion field interface, zero field, box-region field (>= 0 um equivalent).  **v0.1.3:** `identity()` gives a stable token so a reused graph can prove the cost field did not change. |
| src/native3d/search/astar.py | A*: cost = real_path_length + nonnegative congestion, h = max(0, |target - pos| - eps_pos).  Correct termination (goal popped with minimal f), stale heap entries discarded, improved g re-opens a node, and a **documented cost comparison tolerance** (G_COMPARISON_ABS_TOL_UM, G_COMPARISON_REL_TOL, g_improves).  **v0.1.3 additive:** `zero_length_solution`, `n_segments`, `endpoint_errors`, `port_assurance`, and an explicit `"path": null`. |
| src/native3d/search/statuses.py | **v0.1.3:** the result-status vocabulary in ONE place: success, search_exhausted, budget_exhausted, port_attachment_failed, graph_scene_mismatch, invalid_source_pose, invalid_target_pose, unsupported_pose, constraint_failed, uncertified.  `SEARCH_TERMINATIONS` is exactly (success, search_exhausted, budget_exhausted). |
| src/native3d/search/router.py | Strategy dispatch plus the continuous-pose debug engine. |
| src/native3d/search/finite_graph.py | **v0.1.2 production router, hardened in v0.1.3**: integer node ids (i, j, k, direction), six axis directions, straights of n*h, quarter turns of radius m*h, deterministic successors for every node, validated port attachments, per-edge continuous constraint checks, independent-Dijkstra-checkable edge enumeration.  **v0.1.3 adds** the immutable construction snapshot + `graph_scene_mismatch` refusal, the real pose-key bijection guard, per-edge end-pose/junction rejection, `verify_assembled_endpoints()`, `port_assurance` / `endpoint_errors`, and the derived edge upper bound. |
| src/native3d/search/reference_graph.py | The v0.1.1 finite canonical reference graph (max two hops), kept for comparison. |
| src/native3d/constraints/checks.py | Hard constraints: min bend radius, 3D obstacle keep-out, centreline spacing, chip-volume boundary, pose tolerances.  Contains the **certified** segment/segment decision (segment_segment_distance_certified: stable |d1 x d2|^2 denominator + explicit error bound + **exact rational fallback**), the **sagitta domain rule** (chord_count_for_domain, sagitta_bound_um, MAX_CHORD_SWEEP_RAD = pi) and the **witness rule** (a point of the curve inside the keep-out wins over any lower bound). |
| src/native3d/validation/validate.py | **Independent** validator: JSON only, imports nothing from the router.  Checks source AND target pose with their own tolerances, decides line/line, line/arc, arc/line and arc/arc reference centrelines, rejects unsupported or malformed input, enforces the sagitta domain, and refuses to report a pass when a lower bound exceeds the sampled upper bound.  **v0.1.3 adds** the true 3D point-to-planar-arc distance with a witness, the strict pose contract, the mirrored circular-basis contract, and `validate_zero_length_artifact`. |
| src/native3d/io/* | Scene (chip, obstacles, fixed reference centrelines, source/target pose, config), the evidence writer (self_spacing_check is recorded as NOT IMPLEMENTED), parameterised evidence directory (NATIVE3D_ARTIFACT_DIR, default artifacts/v013/cases).  A zero-length success is independently validated instead of being skipped. |
| src/native3d/cases.py | The synthetic case definitions. |
| src/native3d/multinet/* | **v0.2** multi-channel layer: models, occupancy (publication rules), the global-arclength self-spacing check, inter-net spacing, the sequential scheduler, the v0.2 cases.  All inside `artifacts/v020/`; nothing in the v0.1.3 layer changes. |
| src/native3d/validation/joint_validate.py | **v0.2 independent joint validator**: JSON only, standard library only, own geometry and own self-spacing algorithm; re-derives ports, C1, Rmin, boundary, obstacles, spacing, self-spacing and the committed-set invariant; a malformed artifact is rejected with `ContractError`. |
| tests/ | pytest suite: phases 1-7, the v0.1.1 repair regressions, the **v0.1.2 A1-A5 regressions**, the finite-graph tests, the generality matrix, the independent Dijkstra cross-check, the **v0.1.3 A/B/D regressions** (including `v013_helpers.py`, which loads the frozen v0.1.2 snapshot as `native3d_prefix` so the red-to-green transition is provable from a test), and the **v0.2 multichannel regressions** (`test_v020_*.py`, 49 tests). |
| examples/run_core_case.py | Regenerates the core-case evidence (all three strategies) under the parameterised directory. |
| examples/run_multichannel_baseline.py | **v0.2**: runs the multi-channel cases and writes `artifacts/v020/{cases,logs,metrics}`. |
| examples/plot_board_layout.py | **v0.2**: the full-board routing figures from the case JSON. |
| artifacts/v013/ | **v0.1.3 evidence**: `cases/`, `logs/`, `probe/`, and `prefix_src/` (a byte-for-byte v0.1.2 source snapshot taken before the first v0.1.3 edit).  artifacts/cases/, artifacts/v01_preserved/, artifacts/v011/, artifacts/v011_preserved/ and artifacts/v012/ are **frozen** and never written by a v0.1.3 run. |
| artifacts/v020/ | **v0.2 evidence**: `cases/` (`<case>.json` + `<case>.validated.json`), `logs/`, `metrics/` (`summary.json`), `figures/` (board layouts), `probe/` (the pre-implementation feasibility spikes and the adversarial final review with its JSON result). |

## Input contract (Scene)

    {
      "name": "...", "units": "um",
      "chip_volume": {"lo": [x,y,z], "hi": [x,y,z]},
      "obstacles": [{"lo": [...], "hi": [...]}],
      "reference_centerlines": [{"kind": "line", "p0": [...], "p1": [...]},
                                {"kind": "arc", "center": [...], "u": [...],
                                 "v": [...], "radius_um": R, "theta_rad": t}],
      "source_pose": {"position": [...], "tangent": [...]},
      "target_pose": {"position": [...], "tangent": [...]},
      "config": { ...RouterConfig... }
    }

Reference centrelines may be lines **or arcs**; both are validated by the
production checks and by the independent validator.

**Pose contract (v0.1.3, identical in production and in the validator):** a pose
is three finite position numbers plus a **non-zero** three-number direction.
A legal direction is **normalised** (`Pose3D.from_dict` does this, and the
validator does the same); zero, NaN, +-Infinity, a missing field, a wrong
dimension or a non-numeric component is **rejected**.  The independent validator
publishes this as `pose_contract` in every result.

## Output contract

SearchResult / the artifact JSON contains:

* status: success | search_exhausted | budget_exhausted | invalid_source_pose |
  invalid_target_pose | unsupported_pose | port_attachment_failed |
  **graph_scene_mismatch** | **constraint_failed** | uncertified.
  A budget stop is **never** reported as unreachable, an unsupported port is
  **never** reported as physically unreachable, a cache/scene mismatch is
  **never** a search result, and an assembled-curve failure is **never**
  `search_exhausted`;
* **zero_length_solution / n_segments / endpoint_errors / port_assurance**
  (v0.1.3).  A zero-length success and a failure both have `path = None`; the
  flag is what separates them, and no degenerate micro-segment is ever created;
* state_identity and approximate_state_identity;
* the raw searched path (ordered segments, cumulative lengths) - no
  post-processing step exists;
* trace with the action, x/y/z, direction, azimuth, elevation and g of every
  node of the chain - the z-evolution and per-segment provenance evidence;
* initial_candidates: every action generated from the initial state of that
  strategy, with its z/pitch change and validation result;
* cost_um_equivalent, length_um, congestion_cost_um_equivalent, expansions,
  generated, rejected, rejection_summary;
* finite_graph (finite strategy only): node/edge **upper bounds computed from
  the construction** (the edge bound no longer carries a hardcoded `+4096`),
  whether the node bound includes the virtual ports, port attachment report,
  `graph_reuse` (accepted / mismatches / mismatch_detail / graph_cache_state
  cold-or-warm / graph_build_count / successor_cache_hits / the full content
  fingerprint), edge validation counters **split by phase**
  (`edge_check_s_graph_build` + `edge_check_s_search`), the maximum curve-end vs
  canonical-node error, the assembled-path production check and the guarantee
  statement;
* timings: graph build, search, **assembled-path check**, end-to-end (cold
  cache);
* independent_validation: the second-implementation re-check with the sampled
  estimate, the analytic distance, the certified lower bound, the certification
  state of every check, the source/target pose checks, the certification self
  check, the pose contract and the zero-length contract.

### Reusing a prebuilt graph (v0.1.3)

`route_finite_native3d(scene, ..., graph=prebuilt)` is still supported - the
signature is unchanged - but the graph is accepted **only** when an immutable
construction snapshot matches the call by **content**:

* chip volume; **every** obstacle box; **every** reference centreline's **full**
  geometry (not a count); source and target position **and** 3D tangent; mode;
* every config field that can change an edge's geometry, feasibility or cost
  (spacing, straight steps, turn radius steps, Rmin, radius tolerance, dmin,
  obstacle clearance and whether it includes the envelope, envelope radius,
  boundary margin, clearance decision tolerance, subdivision cap, port
  attachment radius and node cap, connector radii, position/tangent/source
  tolerances, the target acceptance ball, budgets);
* the congestion field's identity - an unprovable identity is a mismatch, never
  a silent reuse of stale edge costs.

`Scene` is a **mutable** dataclass, so object identity proves nothing: the
comparison is by content.  A mismatch is refused **before any search expansion**
with `status = graph_scene_mismatch` and the structured diff above.  A genuinely
matching graph is a legal, documented reuse and is reported as a **warm** reuse;
the default branch builds its own **cold** graph.  The frozen 60 s / 200 000
expansion acceptance budget belongs to the **cold default branch** and is never
extrapolated to the warm one.

## Run the tests

    cd "C:\Users\lihao\Desktop\Graduation Project\OpticalWaveguideRouterNative3D"
    .venv\Scripts\python.exe -m pytest

The suite is 617 tests as of v0.2 (568 preserved from v0.1.3 plus 49 new
multi-channel regressions); the authoritative run of this round was
**617 passed / 0 failed / exit code 0 / 185.96 s**
(`artifacts/v020/logs/01_pytest_full.log`).

Regenerate the core-case evidence (all three strategies, timings, peak memory)
under artifacts/v013/cases/:

    .venv\Scripts\python.exe examples\run_core_case.py

Run the v0.1.3 acceptance probe (the original cross-wall case with a cold/warm
timing split and an independent Dijkstra cross-check, the new legal-3D-over-a-
reference-arc acceptance with its crossing / inside-circle / too-close controls,
the refused mismatched graph and the zero-length contract):

    .venv\Scripts\python.exe artifacts\v013\probe\v013_acceptance_probe.py

Re-run the v0.1.3 counterexamples on the **frozen v0.1.2 source** (this is where
the "before" numbers come from) and the PART A / PART B probes:

    .venv\Scripts\python.exe artifacts\v013\probe\v013_prefix_replay.py
    .venv\Scripts\python.exe artifacts\v013\probe\v013_finite_router_probe.py
    .venv\Scripts\python.exe artifacts\v013\probe\v013_validation_probe.py

Run the Lead's independent adversarial review of the six delivered claims (it
re-derives the arc distance from a dense minimisation of the curve, attacks the
graph contract one field at a time, and checks the preserved cross-wall numbers
against the frozen artifact):

    .venv\Scripts\python.exe artifacts\v013\probe\v013_final_review.py

Re-verify a stored artifact from the file alone (no router code involved; the
module imports only the standard library).  A zero-length artifact is validated
too, instead of being skipped:

    $env:PYTHONPATH = "src"
    .venv\Scripts\python.exe -m native3d.validation.validate artifacts\v013\cases\core_case_finite_native3d_native3d.json
    .venv\Scripts\python.exe -m native3d.validation.validate artifacts\v013\cases\v013_zero_length_solution.json

## The finite production router in one paragraph

route_finite_native3d(scene, mode) builds the node set (i, j, k, d) over the chip
volume shrunk by the boundary margin, anchored at the source position, with six
axis directions.  Every node id has exactly one canonical pose; every successor
is generated from that pose (straights of n * finite_spacing_um and quarter turns
of radius m * finite_spacing_um in a coordinate plane).  The node and edge counts
are bounded **by construction** and reported.  Every edge is checked as a full
continuous curve before it may enter OPEN, and its curve must **end on the
canonical pose of the target node** within the documented C1 tolerances or the
edge is rejected (`curve_end_misses_the_canonical_pose`).  The source and target
are ports: if the port pose is itself a canonical pose the node is the port,
otherwise a validated local attachment curve connects them inside a generic
attachment neighbourhood (finite_port_attach_um,
finite_port_attach_max_nodes).  Before `success` is returned, the ASSEMBLED
curve is measured against the poses the caller actually passed: source position
and tangent with the source tolerances, target position and tangent with the
acceptance ball, and every junction with the C1 tolerances.  No waypoint, no
target height and no route is ever supplied by the caller.

## Scope and limitations (summary - see the v0.1.3 report)

* One net only; no multi-net scheduling, no 3D occupancy update, no rip-up and
  reroute, no history congestion (the congestion field is fixed per search).
* **State identity.** pose_float never merges two different stored poses.  It is
  not continuous-space completeness: the enumerated graph of the continuous-pose
  strategy is generally infinite, the identity is the identity of this program's
  floats, and the budget still truncates.  Because nothing is merged, the same
  geometric pose reached through a different evaluation order can appear as two
  nodes - a size/completeness cost, never a correctness cost.
* **Cost optimality** is claimed for the enumerated finite graph only, up to the
  documented comparison tolerance g_improves (1e-12 um absolute + 1e-12
  relative).  It is epsilon-optimality, not optimality over the reals.
* **Self-spacing is NOT enforced and NOT post-checked.**  The search state has no
  path history, and although constraints.checks.path_self_spacing() exists it is
  called by no production code path and by no validator path; it enters no
  verdict.  This is recorded as **not implemented** in the README, the reports and
  every v0.1.3 artifact.  Path-history search is out of scope this round, and it
  is the first item of the next round (see the v0.1.3 report, section 8).
* **Obstacles are AABB only** (no meshes / CSG).
* **Discrete-action limitation** of the finite strategy: a lattice of spacing h
  with quarter turns of radius m*h cannot reproduce every continuous route, so a
  longer legal route than the v0.1.1 reference graph produced (594.159 um vs
  495.301 um on the core case) is expected and disclosed.
* **Terminal connectors** for arbitrary (non-canonical) port tangents use the
  v0.1.1 planar Dubins family: coplanar pose pairs only, no LRL/RLR.  A missing
  connector or attachment means "the supported action set found none", never
  physical unreachability; the status is port_attachment_failed.
* **Numeric scale of the certified decisions:** coordinates and lengths from
  1e-9 um to 1e9 um; below the documented error floor
  (SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9 um) or outside the conditioning of the
  float path, the decision falls back to exact rational arithmetic.
* **Arc basis tolerance (v0.1.3):** the in-plane basis of a circular arc is
  normalised and then rejected when its unit or orthogonality defect exceeds
  1e-12.  Measured over **114 843** real arcs built in-process, the worst unit
  defect is 2.22e-16, the worst orthogonality defect 1.39e-16, the worst
  `|norm(start - center) - R|` is 5.684e-14 um and the worst relative
  `| |P(phi) - C| / R - 1 |` is 1.33e-15, so the threshold has a ~4500x margin
  over real construction.  A basis that is not orthonormal is **rejected**, never
  silently renormalised and then certified.
* **Conservative lower bounds.**  The production segment/arc spacing check
  returns a certified **lower** bound (chord subdivision with the sagitta
  subtracted), not the exact distance: on the legal 64.031242 um fly-over it
  reports 50.0 um and still decides "safe" correctly.  The independent validator
  is the component that reports the achieved distance.
* **`tracemalloc` peak** quoted anywhere in the evidence is the peak **tracked
  Python allocation** of the traced region.  It is **not** the process RSS and
  not the whole interpreter's memory.
* No optical model: no loss, no crosstalk, no dB anywhere (AST-checked).
