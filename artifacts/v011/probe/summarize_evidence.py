import json, os
ART = "artifacts/v011/cases"
def load(n): return json.load(open(os.path.join(ART, n), encoding="utf-8"))
a = load("core_case_reference_graph_native3d.json")
s = a["search"]; v = a["independent_validation"]
print("== reference native3d ==")
print("status", s["status"], "cost", s["cost_um_equivalent"], "len", s["length_um"],
      "exp", s["expansions"], "n_mids", s["reference_graph"]["n_canonical_mids"])
print("trace", [(t["action"], round(t["x_um"],3), round(t["y_um"],3), round(t["z_um"],3)) for t in s["trace"]])
print("z summary", {k: a["trace_z_summary"][k] for k in ("z_min_um","z_max_um","z_varies","actions_changing_z")})
print("verdict", v["verdict"], "minR", v["min_radius_um_refit"], "c1", v["c1_continuous"])
print("obstacle", {k: v["obstacle_clearance"][k] for k in ("required_um","min_sampled_clearance_um","analytic_min_clearance_um","conservative_clearance_lower_bound_um","status")})
print("spacing", {k: v["centerline_spacing"][k] for k in ("required_um","min_sampled_um","conservative_lower_bound_um","status")})
print("boundary", {k: v["boundary"][k] for k in ("min_margin_um_sampled","analytic_min_margin_um","analytic_overshoot_um","certified_contained")})
print("endpoints", {k: v["endpoint_errors"][k] for k in ("start_position_error_um","start_tangent_error_deg","end_position_error_um","end_tangent_error_deg")})
print("cert", v["certification"])
b = load("core_case_reference_graph_2d.json")
print("\n== reference 2d ==", b["search"]["status"], b["search"]["budget_exhausted"], b["search"]["expansions"])
print("reason:", b["search"]["reason"][:200])
c = load("core_case_2d_mode.json"); print("\n== production 2d ==", c["search"]["status"], c["search"]["expansions"], c["search"]["budget"])
d = load("core_case_native3d_mode.json"); print("== production native3d ==", d["search"]["status"], d["search"]["expansions"], d["search"]["budget"])
e = load("core_case_initial_action_probe.json")
print("== probe ==", e["2d_mode"]["n_accepted_z_changing"], e["native3d_mode"]["n_accepted_z_changing"], e["2d_mode"]["n_candidates"], e["native3d_mode"]["n_candidates"])
