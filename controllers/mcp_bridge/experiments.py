"""Pure experiment helpers: physics recipes, declarative-scenario validation /
build planning, run-report comparison, and performance-log parsing.

No Webots (`controller`) imports — unit-testable standalone. Scenario build
planning reuses scene_math for placement/scatter rules.
"""

import scene_math

# ---------------------------------------------------------------------------
# Physics recipes (5.2 / 8.8)
# ---------------------------------------------------------------------------

PHYSICS_RECIPES = {
    "earth": {"gravity": [0, 0, -9.81], "basic_time_step": 32},
    "moon": {"gravity": [0, 0, -1.62], "basic_time_step": 32},
    "mars": {"gravity": [0, 0, -3.72], "basic_time_step": 32},
    "zero_g": {"gravity": [0, 0, 0], "basic_time_step": 32},
    "slow_motion": {"basic_time_step": 8, "fps": 60},
    "high_fidelity": {"basic_time_step": 8},
}


def physics_recipe(name):
    if name not in PHYSICS_RECIPES:
        raise ValueError(f"unknown physics recipe {name!r}; "
                         f"choose from {sorted(PHYSICS_RECIPES)}")
    return dict(PHYSICS_RECIPES[name])


# ---------------------------------------------------------------------------
# Declarative scenarios (10.3)
# ---------------------------------------------------------------------------

def validate_scenario(scn):
    """Return a list of human-readable problems (empty = valid)."""
    errors = []
    if not isinstance(scn, dict):
        return ["scenario must be a JSON object"]
    objs = scn.get("objects")
    if not isinstance(objs, list) or not objs:
        errors.append("scenario needs a non-empty 'objects' list")
        objs = []
    for i, o in enumerate(objs):
        if "node_string" not in o and "proto" not in o:
            errors.append(f"object[{i}] needs 'node_string' or 'proto'")
        has_pose = "position" in o
        has_scatter = "scatter" in o
        if has_scatter:
            sc = o["scatter"]
            for k in ("count", "size", "region"):
                if k not in sc:
                    errors.append(f"object[{i}].scatter needs '{k}'")
        if not has_pose and not has_scatter:
            errors.append(f"object[{i}] needs 'position' or 'scatter'")
    dur = scn.get("duration_s")
    if dur is not None and (not isinstance(dur, (int, float)) or dur <= 0):
        errors.append("'duration_s' must be a positive number")
    return errors


def scenario_build_plan(scn, seed_override=None):
    """Expand a scenario's object list into concrete spawn ops:
    ``[{"node_string", "position", "yaw"}]``. Scatter rules are resolved
    deterministically via scene_math (seed from the rule, or seed_override)."""
    errs = validate_scenario(scn)
    if errs:
        raise ValueError("invalid scenario: " + "; ".join(errs))
    ops = []
    for o in scn["objects"]:
        node_string = o.get("node_string") or o.get("proto")
        if "scatter" in o:
            sc = o["scatter"]
            region = sc["region"]
            region = ((region["min"], region["max"]) if isinstance(region, dict)
                      else (region[0], region[1]))
            seed = seed_override if seed_override is not None else sc.get("seed")
            poses = scene_math.scatter_poses(
                int(sc["count"]), sc["size"], region,
                min_spacing=float(sc.get("min_spacing", 0.0)), seed=seed,
                random_yaw=bool(sc.get("random_yaw", True)))
            for pose in poses:
                if pose is None:
                    continue
                ops.append({"node_string": node_string,
                            "position": pose["position"], "yaw": pose["yaw"]})
        else:
            ops.append({"node_string": node_string,
                        "position": list(o["position"]),
                        "yaw": float(o.get("yaw", 0.0))})
    return ops


# ---------------------------------------------------------------------------
# Run comparison (9.6)
# ---------------------------------------------------------------------------

def compare_reports(a, b, tol=1e-3):
    """Compare two run reports (each with an ``objects`` map of name ->
    {end, displacement_m, ...}). Reports the per-object end-position divergence
    and the maximum. Useful for 'it fails one time in five' debugging."""
    oa = a.get("objects", {})
    ob = b.get("objects", {})
    common = sorted(set(oa) & set(ob))
    per_object = []
    max_div = 0.0
    diverge_at = None
    for name in common:
        ea, eb = oa[name].get("end"), ob[name].get("end")
        if not ea or not eb:
            continue
        d = sum((x - y) ** 2 for x, y in zip(ea, eb)) ** 0.5
        per_object.append({"node": name, "end_divergence": round(d, 5)})
        if d > max_div:
            max_div = d
            diverge_at = name
    per_object.sort(key=lambda r: -r["end_divergence"])
    return {"identical": max_div <= tol,
            "max_divergence": round(max_div, 5),
            "most_divergent": diverge_at,
            "per_object": per_object,
            "only_in_a": sorted(set(oa) - set(ob)),
            "only_in_b": sorted(set(ob) - set(oa))}


# ---------------------------------------------------------------------------
# Performance-log parsing (9.5)
# ---------------------------------------------------------------------------

def parse_performance_log(text):
    """Tolerant parser for Webots ``--log-performance`` output (CSV with a header
    row). Returns average/min/max of the 'speed' column (real-time factor) plus
    the raw column names. Best-effort across Webots versions."""
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()
             and not ln.startswith("#")]
    if not lines:
        return {"samples": 0, "error": "empty log"}
    header = [h.strip().lower() for h in lines[0].split(",")]
    speed_idx = next((i for i, h in enumerate(header) if "speed" in h), None)
    speeds = []
    rows = 0
    for ln in lines[1:]:
        cells = ln.split(",")
        rows += 1
        if speed_idx is not None and speed_idx < len(cells):
            try:
                speeds.append(float(cells[speed_idx]))
            except ValueError:
                pass
    out = {"samples": rows, "columns": header}
    if speeds:
        out["real_time_factor"] = {"avg": round(sum(speeds) / len(speeds), 4),
                                   "min": round(min(speeds), 4),
                                   "max": round(max(speeds), 4)}
    return out
