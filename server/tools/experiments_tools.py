"""Experiment automation (P5.2/8.8, 9.5, 9.6, 10.3): physics config, profiling,
run comparison, and declarative scenarios.

Scenario validation / build-planning / comparison / log-parsing math lives in the
bridge's Webots-free `experiments` module; scenario files are read/written here.
"""

import json
import os
import sys
from typing import Optional

from tools.app import MCP_ROOT

_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import experiments  # noqa: E402

_SCENARIOS = os.path.join(str(MCP_ROOT), "scenarios")


def _scenario_path(file):
    if os.path.isabs(file):
        return file
    if not file.endswith(".json"):
        file += ".json"
    return os.path.join(_SCENARIOS, file)


def register(mcp, bridge):
    @mcp.tool()
    def configure_physics(gravity: Optional[list] = None,
                          basic_time_step: Optional[float] = None,
                          fps: Optional[float] = None,
                          random_seed: Optional[int] = None,
                          optimal_thread_count: Optional[int] = None,
                          recipe: Optional[str] = None) -> dict:
        """Set WorldInfo physics fields without knowing field paths. recipe applies
        a preset ('earth','moon','mars','zero_g','slow_motion','high_fidelity');
        explicit args override the recipe. Set random_seed (then reset) for
        reproducible physics; coordinate_system is reported read-only."""
        return bridge.command("configure_physics",
                              {"gravity": gravity, "basic_time_step": basic_time_step,
                               "fps": fps, "random_seed": random_seed,
                               "optimal_thread_count": optimal_thread_count,
                               "recipe": recipe})

    @mcp.tool()
    def profile_simulation(duration_s: float = 3.0) -> dict:
        """Measure the achieved sim-time/wall-time ratio over duration_s (no
        restart). Tells you whether a world runs faster or slower than real time —
        a slow world usually means mesh bounding objects or a tiny timestep."""
        return bridge.command("profile_simulation", {"duration_s": duration_s},
                              timeout=max(120.0, duration_s * 30))

    @mcp.tool()
    def profile_from_log(log_path: str) -> dict:
        """Parse a Webots --log-performance file into real-time-factor stats
        (avg/min/max). Use when Webots was launched with that flag."""
        if not os.path.exists(log_path):
            return {"error": f"log not found: {log_path}"}
        with open(log_path, encoding="utf-8", errors="replace") as fh:
            return experiments.parse_performance_log(fh.read())

    @mcp.tool()
    def compare_runs(report_a: dict, report_b: dict) -> dict:
        """Compare two run_experiment reports: per-object end-position divergence
        and the maximum, plus objects present in only one. For debugging
        'it fails one time in five' — pass two reports from run_experiment."""
        return experiments.compare_reports(report_a, report_b)

    @mcp.tool()
    def save_scenario(name: str, conditions: Optional[list] = None,
                      duration_s: Optional[float] = None) -> dict:
        """Capture the current world as a reusable scenario.json (object list +
        physics + optional success/failure conditions and duration) under the
        project scenarios/ folder — versionable in git, replayable with
        run_scenario."""
        gen = bridge.command("generate_world_script", {"format": "json"})
        scenario = gen["scenario"]
        if conditions:
            scenario["conditions"] = conditions
        if duration_s is not None:
            scenario["duration_s"] = duration_s
        os.makedirs(_SCENARIOS, exist_ok=True)
        path = _scenario_path(name)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(scenario, fh, indent=1)
        return {"saved": path, "objects": len(scenario.get("objects", []))}

    @mcp.tool()
    def load_scenario(file: str, seed: Optional[int] = None) -> dict:
        """Build a scenario file into the running world: resolves poses and scatter
        rules (deterministic given a seed) and spawns everything. Applies the
        scenario's physics block if present."""
        path = _scenario_path(file)
        if not os.path.exists(path):
            return {"error": f"scenario not found: {path}"}
        with open(path, encoding="utf-8") as fh:
            scn = json.load(fh)
        errs = experiments.validate_scenario(scn)
        if errs:
            return {"error": "invalid scenario", "problems": errs}
        phys = scn.get("physics") or {}
        if phys.get("random_seed") is not None or seed is not None:
            bridge.command("configure_physics",
                           {"random_seed": seed if seed is not None
                            else phys.get("random_seed")})
        ops = experiments.scenario_build_plan(scn, seed_override=seed)
        built = bridge.command("build_scenario", {"ops": ops})
        return {"built": built["count"], "ops": len(ops)}

    @mcp.tool()
    def run_scenario(file: str, runs: int = 1, seeds: Optional[list] = None,
                     restore: str = "auto") -> dict:
        """Batch build→run→report a scenario across variations. Provide 'seeds'
        (one run per seed) or 'runs' (that many with the scenario's own seed).
        Returns each run's report plus, when there are 2+, a divergence comparison
        of the first two — the automated-environment workflow as data."""
        path = _scenario_path(file)
        if not os.path.exists(path):
            return {"error": f"scenario not found: {path}"}
        with open(path, encoding="utf-8") as fh:
            scn = json.load(fh)
        errs = experiments.validate_scenario(scn)
        if errs:
            return {"error": "invalid scenario", "problems": errs}
        seed_list = seeds if seeds is not None else [None] * max(1, runs)
        duration = float(scn.get("duration_s", 5.0))
        until = None
        conds = scn.get("conditions")
        if conds:
            until = conds[0] if len(conds) == 1 else {"type": "any", "conditions": conds}
        reports = []
        for s in seed_list:
            if s is not None:
                bridge.command("configure_physics", {"random_seed": s})
            ops = experiments.scenario_build_plan(scn, seed_override=s)
            bridge.command("build_scenario", {"ops": ops})
            rep = bridge.command("run_experiment",
                                 {"duration_s": duration, "until": until,
                                  "restore": restore},
                                 timeout=max(180.0, duration * 30))
            reports.append({"seed": s, "report": rep})
        out = {"runs": len(reports), "seeds": seed_list, "reports": reports}
        if len(reports) >= 2:
            out["comparison"] = experiments.compare_reports(
                reports[0]["report"], reports[1]["report"])
        return out
