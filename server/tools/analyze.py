"""Run / understand loop (P9, P1.2): event-driven stepping, whole-iteration
experiment reports, anomaly detection, and console diagnostics.

The condition/anomaly/console math lives in the bridge's Webots-free
`run_analysis` module (also used inside Webots); the console tools read the
server-captured Webots log directly.
"""

import os
import sys
from typing import Optional

from tools.app import MCP_ROOT, _state
from tools.sensing import _to_image

# run_analysis is Webots-free; reuse it server-side for console classification.
_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import run_analysis  # noqa: E402


def register(mcp, bridge):
    @mcp.tool()
    def wait_until(condition: dict, timeout_s: float = 30.0) -> dict:
        """Step the simulation until a condition fires, then report what happened
        and when — the missing primitive for "push the box until it reaches the
        wall" so experiments are deterministic instead of guess-the-duration.

        Condition DSL (JSON): {"type":"distance","a":"BOX","b":"WALL","op":"<","value":0.3},
        {"type":"contact","a":"BALL","b":"FLOOR"}, {"type":"speed","node":"BALL","op":"<","value":0.01},
        {"type":"position","node":"BOX","axis":2,"op":"<","value":0.1},
        {"type":"sim_time","op":">","value":12.5}, and
        {"type":"any"|"all","conditions":[...]}. Returns fired, sim_time, steps and
        the poses of involved nodes."""
        return bridge.command("wait_until",
                              {"condition": condition, "timeout_s": timeout_s},
                              timeout=max(120.0, timeout_s * 20))

    @mcp.tool()
    def run_experiment(duration_s: float = 5.0, watch: Optional[list] = None,
                       until: Optional[dict] = None, restore: str = "auto",
                       sample_every: int = 2, floor: float = 0.0,
                       arena: Optional[dict] = None, screenshot: bool = False) -> list:
        """One iteration in one call: checkpoint the world, enable tracking, run for
        duration_s (or until an 'until' condition fires, see wait_until), then return
        a structured run report — per-object motion + displacement, interaction
        timeline, scene diff vs start (added/removed/moved/rotated), and anomalies
        (physics blow-ups, teleports, runaway speed, below-floor). restore:
        'auto' rewinds to the start (default), 'keep' leaves the result, 'on_anomaly'
        rewinds only if something went wrong. This is the build->run->read->adjust
        loop. Set screenshot=True to also get a final image."""
        report = bridge.command(
            "run_experiment",
            {"duration_s": duration_s, "watch": watch, "until": until,
             "restore": restore, "sample_every": sample_every, "floor": floor,
             "arena": arena, "screenshot": screenshot},
            timeout=max(180.0, duration_s * 30))
        shot = report.pop("screenshot_base64", None)
        out = [report]
        if shot:
            out.append(_to_image(shot, 768))
        return out

    @mcp.tool()
    def detect_anomalies(floor: float = 0.0, arena: Optional[dict] = None,
                         teleport_thresh: float = 1.0,
                         speed_thresh: float = 50.0, up: int = 2) -> dict:
        """Scan the currently-tracked motion (start_tracking must be active) for
        physics trouble: NaN/inf positions, teleports (> teleport_thresh in one
        sample), runaway velocity, objects below the floor or outside 'arena'.
        Each anomaly comes with a time, node and a fix hint."""
        return bridge.command("detect_anomalies",
                              {"floor": floor, "arena": arena,
                               "teleport_thresh": teleport_thresh,
                               "speed_thresh": speed_thresh, "up": up})

    @mcp.tool()
    def get_console_diagnostics() -> dict:
        """Classify the captured Webots console into actionable categories with
        fixes: ODE/physics warnings, controller tracebacks, missing assets, parse
        warnings. Answers 'why did my run misbehave' without eyeballing thousands
        of raw log lines. (Only for Webots launched via launch_webots.)"""
        lines = list(_state["log"])
        result = run_analysis.classify_console(lines)
        result["console_lines"] = len(lines)
        return result

    @mcp.tool()
    def get_controller_logs(robot: Optional[str] = None) -> dict:
        """Split the Webots console per controller (Webots prefixes lines with
        [controller_name]). Pass 'robot' for just that controller's lines; omit for
        the full grouping (unprefixed engine lines under '_webots')."""
        grouped = run_analysis.split_controller_logs(list(_state["log"]))
        if robot is not None:
            return {"controller": robot, "lines": grouped.get(robot, [])}
        return {"controllers": {k: len(v) for k, v in grouped.items()},
                "logs": grouped}
