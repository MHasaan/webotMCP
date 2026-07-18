"""Developer-experience & breadth tools (P5.4, P4.4, P7.5) plus workflow prompts
(P6.2). Sample discovery and world-header parsing reuse the bridge's Webots-free
`perception` module; world-file maintenance shells out to the Webots CLI.
"""

import os
import subprocess
import sys

from tools.app import MCP_ROOT
from webots_home import find_webots_executable, find_webots_home

_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import perception  # noqa: E402


def register(mcp, bridge):
    @mcp.tool()
    def get_viewport_labels(width: int = 640, height: int = 480) -> dict:
        """Project every object's position into the CURRENT 3D view and return
        pixel-space labels (near→far, in-frame only, occlusion-unaware). Turns a
        screenshot into a labeled diagram: pair with get_viewport_screenshot and
        overlay the labels at the returned px/py."""
        return bridge.command("get_viewport_labels",
                              {"width": width, "height": height})

    @mcp.tool()
    def describe_sample(path: str) -> dict:
        """Summarize a Webots world file (a sample or your own): Webots version,
        title/info, basicTimeStep, coordinate system, and the robots + their
        controllers — without opening it."""
        if not os.path.isabs(path):
            cand = find_webots_home() / path
            path = str(cand) if cand.exists() else path
        if not os.path.exists(path):
            return {"error": f"world file not found: {path}"}
        with open(path, encoding="utf-8", errors="replace") as fh:
            info = perception.parse_world_header(fh.read())
        info["path"] = path
        return info

    @mcp.tool()
    def list_sample_worlds(query: str = "", max_results: int = 40) -> dict:
        """Find Webots sample/benchmark worlds by name substring (from the Webots
        installation's projects/). Feed a path to describe_sample or launch_webots."""
        root = find_webots_home() / "projects"
        if not root.exists():
            return {"error": f"projects not found at {root}", "worlds": []}
        hits = []
        for p in root.glob("**/worlds/*.wbt"):
            rel = str(p.relative_to(find_webots_home()))
            if not query or query.lower() in rel.lower():
                hits.append(rel)
            if len(hits) >= max_results:
                break
        return {"count": len(hits), "worlds": sorted(hits)}

    @mcp.tool()
    def update_world_file(path: str) -> dict:
        """Batch-migrate an old world to the current Webots format
        (webots --update-world) — useful before installing the bridge into a
        downloaded world. Runs headless."""
        exe = find_webots_executable(console=False)
        try:
            r = subprocess.run([str(exe), "--update-world", "--batch", "--minimize",
                                path], capture_output=True, text=True, timeout=120)
            return {"updated": r.returncode == 0, "path": path,
                    "stdout": r.stdout[-2000:], "stderr": r.stderr[-2000:]}
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}

    @mcp.tool()
    def clear_webots_cache() -> dict:
        """Clear the Webots asset cache (webots --clear-cache) — fixes
        corrupted-asset / stale-PROTO weirdness."""
        exe = find_webots_executable(console=False)
        try:
            r = subprocess.run([str(exe), "--clear-cache", "--batch", "--minimize"],
                               capture_output=True, text=True, timeout=60)
            return {"cleared": r.returncode == 0, "stdout": r.stdout[-1000:]}
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}

    # -- workflow prompts (6.2) --------------------------------------------
    @mcp.prompt()
    def inspect_scene() -> str:
        """Proven workflow: understand an unfamiliar scene end to end."""
        return ("Inspect the current Webots scene: 1) get_simulation_state and read "
                "webots://conventions for the up-axis; 2) get_object_catalog for the "
                "inventory; 3) get_scene_map for the top-down layout; 4) "
                "screenshot_multiview for a visual; 5) validate_world to flag "
                "placement/physics problems. Summarize what's in the world.")

    @mcp.prompt()
    def robot_bringup() -> str:
        """Proven workflow: attach to and probe a robot."""
        return ("Bring up a robot: 1) list_robots; 2) attach_mcp_controller to the "
                "target; 3) get_robot_devices; 4) read sensors with "
                "get_sensor_values / get_camera_image. Report its capabilities.")

    @mcp.prompt()
    def record_demo() -> str:
        """Proven workflow: record a short demo movie of an action."""
        return ("Record a demo: 1) set_viewpoint / frame_node to frame the subject; "
                "2) start_movie_recording; 3) perform the actions (drive, forces, "
                "motions); 4) stop_movie_recording; 5) get_recording_status until "
                "encoding completes.")

    @mcp.prompt()
    def experiment_loop() -> str:
        """Proven workflow: the build→run→understand iteration."""
        return ("Run an experiment loop: 1) snapshot_scene('start'); 2) build/adjust "
                "the world (spawn/scatter/place, then validate_world); 3) "
                "run_experiment with a wait_until 'until' condition; 4) read the "
                "report's anomalies + diff_scene('start'); 5) adjust and repeat.")
