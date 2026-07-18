"""Authoring & automation tools (P8.4/8.5/8.6/8.7, P9.4, P10.1).

Script export and PROTO extraction turn an interactive MCP session into
reproducible code/assets; world templates + backups and appearance/lighting
tools round out world authoring. Text/data generation lives in the bridge's
Webots-free `authoring` module; file writes happen server-side.
"""

import datetime
import os
import shutil
import sys
from typing import Optional

from tools.app import MCP_ROOT
from webots_home import find_webots_home

_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import authoring  # noqa: E402

_WORLDS = os.path.join(str(MCP_ROOT), "worlds")
_PROTOS = os.path.join(str(MCP_ROOT), "protos")
_BACKUPS = os.path.join(_WORLDS, "backups")
_RECORDINGS = os.path.join(str(MCP_ROOT), "recordings")


def _ts():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def register(mcp, bridge):
    @mcp.tool()
    def generate_world_script(format: str = "python") -> dict:
        """Export the CURRENT scene as regenerable code so the user can reproduce
        it without MCP. format='python' → a standalone Supervisor script of
        importMFNodeFromString calls; 'json' → a declarative scenario;
        'wbt' → save a clean world file (via save_world)."""
        if format == "wbt":
            path = os.path.join(_WORLDS, f"generated_{_ts()}.wbt")
            bridge.command("save_world", {"path": path})
            return {"format": "wbt", "path": path}
        return bridge.command("generate_world_script", {"format": format})

    @mcp.tool()
    def extract_proto_from_node(node: str, proto_name: str,
                                expose: Optional[list] = None,
                                save: bool = True) -> dict:
        """Turn a tuned scene node into a reusable PROTO exposing chosen fields
        (default translation/rotation/name) and, with save=True, write it to the
        project protos/ folder — ready for spawn_node('MyProto { ... }'). Turns
        one-off builds into a parts library."""
        result = bridge.command("extract_proto_from_node",
                                {"node": node, "proto_name": proto_name,
                                 "expose": expose})
        if save:
            os.makedirs(_PROTOS, exist_ok=True)
            path = os.path.join(_PROTOS, f"{proto_name}.proto")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(result["proto_text"])
            result["saved_to"] = path
        return result

    @mcp.tool()
    def create_world(name: str, template: str = "empty", size: float = 10.0,
                     coordinate_system: str = "ENU", lighting: str = "indoor") -> dict:
        """Scaffold a ready-to-open .wbt in the project worlds/ folder. templates:
        'empty' (floor only), 'indoor_room' (floor + 4 walls), 'outdoor_flat'.
        Uses base nodes only (no EXTERNPROTO). Run install_bridge_into_world on it
        to enable MCP control."""
        os.makedirs(_WORLDS, exist_ok=True)
        text = authoring.world_template_wbt(name=name, template=template, size=size,
                                            coordinate_system=coordinate_system,
                                            lighting=lighting)
        path = os.path.join(_WORLDS, f"{name}.wbt")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return {"created": path, "template": template}

    @mcp.tool()
    def backup_world(world_path: Optional[str] = None) -> dict:
        """Save a timestamped copy of a world file (defaults to the demo world)
        into worlds/backups/ — a managed edit history."""
        src = world_path or os.path.join(_WORLDS, "demo.wbt")
        if not os.path.exists(src):
            return {"error": f"world not found: {src}"}
        os.makedirs(_BACKUPS, exist_ok=True)
        base = os.path.basename(src)
        dst = os.path.join(_BACKUPS, f"{base}.{_ts()}.bak")
        shutil.copy2(src, dst)
        return {"backed_up": dst, "source": src}

    @mcp.tool()
    def list_world_backups() -> dict:
        """List timestamped world backups made with backup_world (newest first)."""
        if not os.path.isdir(_BACKUPS):
            return {"backups": []}
        files = sorted(os.listdir(_BACKUPS), reverse=True)
        return {"backups": files, "dir": _BACKUPS}

    @mcp.tool()
    def restore_world_backup(backup_name: str, target_path: Optional[str] = None) -> dict:
        """Restore a backup (by file name from list_world_backups) over its
        original world file, or to target_path. Reload the world in Webots after."""
        src = os.path.join(_BACKUPS, backup_name)
        if not os.path.exists(src):
            return {"error": f"backup not found: {backup_name}"}
        dst = target_path or os.path.join(_WORLDS, backup_name.split(".", 1)[0])
        shutil.copy2(src, dst)
        return {"restored": dst, "from": src}

    @mcp.tool()
    def list_appearances(query: str = "", max_results: int = 40) -> dict:
        """Index the built-in Webots appearance PROTOs (Asphalt, BrushedSteel,
        Cardboard, ...) — filter by substring. Use a name with set_appearance."""
        from tools.assets import _build_proto_index
        index = _build_proto_index()
        names = sorted(
            name for name, info in index.items()
            if "/appearances/protos/" in (info.get("url") or ""))
        if query:
            names = [n for n in names if query.lower() in n.lower()]
        return {"count": len(names), "appearances": names[:max_results]}

    @mcp.tool()
    def set_appearance(node: str, base_color: Optional[list] = None,
                       roughness: Optional[float] = None,
                       metalness: Optional[float] = None) -> dict:
        """Recolor/re-material a node's first Shape in one call: base_color=[r,g,b]
        (0-1), plus optional roughness/metalness — no need to know nested field
        paths. The shape must already have a PBRAppearance/Appearance."""
        return bridge.command("set_appearance",
                              {"node": node, "base_color": base_color,
                               "roughness": roughness, "metalness": metalness})

    @mcp.tool()
    def set_recognition_colors(node: str, colors: list) -> dict:
        """Set a Solid's recognitionColors ([[r,g,b], ...]) so camera recognition
        can see it — the field users routinely forget to set."""
        return bridge.command("set_recognition_colors",
                              {"node": node, "colors": colors})

    @mcp.tool()
    def configure_lighting(preset: str = "indoor",
                           intensity: Optional[float] = None) -> dict:
        """Apply a lighting/background preset — 'indoor' | 'outdoor' | 'studio' |
        'night'. Sets the Background sky color and adds matching DirectionalLights.
        Bad lighting is the top cause of useless screenshots and failed camera
        recognition."""
        return bridge.command("configure_lighting",
                              {"preset": preset, "intensity": intensity})

    @mcp.tool()
    def record_states(duration_s: float = 3.0, nodes: Optional[list] = None,
                      fields: Optional[list] = None, interval_ms: int = 0) -> dict:
        """Record object state over a run to a CSV file for offline analysis /
        regression testing. fields ⊆ ['position','velocity','speed'] (default
        position). interval_ms=0 samples every step. Returns the CSV path plus
        summary stats (e.g. max speed) — the raw material for 'assert the box never
        exceeds 0.5 m/s'."""
        result = bridge.command("record_states",
                                {"duration_s": duration_s, "nodes": nodes,
                                 "fields": fields, "interval_ms": interval_ms},
                                timeout=max(120.0, duration_s * 30))
        rows = result["rows"]
        cols = ["sim_time", "node"] + list(result["fields"])
        os.makedirs(_RECORDINGS, exist_ok=True)
        path = os.path.join(_RECORDINGS, f"states_{_ts()}.csv")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(authoring.records_to_csv(rows, cols))
        out = {"csv_path": path, "rows": result["count"], "nodes": result["nodes"]}
        if "speed" in result["fields"]:
            out["speed_summary"] = authoring.summarize_series(rows, "speed")
        return out
