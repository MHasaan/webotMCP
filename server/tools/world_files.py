"""World-file (.wbt) tools: listing, reading, and installing the MCP bridge."""

import os
import re
import shutil
from pathlib import Path
from typing import Optional

MCP_ROOT = Path(__file__).resolve().parents[2]

BRIDGE_SNIPPET = """
Robot {
  translation 0 0 -10
  name "mcp_bridge"
  controller "mcp_bridge"
  supervisor TRUE
}
"""


def install_bridge(wp: Path, copy_controllers: bool = True) -> list:
    """Install the bridge robot + controllers into a world. Returns action log."""
    text = wp.read_text(encoding="utf-8")
    actions = []
    if '"mcp_bridge"' in text:
        actions.append("bridge robot already present in world")
    else:
        shutil.copy2(wp, str(wp) + ".bak")
        wp.write_text(text.rstrip() + "\n" + BRIDGE_SNIPPET, encoding="utf-8")
        actions.append(f"appended mcp_bridge robot (backup: {wp}.bak)")
    if copy_controllers:
        # Webots project layout: <project>/worlds/x.wbt and <project>/controllers/
        project_dir = wp.parent.parent if wp.parent.name == "worlds" else wp.parent
        ctrl_dst = project_dir / "controllers"
        for name in ("mcp_bridge", "mcp_robot"):
            src = MCP_ROOT / "controllers" / name
            dst = ctrl_dst / name
            dst.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src / f"{name}.py", dst / f"{name}.py")
            actions.append(f"copied controller {name} -> {dst}")
    return actions


def register(mcp, bridge):
    @mcp.tool()
    def list_worlds(directory: Optional[str] = None) -> dict:
        """List .wbt world files. Searches the given directory (recursively, 3 levels)
        or the webotMCP worlds folder by default."""
        base = Path(directory) if directory else MCP_ROOT / "worlds"
        if not base.exists():
            return {"error": f"directory not found: {base}"}
        worlds = [str(p) for p in base.glob("**/*.wbt") if len(p.relative_to(base).parts) <= 3]
        return {"directory": str(base), "worlds": worlds}

    @mcp.tool()
    def read_world_file(path: str) -> dict:
        """Read and summarize a .wbt world file: version, EXTERNPROTO declarations,
        and top-level nodes (with DEF names, robot names, controllers). Also says
        whether the mcp_bridge is installed."""
        text = Path(path).read_text(encoding="utf-8", errors="replace")
        version = re.search(r"#VRML_SIM (\S+)", text)
        protos = re.findall(r'EXTERNPROTO\s+"([^"]+)"', text)
        top_nodes = re.findall(r"^(?:DEF\s+(\w+)\s+)?([A-Z]\w*)\s*\{", text, re.MULTILINE)
        names = re.findall(r'name\s+"([^"]+)"', text)
        controllers = re.findall(r'controller\s+"([^"]+)"', text)
        return {
            "path": path,
            "version": version.group(1) if version else None,
            "externproto": protos,
            "top_level_nodes": [{"def": d or None, "type": t} for d, t in top_nodes],
            "robot_names": names,
            "controllers": controllers,
            "mcp_bridge_installed": '"mcp_bridge"' in text,
            "size_bytes": len(text),
        }

    @mcp.tool()
    def install_bridge_into_world(world_path: str, copy_controllers: bool = True) -> dict:
        """Install the MCP bridge into an existing .wbt world: appends the mcp_bridge
        supervisor robot to the world file (backup created as .wbt.bak) and copies the
        mcp_bridge + mcp_robot controllers into the world's project controllers folder
        so Webots can find them. Reload the world afterwards."""
        wp = Path(world_path)
        if not wp.exists():
            raise FileNotFoundError(f"world file not found: {world_path}")
        actions = install_bridge(wp, copy_controllers)
        return {"world": str(wp), "actions": actions,
                "next_step": "reload the world in Webots (or use launch_webots)"}
