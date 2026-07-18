"""Webots MCP Server — full-access MCP for the Webots robot simulator.

Architecture:
  Claude <-stdio-> this FastMCP server <-TCP-> mcp_bridge supervisor controller
  (inside Webots) <-> simulation, plus per-robot mcp_robot agents proxied
  through the bridge.

Run:  python I:\\webotMCP\\server\\main.py
"""

import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mcp.server.fastmcp import FastMCP  # noqa: E402

import resources  # noqa: E402
from connection import BridgeConnection  # noqa: E402
from tools import (analyze, app, assets, authoring_tools, batch, behaviors,  # noqa: E402
                   code_exec, diagnostics, dx, extern, experiments_tools, manage,
                   observe, robot, scene, scene_model, sensing, simulation,
                   world_build, world_files)

logging.basicConfig(level=logging.INFO, stream=sys.stderr,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")

mcp = FastMCP(
    "webots",
    instructions=(
        "Full-access MCP for the Webots robot simulator. Typical workflow: "
        "launch_webots (or install_bridge_into_world for an existing project world, "
        "then open it in Webots) -> get_simulation_state -> get_scene_tree + "
        "get_viewport_screenshot / screenshot_multiview to understand the scene "
        "(find_nodes to search it) -> edit the scene "
        "(spawn/move/delete nodes) or control robots (attach_mcp_controller, then "
        "set_motor / get_camera_image / get_sensor_values). "
        "execute_supervisor_code is the escape hatch for anything else."
    ),
)

bridge = BridgeConnection(
    host=os.environ.get("WEBOTS_MCP_HOST", "127.0.0.1"),
    port=int(os.environ.get("WEBOTS_MCP_PORT", "10022")),
)

# register per group so tool groups can be toggled via manage_tool_groups
_GROUPED_MODULES = (
    ("core", (simulation, scene, sensing, robot, code_exec, batch, diagnostics)),
    ("scene_model", (scene_model,)),
    ("world_build", (world_build,)),
    ("analyze", (analyze,)),
    ("authoring", (authoring_tools,)),
    ("experiments", (experiments_tools,)),
    ("dx", (dx,)),
    ("extern", (extern,)),
    ("behavior", (behaviors,)),
    ("observe", (observe,)),
    ("app", (app, world_files)),
    ("assets", (assets,)),
)
for _group, _modules in _GROUPED_MODULES:
    for _module in _modules:
        _before = set(mcp._tool_manager._tools)
        _module.register(mcp, bridge)
        manage.GROUPS.setdefault(_group, []).extend(
            sorted(set(mcp._tool_manager._tools) - _before))
manage.register(mcp, bridge)
resources.register(mcp, bridge)

if __name__ == "__main__":
    mcp.run()
