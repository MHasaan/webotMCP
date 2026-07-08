"""Read-only MCP resources exposing live simulation state (Unity MCP style).

Resources are quick, LLM-friendly snapshots — reading them never modifies the
simulation. Tools remain the way to act on it.
"""

import json


def register(mcp, bridge):
    @mcp.resource("webots://simulation")
    def simulation_state() -> str:
        """Current simulation state: time, mode, time step, world file, agents."""
        return json.dumps(bridge.command("get_simulation_state"), indent=1)

    @mcp.resource("webots://scene")
    def scene_tree() -> str:
        """Scene tree summary (depth 3, no fields)."""
        return json.dumps(bridge.command("get_scene_tree", {"max_depth": 3}), indent=1)

    @mcp.resource("webots://robots")
    def robots() -> str:
        """All robots: name, model, controller, position, MCP-agent status."""
        return json.dumps(bridge.command("list_robots"), indent=1)

    @mcp.resource("webots://scene/{node}")
    def node_details(node: str) -> str:
        """Full details of one node (fields + world pose) by DEF name/id/name."""
        return json.dumps(bridge.command("get_node_details", {"node": node}), indent=1)
