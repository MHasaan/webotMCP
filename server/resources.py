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

    @mcp.resource("webots://console")
    def console() -> str:
        """Last 50 lines of the captured Webots console (launch_webots only)."""
        from tools.app import _state
        return json.dumps({"lines": list(_state["log"])[-50:]}, indent=1)

    @mcp.resource("webots://conventions")
    def conventions() -> str:
        """Units, axis/coordinate convention and key WorldInfo settings for this
        world — so spatial reasoning never silently assumes the wrong axes."""
        state = bridge.command("get_simulation_state")
        cs = state.get("coordinate_system", "ENU")
        up_axis = {"ENU": "z", "NUE": "y", "EUN": "y"}.get(cs, "z")
        return json.dumps({
            "units": {"length": "meters", "mass": "kilograms", "time": "seconds",
                      "angle": "radians"},
            "coordinate_system": cs,
            "up_axis": up_axis,
            "note": ("Default Webots world is ENU (z up); some sample worlds are "
                     "NUE (y up). Check up_axis before reasoning about height."),
            "basic_time_step_ms": state.get("basic_time_step"),
            "gravity": state.get("gravity"),
            "random_seed": state.get("random_seed"),
        }, indent=1)
