"""batch_execute: run several bridge commands in one MCP call (Unity MCP style)."""

from typing import Optional

from connection import BridgeError

# Server-composed tools (screenshots, robot proxying, app control) are excluded:
# batch covers raw bridge actions, which is what sequences of edits need.
ALLOWED_ACTIONS = {
    "get_simulation_state", "set_simulation_mode", "step_simulation",
    "reset_simulation", "save_world",
    "get_scene_tree", "find_nodes", "get_scene_bounds", "get_node_details",
    "get_node_field", "set_node_field", "get_node_pose", "move_node",
    "spawn_node", "delete_node", "clone_node", "get_node_string",
    "get_selected_node", "set_velocity", "apply_force",
    "insert_field_item", "remove_field_item", "save_checkpoint",
    "restore_checkpoint", "set_joint_position", "frame_node",
    "set_node_visibility", "get_node_proto",
    "set_viewpoint", "set_label", "list_robots", "restart_controller",
    "execute_code",
}


def register(mcp, bridge):
    @mcp.tool()
    def batch_execute(commands: list, stop_on_error: bool = True,
                      timeout_per_command: Optional[float] = None) -> dict:
        """Run several scene/simulation commands in ONE call — much faster than one
        tool call each for bulk edits (e.g. spawning 10 boxes, setting many fields).
        commands = [{"action": "spawn_node", "params": {...}}, ...].
        Allowed actions: get_simulation_state, set_simulation_mode, step_simulation,
        reset_simulation, save_world, get_scene_tree, find_nodes, get_scene_bounds,
        get_node_details, get_node_field, set_node_field, get_node_pose, move_node,
        spawn_node, delete_node, clone_node, get_node_string, get_selected_node,
        set_velocity, apply_force, set_viewpoint, set_label,
        list_robots, restart_controller, execute_code.
        Results are returned per command; with stop_on_error=False failures are
        recorded and execution continues."""
        results = []
        for i, cmd in enumerate(commands):
            action = cmd.get("action")
            if action not in ALLOWED_ACTIONS:
                entry = {"index": i, "action": action, "success": False,
                         "error": f"action not allowed in batch: {action}"}
            else:
                try:
                    r = bridge.command(action, cmd.get("params") or {},
                                       timeout=timeout_per_command or 60.0)
                    entry = {"index": i, "action": action, "success": True, "result": r}
                except (BridgeError, ConnectionError) as exc:
                    entry = {"index": i, "action": action, "success": False,
                             "error": str(exc)}
            results.append(entry)
            if not entry["success"] and stop_on_error:
                break
        ok = sum(1 for r in results if r["success"])
        return {"executed": len(results), "succeeded": ok,
                "failed": len(results) - ok, "results": results}
