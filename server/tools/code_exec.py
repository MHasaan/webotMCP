"""Arbitrary code execution inside Webots — the escape hatch for anything
not covered by a dedicated tool (like Unity MCP's execute_code)."""


def register(mcp, bridge):
    @mcp.tool()
    def execute_supervisor_code(code: str) -> dict:
        """Run arbitrary Python inside the Webots supervisor bridge. Available names:
        'supervisor' (Supervisor instance — full scene/simulation API), 'Node',
        'Field'. Assign to 'result' to return a value; print() output is captured.
        Example: result = supervisor.getFromDef('MY_BOT').getPosition()
        Node.getFromProtoDef requires an actual DEF inside that PROTO, not a
        device's name field. Missing DEF searches can be expensive: do not
        repeat speculative lookups in observation loops. Keep unavailable
        optional data explicit rather than substituting unrelated nodes."""
        return bridge.command("execute_code", {"code": code}, timeout=120.0)

    @mcp.tool()
    def execute_robot_code(robot: str, code: str) -> dict:
        """Run arbitrary Python inside a robot's mcp_robot agent. Available names:
        'robot' (Robot instance), 'devices' (dict name->device), 'agent'.
        Assign to 'result' to return a value. Requires attach_mcp_controller first."""
        return bridge.robot_command(robot, "execute_code", {"code": code}, timeout=120.0)
