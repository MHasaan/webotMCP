"""Tool-group management: keep the default tool surface lean (Unity MCP style).

main.py records which tool names each module registered; this module exposes
list/enable/disable over those groups. Disabled tools are stashed and can be
re-enabled at any time in the same session.
"""

# populated by main.py: {"group": ["tool_name", ...]}
GROUPS = {}
# names that must never be disabled
PROTECTED = {"manage_tool_groups", "list_tool_groups", "preflight"}

_stash = {}  # tool_name -> Tool object


def register(mcp, bridge):
    def _tools():
        return mcp._tool_manager._tools  # dict name -> Tool

    @mcp.tool()
    def list_tool_groups() -> dict:
        """List tool groups, whether each is enabled, and the tools inside.
        Disable groups you don't need (recording, assets...) to keep the tool
        list lean; re-enable them any time."""
        out = {}
        for group, names in GROUPS.items():
            active = [n for n in names if n in _tools()]
            out[group] = {"enabled": bool(active),
                          "tools": names,
                          "disabled_tools": [n for n in names if n not in _tools()]}
        return out

    @mcp.tool()
    def manage_tool_groups(group: str, enabled: bool) -> dict:
        """Enable or disable a whole tool group (see list_tool_groups). Disabling
        removes its tools from the MCP tool list until re-enabled."""
        names = GROUPS.get(group)
        if names is None:
            raise ValueError(f"unknown group '{group}'. Groups: {list(GROUPS)}")
        changed = []
        if enabled:
            for n in names:
                if n in _stash:
                    mcp._tool_manager._tools[n] = _stash.pop(n)
                    changed.append(n)
        else:
            for n in names:
                if n in PROTECTED:
                    continue
                tool = _tools().pop(n, None)
                if tool is not None:
                    _stash[n] = tool
                    changed.append(n)
        return {"group": group, "enabled": enabled, "changed": changed,
                "note": "clients may need a tools/list refresh to see the change"}
