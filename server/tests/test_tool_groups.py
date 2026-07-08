"""Tool-group enable/disable against a real FastMCP instance."""

import pytest

pytest.importorskip("mcp.server.fastmcp")
from mcp.server.fastmcp import FastMCP  # noqa: E402

from tools import manage  # noqa: E402
from conftest import FakeBridge  # noqa: E402


@pytest.fixture
def grouped_mcp():
    mcp = FastMCP("test")
    manage.GROUPS.clear()
    manage._stash.clear()

    @mcp.tool()
    def alpha() -> dict:
        return {}

    @mcp.tool()
    def beta() -> dict:
        return {}

    manage.GROUPS["extras"] = ["alpha", "beta"]
    manage.register(mcp, FakeBridge())
    return mcp


def _names(mcp):
    return set(mcp._tool_manager._tools)


def test_disable_and_reenable_group(grouped_mcp):
    tools = grouped_mcp._tool_manager._tools
    toggle = None
    for name, tool in list(tools.items()):
        if name == "manage_tool_groups":
            toggle = tool.fn
    assert toggle is not None

    r = toggle("extras", False)
    assert set(r["changed"]) == {"alpha", "beta"}
    assert "alpha" not in _names(grouped_mcp)

    r = toggle("extras", True)
    assert set(r["changed"]) == {"alpha", "beta"}
    assert "alpha" in _names(grouped_mcp)


def test_unknown_group_raises(grouped_mcp):
    toggle = grouped_mcp._tool_manager._tools["manage_tool_groups"].fn
    with pytest.raises(ValueError, match="unknown group"):
        toggle("nope", False)


def test_list_tool_groups(grouped_mcp):
    lister = grouped_mcp._tool_manager._tools["list_tool_groups"].fn
    out = lister()
    assert out["extras"]["enabled"] is True
    assert out["extras"]["tools"] == ["alpha", "beta"]
