"""Shared fixtures: a fake MCP registrar and a scriptable fake bridge."""

import os
import sys

import pytest

SERVER_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)


class FakeMCP:
    """Captures @mcp.tool() / @mcp.resource() registrations."""

    def __init__(self):
        self.tools = {}
        self.resources = {}

    def tool(self):
        def deco(fn):
            self.tools[fn.__name__] = fn
            return fn
        return deco

    def resource(self, uri):
        def deco(fn):
            self.resources[uri] = fn
            return fn
        return deco


class FakeBridge:
    """Scriptable bridge: queue responses per action, record all calls."""

    def __init__(self, responses=None):
        self.responses = responses or {}
        self.calls = []

    def _respond(self, action, params):
        r = self.responses.get(action, {"ok": True})
        if callable(r):
            return r(params)
        if isinstance(r, Exception):
            raise r
        return r

    def command(self, action, params=None, timeout=60.0):
        self.calls.append(("command", action, params))
        return self._respond(action, params or {})

    def robot_command(self, robot, action, params=None, timeout=60.0):
        self.calls.append(("robot", robot, action, params))
        return self._respond(action, params or {})


@pytest.fixture
def fake_mcp():
    return FakeMCP()


@pytest.fixture
def fake_bridge():
    return FakeBridge()
