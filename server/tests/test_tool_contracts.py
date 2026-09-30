"""Audit tool dispatch destinations and the repaired public contracts."""
import ast
from pathlib import Path

import pytest

from conftest import FakeBridge
from connection import BridgeError
from tools import code_exec, simulation

ROOT = Path(__file__).resolve().parents[2]


def test_literal_tool_actions_have_controller_handlers():
    handlers = {}
    for kind, name in [('command', 'mcp_bridge'), ('robot_command', 'mcp_robot')]:
        tree = ast.parse((ROOT/'controllers'/name/(name+'.py')).read_text(encoding='utf-8'))
        handlers[kind] = {n.name[4:] for n in ast.walk(tree)
                          if isinstance(n, ast.FunctionDef) and n.name.startswith('cmd_')}
    checked = 0
    for source in (ROOT/'server/tools').glob('*.py'):
        for node in ast.walk(ast.parse(source.read_text(encoding='utf-8'))):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            kind = node.func.attr
            if kind not in handlers:
                continue
            index = 1 if kind == 'robot_command' else 0
            if len(node.args) <= index or not isinstance(node.args[index], ast.Constant):
                continue  # Dynamic batch actions are tested by the whitelist suite.
            action = node.args[index].value
            assert action in handlers[kind], (source.name, node.lineno, kind, action)
            checked += 1
    assert checked > 100


def test_simulation_and_code_tools_forward_results_and_errors(fake_mcp):
    marker = {'time': 1.25, 'stepped': 7}
    bridge = FakeBridge({'step_simulation': marker, 'world_reload': BridgeError('outcome unknown'),
                         'execute_code': {'result': {'answer': 42}, 'stdout': ''}})
    simulation.register(fake_mcp, bridge)
    code_exec.register(fake_mcp, bridge)
    assert fake_mcp.tools['step_simulation'](7) is marker
    assert bridge.calls[-1] == ('command', 'step_simulation', {'steps': 7})
    with pytest.raises(BridgeError, match='outcome unknown'):
        fake_mcp.tools['world_reload']()
    result = fake_mcp.tools['execute_robot_code']('demo', 'result = 42')
    assert result['result'] == {'answer':42}
    assert bridge.calls[-1] == ('robot', 'demo', 'execute_code', {'code':'result = 42'})
