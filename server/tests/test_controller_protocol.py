"""Exercise real controller framing and stepping without a Webots installation."""
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
import types

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def controllers(monkeypatch):
    field = types.SimpleNamespace(**{name: i for i, name in enumerate(
        ['SF_BOOL', 'SF_INT32', 'SF_FLOAT', 'SF_VEC2F', 'SF_VEC3F',
         'SF_ROTATION', 'SF_COLOR', 'SF_STRING', 'SF_NODE', 'MF_BOOL',
         'MF_INT32', 'MF_FLOAT', 'MF_VEC2F', 'MF_VEC3F', 'MF_ROTATION',
         'MF_COLOR', 'MF_STRING', 'MF_NODE'])})
    monkeypatch.setitem(sys.modules, 'controller', types.SimpleNamespace(
        Field=field, Supervisor=object, Node=object, Robot=object, Motion=object))
    modules = []
    for name in ('mcp_bridge', 'mcp_robot'):
        spec = importlib.util.spec_from_file_location(
            'test_' + name, ROOT / 'controllers' / name / (name + '.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        modules.append(module)
    return modules


def frame(obj):
    data = json.dumps(obj).encode()
    return struct.pack('>I', len(data)) + data


def test_agent_partial_body_does_not_block_simulation(controllers):
    _, agent = controllers
    left, right = socket.socketpair()
    data = frame({'id': 1})
    right.sendall(data[:6])
    timer = threading.Timer(.2, lambda: right.sendall(data[6:]))
    timer.start()
    try:
        start = time.monotonic()
        result = agent.recv_frame_nonblocking(left)
        elapsed = time.monotonic() - start
        timer.join()
        assert result is None
        assert elapsed < .15
        assert agent.recv_frame_nonblocking(left) == {'id': 1}
    finally:
        timer.join()
        left.close()
        right.close()


def test_agent_fragmented_header_and_consecutive_frames(controllers):
    _, agent = controllers
    left, right = socket.socketpair()
    try:
        data = frame({'id': 1})
        right.sendall(data[:2])
        assert agent.recv_frame_nonblocking(left) is None
        right.sendall(data[2:] + frame({'id': 2}))
        assert agent.recv_frame_nonblocking(left) == {'id': 1}
        assert agent.recv_frame_nonblocking(left) == {'id': 2}
        assert agent.recv_frame_nonblocking(left) is None
    finally:
        left.close()
        right.close()


def test_registration_eof_fails_and_closes_socket(controllers, monkeypatch):
    _, agent = controllers
    class ClosedPeer:
        calls = 0
        closed = False
        def setsockopt(self, *a): pass
        def settimeout(self, *a): pass
        def connect(self, *a): pass
        def sendall(self, *a): pass
        def recv(self, *a):
            self.calls += 1
            assert self.calls < 4, 'registration spins forever on EOF'
            return b''
        def close(self): self.closed = True
    peer = ClosedPeer()
    monkeypatch.setattr(agent.socket, 'socket', lambda *a: peer)
    instance = agent.Agent.__new__(agent.Agent)
    instance.name = 'test'
    with pytest.raises(ConnectionError):
        instance._connect()
    assert peer.closed


class Supervisor:
    def __init__(self, terminate_after=None):
        self.count = 0
        self.terminate_after = terminate_after
    def step(self, timestep):
        if self.count == self.terminate_after:
            return -1
        self.count += 1
        return 0
    def getTime(self): return self.count * .016


def bridge_double(module, terminate_after=None):
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.sup = Supervisor(terminate_after)
    bridge.timestep = 16
    bridge._sample_tracking = lambda: None
    bridge.agents = {}
    bridge._agents_lock = threading.Lock()
    return bridge


@pytest.mark.parametrize('steps', [-1, 100001, 1.5, True])
def test_invalid_step_counts_do_not_advance(controllers, steps):
    bridge = bridge_double(controllers[0])
    with pytest.raises(ValueError):
        bridge.cmd_step_simulation({'steps': steps})
    assert bridge.sup.count == 0


def test_termination_reports_actual_completed_steps(controllers):
    bridge = bridge_double(controllers[0], 2)
    result = bridge.cmd_step_simulation({'steps': 5})
    assert result['stepped'] == 2
    assert result['terminated'] is True


def test_exact_step_count(controllers):
    bridge = bridge_double(controllers[0])
    assert bridge.cmd_step_simulation({'steps': 0}) == {'stepped': 0, 'time': 0}
    assert bridge.cmd_step_simulation({'steps': 7}) == {'stepped': 7, 'time': .112}


def test_agent_wrong_reply_invalidates_registration(controllers):
    module, _ = controllers
    bridge = bridge_double(module)
    left, right = socket.socketpair()
    bridge.agents['test'] = {'sock': left, 'lock': threading.Lock()}
    def respond():
        req = module.recv_frame(right)
        module.send_frame(right, {'id': req['id'] + 1, 'status': 'ok', 'result': 0})
    thread = threading.Thread(target=respond)
    thread.start()
    try:
        with pytest.raises(ConnectionError, match='outcome.*unknown'):
            bridge.call_agent('test', 'ping', {}, timeout=.1)
        assert 'test' not in bridge.agents
        assert left.fileno() == -1
    finally:
        left.close()
        right.close()
        thread.join()


def test_idle_disconnected_agent_is_unregistered(controllers):
    module, _ = controllers
    bridge = bridge_double(module)
    left, right = socket.socketpair()
    thread = threading.Thread(target=bridge._agent_thread, args=(left,), daemon=True)
    thread.start()
    try:
        module.send_frame(right, {'register': 'test'})
        assert module.recv_frame(right)['status'] == 'ok'
        right.close()
        thread.join(1.5)
        assert not thread.is_alive(), 'disconnected agent stays registered forever'
        assert 'test' not in bridge.agents
        assert left.fileno() == -1
    finally:
        left.close()
        right.close()
        thread.join(1.5)


def test_agent_reply_send_failure_reconnects_on_next_step(controllers, monkeypatch):
    _, module = controllers
    instance = module.Agent.__new__(module.Agent)
    instance.robot = Supervisor(2)
    instance.timestep = 16
    instance.name = 'test'
    left, right = socket.socketpair()
    instance.sock = left
    monkeypatch.setattr(module, 'recv_frame_nonblocking', lambda s: {'id': 1, 'action': 'ping'})
    def lost(*args): raise ConnectionResetError('reply lost')
    monkeypatch.setattr(module, 'send_frame', lost)
    attempts = []
    def reconnect():
        attempts.append(1)
        raise ConnectionRefusedError('bridge unavailable')
    instance._connect = reconnect
    try:
        instance.run()
        assert left.fileno() == -1
        assert attempts
    finally:
        left.close()
        right.close()


def test_agent_partial_reply_obeys_total_deadline(controllers):
    module, _ = controllers
    bridge = bridge_double(module)
    left, right = socket.socketpair()
    bridge.agents['test'] = {'sock': left, 'lock': threading.Lock()}
    def respond():
        try:
            req = module.recv_frame(right)
            data = frame({'id':req['id'], 'status':'ok', 'result':True})
            right.sendall(data[:4])
            for chunk in (data[4:10], data[10:20], data[20:]):
                time.sleep(.04)
                right.sendall(chunk)
        except OSError:
            pass
    thread = threading.Thread(target=respond)
    thread.start()
    try:
        start = time.monotonic()
        with pytest.raises(ConnectionError, match='outcome unknown'):
            bridge.call_agent('test','ping',{},timeout=.07)
        assert time.monotonic()-start < .2
        assert 'test' not in bridge.agents
    finally:
        left.close()
        right.close()
        thread.join()


def test_scene_reference_resolution_refreshes_after_deletion_or_reset(controllers):
    module, _ = controllers
    bridge = bridge_double(module)
    node = object()
    current = {'node':node}
    bridge.sup.getFromId = lambda ref: current['node'] if ref == 42 else None
    bridge.sup.getFromDef = lambda ref: current['node'] if ref == 'BOX' else None
    bridge.sup.getRoot = lambda: None
    name_field = types.SimpleNamespace(getType=lambda:module.Field.SF_STRING,
                                       getSFString=lambda:'box name')
    named = types.SimpleNamespace(getField=lambda key:name_field if key == 'name' else None)
    depths = []
    def nodes(root, max_depth):
        depths.append(max_depth)
        return iter([named])
    bridge._iter_nodes = nodes
    assert bridge.find_node(42) is node
    assert bridge.find_node('42') is node
    assert bridge.find_node('BOX') is node
    assert bridge.find_node('box name') is named
    assert depths == [4]
    current['node'] = None
    for ref in (42, '42', 'BOX', 'missing'):
        with pytest.raises(ValueError, match='node not found'):
            bridge.find_node(ref)
    replacement = object()
    current['node'] = replacement
    assert bridge.find_node('BOX') is replacement


def test_scene_traversal_stops_at_depth_bound(controllers):
    bridge = bridge_double(controllers[0])
    visited = []
    def children(node):
        visited.append(node)
        yield 'children', node+1
    bridge._child_nodes = children
    assert list(bridge._iter_nodes(0,max_depth=4)) == [0,1,2,3,4]
    assert visited == [0,1,2,3,4]
