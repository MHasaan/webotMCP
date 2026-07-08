"""Frame-protocol tests for BridgeConnection against a real TCP socket."""

import json
import socket
import struct
import threading

import pytest

from connection import BridgeConnection, BridgeError


def _frame(obj):
    data = json.dumps(obj).encode()
    return struct.pack(">I", len(data)) + data


class StubBridgeServer:
    """Accepts one connection and answers each request from a script list.
    Script entries: dict (response result), Exception (error response),
    'heartbeat' (zero-length frame then next entry)."""

    def __init__(self, script):
        self.script = list(script)
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        conn, _ = self.sock.accept()
        try:
            while self.script:
                header = conn.recv(4)
                if not header:
                    return
                (length,) = struct.unpack(">I", header)
                buf = b""
                while len(buf) < length:
                    buf += conn.recv(length - len(buf))
                req = json.loads(buf)
                entry = self.script.pop(0)
                while entry == "heartbeat":
                    conn.sendall(struct.pack(">I", 0))
                    entry = self.script.pop(0)
                if isinstance(entry, Exception):
                    resp = {"id": req["id"], "status": "error", "error": str(entry)}
                else:
                    resp = {"id": req["id"], "status": "ok", "result": entry}
                conn.sendall(_frame(resp))
        finally:
            conn.close()


def test_command_roundtrip():
    server = StubBridgeServer([{"pong": True}])
    c = BridgeConnection(port=server.port)
    assert c.command("ping") == {"pong": True}
    c.close()


def test_heartbeats_are_skipped():
    server = StubBridgeServer(["heartbeat", "heartbeat", {"done": 1}])
    c = BridgeConnection(port=server.port)
    assert c.command("slow_op", timeout=10.0) == {"done": 1}
    c.close()


def test_bridge_error_raises():
    server = StubBridgeServer([ValueError("node not found: X")])
    c = BridgeConnection(port=server.port)
    with pytest.raises(BridgeError, match="node not found"):
        c.command("get_node_details", {"node": "X"})
    c.close()


def test_unreachable_bridge_message():
    c = BridgeConnection(port=1)  # nothing listens there
    with pytest.raises(ConnectionError, match="mcp_bridge"):
        c.command("ping")


def test_sequential_commands_share_connection():
    server = StubBridgeServer([{"n": 1}, {"n": 2}])
    c = BridgeConnection(port=server.port)
    assert c.command("a")["n"] == 1
    assert c.command("b")["n"] == 2
    c.close()
