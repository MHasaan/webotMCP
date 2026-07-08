"""TCP client to the mcp_bridge supervisor controller running inside Webots."""

import json
import logging
import socket
import struct
import threading
import time

logger = logging.getLogger("webots-mcp")

RETRY_DELAYS = [0.0, 1.0, 3.0, 5.0]


class BridgeError(Exception):
    """Raised when the bridge reports an error executing a command."""


class BridgeConnection:
    def __init__(self, host="127.0.0.1", port=10022):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self._next_id = 0

    # -- low level ----------------------------------------------------------

    def _connect(self):
        for delay in RETRY_DELAYS:
            if delay:
                time.sleep(delay)
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = sock
                logger.info("connected to mcp_bridge at %s:%s", self.host, self.port)
                return
            except OSError as exc:
                last = exc
        raise ConnectionError(
            f"Cannot reach the Webots MCP bridge at {self.host}:{self.port} ({last}). "
            "Is Webots running with a world that contains the mcp_bridge supervisor robot? "
            "Use launch_webots / install_bridge_into_world if not."
        )

    def _ensure_connected(self):
        if self.sock is None:
            self._connect()
            return
        # stale-socket detection: peek without blocking
        try:
            self.sock.setblocking(False)
            data = self.sock.recv(1, socket.MSG_PEEK)
            if data == b"":
                raise OSError("peer closed")
        except BlockingIOError:
            pass  # healthy: nothing to read
        except OSError:
            self.close()
            self._connect()
        finally:
            if self.sock is not None:
                self.sock.setblocking(True)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _send_frame(self, obj):
        data = json.dumps(obj).encode("utf-8")
        self.sock.sendall(struct.pack(">I", len(data)) + data)

    def _recv_exact(self, n, timeout):
        self.sock.settimeout(timeout)
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("bridge closed the connection")
            buf += chunk
        return buf

    def _recv_frame(self, timeout):
        """Receive one frame; zero-length frames are heartbeats and are skipped."""
        deadline = time.time() + timeout
        while True:
            remaining = max(0.5, deadline - time.time())
            (length,) = struct.unpack(">I", self._recv_exact(4, remaining))
            if length == 0:
                # heartbeat during a long operation -> extend the deadline
                deadline = time.time() + timeout
                continue
            data = self._recv_exact(length, max(0.5, deadline - time.time()))
            return json.loads(data.decode("utf-8"))

    # -- public API -----------------------------------------------------------

    def command(self, action, params=None, timeout=60.0):
        """Send a command to the bridge and return its result (or raise BridgeError)."""
        with self.lock:
            self._ensure_connected()
            self._next_id += 1
            req = {"id": self._next_id, "action": action, "params": params or {}}
            try:
                self._send_frame(req)
                resp = self._recv_frame(timeout)
            except (OSError, ConnectionError):
                # one reconnect-and-retry for transient failures
                self.close()
                self._connect()
                self._send_frame(req)
                resp = self._recv_frame(timeout)
        if resp.get("status") != "ok":
            msg = resp.get("error", "unknown bridge error")
            tb = resp.get("traceback")
            raise BridgeError(f"{msg}\n{tb}" if tb else msg)
        return resp.get("result")

    def robot_command(self, robot, action, params=None, timeout=60.0):
        """Proxy a command to a per-robot mcp_robot agent via the bridge."""
        resp = self.command(
            "robot_command",
            {"robot": robot, "action": action, "params": params or {}, "timeout": timeout - 5},
            timeout=timeout,
        )
        # resp is the agent's own {status, result|error} envelope
        if isinstance(resp, dict) and resp.get("status") == "error":
            msg = resp.get("error", "unknown agent error")
            tb = resp.get("traceback")
            raise BridgeError(f"{msg}\n{tb}" if tb else msg)
        if isinstance(resp, dict) and "result" in resp:
            return resp["result"]
        return resp
