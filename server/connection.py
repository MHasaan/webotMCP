"""TCP client to the mcp_bridge supervisor controller running inside Webots."""

import json
import logging
import math
import socket
import struct
import threading
import time

logger = logging.getLogger("webots-mcp")

RETRY_DELAYS = [0.0, 1.0, 3.0, 5.0]
MAX_FRAME = 64 * 1024 * 1024


class BridgeError(Exception):
    """Raised when the bridge reports an error executing a command."""


class CommandOutcomeUnknown(BridgeError):
    """Transmission started, but no trustworthy execution result was received."""


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("command deadline expired")
    return remaining


class BridgeConnection:
    def __init__(self, host="127.0.0.1", port=10022):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self._next_id = 0

    # -- low level ----------------------------------------------------------

    def _connect(self, deadline):
        last = "deadline expired"
        for delay in RETRY_DELAYS:
            if delay:
                time.sleep(min(delay, _remaining(deadline)))
            sock = None
            try:
                sock = socket.create_connection((self.host, self.port),
                                                timeout=min(5.0, _remaining(deadline)))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = sock
                logger.info("connected to mcp_bridge at %s:%s", self.host, self.port)
                return
            except OSError as exc:
                last = exc
                if sock is not None:
                    sock.close()
        raise ConnectionError(
            f"Cannot reach the Webots MCP bridge at {self.host}:{self.port} ({last}). "
            "Is Webots running with a world that contains the mcp_bridge supervisor robot? "
            "Use launch_webots / install_bridge_into_world if not."
        )

    def _ensure_connected(self, deadline):
        if self.sock is None:
            self._connect(deadline)
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
            self._connect(deadline)
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

    def _recv_exact(self, n, deadline):
        buf = bytearray()
        while len(buf) < n:
            self.sock.settimeout(_remaining(deadline))
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("bridge closed the connection")
            buf += chunk
        return buf

    def _recv_frame(self, deadline):
        """Receive one frame; zero-length frames are heartbeats and are skipped."""
        while True:
            (length,) = struct.unpack(">I", self._recv_exact(4, deadline))
            if length == 0:
                # A heartbeat proves liveness, not completion or cancellation.
                continue
            if length > MAX_FRAME:
                raise ValueError(f"frame too large: {length}")
            data = self._recv_exact(length, deadline)
            return json.loads(data.decode("utf-8"))

    # -- public API -----------------------------------------------------------

    def command(self, action, params=None, timeout=60.0):
        """Send once; timeout does not cancel an operation already queued in Webots.

        One monotonic deadline covers lock, connection, send and receive.
        An invalid or lost reply is never automatically retried.
        """
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        if not self.lock.acquire(timeout=timeout):
            raise BridgeError("command deadline expired waiting for connection lock; not sent")
        try:
            try:
                self._ensure_connected(deadline)
            except OSError as exc:
                self.close()
                raise ConnectionError(f"command not sent: {exc}") from exc
            self._next_id += 1
            req = {"id": self._next_id, "action": action, "params": params or {}}
            # Serialization errors precede transmission and have known outcomes.
            data = json.dumps(req).encode("utf-8")
            if len(data) > MAX_FRAME:
                raise ValueError("command frame too large")
            try:
                self.sock.settimeout(_remaining(deadline))
                self.sock.sendall(struct.pack(">I", len(data)) + data)
                resp = self._recv_frame(deadline)
                if (not isinstance(resp, dict) or type(resp.get("id")) is not int
                        or resp["id"] != req["id"]
                        or resp.get("status") not in ("ok", "error")):
                    raise ValueError("invalid response envelope or mismatched response ID")
            except (OSError, ValueError) as exc:
                self.close()
                raise CommandOutcomeUnknown(
                    f"Command '{action}' (id {req['id']}) outcome unknown: {exc}. "
                    "It may already have executed; a timeout does not cancel it. "
                    "The command was not retried. Inspect state before retrying; "
                    "the next command will reconnect."
                ) from exc
        finally:
            self.lock.release()
        if resp.get("status") != "ok":
            msg = resp.get("error", "unknown bridge error")
            tb = resp.get("traceback")
            raise BridgeError(f"{msg}\n{tb}" if tb else msg)
        return resp.get("result")

    def robot_command(self, robot, action, params=None, timeout=60.0):
        """Proxy a command to a per-robot mcp_robot agent via the bridge."""
        resp = self.command(
            "robot_command",
            {"robot": robot, "action": action, "params": params or {},
             "timeout": timeout - min(5.0, timeout * 0.1)},
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
