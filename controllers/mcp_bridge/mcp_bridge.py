"""MCP Bridge - Supervisor controller that exposes the whole Webots simulation over TCP.

Runs inside Webots as the controller of a `Robot { supervisor TRUE }` node.
Listens on two TCP ports:
  - COMMAND_PORT (default 10022): the MCP server connects here and sends JSON commands.
  - AGENT_PORT   (default 10023): mcp_robot agents (generic per-robot controllers)
    register here; the bridge proxies robot-level commands to them.

Wire protocol (both ports): 4-byte big-endian length prefix + UTF-8 JSON payload.
Command frames:  {"id": int, "action": str, "params": {...}}
Response frames: {"id": int, "status": "ok"|"error", "result": ..., "error": str}

Commands are queued by socket threads and executed on the main thread between
supervisor.step() calls, so all Webots API access is single-threaded and the
simulation keeps running.
"""

import base64
import collections
import io
import json
import math
import os
import queue
import socket
import struct
import sys
import tempfile
import threading
import time
import traceback

# Make sure the Webots python API is importable even if PYTHONPATH is not set.
# (Webots sets WEBOTS_HOME for controller processes it spawns.)
for _home in filter(None, (os.environ.get("WEBOTS_HOME"),
                           r"C:\Program Files\Webots", "/usr/local/webots",
                           "/Applications/Webots.app")):
    _py_api = os.path.join(_home, "lib", "controller", "python")
    if os.path.isdir(_py_api):
        if _py_api not in sys.path:
            sys.path.insert(0, _py_api)
        break

from controller import Supervisor, Node, Field  # noqa: E402

COMMAND_PORT = int(os.environ.get("WEBOTS_MCP_PORT", "10022"))
AGENT_PORT = int(os.environ.get("WEBOTS_MCP_AGENT_PORT", "10023"))
MAX_FRAME = 64 * 1024 * 1024


# ---------------------------------------------------------------------------
# Framing helpers
# ---------------------------------------------------------------------------

def send_frame(sock, obj):
    data = json.dumps(obj).encode("utf-8")
    sock.sendall(struct.pack(">I", len(data)) + data)


def recv_frame(sock):
    header = _recv_exact(sock, 4)
    if header is None:
        return None
    (length,) = struct.unpack(">I", header)
    if length > MAX_FRAME:
        raise ValueError(f"frame too large: {length}")
    data = _recv_exact(sock, length)
    if data is None:
        return None
    return json.loads(data.decode("utf-8"))


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


# ---------------------------------------------------------------------------
# Field value serialization
# ---------------------------------------------------------------------------

_SF_GETTERS = {
    Field.SF_BOOL: "getSFBool",
    Field.SF_INT32: "getSFInt32",
    Field.SF_FLOAT: "getSFFloat",
    Field.SF_VEC2F: "getSFVec2f",
    Field.SF_VEC3F: "getSFVec3f",
    Field.SF_ROTATION: "getSFRotation",
    Field.SF_COLOR: "getSFColor",
    Field.SF_STRING: "getSFString",
}

_MF_GETTERS = {
    Field.MF_BOOL: "getMFBool",
    Field.MF_INT32: "getMFInt32",
    Field.MF_FLOAT: "getMFFloat",
    Field.MF_VEC2F: "getMFVec2f",
    Field.MF_VEC3F: "getMFVec3f",
    Field.MF_ROTATION: "getMFRotation",
    Field.MF_COLOR: "getMFColor",
    Field.MF_STRING: "getMFString",
}

_SF_SETTERS = {
    Field.SF_BOOL: "setSFBool",
    Field.SF_INT32: "setSFInt32",
    Field.SF_FLOAT: "setSFFloat",
    Field.SF_VEC2F: "setSFVec2f",
    Field.SF_VEC3F: "setSFVec3f",
    Field.SF_ROTATION: "setSFRotation",
    Field.SF_COLOR: "setSFColor",
    Field.SF_STRING: "setSFString",
}

_MF_SETTERS = {
    Field.MF_BOOL: "setMFBool",
    Field.MF_INT32: "setMFInt32",
    Field.MF_FLOAT: "setMFFloat",
    Field.MF_VEC2F: "setMFVec2f",
    Field.MF_VEC3F: "setMFVec3f",
    Field.MF_ROTATION: "setMFRotation",
    Field.MF_COLOR: "setMFColor",
    Field.MF_STRING: "setMFString",
}


def read_field_value(field, max_items=20):
    """Serialize a field value to a JSON-friendly structure."""
    ftype = field.getType()
    if ftype == Field.SF_NODE:
        node = field.getSFNode()
        return {"node": node.getTypeName()} if node else None
    if ftype == Field.MF_NODE:
        count = field.getCount()
        items = []
        for i in range(min(count, max_items)):
            n = field.getMFNode(i)
            items.append(n.getTypeName() if n else None)
        return {"nodes": items, "count": count}
    getter = _SF_GETTERS.get(ftype)
    if getter:
        return getattr(field, getter)()
    getter = _MF_GETTERS.get(ftype)
    if getter:
        count = field.getCount()
        return [getattr(field, getter)(i) for i in range(min(count, max_items))]
    return f"<unsupported type {field.getTypeName()}>"


def write_field_value(field, value, index=None):
    ftype = field.getType()
    setter = _SF_SETTERS.get(ftype)
    if setter:
        getattr(field, setter)(value)
        return
    setter = _MF_SETTERS.get(ftype)
    if setter:
        if index is None:
            raise ValueError(f"field is multi-valued ({field.getTypeName()}); provide 'index'")
        getattr(field, setter)(index, value)
        return
    raise ValueError(f"cannot write field of type {field.getTypeName()}")


# ---------------------------------------------------------------------------
# The bridge
# ---------------------------------------------------------------------------

class Bridge:
    def __init__(self):
        self.sup = Supervisor()
        self.timestep = int(self.sup.getBasicTimeStep())
        self.logical_mode = "realtime"
        # motion/interaction tracking state
        self.tracking = None  # {"nodes": {id: {"node", "name", "buf"}}, "sample_every", "events", "prev_contacts", "count"}
        self.commands = queue.Queue()  # (request dict, reply callable)
        self.agents = {}  # robot name -> {"sock": socket, "lock": Lock}
        self._start_server(COMMAND_PORT, self._client_thread)
        self._start_server(AGENT_PORT, self._agent_thread)
        print(f"[mcp_bridge] command port {COMMAND_PORT}, agent port {AGENT_PORT}", flush=True)

    # -- networking --------------------------------------------------------

    def _start_server(self, port, handler):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(4)

        def accept_loop():
            while True:
                try:
                    conn, _ = srv.accept()
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    threading.Thread(target=handler, args=(conn,), daemon=True).start()
                except OSError:
                    return

        threading.Thread(target=accept_loop, daemon=True).start()

    def _client_thread(self, conn):
        """One MCP-server connection: read commands, enqueue, send replies."""
        send_lock = threading.Lock()
        try:
            while True:
                req = recv_frame(conn)
                if req is None:
                    return
                # Some actions must NOT wait on the main thread:
                #  - robot_command only touches agent sockets (no Webots API), and
                #    blocking the main loop on it would freeze the simulation and
                #    deadlock the agent. Everything else runs on the main thread.
                if req.get("action") == "robot_command":
                    resp = {"id": req.get("id")}
                    try:
                        resp["status"] = "ok"
                        resp["result"] = self.dispatch(req.get("action"), req.get("params") or {})
                    except Exception as exc:  # noqa: BLE001
                        resp["status"] = "error"
                        resp["error"] = f"{type(exc).__name__}: {exc}"
                    with send_lock:
                        send_frame(conn, resp)
                    continue
                done = threading.Event()
                holder = {}

                def reply(resp, _done=done, _holder=holder):
                    _holder["resp"] = resp
                    _done.set()

                self.commands.put((req, reply))
                # Wait for main thread to execute; heartbeat empty frames on long ops.
                while not done.wait(timeout=10.0):
                    with send_lock:
                        conn.sendall(struct.pack(">I", 0))  # heartbeat
                with send_lock:
                    send_frame(conn, holder["resp"])
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _agent_thread(self, conn):
        """An mcp_robot agent registering itself."""
        try:
            reg = recv_frame(conn)
            if not reg or "register" not in reg:
                conn.close()
                return
            name = reg["register"]
            self.agents[name] = {"sock": conn, "lock": threading.Lock()}
            print(f"[mcp_bridge] agent registered: {name}", flush=True)
            send_frame(conn, {"status": "ok"})
            # Keep the thread alive to detect disconnect (agent only speaks when asked).
            while True:
                time.sleep(1.0)
                if conn.fileno() == -1:
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            for k, v in list(self.agents.items()):
                if v["sock"] is conn:
                    del self.agents[k]
                    print(f"[mcp_bridge] agent disconnected: {k}", flush=True)

    def call_agent(self, robot_name, action, params, timeout=30.0):
        agent = self.agents.get(robot_name)
        if not agent:
            available = list(self.agents.keys())
            raise ValueError(
                f"no mcp_robot agent registered for '{robot_name}'. "
                f"Registered agents: {available}. Use attach_mcp_controller first."
            )
        with agent["lock"]:
            sock = agent["sock"]
            sock.settimeout(timeout)
            agent["req_id"] = agent.get("req_id", 0) + 1
            req_id = agent["req_id"]
            send_frame(sock, {"id": req_id, "action": action, "params": params})
            while True:
                resp = recv_frame(sock)
                if resp is None:
                    raise ConnectionError(f"agent '{robot_name}' closed the connection")
                if resp.get("id") == req_id:
                    return resp
                # stale reply from a previously timed-out request: discard

    # -- main loop ----------------------------------------------------------

    def run(self):
        # "pause" is implemented by NOT stepping: the bridge is a synchronous
        # controller, so the whole simulation waits for it. This keeps the command
        # queue responsive while paused (calling simulationSetMode(PAUSE) would
        # block our own next step() and wedge the bridge).
        self.logical_mode = "realtime"
        while True:
            self._drain_commands()
            if self.logical_mode == "pause":
                time.sleep(0.02)
                continue
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()

    # -- motion / interaction tracking ------------------------------------

    def _sample_tracking(self):
        tr = self.tracking
        if tr is None:
            return
        tr["count"] += 1
        if tr["count"] % tr["sample_every"]:
            return
        t = round(self.sup.getTime(), 4)
        # NOTE: ContactPoint.node_id identifies the touching descendant of the
        # queried node, NOT the other object. To attribute contacts we match
        # contact-point positions across tracked nodes: the same world point
        # reported by two objects means they touch each other; unmatched points
        # are contacts with the (untracked) environment, e.g. the floor.
        points_by_node = {}
        for nid, entry in tr["nodes"].items():
            node = entry["node"]
            try:
                pos = node.getPosition()
                vel = node.getVelocity()
                speed = math.sqrt(sum(v * v for v in vel[:3]))
                entry["buf"].append((t, [round(v, 4) for v in pos], round(speed, 4)))
                points_by_node[nid] = [tuple(cp.point) for cp in node.getContactPoints(True)]
            except Exception:  # noqa: BLE001 - node may have been deleted
                continue
        contacts = set()
        contact_pos = {}
        eps = 1e-4
        node_ids = list(points_by_node)
        matched = {nid: [False] * len(points_by_node[nid]) for nid in node_ids}
        for i, a in enumerate(node_ids):
            for b in node_ids[i + 1:]:
                for ia, pa in enumerate(points_by_node[a]):
                    for ib, pb in enumerate(points_by_node[b]):
                        if (abs(pa[0] - pb[0]) < eps and abs(pa[1] - pb[1]) < eps
                                and abs(pa[2] - pb[2]) < eps):
                            pair = tuple(sorted((a, b)))
                            contacts.add(pair)
                            contact_pos.setdefault(pair, [round(v, 4) for v in pa])
                            matched[a][ia] = matched[b][ib] = True
        for nid in node_ids:
            unmatched = [p for i, p in enumerate(points_by_node[nid]) if not matched[nid][i]]
            if unmatched:
                pair = (nid, -1)
                contacts.add(pair)
                contact_pos.setdefault(pair, [round(v, 4) for v in unmatched[0]])
        for pair in contacts - tr["prev_contacts"]:
            tr["events"].append({"t": t, "event": "contact_start",
                                 "between": self._pair_names(pair, tr),
                                 "at": contact_pos.get(pair)})
        for pair in tr["prev_contacts"] - contacts:
            tr["events"].append({"t": t, "event": "contact_end",
                                 "between": self._pair_names(pair, tr)})
        tr["prev_contacts"] = contacts

    def _pair_names(self, pair, tr):
        names = []
        for nid in pair:
            if nid in tr["nodes"]:
                names.append(tr["nodes"][nid]["name"])
            elif nid == -1:
                names.append("<static environment>")
            else:
                node = self.sup.getFromId(nid)
                names.append(self._node_summary(node).get("name") or
                             self._node_summary(node).get("def") or
                             node.getTypeName() if node else f"node#{nid}")
        return names

    def _dynamic_nodes(self):
        """All nodes that can move: physics-enabled solids and robots."""
        out = []
        for node in self._iter_nodes(self.sup.getRoot(), max_depth=2):
            base = node.getBaseTypeName()
            if base == "Robot":
                if (node.getField("name") and
                        node.getField("name").getSFString() == "mcp_bridge"):
                    continue
                out.append(node)
            elif base in ("Solid",):
                phys = node.getField("physics")
                if phys and phys.getType() == Field.SF_NODE and phys.getSFNode():
                    out.append(node)
        return out

    def cmd_start_tracking(self, p):
        nodes = {}
        targets = ([self.find_node(r) for r in p["nodes"]] if p.get("nodes")
                   else self._dynamic_nodes())
        for node in targets:
            s = self._node_summary(node)
            nodes[node.getId()] = {"node": node,
                                   "name": s.get("name") or s.get("def") or s["type"],
                                   "buf": collections.deque(maxlen=3000)}
        self.tracking = {"nodes": nodes, "sample_every": max(1, int(p.get("sample_every", 2))),
                         "events": [], "prev_contacts": set(), "count": 0}
        return {"tracking": [e["name"] for e in nodes.values()],
                "sample_every": self.tracking["sample_every"]}

    def cmd_stop_tracking(self, p):
        result = self.cmd_get_tracking(p)
        self.tracking = None
        return result

    def cmd_get_tracking(self, p):
        tr = self.tracking
        if tr is None:
            raise ValueError("tracking is not active; call start_tracking or watch_simulation")
        max_points = int(p.get("max_points", 40))
        wanted = p.get("node")
        objects = {}
        for entry in tr["nodes"].values():
            name = entry["name"]
            if wanted and name != wanted:
                continue
            buf = list(entry["buf"])
            if not buf:
                objects[name] = {"samples": 0}
                continue
            start, end = buf[0][1], buf[-1][1]
            displacement = math.sqrt(sum((a - b) ** 2 for a, b in zip(start, end)))
            path_len = sum(
                math.sqrt(sum((a - b) ** 2 for a, b in zip(buf[i][1], buf[i + 1][1])))
                for i in range(len(buf) - 1))
            max_speed = max(s for _, _, s in buf)
            moved = path_len > 0.005
            info = {"moved": moved, "start": start, "end": end,
                    "displacement_m": round(displacement, 4),
                    "path_length_m": round(path_len, 4),
                    "max_speed_mps": max_speed,
                    "time_range": [buf[0][0], buf[-1][0]], "samples": len(buf)}
            if moved:
                stride = max(1, len(buf) // max_points)
                info["trajectory"] = [{"t": b[0], "pos": b[1], "speed": b[2]}
                                      for b in buf[::stride]]
            objects[name] = info
        return {"objects": objects, "interactions": tr["events"][-200:],
                "sim_time": self.sup.getTime()}

    def cmd_capture_sequence(self, p):
        steps = int(p.get("steps", 100))
        frames = min(int(p.get("frames", 5)), 10)
        quality = int(p.get("quality", 85))
        capture_at = {round(i * (steps - 1) / max(frames - 1, 1)) for i in range(frames)}
        images = []
        for i in range(steps):
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            if i in capture_at:
                path = os.path.join(tempfile.gettempdir(), f"webots_mcp_seq_{i}.jpg")
                self.sup.exportImage(path, quality)
                self.sup.step(self.timestep)
                self._sample_tracking()
                deadline = time.time() + 5.0
                while not os.path.exists(path) and time.time() < deadline:
                    self.sup.step(self.timestep)
                with open(path, "rb") as fh:
                    images.append({"t": round(self.sup.getTime(), 3),
                                   "base64": base64.b64encode(fh.read()).decode("ascii")})
                os.remove(path)
        return {"frames": images, "stepped": steps}

    def _drain_commands(self):
        while True:
            try:
                req, reply = self.commands.get_nowait()
            except queue.Empty:
                return
            resp = {"id": req.get("id")}
            try:
                result = self.dispatch(req.get("action"), req.get("params") or {})
                resp["status"] = "ok"
                resp["result"] = result
            except Exception as exc:  # noqa: BLE001 - report all errors to client
                resp["status"] = "error"
                resp["error"] = f"{type(exc).__name__}: {exc}"
                resp["traceback"] = traceback.format_exc(limit=6)
            reply(resp)

    # -- dispatch ------------------------------------------------------------

    def dispatch(self, action, p):
        handler = getattr(self, "cmd_" + str(action), None)
        if handler is None:
            raise ValueError(f"unknown action '{action}'")
        return handler(p)

    # -- node lookup ----------------------------------------------------------

    def find_node(self, ref):
        """Resolve a node by DEF name, unique id (int), or robot 'name' field."""
        if isinstance(ref, int) or (isinstance(ref, str) and ref.lstrip("-").isdigit()):
            node = self.sup.getFromId(int(ref))
            if node:
                return node
        if isinstance(ref, str):
            node = self.sup.getFromDef(ref)
            if node:
                return node
            # search top-level (and robot) nodes by their 'name' field
            for node in self._iter_nodes(self.sup.getRoot(), max_depth=4):
                f = node.getField("name")
                if f and f.getType() == Field.SF_STRING and f.getSFString() == ref:
                    return node
        raise ValueError(f"node not found: {ref!r} (use DEF name, id, or 'name' field)")

    # node-typed fields worth traversing: scene children, joint endpoints,
    # joint devices, robot slots
    _TREE_FIELDS = ("children", "endPoint", "device", "endpoint", "slot")

    def _child_nodes(self, node):
        for fname in self._TREE_FIELDS:
            f = node.getField(fname)
            if f is None:
                continue
            if f.getType() == Field.MF_NODE:
                for i in range(f.getCount()):
                    child = f.getMFNode(i)
                    if child:
                        yield fname, child
            elif f.getType() == Field.SF_NODE:
                child = f.getSFNode()
                if child:
                    yield fname, child

    def _iter_nodes(self, node, max_depth=10, depth=0):
        if depth > max_depth:
            return
        yield node
        for _, child in self._child_nodes(node):
            yield from self._iter_nodes(child, max_depth, depth + 1)

    def _node_summary(self, node):
        info = {
            "id": node.getId(),
            "type": node.getTypeName(),
            "base_type": node.getBaseTypeName(),
        }
        d = node.getDef()
        if d:
            info["def"] = d
        name_f = node.getField("name")
        if name_f and name_f.getType() == Field.SF_STRING:
            info["name"] = name_f.getSFString()
        trans_f = node.getField("translation")
        if trans_f and trans_f.getType() == Field.SF_VEC3F:
            info["translation"] = [round(v, 4) for v in trans_f.getSFVec3f()]
        return info

    # ======================================================================
    # Commands: general / simulation
    # ======================================================================

    def cmd_ping(self, p):
        return {"pong": True, "time": self.sup.getTime()}

    def cmd_get_simulation_state(self, p):
        return {
            "time": self.sup.getTime(),
            "mode": self.logical_mode,
            "basic_time_step": self.timestep,
            "world": self.sup.getWorldPath(),
            "webots_version": os.environ.get("WEBOTS_VERSION", "unknown"),
            "registered_agents": list(self.agents.keys()),
        }

    def cmd_set_simulation_mode(self, p):
        mode = p.get("mode")
        if mode not in ("pause", "realtime", "fast"):
            raise ValueError("mode must be one of ['pause', 'realtime', 'fast']")
        if mode == "pause":
            # implemented by the run loop not stepping (see run()); the Webots GUI
            # will show the sim as running at 0.00x, which is effectively paused.
            self.logical_mode = "pause"
        else:
            self.sup.simulationSetMode(
                Supervisor.SIMULATION_MODE_REAL_TIME if mode == "realtime"
                else Supervisor.SIMULATION_MODE_FAST)
            self.logical_mode = mode
        return {"mode": mode}

    def cmd_step_simulation(self, p):
        steps = int(p.get("steps", 1))
        for _ in range(min(steps, 100000)):
            if self.sup.step(self.timestep) == -1:
                return {"stepped": True, "terminated": True}
            self._sample_tracking()
        return {"stepped": steps, "time": self.sup.getTime()}

    def cmd_reset_simulation(self, p):
        if p.get("reload_world"):
            self.sup.worldReload()
        else:
            self.sup.simulationReset()
        return {"reset": True, "reload": bool(p.get("reload_world"))}

    def cmd_save_world(self, p):
        path = p.get("path")
        ok = self.sup.worldSave(path) if path else self.sup.worldSave()
        return {"saved": bool(ok), "path": path or self.sup.getWorldPath()}

    def cmd_load_world(self, p):
        self.sup.worldLoad(p["path"])
        return {"loading": p["path"]}

    # ======================================================================
    # Commands: scene tree
    # ======================================================================

    def cmd_find_nodes(self, p):
        """Search the scene by substring (case-insensitive) against DEF name,
        'name' field, and type; optional base_type filter (e.g. 'Robot', 'Solid')."""
        query = str(p.get("query", "")).lower()
        base_type = p.get("base_type")
        max_results = int(p.get("max_results", 20))
        max_depth = int(p.get("max_depth", 8))
        results = []
        for node in self._iter_nodes(self.sup.getRoot(), max_depth=max_depth):
            if node is self.sup.getRoot():
                continue
            s = self._node_summary(node)
            if base_type and node.getBaseTypeName() != base_type:
                continue
            hay = " ".join(str(s.get(k, "")) for k in ("def", "name", "type", "base_type")).lower()
            if query and query not in hay:
                continue
            try:
                s["position"] = [round(v, 4) for v in node.getPosition()]
            except Exception:  # noqa: BLE001
                pass
            results.append(s)
            if len(results) >= max_results:
                break
        return {"query": p.get("query"), "count": len(results), "nodes": results}

    def cmd_get_scene_tree(self, p):
        max_depth = int(p.get("max_depth", 3))
        include_fields = bool(p.get("include_fields", False))

        # Paged flat mode (Unity-MCP style): direct children of 'parent', sliced.
        if p.get("page_size"):
            page_size = max(1, int(p["page_size"]))
            cursor = int(p.get("cursor", 0))
            parent = self.find_node(p["parent"]) if p.get("parent") else self.sup.getRoot()
            kids = list(self._child_nodes(parent))
            page = []
            for fname, child in kids[cursor:cursor + page_size]:
                entry = self._node_summary(child)
                entry["via_field"] = fname
                entry["children_count"] = len(list(self._child_nodes(child)))
                if include_fields:
                    entry["fields"] = self._read_all_fields(child, max_items=8)
                page.append(entry)
            nxt = cursor + page_size
            return {"world": self.sup.getWorldPath(),
                    "parent": p.get("parent") or "<root>",
                    "total": len(kids), "cursor": cursor,
                    "next_cursor": nxt if nxt < len(kids) else None,
                    "nodes": page}

        def walk(node, depth):
            entry = self._node_summary(node)
            if include_fields:
                entry["fields"] = self._read_all_fields(node, max_items=8)
            kids = list(self._child_nodes(node))
            if kids and depth < max_depth:
                entry["children"] = [dict(walk(c, depth + 1), via_field=f) for f, c in kids]
            elif kids:
                entry["children_count"] = len(kids)
            return entry

        root = self.sup.getRoot()
        tree = [walk(child, 1) for _, child in self._child_nodes(root)]
        return {"world": self.sup.getWorldPath(), "nodes": tree}

    def _read_all_fields(self, node, max_items=20):
        fields = {}
        for i in range(node.getNumberOfFields()):
            f = node.getFieldByIndex(i)
            if f is None:
                continue
            try:
                fields[f.getName()] = read_field_value(f, max_items=max_items)
            except Exception as exc:  # noqa: BLE001
                fields[f.getName()] = f"<error: {exc}>"
        return fields

    def cmd_get_node_details(self, p):
        node = self.find_node(p["node"])
        info = self._node_summary(node)
        info["fields"] = self._read_all_fields(node)
        try:
            info["position"] = list(node.getPosition())
            info["orientation"] = list(node.getOrientation())
        except Exception:  # noqa: BLE001 - not all nodes have a pose
            pass
        return info

    def cmd_set_node_field(self, p):
        node = self.find_node(p["node"])
        field = node.getField(p["field"])
        if field is None:
            raise ValueError(f"node has no field '{p['field']}'")
        write_field_value(field, p["value"], p.get("index"))
        return {"set": p["field"], "value": p["value"]}

    def cmd_get_node_pose(self, p):
        node = self.find_node(p["node"])
        out = {
            "position": list(node.getPosition()),
            "orientation": list(node.getOrientation()),
            "velocity": list(node.getVelocity()) if p.get("include_velocity") else None,
        }
        if p.get("relative_to"):
            ref = self.find_node(p["relative_to"])
            out["pose_relative_to"] = p["relative_to"]
            out["pose_matrix_4x4"] = [round(v, 6) for v in node.getPose(ref)]
        if p.get("include_center_of_mass"):
            try:
                out["center_of_mass"] = [round(v, 4) for v in node.getCenterOfMass()]
                out["statically_balanced"] = bool(node.getStaticBalance())
            except Exception as exc:  # noqa: BLE001 - needs physics
                out["center_of_mass_error"] = str(exc)
        return out

    def cmd_get_node_string(self, p):
        node = self.find_node(p["node"])
        return {"node": self._node_summary(node), "node_string": node.exportString()}

    def cmd_clone_node(self, p):
        src = self.find_node(p["node"])
        s = src.exportString()
        # strip any DEF so the copy doesn't collide; apply new_def if given
        if s.startswith("DEF "):
            s = s.split(None, 2)[2]
        if p.get("new_def"):
            s = f"DEF {p['new_def']} " + s
        if p.get("parent"):
            field = self.find_node(p["parent"]).getField(p.get("field", "children"))
        else:
            field = src.getParentNode().getField("children") \
                if src.getParentNode() else self.sup.getRoot().getField("children")
            if field is None:
                field = self.sup.getRoot().getField("children")
        field.importMFNodeFromString(-1, s)
        new_node = field.getMFNode(field.getCount() - 1)
        if p.get("position") and new_node.getField("translation"):
            new_node.getField("translation").setSFVec3f([float(v) for v in p["position"]])
        elif new_node.getField("translation") and new_node.getField("translation").getType() == Field.SF_VEC3F:
            # nudge so the copy isn't perfectly inside the original
            t = new_node.getField("translation").getSFVec3f()
            new_node.getField("translation").setSFVec3f([t[0] + 0.25, t[1] + 0.25, t[2]])
        return {"cloned_from": self._node_summary(src),
                "new_node": self._node_summary(new_node)}

    _MF_INSERTERS = {
        Field.MF_BOOL: "insertMFBool",
        Field.MF_INT32: "insertMFInt32",
        Field.MF_FLOAT: "insertMFFloat",
        Field.MF_VEC2F: "insertMFVec2f",
        Field.MF_VEC3F: "insertMFVec3f",
        Field.MF_ROTATION: "insertMFRotation",
        Field.MF_COLOR: "insertMFColor",
        Field.MF_STRING: "insertMFString",
    }

    def cmd_insert_field_item(self, p):
        """Insert a value into a multi-valued (MF) field at an index (-1 = append)."""
        node = self.find_node(p["node"])
        field = node.getField(p["field"])
        if field is None:
            raise ValueError(f"node has no field '{p['field']}'")
        index = int(p.get("index", -1))
        if field.getType() == Field.MF_NODE:
            field.importMFNodeFromString(index, str(p["value"]))
        else:
            inserter = self._MF_INSERTERS.get(field.getType())
            if inserter is None:
                raise ValueError(f"cannot insert into field of type {field.getTypeName()}")
            getattr(field, inserter)(index, p["value"])
        return {"inserted": True, "field": p["field"], "count": field.getCount()}

    def cmd_remove_field_item(self, p):
        """Remove one item of an MF field by index, or clear an SF_NODE field."""
        node = self.find_node(p["node"])
        field = node.getField(p["field"])
        if field is None:
            raise ValueError(f"node has no field '{p['field']}'")
        if p.get("index") is None:
            field.removeSF()
            return {"removed": "SF value", "field": p["field"]}
        field.removeMF(int(p["index"]))
        return {"removed": int(p["index"]), "field": p["field"],
                "count": field.getCount()}

    def cmd_save_checkpoint(self, p):
        """Save pose+physics state of nodes under a named checkpoint."""
        name = str(p.get("name", "default"))
        nodes = ([self.find_node(r) for r in p["nodes"]]
                 if p.get("nodes") else self._dynamic_nodes())
        if not hasattr(self, "checkpoints"):
            self.checkpoints = {}
        saved = []
        for node in nodes:
            node.saveState(f"mcp_{name}")
            saved.append(node.getId())
        self.checkpoints[name] = saved
        return {"checkpoint": name, "nodes_saved": len(saved),
                "sim_time": round(self.sup.getTime(), 3)}

    def cmd_restore_checkpoint(self, p):
        """Restore a named checkpoint (rewind objects to their saved states)."""
        name = str(p.get("name", "default"))
        ids = getattr(self, "checkpoints", {}).get(name)
        if ids is None:
            raise ValueError(f"no checkpoint '{name}'. "
                             f"Saved: {list(getattr(self, 'checkpoints', {}))}")
        restored = 0
        for nid in ids:
            node = self.sup.getFromId(nid)
            if node is None:
                continue  # deleted since the save
            node.loadState(f"mcp_{name}")
            node.resetPhysics()
            restored += 1
        self.sup.simulationResetPhysics()
        return {"checkpoint": name, "nodes_restored": restored,
                "nodes_missing": len(ids) - restored}

    def cmd_set_joint_position(self, p):
        """Pose a joint directly through the supervisor (no motor/controller needed)."""
        node = self.find_node(p["node"])
        if "Joint" not in node.getBaseTypeName():
            raise ValueError(f"'{p['node']}' is a {node.getBaseTypeName()}, not a Joint. "
                             "Pass the HingeJoint/SliderJoint/BallJoint node itself "
                             "(find them via get_scene_tree with a robot parent).")
        node.setJointPosition(float(p["position"]), int(p.get("index", 1)))
        return {"joint": self._node_summary(node), "position": float(p["position"])}

    def cmd_set_node_visibility(self, p):
        """Hide/show a node for a specific viewer (Viewpoint or a camera node)."""
        node = self.find_node(p["node"])
        if p.get("from_node"):
            viewer = self.find_node(p["from_node"])
        else:
            viewer = self._get_viewpoint()
        node.setVisibility(viewer, bool(p.get("visible", True)))
        return {"node": self._node_summary(node), "visible": bool(p.get("visible", True)),
                "from": p.get("from_node") or "<Viewpoint>"}

    def cmd_get_node_proto(self, p):
        """Introspect a PROTO instance: its parameters (name, type, value) and
        derivation chain."""
        node = self.find_node(p["node"])
        if not node.isProto():
            return {"node": self._node_summary(node), "is_proto": False}
        out = {"node": self._node_summary(node), "is_proto": True, "protos": []}
        proto = node.getProto()
        while proto is not None:
            params = {}
            for i in range(proto.getNumberOfFields()):
                f = proto.getFieldByIndex(i)
                if f is None:
                    continue
                try:
                    params[f.getName()] = read_field_value(f, max_items=8)
                except Exception as exc:  # noqa: BLE001
                    params[f.getName()] = f"<error: {exc}>"
            out["protos"].append({"type": proto.getTypeName(),
                                  "derived": bool(proto.isDerived()),
                                  "parameters": params})
            proto = proto.getParent()
        return out

    def cmd_frame_node(self, p):
        """Move the Viewpoint to frame a node (Webots' built-in 'move viewpoint to object')."""
        node = self.find_node(p["node"])
        node.moveViewpoint()
        return {"framed": self._node_summary(node)}

    def cmd_get_selected_node(self, p):
        node = self.sup.getSelected()
        if node is None:
            return {"selected": None,
                    "hint": "no node is selected in the Webots scene tree / 3D view"}
        info = self._node_summary(node)
        try:
            info["position"] = [round(v, 4) for v in node.getPosition()]
        except Exception:  # noqa: BLE001
            pass
        return {"selected": info}

    def cmd_enable_camera_recognition(self, p):
        """Add (or configure) a Recognition node on a robot's camera so
        get_recognition / segmentation work."""
        robot_node = self.find_node(p["robot"])
        wanted = p.get("camera")
        segmentation = bool(p.get("segmentation", False))
        cam = None
        for node in self._iter_nodes(robot_node, max_depth=10):
            if node.getBaseTypeName() == "Camera":
                nf = node.getField("name")
                cam_name = nf.getSFString() if nf else ""
                if wanted is None or cam_name == wanted:
                    cam = node
                    break
        if cam is None:
            raise ValueError(f"no Camera{f' named {wanted!r}' if wanted else ''} "
                             f"found under robot '{p['robot']}'")
        rec_field = cam.getField("recognition")
        if rec_field is None:
            raise ValueError("camera node has no 'recognition' field")
        existing = rec_field.getSFNode()
        if existing is None:
            seg = "TRUE" if segmentation else "FALSE"
            rec_field.importSFNodeFromString(f"Recognition {{ segmentation {seg} }}")
            action = "added Recognition node"
        else:
            action = "Recognition node already present"
            if segmentation:
                sf = existing.getField("segmentation")
                if sf:
                    sf.setSFBool(True)
                    action += "; segmentation enabled"
        return {"camera": (cam.getField("name").getSFString()
                           if cam.getField("name") else cam.getTypeName()),
                "action": action, "segmentation": segmentation,
                "note": "re-run attach/restart is NOT needed; recognition activates "
                        "on the next get_recognition call"}

    def cmd_move_node(self, p):
        node = self.find_node(p["node"])
        if "position" in p and p["position"] is not None:
            f = node.getField("translation")
            if f is None:
                raise ValueError("node has no translation field")
            f.setSFVec3f([float(v) for v in p["position"]])
        if "rotation" in p and p["rotation"] is not None:
            f = node.getField("rotation")
            if f is None:
                raise ValueError("node has no rotation field")
            f.setSFRotation([float(v) for v in p["rotation"]])
        if p.get("reset_physics", True):
            node.resetPhysics()
        return {"moved": True}

    def cmd_spawn_node(self, p):
        node_string = p["node_string"]
        parent_ref = p.get("parent")
        if parent_ref:
            parent = self.find_node(parent_ref)
            field = parent.getField(p.get("field", "children"))
        else:
            field = self.sup.getRoot().getField("children")
        position = int(p.get("position", -1))
        field.importMFNodeFromString(position, node_string)
        count = field.getCount()
        new_node = field.getMFNode(count - 1 if position == -1 else position)
        return {"spawned": True, "node": self._node_summary(new_node) if new_node else None}

    def cmd_delete_node(self, p):
        node = self.find_node(p["node"])
        summary = self._node_summary(node)
        node.remove()
        return {"deleted": summary}

    def cmd_get_node_field(self, p):
        node = self.find_node(p["node"])
        field = node.getField(p["field"])
        if field is None:
            available = [node.getFieldByIndex(i).getName()
                         for i in range(node.getNumberOfFields())]
            raise ValueError(f"no field '{p['field']}'. Fields: {available}")
        return {"field": p["field"], "type": field.getTypeName(),
                "value": read_field_value(field, max_items=int(p.get("max_items", 1000)))}

    def cmd_set_velocity(self, p):
        node = self.find_node(p["node"])
        lin = p.get("linear") or [0, 0, 0]
        ang = p.get("angular") or [0, 0, 0]
        node.setVelocity([float(v) for v in (*lin, *ang)])
        return {"velocity_set": True}

    def cmd_apply_force(self, p):
        node = self.find_node(p["node"])
        relative = bool(p.get("relative", False))
        duration_steps = max(1, int(p.get("duration_steps", 1)))

        def apply_once():
            if p.get("force"):
                if p.get("offset"):
                    node.addForceWithOffset([float(v) for v in p["force"]],
                                            [float(v) for v in p["offset"]], relative)
                else:
                    node.addForce([float(v) for v in p["force"]], relative)
            if p.get("torque"):
                node.addTorque([float(v) for v in p["torque"]], relative)

        # a force lasts one physics step; re-apply across duration_steps
        apply_once()
        for _ in range(duration_steps - 1):
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            apply_once()
        return {"applied": True, "duration_steps": duration_steps,
                "sim_time": self.sup.getTime()}

    def cmd_restart_controller(self, p):
        node = self.find_node(p["robot"])
        node.restartController()
        return {"restarted": True}

    @staticmethod
    def _look_at_orientation(pos, target, up=(0.0, 0.0, 1.0)):
        """Axis-angle so a Webots Viewpoint at pos looks at target. Webots (ENU/FLU)
        cameras look along their local +x axis with +z up (verified against the
        bundled sample worlds)."""
        def sub(a, b):
            return [a[i] - b[i] for i in range(3)]

        def norm(v):
            n = math.sqrt(sum(x * x for x in v)) or 1.0
            return [x / n for x in v]

        def cross(a, b):
            return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]]

        x = norm(sub(target, pos))          # camera forward = +x
        y = norm(cross(up, x))              # camera left
        if sum(abs(v) for v in y) < 1e-6:   # looking straight up/down
            y = [0.0, 1.0, 0.0]
        z = cross(x, y)                     # camera up
        # rotation matrix columns are the camera basis
        r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
        trace = r[0][0] + r[1][1] + r[2][2]
        angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        if angle < 1e-6:
            return [0.0, 0.0, 1.0, 0.0]
        s = 2.0 * math.sin(angle)
        axis = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s]
        return [round(v, 5) for v in (*axis, angle)]

    def _get_viewpoint(self):
        root_children = self.sup.getRoot().getField("children")
        for i in range(root_children.getCount()):
            n = root_children.getMFNode(i)
            if n and n.getTypeName() == "Viewpoint":
                return n
        raise ValueError("no Viewpoint node found in world")

    def _save_viewpoint(self, vp):
        state = {"position": list(vp.getField("position").getSFVec3f()),
                 "orientation": list(vp.getField("orientation").getSFRotation())}
        f = vp.getField("follow")
        state["follow"] = f.getSFString() if f else None
        return state

    def _restore_viewpoint(self, vp, state):
        vp.getField("position").setSFVec3f(state["position"])
        vp.getField("orientation").setSFRotation(state["orientation"])
        f = vp.getField("follow")
        if f is not None and state["follow"] is not None:
            f.setSFString(state["follow"])

    def cmd_set_viewpoint(self, p):
        vp = self._get_viewpoint()
        orientation = p.get("orientation")
        position = p.get("position")
        if p.get("look_at"):
            target = [float(v) for v in p["look_at"]]
            if position is None:
                position = vp.getField("position").getSFVec3f()
            orientation = self._look_at_orientation(position, target)
        if position:
            vp.getField("position").setSFVec3f([float(v) for v in position])
        if orientation:
            vp.getField("orientation").setSFRotation([float(v) for v in orientation])
        if p.get("follow") is not None:
            f = vp.getField("follow")
            if f:
                f.setSFString(str(p["follow"]))
        return {"viewpoint_updated": True,
                "orientation": orientation, "position": position}

    # ======================================================================
    # Commands: visual / labels
    # ======================================================================

    def _capture_jpeg(self, quality):
        path = os.path.join(tempfile.gettempdir(), f"webots_mcp_view_{os.getpid()}.jpg")
        self.sup.exportImage(path, quality)
        # exportImage is asynchronous-ish; step once so the file is written.
        self.sup.step(self.timestep)
        deadline = time.time() + 5.0
        while not os.path.exists(path) and time.time() < deadline:
            self.sup.step(self.timestep)
        with open(path, "rb") as fh:
            data = fh.read()
        os.remove(path)
        return base64.b64encode(data).decode("ascii")

    def _resolve_point(self, target):
        """Resolve a capture target: node ref -> its world position; [x,y,z] -> itself."""
        if isinstance(target, (list, tuple)):
            return [float(v) for v in target]
        return list(self.find_node(target).getPosition())

    def cmd_get_scene_bounds(self, p):
        """Center and radius of the interesting part of the scene (dynamic
        objects and robots; falls back to all top-level solids)."""
        nodes = self._dynamic_nodes()
        if not nodes:
            nodes = [n for _, n in self._child_nodes(self.sup.getRoot())
                     if n.getBaseTypeName() not in ("WorldInfo", "Viewpoint", "Background",
                                                    "TexturedBackground", "DirectionalLight",
                                                    "TexturedBackgroundLight", "PointLight")]
        positions = []
        for n in nodes:
            try:
                positions.append(n.getPosition())
            except Exception:  # noqa: BLE001
                continue
        if not positions:
            return {"center": [0.0, 0.0, 0.0], "radius": 2.0, "objects": 0}
        center = [sum(pv[i] for pv in positions) / len(positions) for i in range(3)]
        radius = max((math.dist(pv, center) for pv in positions), default=0.0)
        return {"center": [round(v, 4) for v in center],
                "radius": round(max(radius, 0.5), 4),
                "objects": len(positions)}

    def cmd_screenshot(self, p):
        quality = int(p.get("quality", 90))
        vp = None
        saved = None
        if p.get("view_position") or p.get("view_target") is not None:
            vp = self._get_viewpoint()
            saved = self._save_viewpoint(vp)
            follow_f = vp.getField("follow")
            if follow_f:
                follow_f.setSFString("")
            position = p.get("view_position")
            target = p.get("view_target")
            if target is not None:
                tp = self._resolve_point(target)
                if position is None:
                    # frame the target from a 3/4 view scaled to scene size
                    r = max(self.cmd_get_scene_bounds({})["radius"] * 1.5, 1.5)
                    position = [tp[0] + r, tp[1] - r, tp[2] + r * 0.8]
                vp.getField("orientation").setSFRotation(
                    self._look_at_orientation(position, tp))
            if position is not None:
                vp.getField("position").setSFVec3f([float(v) for v in position])
            self.sup.step(self.timestep)  # let the render catch up
        try:
            b64 = self._capture_jpeg(quality)
        finally:
            if vp is not None and not p.get("keep_viewpoint", False):
                self._restore_viewpoint(vp, saved)
        return {"format": "jpeg", "base64": b64}

    def cmd_screenshot_batch(self, p):
        """Multi-angle capture around a target (or the whole scene):
        batch='surround' -> 6 fixed views; batch='orbit' -> azimuths x elevations."""
        quality = int(p.get("quality", 80))
        bounds = self.cmd_get_scene_bounds({})
        target = p.get("target")
        center = self._resolve_point(target) if target is not None else bounds["center"]
        radius = float(p.get("radius") or max(bounds["radius"] * 2.2, 1.5))
        if p.get("batch", "surround") == "surround":
            views = [(0, 25), (90, 25), (180, 25), (270, 25), (45, 0), (0, 85)]
        else:
            azimuths = min(int(p.get("azimuths", 8)), 36)
            elevations = p.get("elevations") or [0, 30, -15]
            views = [(a * 360.0 / azimuths, e) for e in elevations
                     for a in range(azimuths)]
            views = views[:12]  # payload safety cap
        vp = self._get_viewpoint()
        saved = self._save_viewpoint(vp)
        follow_f = vp.getField("follow")
        if follow_f:
            follow_f.setSFString("")
        shots = []
        try:
            for az, el in views:
                a, e = math.radians(az), math.radians(el)
                pos = [center[0] + radius * math.cos(e) * math.cos(a),
                       center[1] + radius * math.cos(e) * math.sin(a),
                       center[2] + radius * math.sin(e)]
                vp.getField("position").setSFVec3f(pos)
                vp.getField("orientation").setSFRotation(
                    self._look_at_orientation(pos, center))
                self.sup.step(self.timestep)  # render the new viewpoint
                shots.append({"azimuth": az, "elevation": el,
                              "position": [round(v, 3) for v in pos],
                              "base64": self._capture_jpeg(quality)})
        finally:
            self._restore_viewpoint(vp, saved)
        return {"scene_center": center, "capture_radius": radius,
                "screenshots": shots}



    def cmd_set_label(self, p):
        self.sup.setLabel(
            int(p.get("label_id", 0)),
            str(p.get("text", "")),
            float(p.get("x", 0.05)),
            float(p.get("y", 0.05)),
            float(p.get("size", 0.08)),
            int(str(p.get("color", "0xFFFFFF")), 0),
            float(p.get("transparency", 0.0)),
            str(p.get("font", "Arial")),
        )
        return {"label_set": True}

    def cmd_export_screenshot(self, p):
        """Save the 3D view to an image file on disk (any resolution Webots renders)."""
        path = p["path"]
        self.sup.exportImage(path, int(p.get("quality", 90)))
        self.sup.step(self.timestep)
        return {"exported": path}

    def cmd_get_recording_status(self, p):
        return {"movie_ready": bool(self.sup.movieIsReady()),
                "movie_failed": bool(self.sup.movieFailed())}

    def cmd_world_reload(self, p):
        self.sup.worldReload()
        return {"reloading": True,
                "note": "all controllers restart, including this bridge — expect a "
                        "brief disconnect, then reconnect automatically"}

    def cmd_start_movie(self, p):
        self.sup.movieStartRecording(
            p["path"], int(p.get("width", 1280)), int(p.get("height", 720)),
            int(p.get("codec", 0)), int(p.get("quality", 90)),
            int(p.get("acceleration", 1)), bool(p.get("caption", False)))
        return {"recording": p["path"]}

    def cmd_stop_movie(self, p):
        self.sup.movieStopRecording()
        # movie encoding is asynchronous; poll readiness briefly
        for _ in range(int(p.get("wait_steps", 100))):
            if self.sup.movieIsReady():
                break
            self.sup.step(self.timestep)
        return {"stopped": True, "ready": self.sup.movieIsReady(),
                "failed": self.sup.movieFailed()}

    def cmd_start_animation(self, p):
        self.sup.animationStartRecording(p["path"])
        return {"recording": p["path"]}

    def cmd_stop_animation(self, p):
        self.sup.animationStopRecording()
        return {"stopped": True}

    # ======================================================================
    # Commands: robots
    # ======================================================================

    def cmd_list_robots(self, p):
        robots = []
        for node in self._iter_nodes(self.sup.getRoot(), max_depth=int(p.get("max_depth", 4))):
            if node.getBaseTypeName() == "Robot":
                info = self._node_summary(node)
                ctrl = node.getField("controller")
                if ctrl:
                    info["controller"] = ctrl.getSFString()
                info["has_mcp_agent"] = info.get("name") in self.agents
                try:
                    info["position"] = [round(v, 4) for v in node.getPosition()]
                except Exception:  # noqa: BLE001
                    pass
                robots.append(info)
        return {"robots": robots, "registered_agents": list(self.agents.keys())}

    def cmd_attach_mcp_controller(self, p):
        node = self.find_node(p["robot"])
        if node.getBaseTypeName() != "Robot":
            raise ValueError(f"node is a {node.getBaseTypeName()}, not a Robot")
        ctrl = node.getField("controller")
        old = ctrl.getSFString()
        ctrl.setSFString("mcp_robot")
        node.restartController()
        return {"attached": True, "previous_controller": old,
                "note": "agent will register within a few simulation steps"}

    def cmd_robot_command(self, p):
        """Proxy a command to an mcp_robot agent."""
        return self.call_agent(p["robot"], p["action"], p.get("params") or {},
                               timeout=float(p.get("timeout", 30.0)))

    # ======================================================================
    # Commands: code execution
    # ======================================================================

    def cmd_execute_code(self, p):
        code = p["code"]
        namespace = {
            "supervisor": self.sup,
            "sup": self.sup,
            "Node": Node,
            "Field": Field,
            "bridge": self,
            "result": None,
        }
        stdout = io.StringIO()
        old_stdout = sys.stdout
        sys.stdout = stdout
        try:
            exec(compile(code, "<mcp>", "exec"), namespace)  # noqa: S102 - intentional escape hatch
        finally:
            sys.stdout = old_stdout
        result = namespace.get("result")
        try:
            json.dumps(result)
        except (TypeError, ValueError):
            result = repr(result)
        return {"result": result, "stdout": stdout.getvalue()}


if __name__ == "__main__":
    Bridge().run()
