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

# scene_math / run_analysis live next to this controller; make them importable.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_math  # noqa: E402
import run_analysis  # noqa: E402
import authoring  # noqa: E402
import experiments  # noqa: E402
import perception  # noqa: E402

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
        self._contact_tracked = set()  # node ids with contact-points tracking enabled
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
                self._ensure_contact_tracking(node)
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
    # Geometry / semantic-scene helpers (P7)
    # ======================================================================

    def _yaw_from_orientation(self, node):
        """Rotation about the world vertical axis (rad), or None."""
        try:
            o = node.getOrientation()  # 3x3 row-major
            return round(math.atan2(o[3], o[0]), 4)
        except Exception:  # noqa: BLE001
            return None

    @staticmethod
    def _geo_field(geo, name, default=None):
        f = geo.getField(name)
        if f is None:
            return default
        try:
            t = f.getType()
            if t == Field.SF_FLOAT:
                return f.getSFFloat()
            if t == Field.SF_VEC3F:
                return f.getSFVec3f()
            if t == Field.SF_VEC2F:
                return f.getSFVec2f()
        except Exception:  # noqa: BLE001
            return default
        return default

    def _geometry_extents(self, geo):
        """Local half-extents [hx,hy,hz] of a geometry node, or None.
        Approximate: Cylinder/Capsule assumed y-axis aligned (Webots default)."""
        if geo is None:
            return None
        t = geo.getTypeName()
        if t == "Box":
            s = self._geo_field(geo, "size", [0, 0, 0]) or [0, 0, 0]
            return [abs(s[0]) / 2, abs(s[1]) / 2, abs(s[2]) / 2]
        if t == "Sphere":
            r = self._geo_field(geo, "radius", 0.0) or 0.0
            return [r, r, r]
        if t in ("Cylinder", "Capsule"):
            r = self._geo_field(geo, "radius", 0.0) or 0.0
            h = self._geo_field(geo, "height", 0.0) or 0.0
            return [r, h / 2, r]
        if t == "Plane":
            s = self._geo_field(geo, "size", [1, 1]) or [1, 1]
            return [abs(s[0]) / 2, abs(s[1]) / 2, 0.0]
        return None

    @staticmethod
    def _shape_geometry(shape):
        f = shape.getField("geometry")
        if f and f.getType() == Field.SF_NODE:
            return f.getSFNode()
        return None

    def _extents_from_bounding(self, bo, depth=0):
        """Half-extents from a boundingObject subtree (Shape/geometry/Group/Pose)."""
        if bo is None or depth > 4:
            return None
        ext = self._geometry_extents(bo)
        if ext:
            return ext
        if bo.getBaseTypeName() == "Shape":
            return self._geometry_extents(self._shape_geometry(bo))
        f = bo.getField("children")
        if f and f.getType() == Field.MF_NODE and f.getCount():
            return self._extents_from_bounding(f.getMFNode(0), depth + 1)
        f = bo.getField("geometry")
        if f and f.getType() == Field.SF_NODE and f.getSFNode():
            return self._geometry_extents(f.getSFNode())
        return None

    def _node_extents(self, node):
        """Local half-extents from boundingObject, else first Shape geometry."""
        bo = node.getField("boundingObject")
        if bo and bo.getType() == Field.SF_NODE and bo.getSFNode():
            ext = self._extents_from_bounding(bo.getSFNode())
            if ext:
                return ext
        for _, child in self._child_nodes(node):
            if child.getBaseTypeName() == "Shape":
                ext = self._geometry_extents(self._shape_geometry(child))
                if ext:
                    return ext
        return None

    def _world_aabb(self, node):
        """Approximate world-frame AABB (min, max) from local extents + pose, or None."""
        ext = self._node_extents(node)
        if ext is None:
            return None
        try:
            pos = node.getPosition()
            o = node.getOrientation()
        except Exception:  # noqa: BLE001
            return None
        hx, hy, hz = ext
        wx = abs(o[0]) * hx + abs(o[1]) * hy + abs(o[2]) * hz
        wy = abs(o[3]) * hx + abs(o[4]) * hy + abs(o[5]) * hz
        wz = abs(o[6]) * hx + abs(o[7]) * hy + abs(o[8]) * hz
        return ([pos[0] - wx, pos[1] - wy, pos[2] - wz],
                [pos[0] + wx, pos[1] + wy, pos[2] + wz])

    @staticmethod
    def _aabb_overlap(a, b):
        return all(a[0][i] <= b[1][i] and b[0][i] <= a[1][i] for i in range(3))

    def _node_color(self, node):
        """Best-effort RGB color from recognitionColors or a Shape appearance."""
        rc = node.getField("recognitionColors")
        if rc and rc.getType() == Field.MF_COLOR and rc.getCount():
            return [round(v, 3) for v in rc.getMFColor(0)]
        for n in self._iter_nodes(node, max_depth=4):
            if n.getBaseTypeName() != "Shape":
                continue
            appf = n.getField("appearance")
            app = appf.getSFNode() if appf and appf.getType() == Field.SF_NODE else None
            if app is None:
                continue
            bc = app.getField("baseColor")
            if bc and bc.getType() == Field.SF_COLOR:
                try:
                    return [round(v, 3) for v in bc.getSFColor()]
                except Exception:  # noqa: BLE001
                    pass
            mat = app.getField("material")
            if mat and mat.getType() == Field.SF_NODE and mat.getSFNode():
                dc = mat.getSFNode().getField("diffuseColor")
                if dc and dc.getType() == Field.SF_COLOR:
                    try:
                        return [round(v, 3) for v in dc.getSFColor()]
                    except Exception:  # noqa: BLE001
                        pass
        return None

    def _node_mass_kind(self, node):
        """(mass_or_None, 'static'|'dynamic') from the Physics node."""
        pf = node.getField("physics")
        if not (pf and pf.getType() == Field.SF_NODE and pf.getSFNode()):
            return None, "static"
        mf = pf.getSFNode().getField("mass")
        try:
            mass = mf.getSFFloat() if mf else -1.0
        except Exception:  # noqa: BLE001
            mass = -1.0
        return (round(mass, 4) if mass and mass > 0 else None), "dynamic"

    def _catalog_entry(self, node):
        s = self._node_summary(node)
        entry = {"id": node.getId(), "name": s.get("name"), "def": s.get("def"),
                 "type": node.getTypeName(), "base_type": node.getBaseTypeName()}
        try:
            entry["position"] = [round(v, 4) for v in node.getPosition()]
        except Exception:  # noqa: BLE001
            entry["position"] = None
        entry["yaw"] = self._yaw_from_orientation(node)
        ext = self._node_extents(node)
        entry["size"] = [round(2 * e, 4) for e in ext] if ext else None
        mass, kind = self._node_mass_kind(node)
        entry["mass"] = mass
        entry["kind"] = kind
        entry["color"] = self._node_color(node)
        par = node.getParentNode()
        entry["parent"] = par.getId() if par else None
        return entry

    def _catalog_nodes(self, max_depth=3):
        out = []
        for node in self._iter_nodes(self.sup.getRoot(), max_depth=max_depth):
            base = node.getBaseTypeName()
            if base not in ("Solid", "Robot"):
                continue
            nm = node.getField("name")
            if base == "Robot" and nm and nm.getSFString() == "mcp_bridge":
                continue
            out.append(node)
        return out

    def _ensure_contact_tracking(self, node, include_descendants=True):
        """R2025a only populates getContactPoints() after tracking is enabled;
        enable it once per node (idempotent) so contact queries actually work."""
        try:
            nid = node.getId()
        except Exception:  # noqa: BLE001
            return
        if nid in self._contact_tracked:
            return
        try:
            node.enableContactPointsTracking(self.timestep, bool(include_descendants))
            self._contact_tracked.add(nid)
        except Exception:  # noqa: BLE001
            pass

    def _contact_partners(self, node):
        """Summarize contacts as [{other_node_id, other_name, points}]."""
        self._ensure_contact_tracking(node)
        try:
            pts = node.getContactPoints(True)
        except Exception:  # noqa: BLE001
            return []
        seen = {}
        for cp in pts:
            nid = getattr(cp, "node_id", None)
            info = seen.setdefault(nid, {"other_node_id": nid, "points": 0})
            info["points"] += 1
        for nid, info in seen.items():
            other = self.sup.getFromId(nid) if nid else None
            if other:
                nm = other.getField("name")
                info["other_name"] = ((nm.getSFString() if nm else None)
                                      or other.getDef() or other.getTypeName())
        return list(seen.values())

    # ======================================================================
    # Commands: semantic scene model (P7) & contacts (P2)
    # ======================================================================

    def cmd_get_contact_points(self, p):
        node = self.find_node(p["node"])
        include = bool(p.get("include_descendants", False))
        # R2025a needs tracking enabled before points populate; enable then step
        # one timestep so the query reflects the contacts touching "right now".
        newly = node.getId() not in self._contact_tracked
        self._ensure_contact_tracking(node, include)
        if newly:
            # one step so the just-enabled tracker captures current contacts
            self.sup.step(self.timestep)
            self._sample_tracking()
        try:
            pts = node.getContactPoints(include)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(f"getContactPoints failed: {exc}")
        result = []
        for cp in pts:
            entry = {"point": [round(v, 4) for v in cp.point]}
            nid = getattr(cp, "node_id", None)
            if nid is not None:
                entry["other_node_id"] = nid
                other = self.sup.getFromId(nid)
                if other:
                    nm = other.getField("name")
                    entry["other_name"] = ((nm.getSFString() if nm else None)
                                           or other.getDef() or other.getTypeName())
            result.append(entry)
        return {"node": self._node_summary(node), "count": len(result),
                "contact_points": result}

    def cmd_get_object_catalog(self, p):
        max_depth = int(p.get("max_depth", 3))
        nodes = self._catalog_nodes(max_depth)
        total = len(nodes)
        cursor = int(p.get("cursor", 0))
        page_size = p.get("page_size")
        page = nodes[cursor:cursor + int(page_size)] if page_size else nodes
        out = {"objects": [self._catalog_entry(n) for n in page],
               "count": len(page), "total": total}
        if page_size and cursor + int(page_size) < total:
            out["next_cursor"] = cursor + int(page_size)
        return out

    def cmd_get_object_properties(self, p):
        node = self.find_node(p["node"])
        entry = self._catalog_entry(node)
        aabb = self._world_aabb(node)
        if aabb:
            entry["aabb"] = {"min": [round(v, 4) for v in aabb[0]],
                             "max": [round(v, 4) for v in aabb[1]]}
        try:
            entry["velocity"] = [round(v, 4) for v in node.getVelocity()]
        except Exception:  # noqa: BLE001
            pass
        try:
            entry["center_of_mass"] = [round(v, 4) for v in node.getCenterOfMass()]
            entry["statically_balanced"] = bool(node.getStaticBalance())
        except Exception:  # noqa: BLE001
            pass
        entry["contacts"] = self._contact_partners(node)
        devf = node.getField("controller")
        if devf is not None:
            devices = []
            for _, child in self._child_nodes(node):
                dn = child.getField("name")
                devices.append({"type": child.getTypeName(),
                                "name": dn.getSFString() if dn else None})
            entry["is_robot"] = True
            entry["devices"] = devices
        return entry

    def _spatial_catalog(self, max_depth=3):
        """[(node, entry, aabb)] for spatial queries."""
        rows = []
        for node in self._catalog_nodes(max_depth):
            rows.append((node, self._catalog_entry(node), self._world_aabb(node)))
        return rows

    def cmd_find_nodes_near(self, p):
        center = self._resolve_point(p.get("point") if p.get("point") is not None
                                     else p["node"])
        radius = float(p["radius"])
        exclude = None
        if p.get("node") is not None and p.get("point") is None:
            exclude = self.find_node(p["node"]).getId()
        found = []
        for node, entry, _ in self._spatial_catalog():
            if node.getId() == exclude or entry["position"] is None:
                continue
            d = math.dist(entry["position"], center)
            if d <= radius:
                e = dict(entry)
                e["distance"] = round(d, 4)
                found.append(e)
        found.sort(key=lambda e: e["distance"])
        return {"center": [round(v, 4) for v in center], "radius": radius,
                "count": len(found), "nodes": found}

    def cmd_objects_in_region(self, p):
        mn = [float(v) for v in p["min"]]
        mx = [float(v) for v in p["max"]]
        found = []
        for _, entry, _ in self._spatial_catalog():
            pos = entry["position"]
            if pos and all(mn[i] <= pos[i] <= mx[i] for i in range(3)):
                found.append(entry)
        return {"min": mn, "max": mx, "count": len(found), "nodes": found}

    def cmd_check_overlap(self, p):
        a = self.find_node(p["node_a"])
        b = self.find_node(p["node_b"])
        aabb_a, aabb_b = self._world_aabb(a), self._world_aabb(b)
        if aabb_a is None or aabb_b is None:
            return {"overlap": None,
                    "error": "AABB unavailable for one or both nodes"}
        return {"node_a": self._node_summary(a), "node_b": self._node_summary(b),
                "aabb_a": {"min": aabb_a[0], "max": aabb_a[1]},
                "aabb_b": {"min": aabb_b[0], "max": aabb_b[1]},
                "overlap": self._aabb_overlap(aabb_a, aabb_b),
                "note": "approximate axis-aligned bounding-box test"}

    def cmd_find_overlapping_pairs(self, p):
        rows = [(e, a) for _, e, a in self._spatial_catalog() if a is not None]
        pairs = []
        for i in range(len(rows)):
            for j in range(i + 1, len(rows)):
                if self._aabb_overlap(rows[i][1], rows[j][1]):
                    pairs.append({"a": rows[i][0]["id"], "a_name": rows[i][0]["name"] or rows[i][0]["def"],
                                  "b": rows[j][0]["id"], "b_name": rows[j][0]["name"] or rows[j][0]["def"]})
        return {"count": len(pairs), "pairs": pairs,
                "note": "approximate axis-aligned bounding-box test"}

    def cmd_get_spatial_relations(self, p):
        near_thresh = float(p.get("near_threshold", 0.5))
        rows = self._spatial_catalog()
        if p.get("node") is not None:
            target_id = self.find_node(p["node"]).getId()
            rows = [r for r in rows if r[0].getId() == target_id] + \
                   [r for r in rows if r[0].getId() != target_id]
            focus = {target_id}
        else:
            focus = None
        relations = []
        for i, (na, ea, aa) in enumerate(rows):
            if focus is not None and na.getId() not in focus:
                continue
            contacts = {c["other_node_id"] for c in self._contact_partners(na)}
            for nb, eb, ab in rows:
                if nb.getId() == na.getId():
                    continue
                rel = []
                if nb.getId() in contacts:
                    rel.append("touching")
                    if aa and ab and aa[0][2] > ab[0][2] + 1e-3:
                        rel.append("on_top_of")
                if aa and ab and self._aabb_contains(ab, aa):
                    rel.append("inside")
                if ea["position"] and eb["position"]:
                    d = math.dist(ea["position"], eb["position"])
                    if d <= near_thresh and not rel:
                        rel.append("near")
                    if rel:
                        relations.append({"a": ea["name"] or ea["def"] or ea["id"],
                                          "b": eb["name"] or eb["def"] or eb["id"],
                                          "relations": rel, "distance": round(d, 4)})
        return {"count": len(relations), "relations": relations,
                "near_threshold": near_thresh}

    @staticmethod
    def _aabb_contains(outer, inner):
        return all(outer[0][i] <= inner[0][i] and inner[1][i] <= outer[1][i]
                   for i in range(3))

    def cmd_reset_node_physics(self, p):
        node = self.find_node(p["node"])
        node.resetPhysics()
        return {"reset_physics": self._node_summary(node)}

    # ======================================================================
    # Commands: world building (P8) & map/diff (P7)
    # ======================================================================

    @staticmethod
    def _region(p_region):
        """Accept {'min':..,'max':..} or [min,max]."""
        if isinstance(p_region, dict):
            return (p_region["min"], p_region["max"])
        return (p_region[0], p_region[1])

    def _size_of(self, node):
        ext = self._node_extents(node)
        return [round(2 * e, 5) for e in ext] if ext else None

    def _apply_move(self, node, position=None, yaw=None, reset=True):
        if position is not None:
            f = node.getField("translation")
            if f:
                f.setSFVec3f([float(v) for v in position])
        if yaw is not None:
            f = node.getField("rotation")
            if f:
                f.setSFRotation([0.0, 0.0, 1.0, float(yaw)])
        if reset:
            try:
                node.resetPhysics()
            except Exception:  # noqa: BLE001
                pass

    def _spawn_string_at(self, node_string, position, yaw=None):
        field = self.sup.getRoot().getField("children")
        field.importMFNodeFromString(-1, node_string)
        node = field.getMFNode(field.getCount() - 1)
        self._apply_move(node, position=position, yaw=yaw, reset=False)
        return node

    def _spawn_many(self, node_string, positions, yaw=None):
        return [self._node_summary(self._spawn_string_at(node_string, pos, yaw))
                for pos in positions]

    def _occupied_aabbs(self, exclude_id=None):
        out = []
        for n in self._catalog_nodes():
            if exclude_id is not None and n.getId() == exclude_id:
                continue
            ab = self._world_aabb(n)
            if ab:
                out.append(ab)
        return out

    def _support_aabbs(self, region, up):
        """Occupied AABBs that could actually obstruct placement ON a support
        plane: drop anything lying entirely at/below the region floor (e.g. the
        ground/floor itself), which objects are meant to rest on, not avoid."""
        floor_lvl = region[0][up]
        return [a for a in self._occupied_aabbs() if a[1][up] > floor_lvl + 1e-4]

    def _catalog_data(self, max_depth=3, with_flags=False):
        rows = []
        for node in self._catalog_nodes(max_depth):
            e = self._catalog_entry(node)
            if with_flags:
                bo = node.getField("boundingObject")
                e["has_bounding"] = bool(bo and bo.getType() == Field.SF_NODE
                                         and bo.getSFNode())
                pf = node.getField("physics")
                e["has_physics"] = bool(pf and pf.getType() == Field.SF_NODE
                                        and pf.getSFNode())
            rows.append(e)
        return rows

    def cmd_drop_to_ground(self, p):
        node = self.find_node(p["node"])
        size = self._size_of(node)
        if size is None:
            raise ValueError("cannot determine object size (no boundingObject)")
        up = int(p.get("up", 2))
        pos = scene_math.drop_position(
            list(node.getPosition()), size, self._occupied_aabbs(node.getId()),
            floor=float(p.get("floor", 0.0)), up=up, gap=float(p.get("gap", 0.0)))
        self._apply_move(node, position=pos)
        return {"node": self._node_summary(node),
                "position": [round(v, 4) for v in pos]}

    def cmd_place_on(self, p):
        node = self.find_node(p["node"])
        target = self.find_node(p["target"])
        size = self._size_of(node)
        tab = self._world_aabb(target)
        if size is None or tab is None:
            raise ValueError("need bounding sizes for both node and target")
        pos = scene_math.place_on_position(
            size, tab, offset=p.get("offset") or [0.0, 0.0],
            up=int(p.get("up", 2)), gap=float(p.get("gap", 0.0)))
        self._apply_move(node, position=pos)
        return {"node": self._node_summary(node),
                "position": [round(v, 4) for v in pos]}

    def cmd_align_objects(self, p):
        nodes = [self.find_node(r) for r in p["nodes"]]
        items = [{"id": n.getId(), "position": list(n.getPosition())} for n in nodes]
        newpos = scene_math.align_positions(items, int(p["axis"]),
                                            p.get("mode", "center"))
        for n in nodes:
            self._apply_move(n, position=newpos[n.getId()])
        return {"aligned": {nid: [round(v, 4) for v in pos]
                            for nid, pos in newpos.items()}}

    def cmd_distribute_objects(self, p):
        nodes = [self.find_node(r) for r in p["nodes"]]
        items = [{"id": n.getId(), "position": list(n.getPosition())} for n in nodes]
        newpos = scene_math.distribute_positions(
            items, int(p["axis"]), spacing=p.get("spacing"), extent=p.get("extent"))
        for n in nodes:
            self._apply_move(n, position=newpos[n.getId()])
        return {"distributed": {nid: [round(v, 4) for v in pos]
                                for nid, pos in newpos.items()}}

    def cmd_place_row(self, p):
        positions = scene_math.row_positions(p["start"], p["step"], int(p["count"]))
        return {"spawned": self._spawn_many(p["node_string"], positions, p.get("yaw"))}

    def cmd_place_grid(self, p):
        positions = scene_math.grid_positions(
            p["start"], p["step_row"], p["step_col"], int(p["rows"]), int(p["cols"]))
        return {"spawned": self._spawn_many(p["node_string"], positions, p.get("yaw"))}

    def cmd_find_free_space(self, p):
        near = self._resolve_point(p["near"]) if p.get("near") is not None else None
        up = int(p.get("up", 2))
        region = self._region(p["region"])
        c = scene_math.find_free_space(
            p["size"], region, self._support_aabbs(region, up),
            up=up, near=near)
        return {"found": c is not None,
                "position": [round(v, 4) for v in c] if c else None}

    def cmd_scatter_objects(self, p):
        up = int(p.get("up", 2))
        region = self._region(p["region"])
        poses = scene_math.scatter_poses(
            int(p["count"]), p["size"], region,
            occupied=self._support_aabbs(region, up),
            min_spacing=float(p.get("min_spacing", 0.0)), seed=p.get("seed"),
            random_yaw=bool(p.get("random_yaw", True)), up=up)
        spawned = None
        if p.get("node_string"):
            spawned = []
            for pose in poses:
                if pose is None:
                    spawned.append(None)
                    continue
                n = self._spawn_string_at(p["node_string"], pose["position"], pose["yaw"])
                spawned.append(self._node_summary(n))
        return {"requested": int(p["count"]),
                "placed": sum(1 for x in poses if x),
                "poses": poses, "spawned": spawned}

    def cmd_validate_world(self, p):
        entries = self._catalog_data(with_flags=True)
        state = self.cmd_get_simulation_state({})
        arena = self._region(p["arena"]) if p.get("arena") else None
        issues = scene_math.validate_world(
            entries, floor=float(p.get("floor", 0.0)), up=int(p.get("up", 2)),
            coordinate_system=state.get("coordinate_system", "ENU"), arena=arena)
        by_sev = {}
        for i in issues:
            by_sev[i["severity"]] = by_sev.get(i["severity"], 0) + 1
        return {"issues": issues, "count": len(issues), "by_severity": by_sev}

    def _scene_snapshot(self):
        snap = {}
        for n in self._catalog_nodes():
            e = self._catalog_entry(n)
            snap[n.getId()] = {"name": e["name"] or e["def"], "position": e["position"],
                               "yaw": e["yaw"], "size": e["size"]}
        return snap

    def cmd_snapshot_scene(self, p):
        if not hasattr(self, "snapshots"):
            self.snapshots = {}
        name = str(p.get("name", "default"))
        self.snapshots[name] = self._scene_snapshot()
        return {"snapshot": name, "objects": len(self.snapshots[name])}

    def cmd_diff_scene(self, p):
        if not hasattr(self, "snapshots"):
            self.snapshots = {}
        a = self.snapshots.get(p["name_a"])
        if a is None:
            raise ValueError(f"no snapshot named {p['name_a']!r} (call snapshot_scene)")
        b_name = p.get("name_b", "now")
        b = self._scene_snapshot() if b_name == "now" else self.snapshots.get(b_name)
        if b is None:
            raise ValueError(f"no snapshot named {b_name!r}")
        return scene_math.diff_snapshots(a, b)

    def cmd_get_scene_map(self, p):
        entries = self._catalog_data(with_flags=False)
        svg = scene_math.svg_scene_map(
            entries, up=int(p.get("up", 2)), width=int(p.get("width", 640)),
            height=int(p.get("height", 640)))
        return {"format": "svg", "objects": len(entries), "svg": svg}

    # ======================================================================
    # Commands: run / understand loop (P9, P1.2)
    # ======================================================================

    def _build_world(self, names):
        """World snapshot for the condition DSL: positions, speeds, and
        pairwise contacts among the named nodes."""
        points = {}
        world_nodes = {}
        for name in names:
            node = self.find_node(name)
            pos = list(node.getPosition())
            try:
                vel = node.getVelocity()
                speed = math.sqrt(sum(v * v for v in vel[:3]))
            except Exception:  # noqa: BLE001
                speed = 0.0
            world_nodes[name] = {"position": [round(v, 5) for v in pos],
                                 "speed": round(speed, 5), "contacts": set()}
            try:
                self._ensure_contact_tracking(node)
                points[name] = [tuple(cp.point) for cp in node.getContactPoints(True)]
            except Exception:  # noqa: BLE001
                points[name] = []
        eps = 1e-4
        ns = list(points)
        for i, a in enumerate(ns):
            for b in ns[i + 1:]:
                if any(abs(pa[0] - pb[0]) < eps and abs(pa[1] - pb[1]) < eps
                       and abs(pa[2] - pb[2]) < eps
                       for pa in points[a] for pb in points[b]):
                    world_nodes[a]["contacts"].add(b)
                    world_nodes[b]["contacts"].add(a)
        return {"sim_time": round(self.sup.getTime(), 4), "nodes": world_nodes}

    def cmd_wait_until(self, p):
        cond = p["condition"]
        names = sorted(run_analysis.referenced_nodes(cond))
        timeout = float(p.get("timeout_s", 30.0))
        max_steps = max(1, int(timeout * 1000 / self.timestep))
        fired = False
        steps = 0
        world = self._build_world(names)
        for _ in range(max_steps):
            world = self._build_world(names)
            if run_analysis.evaluate_condition(cond, world):
                fired = True
                break
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            steps += 1
        return {"fired": fired, "sim_time": round(self.sup.getTime(), 4),
                "steps": steps, "condition": cond,
                "poses": {n: world["nodes"][n]["position"] for n in names},
                "reason": "condition met" if fired else "timeout"}

    def cmd_detect_anomalies(self, p):
        tr = self.tracking
        if tr is None:
            raise ValueError("tracking is not active; call start_tracking first")
        buffers = {e["name"]: list(e["buf"]) for e in tr["nodes"].values()}
        arena = self._region(p["arena"]) if p.get("arena") else None
        an = run_analysis.detect_anomalies(
            buffers, floor=float(p.get("floor", 0.0)), up=int(p.get("up", 2)),
            arena=arena, teleport_thresh=float(p.get("teleport_thresh", 1.0)),
            speed_thresh=float(p.get("speed_thresh", 50.0)))
        return {"anomalies": an, "count": len(an)}

    def cmd_run_experiment(self, p):
        duration = float(p.get("duration_s", 5.0))
        before = self._scene_snapshot()
        self.cmd_save_checkpoint({"name": "_experiment"})
        self.cmd_start_tracking({"nodes": p.get("watch"),
                                 "sample_every": int(p.get("sample_every", 2))})
        cond = p.get("until")
        names = sorted(run_analysis.referenced_nodes(cond)) if cond else []
        steps_total = max(1, int(duration * 1000 / self.timestep))
        fired = False
        i = 0
        while i < steps_total:
            if cond and run_analysis.evaluate_condition(cond, self._build_world(names)):
                fired = True
                break
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            i += 1
        tracking = self.cmd_get_tracking({"max_points": int(p.get("max_points", 30))})
        buffers = {e["name"]: list(e["buf"]) for e in self.tracking["nodes"].values()}
        arena = self._region(p["arena"]) if p.get("arena") else None
        anomalies = run_analysis.detect_anomalies(
            buffers, floor=float(p.get("floor", 0.0)), up=int(p.get("up", 2)),
            arena=arena)
        after = self._scene_snapshot()
        diff = scene_math.diff_snapshots(before, after)
        self.tracking = None
        shot = None
        if p.get("screenshot"):
            try:
                shot = self.cmd_screenshot({"quality": int(p.get("quality", 85))}).get("base64")
            except Exception:  # noqa: BLE001
                shot = None
        restore = p.get("restore", "auto")
        restored = False
        if restore == "keep":
            pass
        elif restore == "on_anomaly":
            if anomalies:
                self.cmd_restore_checkpoint({"name": "_experiment"})
                restored = True
        else:
            self.cmd_restore_checkpoint({"name": "_experiment"})
            restored = True
        return {"duration_s": duration, "fired": fired,
                "sim_time": round(self.sup.getTime(), 4),
                "objects": tracking["objects"],
                "interactions": tracking["interactions"],
                "diff": diff["summary"], "diff_detail": diff,
                "anomalies": anomalies, "anomaly_count": len(anomalies),
                "restored": restored, "restore_mode": restore,
                "screenshot_base64": shot}

    # ======================================================================
    # Commands: authoring / script export (P8.4/8.5/8.7, P9.4, P10.1)
    # ======================================================================

    def _first_shape(self, node):
        for n in self._iter_nodes(node, max_depth=5):
            if n.getBaseTypeName() == "Shape":
                return n
        return None

    def cmd_generate_world_script(self, p):
        fmt = p.get("format", "python")
        strings, objects = [], []
        for n in self._catalog_nodes():
            try:
                s = n.exportString()
            except Exception:  # noqa: BLE001
                continue
            strings.append(s)
            e = self._catalog_entry(n)
            objects.append({"name": e["name"], "def": e["def"],
                            "position": e["position"], "node_string": s})
        if fmt == "python":
            return {"format": "python",
                    "script": authoring.python_script_from_nodes(strings),
                    "objects": len(strings)}
        if fmt == "json":
            state = self.cmd_get_simulation_state({})
            phys = {"coordinate_system": state.get("coordinate_system"),
                    "gravity": state.get("gravity"),
                    "basic_time_step": self.timestep}
            return {"format": "json",
                    "scenario": authoring.json_scenario_from_objects(objects, physics=phys),
                    "objects": len(objects)}
        raise ValueError("format must be 'python' or 'json' (use save_world for 'wbt')")

    def cmd_extract_proto_from_node(self, p):
        node = self.find_node(p["node"])
        s = node.exportString()
        expose = tuple(p.get("expose") or ["translation", "rotation", "name"])
        proto = authoring.wrap_proto(s, p["proto_name"], expose=expose)
        return {"proto_name": p["proto_name"], "proto_text": proto}

    def cmd_set_appearance(self, p):
        node = self.find_node(p["node"])
        shape = self._first_shape(node)
        if shape is None:
            raise ValueError("node has no Shape to style")
        appf = shape.getField("appearance")
        app = appf.getSFNode() if appf and appf.getType() == Field.SF_NODE else None
        if p.get("base_color") is not None:
            color = [float(v) for v in p["base_color"]]
            if app is not None and app.getField("baseColor"):
                app.getField("baseColor").setSFColor(color)
            elif app is not None and app.getField("material") \
                    and app.getField("material").getSFNode():
                app.getField("material").getSFNode().getField("diffuseColor").setSFColor(color)
            else:
                raise ValueError("shape has no PBRAppearance/Appearance to recolor; "
                                 "respawn it with an appearance node")
        if p.get("roughness") is not None and app and app.getField("roughness"):
            app.getField("roughness").setSFFloat(float(p["roughness"]))
        if p.get("metalness") is not None and app and app.getField("metalness"):
            app.getField("metalness").setSFFloat(float(p["metalness"]))
        return {"styled": self._node_summary(shape)}

    def cmd_set_recognition_colors(self, p):
        node = self.find_node(p["node"])
        f = node.getField("recognitionColors")
        if f is None or f.getType() != Field.MF_COLOR:
            raise ValueError("node has no recognitionColors field (not a Solid?)")
        while f.getCount():
            f.removeMF(0)
        for c in p["colors"]:
            f.insertMFColor(-1, [float(v) for v in c])
        return {"node": self._node_summary(node), "colors": f.getCount()}

    def cmd_configure_lighting(self, p):
        preset = authoring.lighting_preset(p["preset"])
        intensity = p.get("intensity")
        changed = {}
        for _, n in self._child_nodes(self.sup.getRoot()):
            if n.getBaseTypeName() in ("Background",) and n.getField("skyColor"):
                sk = n.getField("skyColor")
                if sk.getCount():
                    sk.setMFColor(0, preset["sky"])
                else:
                    sk.insertMFColor(-1, preset["sky"])
                changed["background"] = preset["sky"]
        root = self.sup.getRoot().getField("children")
        added = 0
        for spec in preset["lights"]:
            root.importMFNodeFromString(-1, authoring._light_node(spec, intensity))
            added += 1
        changed["lights_added"] = added
        return {"preset": p["preset"], "changed": changed}

    def cmd_record_states(self, p):
        nodes = ([self.find_node(r) for r in p["nodes"]] if p.get("nodes")
                 else self._catalog_nodes())
        fields = p.get("fields") or ["position"]
        interval = int(p.get("interval_ms", self.timestep))
        duration = float(p.get("duration_s", 3.0))
        every = max(1, round(interval / self.timestep))
        total = max(1, int(duration * 1000 / self.timestep))
        names = {n.getId(): (self._node_summary(n).get("name")
                             or self._node_summary(n).get("def") or str(n.getId()))
                 for n in nodes}
        rows = []
        step = 0
        while step < total:
            if step % every == 0:
                t = round(self.sup.getTime(), 4)
                for n in nodes:
                    row = {"sim_time": t, "node": names[n.getId()]}
                    if "position" in fields:
                        row["position"] = [round(v, 5) for v in n.getPosition()]
                    try:
                        vel = n.getVelocity()
                    except Exception:  # noqa: BLE001
                        vel = None
                    if "velocity" in fields:
                        row["velocity"] = [round(v, 5) for v in vel] if vel else None
                    if "speed" in fields:
                        row["speed"] = (round(math.sqrt(sum(x * x for x in vel[:3])), 5)
                                        if vel else None)
                    rows.append(row)
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            step += 1
        return {"rows": rows, "count": len(rows), "fields": fields,
                "nodes": list(names.values())}

    # ======================================================================
    # Commands: physics config / profiling / scenarios (P5.2/8.8, 9.5, 10.3)
    # ======================================================================

    def cmd_configure_physics(self, p):
        wi = self._world_info()
        if wi is None:
            raise ValueError("no WorldInfo node in this world")
        # drop None values so recipe defaults aren't blocked by unset params
        settings = {k: v for k, v in p.items() if k != "recipe" and v is not None}
        if p.get("recipe"):
            recipe = experiments.physics_recipe(p["recipe"])
            for k, v in recipe.items():
                settings.setdefault(k, v)
        applied = {}
        g = settings.get("gravity")
        if g is not None:
            f = wi.getField("gravity")
            if f and f.getType() == Field.SF_VEC3F:
                vec = ([float(v) for v in g] if isinstance(g, (list, tuple))
                       else [0.0, 0.0, -abs(float(g))])
                f.setSFVec3f(vec)
                applied["gravity"] = vec
            elif f and f.getType() == Field.SF_FLOAT:
                # R2025a WorldInfo.gravity is a scalar magnitude; direction follows
                # the coordinate system. Accept a vector (use its magnitude) or scalar.
                mag = (sum(float(v) ** 2 for v in g) ** 0.5
                       if isinstance(g, (list, tuple)) else abs(float(g)))
                f.setSFFloat(mag)
                applied["gravity"] = mag
        for key, field_name, cast in (
                ("basic_time_step", "basicTimeStep", float),
                ("fps", "fps", float),
                ("random_seed", "randomSeed", int),
                ("optimal_thread_count", "optimalThreadCount", int)):
            if settings.get(key) is not None:
                f = wi.getField(field_name)
                if f:
                    val = cast(settings[key])
                    (f.setSFFloat if cast is float else f.setSFInt32)(val)
                    applied[key] = val
        cs = wi.getField("coordinateSystem")
        return {"applied": applied,
                "coordinate_system": cs.getSFString() if cs else "ENU",
                "note": "basicTimeStep/thread changes may need a world reset to "
                        "fully take effect; set random_seed then reset for "
                        "reproducible physics"}

    def cmd_profile_simulation(self, p):
        duration = float(p.get("duration_s", 3.0))
        steps = max(1, int(duration * 1000 / self.timestep))
        sim0 = self.sup.getTime()
        wall0 = time.time()
        done = 0
        for _ in range(steps):
            if self.sup.step(self.timestep) == -1:
                break
            self._sample_tracking()
            done += 1
        wall = time.time() - wall0
        sim = self.sup.getTime() - sim0
        rtf = (sim / wall) if wall > 0 else None
        return {"sim_time_s": round(sim, 4), "wall_time_s": round(wall, 4),
                "real_time_factor": round(rtf, 4) if rtf else None,
                "steps": done,
                "note": "achieved sim/wall ratio; < 1 means slower than real time "
                        "(usually mesh bounding objects or a tiny timestep)"}

    def cmd_build_scenario(self, p):
        spawned = []
        for op in p["ops"]:
            n = self._spawn_string_at(op["node_string"], op["position"],
                                      op.get("yaw"))
            spawned.append(self._node_summary(n))
        return {"spawned": spawned, "count": len(spawned)}

    def cmd_get_viewport_labels(self, p):
        vp = self._get_viewpoint()
        pos = list(vp.getField("position").getSFVec3f())
        rot = list(vp.getField("orientation").getSFRotation())
        fov_f = vp.getField("fieldOfView")
        fov = fov_f.getSFFloat() if fov_f else 0.785
        m = perception.axis_angle_to_matrix(rot[:3], rot[3])
        fwd, up = perception.camera_basis(m)
        w = int(p.get("width", 640))
        h = int(p.get("height", 480))
        objs = [{"name": e["name"] or e["def"] or str(e["id"]),
                 "position": e["position"]} for e in self._catalog_data()]
        labels = perception.viewport_labels(objs, pos, fwd, up, fov, w, h)
        return {"labels": labels, "count": len(labels), "width": w, "height": h,
                "camera": {"position": [round(v, 4) for v in pos],
                           "fov": round(fov, 4)},
                "note": "occlusion-unaware: labels are for objects in front and "
                        "inside the frame, sorted near to far"}

    # ======================================================================
    # Commands: general / simulation
    # ======================================================================

    def cmd_ping(self, p):
        return {"pong": True, "time": self.sup.getTime()}

    def _world_info(self):
        """The WorldInfo node, or None."""
        for _, n in self._child_nodes(self.sup.getRoot()):
            if n.getBaseTypeName() == "WorldInfo":
                return n
        return None

    def cmd_get_simulation_state(self, p):
        state = {
            "time": self.sup.getTime(),
            "mode": self.logical_mode,
            "basic_time_step": self.timestep,
            "world": self.sup.getWorldPath(),
            "webots_version": os.environ.get("WEBOTS_VERSION", "unknown"),
            "registered_agents": list(self.agents.keys()),
        }
        wi = self._world_info()
        if wi is not None:
            cs = wi.getField("coordinateSystem")
            state["coordinate_system"] = cs.getSFString() if cs else "ENU"
            g = wi.getField("gravity")
            if g and g.getType() == Field.SF_VEC3F:
                state["gravity"] = [round(v, 4) for v in g.getSFVec3f()]
            rs = wi.getField("randomSeed")
            if rs:
                state["random_seed"] = rs.getSFInt32()
        return state

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
        try:
            info["velocity"] = [round(v, 4) for v in node.getVelocity()]
        except Exception:  # noqa: BLE001 - needs physics
            pass
        try:
            self._ensure_contact_tracking(node)
            info["contact_count"] = len(node.getContactPoints(True))
        except Exception:  # noqa: BLE001
            pass
        try:
            info["center_of_mass"] = [round(v, 4) for v in node.getCenterOfMass()]
        except Exception:  # noqa: BLE001 - needs physics
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
