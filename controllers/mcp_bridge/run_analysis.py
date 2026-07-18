"""Pure analysis helpers for the run/understand loop (P9, P1.2).

No Webots (`controller`) imports, so it is unit-testable standalone. The bridge
builds plain-data snapshots (world state dicts, trajectory buffers) and delegates
condition evaluation and anomaly detection here; console classification is used by
the server (which owns the captured log).
"""

import math
import re

_OPS = {
    "<": lambda a, b: a < b,
    "<=": lambda a, b: a <= b,
    ">": lambda a, b: a > b,
    ">=": lambda a, b: a >= b,
    "==": lambda a, b: a == b,
    "!=": lambda a, b: a != b,
}


# ---------------------------------------------------------------------------
# wait_until condition DSL (1.2)
# ---------------------------------------------------------------------------

def referenced_nodes(cond):
    """Collect every node name a condition (tree) refers to — so the caller knows
    which nodes to snapshot each step."""
    out = set()
    t = cond.get("type")
    if t in ("any", "all"):
        for c in cond.get("conditions", []):
            out |= referenced_nodes(c)
        return out
    for key in ("a", "b", "node"):
        v = cond.get(key)
        if isinstance(v, str):
            out.add(v)
    return out


def evaluate_condition(cond, world):
    """Evaluate a condition against a world snapshot:
    ``{"sim_time": float, "nodes": {name: {"position":[x,y,z], "speed":float,
    "contacts": set(names)}}}``. Returns bool. Raises ValueError on malformed
    conditions or unknown nodes.

    Types: sim_time, distance, speed, position, contact, any, all.
    """
    t = cond.get("type")
    if t == "any":
        return any(evaluate_condition(c, world) for c in cond["conditions"])
    if t == "all":
        return all(evaluate_condition(c, world) for c in cond["conditions"])

    nodes = world.get("nodes", {})

    def pos(name):
        if name not in nodes:
            raise ValueError(f"condition references unknown node {name!r}")
        return nodes[name]["position"]

    def op(default=">="):
        o = cond.get("op", default)
        if o not in _OPS:
            raise ValueError(f"unknown operator {o!r}")
        return _OPS[o]

    if t == "sim_time":
        return op(">=")(world["sim_time"], float(cond["value"]))
    if t == "distance":
        d = math.dist(pos(cond["a"]), pos(cond["b"]))
        return op("<")(d, float(cond["value"]))
    if t == "speed":
        return op("<")(nodes[cond["node"]]["speed"], float(cond["value"]))
    if t == "position":
        axis = int(cond.get("axis", 2))
        return op(">=")(pos(cond["node"])[axis], float(cond["value"]))
    if t == "contact":
        a = cond["a"]
        b = cond.get("b")
        contacts = nodes.get(a, {}).get("contacts", set())
        if b is None:
            return len(contacts) > 0
        return b in contacts
    raise ValueError(f"unknown condition type {t!r}")


# ---------------------------------------------------------------------------
# Anomaly detection (9.2)
# ---------------------------------------------------------------------------

def _finite(vec):
    return all(math.isfinite(v) for v in vec)


def detect_anomalies(buffers, floor=0.0, up=2, arena=None,
                     teleport_thresh=1.0, speed_thresh=50.0, below_tol=0.05):
    """Scan trajectory buffers for physics trouble. ``buffers`` = ``{name:
    [(t, [x,y,z], speed), ...]}``. Returns a list of
    ``{type, node, t, detail, hint}`` sorted by time."""
    out = []

    def add(kind, node, t, detail, hint):
        out.append({"type": kind, "node": node, "t": t, "detail": detail, "hint": hint})

    for name, buf in buffers.items():
        prev = None
        for t, p, speed in buf:
            if not _finite(p) or not math.isfinite(speed):
                add("nan_or_inf", name, t, "non-finite position/speed",
                    "physics blow-up: reduce basicTimeStep or check boundingObject")
                prev = None
                continue
            if speed > speed_thresh:
                add("runaway_velocity", name, t, f"speed {round(speed, 2)} m/s",
                    "add damping, reduce basicTimeStep, or check for interpenetration")
            if p[up] < floor - below_tol:
                add("below_floor", name, t,
                    f"{round(floor - p[up], 3)} m below floor",
                    "object fell through: add/enlarge floor boundingObject or a wall")
            if arena is not None and not all(
                    arena[0][i] <= p[i] <= arena[1][i] for i in range(3)):
                add("outside_arena", name, t, f"position {p}",
                    "object left the arena region")
            if prev is not None:
                step_d = math.dist(prev, p)
                if step_d > teleport_thresh:
                    add("teleport", name, t,
                        f"moved {round(step_d, 3)} m in one sample",
                        "large jump: unstable contact or a supervisor teleport")
            prev = p
    out.sort(key=lambda a: a["t"])
    return out


# ---------------------------------------------------------------------------
# Console diagnostics (9.3)
# ---------------------------------------------------------------------------

_CONSOLE_PATTERNS = [
    ("physics_ode", re.compile(r"\bODE\b|ode_error|LCP|contact joint", re.I),
     "physics solver strain: reduce basicTimeStep, add damping, or fix "
     "overlapping/oversized boundingObjects"),
    ("controller_crash",
     re.compile(r"Traceback \(most recent call last\)|Exception|"
                r"exited with status [1-9]|segmentation fault", re.I),
     "a controller crashed: check its code / the traceback below"),
    ("missing_asset",
     re.compile(r"could not (open|find)|no such file|failed to load|"
                r"unknown proto|missing", re.I),
     "missing asset: fix the url/path or run clear_webots_cache"),
    ("parse_warning", re.compile(r"WARNING.*(field|node|expected|deprecated)", re.I),
     "world/PROTO parse issue: check the reported field/node"),
    ("generic_error", re.compile(r"\bERROR\b|\bCRITICAL\b", re.I),
     "see message"),
    ("generic_warning", re.compile(r"\bWARNING\b", re.I),
     "see message"),
]

_CONTROLLER_PREFIX = re.compile(r"^\[([^\]]+)\]\s?(.*)$")


def classify_console(lines):
    """Classify Webots console lines into categories with suggested fixes.
    Returns ``{"issues": [{category, hint, count, examples}], "totals": {...}}``."""
    buckets = {}
    for line in lines:
        for cat, pat, hint in _CONSOLE_PATTERNS:
            if pat.search(line):
                b = buckets.setdefault(cat, {"category": cat, "hint": hint,
                                             "count": 0, "examples": []})
                b["count"] += 1
                if len(b["examples"]) < 3:
                    b["examples"].append(line.strip()[:200])
                break
    issues = sorted(buckets.values(), key=lambda b: -b["count"])
    totals = {c: b["count"] for c, b in buckets.items()}
    return {"issues": issues, "totals": totals}


def split_controller_logs(lines):
    """Group console lines by their ``[controller_name]`` prefix. Lines without a
    prefix go under ``"_webots"``. Returns ``{name: [lines]}``."""
    out = {}
    for line in lines:
        m = _CONTROLLER_PREFIX.match(line.strip())
        if m:
            out.setdefault(m.group(1), []).append(m.group(2))
        else:
            out.setdefault("_webots", []).append(line.strip())
    return out
