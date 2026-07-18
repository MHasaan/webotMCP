"""Unit tests for the bridge's pure run-analysis helpers (no Webots needed)."""

import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import run_analysis as ra  # noqa: E402


def _world(t, nodes):
    return {"sim_time": t, "nodes": nodes}


def test_sim_time_and_distance():
    w = _world(5.0, {"A": {"position": [0, 0, 0], "speed": 0, "contacts": set()},
                     "B": {"position": [0.2, 0, 0], "speed": 0, "contacts": set()}})
    assert ra.evaluate_condition({"type": "sim_time", "op": ">", "value": 4}, w)
    assert not ra.evaluate_condition({"type": "sim_time", "op": ">", "value": 6}, w)
    assert ra.evaluate_condition({"type": "distance", "a": "A", "b": "B",
                                  "op": "<", "value": 0.3}, w)
    assert not ra.evaluate_condition({"type": "distance", "a": "A", "b": "B",
                                      "op": "<", "value": 0.1}, w)


def test_speed_contact_position_combinators():
    w = _world(1.0, {"BALL": {"position": [0, 0, 0.4], "speed": 0.005,
                              "contacts": {"FLOOR"}}})
    assert ra.evaluate_condition({"type": "speed", "node": "BALL", "op": "<",
                                  "value": 0.01}, w)
    assert ra.evaluate_condition({"type": "contact", "a": "BALL", "b": "FLOOR"}, w)
    assert not ra.evaluate_condition({"type": "contact", "a": "BALL", "b": "WALL"}, w)
    assert ra.evaluate_condition({"type": "position", "node": "BALL", "axis": 2,
                                  "op": "<", "value": 0.5}, w)
    both = {"type": "all", "conditions": [
        {"type": "speed", "node": "BALL", "op": "<", "value": 0.01},
        {"type": "contact", "a": "BALL", "b": "FLOOR"}]}
    assert ra.evaluate_condition(both, w)
    either = {"type": "any", "conditions": [
        {"type": "speed", "node": "BALL", "op": ">", "value": 100},
        {"type": "sim_time", "op": ">", "value": 0.5}]}
    assert ra.evaluate_condition(either, w)


def test_referenced_nodes():
    cond = {"type": "all", "conditions": [
        {"type": "distance", "a": "A", "b": "B", "op": "<", "value": 1},
        {"type": "speed", "node": "C", "op": "<", "value": 1}]}
    assert ra.referenced_nodes(cond) == {"A", "B", "C"}


def test_unknown_node_and_type():
    w = _world(0, {})
    for bad in ({"type": "speed", "node": "X", "op": "<", "value": 1},
                {"type": "bogus"}):
        try:
            ra.evaluate_condition(bad, w)
            assert False, bad
        except (ValueError, KeyError):
            pass


def test_detect_anomalies():
    buffers = {
        "ball": [(0.0, [0, 0, 1.0], 0.1), (0.1, [0, 0, 0.5], 0.2),
                 (0.2, [0, 0, -0.3], 0.3)],
        "rocket": [(0.0, [0, 0, 0.2], 0.1), (0.1, [0, 0, 0.2], 99.0)],
        "glitch": [(0.0, [0, 0, 0.2], 0.1), (0.1, [5, 0, 0.2], 0.1)],
        "broken": [(0.0, [float("nan"), 0, 0], 0.1)],
    }
    an = ra.detect_anomalies(buffers, floor=0.0, teleport_thresh=1.0, speed_thresh=50.0)
    types = {a["type"] for a in an}
    assert {"below_floor", "runaway_velocity", "teleport", "nan_or_inf"} <= types
    an2 = ra.detect_anomalies({"x": [(0.0, [10, 0, 0.2], 0.1)]},
                              arena=([-1, -1, -1], [1, 1, 1]))
    assert any(a["type"] == "outside_arena" for a in an2)


def test_classify_console_and_split():
    lines = [
        "[my_robot] starting up",
        "WARNING: 'foo' is not a valid field of node Bar",
        "Traceback (most recent call last):",
        "ERROR: could not open file 'missing.png'",
        "ODE Message 3: LCP internal error",
        "[my_robot] step 5",
    ]
    c = ra.classify_console(lines)
    cats = {i["category"] for i in c["issues"]}
    assert {"controller_crash", "missing_asset", "physics_ode", "parse_warning"} <= cats
    split = ra.split_controller_logs(lines)
    assert "my_robot" in split and len(split["my_robot"]) == 2 and "_webots" in split
