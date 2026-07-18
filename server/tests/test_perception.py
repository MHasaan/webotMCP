"""Unit tests for the bridge's pure perception/projection helpers (no Webots)."""

import math
import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import perception as pc  # noqa: E402


def test_axis_angle_identity_and_z90():
    assert pc.axis_angle_to_matrix([0, 0, 1], 0) == (1, 0, 0, 0, 1, 0, 0, 0, 1)
    m = pc.axis_angle_to_matrix([0, 0, 1], math.pi / 2)
    v = pc._matvec(m, [1, 0, 0])
    assert abs(v[0]) < 1e-9 and abs(v[1] - 1) < 1e-9


def test_project_center_and_behind():
    r = pc.project_point([0, 0, -5], [0, 0, 0], [0, 0, -1], [0, 1, 0],
                         math.radians(60), 640, 480)
    assert r["visible"] and abs(r["px"] - 320) < 1e-6 and abs(r["py"] - 240) < 1e-6
    assert abs(r["depth"] - 5) < 1e-6
    b = pc.project_point([0, 0, 5], [0, 0, 0], [0, 0, -1], [0, 1, 0],
                         math.radians(60), 640, 480)
    assert not b["visible"]


def test_project_offset_right_and_up():
    r = pc.project_point([1, 1, -5], [0, 0, 0], [0, 0, -1], [0, 1, 0],
                         math.radians(60), 640, 480)
    assert r["px"] > 320 and r["py"] < 240


def test_viewport_labels_sorted_and_filtered():
    objs = [{"name": "near", "position": [0, 0, -2]},
            {"name": "far", "position": [0, 0, -8]},
            {"name": "behind", "position": [0, 0, 3]},
            {"name": "offscreen", "position": [100, 0, -2]}]
    labels = pc.viewport_labels(objs, [0, 0, 0], [0, 0, -1], [0, 1, 0],
                                math.radians(60), 640, 480)
    assert [l["name"] for l in labels] == ["near", "far"]


def test_parse_world_header():
    wbt = ('#VRML_SIM R2025a utf8\n'
           'WorldInfo { title "Demo" info [ "a demo" ] basicTimeStep 16 '
           'coordinateSystem "NUE" }\n'
           'Viewpoint { }\n'
           'Robot { name "bot" controller "my_ctrl" }\n'
           'E-puck { controller "<extern>" }\n')
    h = pc.parse_world_header(wbt)
    assert h["webots_version"] == "R2025a" and h["title"] == "Demo"
    assert h["basic_time_step"] == 16 and h["coordinate_system"] == "NUE"
    assert h["robot_count"] == 2
    ctrls = {r["controller"] for r in h["robots"]}
    assert "my_ctrl" in ctrls and "<extern>" in ctrls
