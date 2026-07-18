"""Unit tests for the bridge's pure robot-behavior math (no Webots needed)."""

import math
import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import kinematics as kn  # noqa: E402


def test_diff_drive_straight_and_turn():
    l, r = kn.diff_drive_wheel_speeds(0.2, 0.0, wheel_radius=0.05, track_width=0.3)
    assert abs(l - r) < 1e-9 and abs(l - 4.0) < 1e-9
    l, r = kn.diff_drive_wheel_speeds(0.0, 1.0, 0.05, 0.3)
    assert l < 0 < r and abs(l + r) < 1e-9


def test_clamp_wheel_speeds_preserves_ratio():
    l, r = kn.clamp_wheel_speeds(10, 5, max_speed=4)
    assert abs(max(abs(l), abs(r)) - 4) < 1e-9 and abs((l / r) - 2.0) < 1e-9


def test_heading_control_done_and_turn():
    lin, ang, done = kn.heading_control([0, 0], 0.0, [0.05, 0], tolerance=0.1)
    assert done and lin == 0 and ang == 0
    lin, ang, done = kn.heading_control([0, 0], 0.0, [0, 5], tolerance=0.1)
    assert not done and ang > 0 and lin < 1e-9
    lin, ang, done = kn.heading_control([0, 0], 0.0, [5, 0], tolerance=0.1)
    assert lin > 0 and abs(ang) < 1e-6


def test_wrap_and_yaw():
    assert abs(kn.wrap_angle(math.pi * 1.5) - (-math.pi / 2)) < 1e-9
    m = (0, -1, 0, 1, 0, 0, 0, 0, 1)
    assert abs(kn.yaw_from_matrix(m) - math.pi / 2) < 1e-9


def test_pick_wheel_motors():
    devs = [{"name": "left wheel motor", "type": "RotationalMotor"},
            {"name": "right wheel motor", "type": "RotationalMotor"},
            {"name": "camera", "type": "Camera"}]
    assert kn.pick_wheel_motors(devs) == ("left wheel motor", "right wheel motor")
    two = [{"name": "m1", "type": "RotationalMotor"},
           {"name": "m2", "type": "RotationalMotor"}]
    assert kn.pick_wheel_motors(two) == ("m1", "m2")
    assert kn.pick_wheel_motors([{"name": "arm", "type": "RotationalMotor"},
                                 {"name": "b", "type": "RotationalMotor"},
                                 {"name": "c", "type": "RotationalMotor"}]) == (None, None)


def test_occupancy_grid():
    g = kn.occupancy_grid([[0.0, 0.0], [1.0, 1.0]], resolution=0.5, size=4.0)
    assert g["cells"] == 8 and len(g["grid"]) == 8
    assert any("#" in row for row in g["grid"]) and g["obstacles"]
    empty = kn.occupancy_grid([], resolution=0.5, size=2.0)
    assert all(set(row) == {"."} for row in empty["grid"])
