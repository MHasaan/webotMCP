"""Unit tests for the bridge's pure geometry/placement math (no Webots needed)."""

import math
import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import scene_math as sm  # noqa: E402


def test_aabb_basics():
    a = sm.aabb_from_center_size([0, 0, 0], [2, 2, 2])
    assert a == ([-1, -1, -1], [1, 1, 1])
    assert sm.center_of(a) == [0, 0, 0]
    assert sm.size_of(a) == [2, 2, 2]
    b = sm.aabb_from_center_size([1.5, 0, 0], [2, 2, 2])
    assert sm.aabb_overlap(a, b)
    c = sm.aabb_from_center_size([5, 0, 0], [2, 2, 2])
    assert not sm.aabb_overlap(a, c)
    assert sm.aabb_contains(([-2, -2, -2], [2, 2, 2]), a)


def test_place_on():
    target = sm.aabb_from_center_size([1, 2, 0.5], [2, 2, 1])
    pos = sm.place_on_position([0.4, 0.4, 0.4], target, offset=(0.1, -0.1))
    assert pos[0] == 1.1 and pos[1] == 1.9
    assert abs(pos[2] - 1.2) < 1e-9


def test_drop_to_support_and_floor():
    table = sm.aabb_from_center_size([0, 0, 0.5], [2, 2, 1])
    pos = sm.drop_position([0, 0, 5], [0.2, 0.2, 0.4], [table], floor=0.0)
    assert abs(pos[2] - 1.2) < 1e-9
    pos2 = sm.drop_position([9, 9, 5], [0.2, 0.2, 0.4], [table], floor=0.0)
    assert abs(pos2[2] - 0.2) < 1e-9


def test_align_distribute():
    items = [{"id": 1, "position": [0, 0, 0]},
             {"id": 2, "position": [2, 5, 0]},
             {"id": 3, "position": [4, 10, 0]}]
    aligned = sm.align_positions(items, axis=1, mode="center")
    assert all(abs(p[1] - 5.0) < 1e-9 for p in aligned.values())
    dist = sm.distribute_positions(items, axis=0, spacing=1.0)
    assert dist[1][0] == 0 and dist[2][0] == 1 and dist[3][0] == 2


def test_row_grid():
    assert sm.row_positions([0, 0, 0], [1, 0, 0], 3) == [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    g = sm.grid_positions([0, 0, 0], [0, 1, 0], [1, 0, 0], 2, 2)
    assert len(g) == 4 and g[3] == [1, 1, 0]


def test_find_free_space():
    region = ([0, 0, 0], [10, 10, 2])
    occ = [sm.aabb_from_center_size([1, 1, 0.5], [2, 2, 1])]
    c = sm.find_free_space([1, 1, 1], region, occ, near=[1, 1, 0])
    assert c is not None
    assert not sm.aabb_overlap(sm.aabb_from_center_size(c, [1, 1, 1]), occ[0])
    assert sm.find_free_space([20, 20, 1], region, []) is None


def test_scatter_deterministic_no_overlap():
    region = ([0, 0, 0], [10, 10, 2])
    p1 = sm.scatter_poses(8, [0.5, 0.5, 0.5], region, min_spacing=1.0, seed=42)
    p2 = sm.scatter_poses(8, [0.5, 0.5, 0.5], region, min_spacing=1.0, seed=42)
    assert p1 == p2
    placed = [p for p in p1 if p]
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            assert math.dist(placed[i]["position"][:2], placed[j]["position"][:2]) >= 1.0 - 1e-6


def test_validate_world():
    entries = [
        {"id": 1, "name": "floor", "def": None, "position": [0, 0, -0.01],
         "size": [10, 10, 0.02], "kind": "static", "has_physics": False,
         "has_bounding": True, "base_type": "Solid"},
        {"id": 2, "name": "ball", "def": "B", "position": [0, 0, 3],
         "size": [0.4, 0.4, 0.4], "kind": "dynamic", "has_physics": True,
         "has_bounding": True, "base_type": "Solid"},
        {"id": 3, "name": "sunk", "def": "B", "position": [2, 2, -1],
         "size": [0.4, 0.4, 0.4], "kind": "static", "has_physics": False,
         "has_bounding": True, "base_type": "Solid"},
        {"id": 4, "name": "nobound", "def": None, "position": [5, 5, 0.2],
         "size": [0.4, 0.4, 0.4], "kind": "dynamic", "has_physics": True,
         "has_bounding": False, "base_type": "Solid"},
    ]
    issues = sm.validate_world(entries, floor=0.0, coordinate_system="ENU")
    codes = {i["code"] for i in issues}
    assert {"floating", "below_floor", "missing_bounding", "duplicate_def"} <= codes
    assert issues[0]["severity"] == "error"


def test_diff_snapshots():
    a = {1: {"name": "a", "position": [0, 0, 0], "yaw": 0.0},
         2: {"name": "b", "position": [1, 1, 1], "yaw": 0.0}}
    b = {2: {"name": "b", "position": [1, 1, 2], "yaw": 1.0},
         3: {"name": "c", "position": [5, 5, 5], "yaw": 0.0}}
    d = sm.diff_snapshots(a, b)
    assert d["summary"] == {"added": 1, "removed": 1, "moved": 1, "rotated": 1}
    assert d["added"][0]["id"] == 3 and d["removed"][0]["id"] == 1
    assert d["moved"][0]["id"] == 2 and abs(d["moved"][0]["displacement"] - 1.0) < 1e-6


def test_svg_scene_map():
    entries = [{"id": 1, "name": "box", "position": [0, 0, 0.5],
                "size": [1, 1, 1], "color": [1, 0, 0]}]
    svg = sm.svg_scene_map(entries)
    assert svg.startswith("<svg") and "box" in svg and "rgb(255,0,0)" in svg
    assert sm.svg_scene_map([]).startswith("<svg")
