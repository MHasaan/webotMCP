"""Unit tests for the bridge's pure experiment helpers (no Webots needed)."""

import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import experiments as ex  # noqa: E402


def test_physics_recipe():
    assert ex.physics_recipe("moon")["gravity"][2] == -1.62
    assert ex.physics_recipe("slow_motion")["basic_time_step"] == 8
    try:
        ex.physics_recipe("jupiter")
        assert False
    except ValueError:
        pass


def test_validate_scenario():
    good = {"objects": [{"node_string": "Sphere {}", "position": [0, 0, 1]}],
            "duration_s": 5}
    assert ex.validate_scenario(good) == []
    bad = {"objects": [{"position": [0, 0, 1]}, {"node_string": "X"}],
           "duration_s": -1}
    errs = ex.validate_scenario(bad)
    assert any("node_string" in e for e in errs)
    assert any("position" in e or "scatter" in e for e in errs)
    assert any("duration_s" in e for e in errs)
    assert ex.validate_scenario({})


def test_scenario_build_plan_with_scatter():
    scn = {"objects": [
        {"node_string": "DEF A Solid {}", "position": [1, 2, 0.5], "yaw": 0.3},
        {"node_string": "Ball {}", "scatter": {"count": 4, "size": [0.3, 0.3, 0.3],
         "region": {"min": [0, 0, 0], "max": [5, 5, 1]}, "min_spacing": 0.5, "seed": 1}},
    ]}
    plan1 = ex.scenario_build_plan(scn)
    assert plan1 == ex.scenario_build_plan(scn)
    assert plan1[0]["position"] == [1, 2, 0.5] and plan1[0]["yaw"] == 0.3
    assert 1 <= len(plan1) - 1 <= 4
    assert all(op["node_string"] == "Ball {}" for op in plan1[1:])
    other = ex.scenario_build_plan(scn, seed_override=99)
    assert other[1:] != plan1[1:] or len(other) != len(plan1)


def test_scenario_build_plan_rejects_invalid():
    try:
        ex.scenario_build_plan({"objects": [{"foo": 1}]})
        assert False
    except ValueError:
        pass


def test_compare_reports():
    a = {"objects": {"ball": {"end": [0, 0, 1.0]}, "box": {"end": [1, 1, 0]}}}
    b = {"objects": {"ball": {"end": [0, 0, 1.5]}, "box": {"end": [1, 1, 0]},
                     "extra": {"end": [9, 9, 9]}}}
    c = ex.compare_reports(a, b)
    assert not c["identical"] and c["most_divergent"] == "ball"
    assert abs(c["max_divergence"] - 0.5) < 1e-6 and c["only_in_b"] == ["extra"]
    same = ex.compare_reports(a, a)
    assert same["identical"] and same["max_divergence"] == 0.0


def test_parse_performance_log():
    log = "step,speed,realtime\n0,0.9,1.0\n1,1.1,1.0\n2,1.0,1.0\n"
    r = ex.parse_performance_log(log)
    assert r["samples"] == 3 and r["real_time_factor"]["max"] == 1.1
    assert abs(r["real_time_factor"]["avg"] - 1.0) < 1e-6
    assert ex.parse_performance_log("")["samples"] == 0
