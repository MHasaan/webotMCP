"""Registration + behavior tests for every server tool module (fake bridge)."""

import os

import pytest

from conftest import FakeBridge

from tools import (batch, code_exec, observe, robot, scene, scene_model, sensing,
                   simulation, world_build, world_files)
import resources


EXPECTED = {
    scene: {"get_scene_tree", "find_nodes", "get_scene_bounds", "get_node_details",
            "set_node_field", "get_node_field", "set_velocity", "apply_force",
            "get_node_pose", "get_node_string", "clone_node", "move_node",
            "spawn_node", "delete_node", "set_viewpoint", "insert_field_item",
            "remove_field_item", "save_checkpoint", "restore_checkpoint",
            "set_joint_position", "frame_node", "set_node_visibility",
            "get_node_proto", "get_selected_node"},
    sensing: {"get_viewport_screenshot", "screenshot_multiview", "get_camera_image",
              "get_camera_recognition", "enable_camera_recognition",
              "get_segmentation_image", "get_depth_image", "get_radar_targets",
              "get_recording_status", "get_sensor_values", "get_lidar_summary",
              "export_screenshot", "start_movie_recording", "stop_movie_recording",
              "start_animation_recording", "stop_animation_recording", "set_label"},
    robot: {"list_robots", "attach_mcp_controller", "restart_controller",
            "get_robot_devices", "set_motor", "get_motor_state", "configure_motor",
            "export_urdf", "send_message", "get_messages", "set_connector",
            "vacuum_gripper", "speak", "set_brake", "display_draw", "get_battery",
            "robot_custom_data", "set_led", "play_motion", "get_motion_state",
            "stop_motion"},
    simulation: {"world_reload", "get_simulation_state", "set_simulation_mode",
                 "step_simulation", "reset_simulation", "save_world", "load_world"},
    code_exec: {"execute_supervisor_code", "execute_robot_code"},
    batch: {"batch_execute"},
    observe: {"watch_simulation", "start_tracking", "stop_tracking",
              "get_object_trajectories", "get_interactions", "capture_sequence"},
    world_files: {"list_worlds", "read_world_file", "install_bridge_into_world"},
    scene_model: {"get_object_catalog", "get_object_properties",
                  "get_contact_points", "reset_node_physics", "find_nodes_near",
                  "objects_in_region", "check_overlap", "find_overlapping_pairs",
                  "get_spatial_relations"},
    world_build: {"drop_to_ground", "place_on", "align_objects",
                  "distribute_objects", "place_row", "place_grid",
                  "find_free_space", "scatter_objects", "validate_world",
                  "get_scene_map", "snapshot_scene", "diff_scene"},
}


def test_dx_registers_tools_and_prompts(fake_mcp, fake_bridge):
    from tools import dx
    dx.register(fake_mcp, fake_bridge)
    assert {"get_viewport_labels", "describe_sample", "list_sample_worlds",
            "update_world_file", "clear_webots_cache"} <= set(fake_mcp.tools)
    assert {"inspect_scene", "robot_bringup", "record_demo",
            "experiment_loop"} <= set(fake_mcp.prompts)
    # prompts return non-empty guidance strings
    assert "get_object_catalog" in fake_mcp.prompts["inspect_scene"]()
    fake_mcp.tools["get_viewport_labels"](width=800)
    _, action, params = fake_bridge.calls[-1]
    assert action == "get_viewport_labels" and params["width"] == 800


def test_extern_controller_lifecycle(fake_mcp):
    from conftest import FakeBridge
    from tools import extern

    class FakeProc:
        def __init__(self, *a, **k):
            self.pid = 4242
            self._alive = True
            self.stdout = None  # no pump thread
            self.terminated = False
        def poll(self):
            return None if self._alive else 0
        def terminate(self):
            self.terminated = True
            self._alive = False
        def wait(self, timeout=None):
            return 0

    extern._state["proc"] = None
    extern._state["popen"] = lambda *a, **k: FakeProc()
    bridge = FakeBridge({"spawn_node": {"spawned": True}})
    extern.register(fake_mcp, bridge)
    assert {"run_extern_controller", "get_extern_controller_output",
            "stop_extern_controller", "create_supervisor_script",
            "run_supervisor_script", "import_cad_model",
            "convert_proto"} <= set(fake_mcp.tools)

    r = fake_mcp.tools["run_extern_controller"]("/ctrl.py", "bot", protocol="tcp")
    assert r["started"] and "--robot-name=bot" in r["command"]
    # setting controller "<extern>" went to the bridge
    setc = [c for c in bridge.calls if c[1] == "set_node_field"][-1]
    assert setc[2]["field"] == "controller" and setc[2]["value"] == "<extern>"
    assert fake_mcp.tools["get_extern_controller_output"]()["running"] is True
    assert fake_mcp.tools["stop_extern_controller"]()["stopped"] is True
    # CAD import builds a CadShape node string and spawns it
    cad = fake_mcp.tools["import_cad_model"]("m.obj", physics=True, bounding_box=[1, 1, 1])
    assert "CadShape" in cad["node_string"]
    _, action, params = bridge.calls[-1]
    assert action == "spawn_node" and "CadShape" in params["node_string"]


def test_drive_robot_composite(fake_mcp):
    from conftest import FakeBridge
    from tools import behaviors

    class DriveBridge(FakeBridge):
        def __init__(self):
            super().__init__()
            self._t = 0
        def command(self, action, params=None, timeout=60.0):
            self.calls.append(("command", action, params))
            if action == "get_simulation_state":
                return {"basic_time_step": 32}
            if action == "get_node_pose":
                self._t += 1
                return {"position": [0.0, 0.0, 0.0] if self._t == 1 else [1.0, 0.0, 0.0],
                        "orientation": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
            return {"ok": True}
        def robot_command(self, robot, action, params=None, timeout=60.0):
            self.calls.append(("robot", robot, action, params))
            if action == "get_devices":
                return {"devices": [{"name": "left wheel motor", "type": "RotationalMotor"},
                                    {"name": "right wheel motor", "type": "RotationalMotor"}]}
            return {"ok": True}

    bridge = DriveBridge()
    behaviors.register(fake_mcp, bridge)
    assert {"drive_robot", "move_robot_to", "build_occupancy_grid"} <= set(fake_mcp.tools)
    r = fake_mcp.tools["drive_robot"]("bot", linear=0.2, duration_s=1.0,
                                      wheel_radius=0.05, track_width=0.3)
    # straight drive: both wheels 4.0 rad/s, distance 1.0m between the two poses
    assert abs(r["wheel_speeds"]["left wheel motor"] - 4.0) < 1e-6
    assert abs(r["distance_m"] - 1.0) < 1e-6
    # wheels were set then zeroed
    setmotor = [c for c in bridge.calls if c[0] == "robot" and c[2] == "set_motor"]
    assert setmotor[-1][3]["motors"]["left wheel motor"]["velocity"] == 0.0


def test_move_robot_to_reaches_target(fake_mcp):
    from conftest import FakeBridge
    from tools import behaviors

    class GotoBridge(FakeBridge):
        def __init__(self):
            super().__init__()
            self.x = 0.0
        def command(self, action, params=None, timeout=60.0):
            self.calls.append(("command", action, params))
            if action == "get_simulation_state":
                return {"basic_time_step": 32}
            if action == "step_simulation":
                self.x += 0.5  # advance toward target each leg
                return {"stepped": True}
            if action == "get_node_pose":
                return {"position": [self.x, 0.0, 0.0],
                        "orientation": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
            return {"ok": True}
        def robot_command(self, robot, action, params=None, timeout=60.0):
            self.calls.append(("robot", robot, action, params))
            if action == "get_devices":
                return {"devices": [{"name": "left wheel motor", "type": "RotationalMotor"},
                                    {"name": "right wheel motor", "type": "RotationalMotor"}]}
            return {"ok": True}

    bridge = GotoBridge()
    behaviors.register(fake_mcp, bridge)
    r = fake_mcp.tools["move_robot_to"]("bot", target=[1.0, 0.0], tolerance=0.15,
                                        max_duration_s=5.0)
    assert r["reached"] is True and r["final"][0] >= 0.85


def test_build_occupancy_grid(fake_mcp):
    from conftest import FakeBridge
    from tools import behaviors
    bridge = FakeBridge({
        "get_lidar_summary": {"point_cloud": [[0.5, 0.5, 0.0], [-0.5, -0.5, 0.0]]},
        "get_node_pose": {"position": [0.0, 0.0, 0.0]},
    })
    behaviors.register(fake_mcp, bridge)
    g = fake_mcp.tools["build_occupancy_grid"]("bot", resolution=0.5, size=4.0)
    assert g["points"] == 2 and "#" in g["ascii"] and g["cells"] == 8


def test_console_resource(fake_mcp, fake_bridge):
    import resources
    from tools.app import _state
    _state["log"].append("[bot] hello")
    resources.register(fake_mcp, fake_bridge)
    import json
    out = json.loads(fake_mcp.resources["webots://console"]())
    assert any("hello" in ln for ln in out["lines"])


def test_experiments_tools_pass_through(fake_mcp):
    from conftest import FakeBridge
    from tools import experiments_tools
    bridge = FakeBridge({
        "configure_physics": {"applied": {"random_seed": 7}},
        "profile_simulation": {"real_time_factor": 0.8},
    })
    experiments_tools.register(fake_mcp, bridge)
    assert {"configure_physics", "profile_simulation", "profile_from_log",
            "compare_runs", "save_scenario", "load_scenario",
            "run_scenario"} <= set(fake_mcp.tools)
    fake_mcp.tools["configure_physics"](recipe="moon", random_seed=7)
    _, action, params = bridge.calls[-1]
    assert action == "configure_physics" and params["recipe"] == "moon"
    # compare_runs is pure (no bridge)
    a = {"objects": {"ball": {"end": [0, 0, 1.0]}}}
    b = {"objects": {"ball": {"end": [0, 0, 1.4]}}}
    cmp = fake_mcp.tools["compare_runs"](a, b)
    assert not cmp["identical"] and abs(cmp["max_divergence"] - 0.4) < 1e-6


def test_authoring_tools_pass_through(fake_mcp):
    from conftest import FakeBridge
    from tools import authoring_tools
    bridge = FakeBridge({
        "generate_world_script": {"format": "python", "script": "x", "objects": 0},
        "extract_proto_from_node": {"proto_name": "P", "proto_text": "#VRML_SIM R2025a utf8\nPROTO P [] {}\n"},
        "record_states": {"rows": [{"sim_time": 0.0, "node": "b", "speed": 0.2}],
                          "count": 1, "fields": ["speed"], "nodes": ["b"]},
    })
    authoring_tools.register(fake_mcp, bridge)
    assert {"generate_world_script", "extract_proto_from_node", "create_world",
            "backup_world", "list_world_backups", "restore_world_backup",
            "list_appearances", "set_appearance", "set_recognition_colors",
            "configure_lighting", "record_states"} <= set(fake_mcp.tools)
    r = fake_mcp.tools["generate_world_script"]("python")
    assert r["format"] == "python"
    # create_world writes a real file under MCP_ROOT/worlds
    cw = fake_mcp.tools["create_world"]("unittest_world", template="indoor_room")
    assert os.path.exists(cw["created"])
    with open(cw["created"], encoding="utf-8") as fh:
        assert fh.read().startswith("#VRML_SIM R2025a utf8")
    os.remove(cw["created"])
    # record_states writes a CSV and summarizes speed
    rs = fake_mcp.tools["record_states"](duration_s=0.1, fields=["speed"])
    assert os.path.exists(rs["csv_path"]) and rs["speed_summary"]["max"] == 0.2
    os.remove(rs["csv_path"])


def test_analyze_pass_through(fake_mcp):
    from conftest import FakeBridge
    from tools import analyze
    bridge = FakeBridge({"run_experiment": {"objects": {}, "interactions": [],
                                            "diff": {}, "anomalies": [],
                                            "anomaly_count": 0}})
    analyze.register(fake_mcp, bridge)
    assert {"wait_until", "run_experiment", "detect_anomalies",
            "get_console_diagnostics", "get_controller_logs"} <= set(fake_mcp.tools)
    cond = {"type": "contact", "a": "BALL", "b": "FLOOR"}
    fake_mcp.tools["wait_until"](cond, timeout_s=5)
    _, action, params = bridge.calls[-1]
    assert action == "wait_until" and params["condition"] == cond
    r = fake_mcp.tools["run_experiment"](duration_s=2.0, until=cond)
    assert isinstance(r, list) and r[0]["anomaly_count"] == 0
    # console tools run server-side (no bridge); should not raise
    fake_mcp.tools["get_console_diagnostics"]()
    fake_mcp.tools["get_controller_logs"]("robot")


def test_world_build_pass_through(fake_mcp, fake_bridge):
    world_build.register(fake_mcp, fake_bridge)
    fake_mcp.tools["drop_to_ground"]("BOX", floor=0.1)
    _, action, params = fake_bridge.calls[-1]
    assert action == "drop_to_ground" and params["floor"] == 0.1
    fake_mcp.tools["place_on"]("A", "B", offset=[0.1, 0.0])
    _, action, params = fake_bridge.calls[-1]
    assert action == "place_on" and params["target"] == "B"
    fake_mcp.tools["scatter_objects"](5, [0.3, 0.3, 0.3],
                                      {"min": [0, 0, 0], "max": [2, 2, 1]}, seed=7)
    _, action, params = fake_bridge.calls[-1]
    assert action == "scatter_objects" and params["seed"] == 7 and params["count"] == 5
    fake_mcp.tools["diff_scene"]("start")
    _, action, params = fake_bridge.calls[-1]
    assert action == "diff_scene" and params["name_a"] == "start" and params["name_b"] == "now"


@pytest.mark.parametrize("module", list(EXPECTED), ids=lambda m: m.__name__)
def test_module_registers_expected_tools(module, fake_mcp, fake_bridge):
    module.register(fake_mcp, fake_bridge)
    assert set(fake_mcp.tools) == EXPECTED[module]


def test_resources_register(fake_mcp, fake_bridge):
    resources.register(fake_mcp, fake_bridge)
    assert set(fake_mcp.resources) == {"webots://simulation", "webots://scene",
                                       "webots://robots", "webots://scene/{node}",
                                       "webots://conventions", "webots://console"}


def test_scene_model_tools_pass_through(fake_mcp, fake_bridge):
    scene_model.register(fake_mcp, fake_bridge)

    fake_mcp.tools["get_object_catalog"](page_size=10, cursor=5)
    _, action, params = fake_bridge.calls[-1]
    assert action == "get_object_catalog"
    assert params["page_size"] == 10 and params["cursor"] == 5

    fake_mcp.tools["get_contact_points"]("BOX", include_descendants=True)
    _, action, params = fake_bridge.calls[-1]
    assert action == "get_contact_points" and params["include_descendants"] is True

    fake_mcp.tools["find_nodes_near"](0.4, node="ROBOT")
    _, action, params = fake_bridge.calls[-1]
    assert action == "find_nodes_near" and params["radius"] == 0.4
    assert params["node"] == "ROBOT" and params["point"] is None

    fake_mcp.tools["check_overlap"]("A", "B")
    _, action, params = fake_bridge.calls[-1]
    assert action == "check_overlap"
    assert params["node_a"] == "A" and params["node_b"] == "B"

    fake_mcp.tools["get_spatial_relations"](near_threshold=1.2)
    _, action, params = fake_bridge.calls[-1]
    assert action == "get_spatial_relations" and params["near_threshold"] == 1.2


def test_conventions_resource_derives_up_axis(fake_mcp):
    bridge = FakeBridge({"get_simulation_state": {"coordinate_system": "NUE",
                                                  "basic_time_step": 16}})
    resources.register(fake_mcp, bridge)
    import json
    out = json.loads(fake_mcp.resources["webots://conventions"]())
    assert out["coordinate_system"] == "NUE" and out["up_axis"] == "y"


def test_scene_tools_pass_through_params(fake_mcp, fake_bridge):
    scene.register(fake_mcp, fake_bridge)
    fake_mcp.tools["get_scene_tree"](page_size=5, parent="ROOT", cursor=10)
    kind, action, params = fake_bridge.calls[-1]
    assert (kind, action) == ("command", "get_scene_tree")
    assert params["page_size"] == 5 and params["parent"] == "ROOT" and params["cursor"] == 10

    fake_mcp.tools["clone_node"]("BOX", new_def="BOX2", position=[1, 2, 3])
    _, action, params = fake_bridge.calls[-1]
    assert action == "clone_node" and params["new_def"] == "BOX2"


def test_robot_custom_data_get_vs_set(fake_mcp, fake_bridge):
    robot.register(fake_mcp, fake_bridge)
    fake_mcp.tools["robot_custom_data"]("bot")
    assert fake_bridge.calls[-1][2] == "get_custom_data"
    fake_mcp.tools["robot_custom_data"]("bot", data="x=1")
    assert fake_bridge.calls[-1][2] == "set_custom_data"
    assert fake_bridge.calls[-1][3] == {"data": "x=1"}


def test_batch_execute_stop_on_error_and_whitelist(fake_mcp):
    from connection import BridgeError
    bridge = FakeBridge({"delete_node": BridgeError("boom")})
    batch.register(fake_mcp, bridge)
    be = fake_mcp.tools["batch_execute"]

    r = be([{"action": "spawn_node"}, {"action": "delete_node"},
            {"action": "get_scene_tree"}], stop_on_error=False)
    assert (r["executed"], r["succeeded"], r["failed"]) == (3, 2, 1)

    r = be([{"action": "delete_node"}, {"action": "spawn_node"}])
    assert r["executed"] == 1  # stopped at first failure

    r = be([{"action": "quit_webots"}])
    assert not r["results"][0]["success"]
    assert "not allowed" in r["results"][0]["error"]


def test_watch_simulation_composes_bridge_calls(fake_mcp):
    bridge = FakeBridge({
        "get_simulation_state": {"basic_time_step": 32},
        "stop_tracking": {"trajectories": {}, "interactions": []},
    })
    observe.register(fake_mcp, bridge)
    fake_mcp.tools["watch_simulation"](duration_s=1.0)
    actions = [c[1] for c in bridge.calls if c[0] == "command"]
    assert actions == ["get_simulation_state", "start_tracking",
                       "step_simulation", "stop_tracking"]
    step_params = bridge.calls[2][2]
    assert step_params["steps"] == 31  # 1000ms / 32ms


def test_get_interactions_projection(fake_mcp):
    bridge = FakeBridge({"get_tracking": {"interactions": [{"e": 1}],
                                          "sim_time": 4.2, "trajectories": {}}})
    observe.register(fake_mcp, bridge)
    r = fake_mcp.tools["get_interactions"]()
    assert r == {"interactions": [{"e": 1}], "sim_time": 4.2}
