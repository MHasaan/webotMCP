"""Registration + behavior tests for every server tool module (fake bridge)."""

import pytest

from conftest import FakeBridge

from tools import batch, code_exec, observe, robot, scene, sensing, simulation, world_files
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
}


@pytest.mark.parametrize("module", list(EXPECTED), ids=lambda m: m.__name__)
def test_module_registers_expected_tools(module, fake_mcp, fake_bridge):
    module.register(fake_mcp, fake_bridge)
    assert set(fake_mcp.tools) == EXPECTED[module]


def test_resources_register(fake_mcp, fake_bridge):
    resources.register(fake_mcp, fake_bridge)
    assert set(fake_mcp.resources) == {"webots://simulation", "webots://scene",
                                       "webots://robots", "webots://scene/{node}"}


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
