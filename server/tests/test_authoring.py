"""Unit tests for the bridge's pure authoring helpers (no Webots needed)."""

import os
import sys

_BRIDGE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)

import authoring as au  # noqa: E402


def test_world_templates():
    for tmpl in ("empty", "indoor_room", "outdoor_flat"):
        w = au.world_template_wbt(template=tmpl, size=8, lighting="studio")
        assert w.startswith("#VRML_SIM R2025a utf8")
        assert "WorldInfo" in w and "Viewpoint" in w and "DEF FLOOR" in w
        assert "DirectionalLight" in w
    assert au.world_template_wbt(template="indoor_room").count("DEF WALL_") == 4
    try:
        au.world_template_wbt(template="bogus")
        assert False
    except ValueError:
        pass


def test_lighting_preset():
    assert au.lighting_preset("night")["lights"][0]["intensity"] == 0.4
    try:
        au.lighting_preset("disco")
        assert False
    except ValueError:
        pass


def test_python_script():
    ns = ['DEF A Solid { translation 0 0 1 }', 'Sphere { radius 0.2 }']
    s = au.python_script_from_nodes(ns)
    assert "from controller import Supervisor" in s
    assert "importMFNodeFromString(-1, node_string)" in s
    assert "DEF A Solid" in s


def test_json_scenario():
    sc = au.json_scenario_from_objects([{"proto": "Ball"}],
                                       physics={"gravity": [0, 0, -9.81]},
                                       conditions=[{"type": "sim_time"}], duration_s=5)
    assert sc["objects"][0]["proto"] == "Ball" and sc["duration_s"] == 5
    assert sc["physics"]["gravity"][2] == -9.81


def test_wrap_proto():
    ns = ('DEF BOX Solid { translation 1 2 3 rotation 0 0 1 0.5 name "box" '
          'children [ Shape { geometry Box { size 1 1 1 } } ] }')
    proto = au.wrap_proto(ns, "MyBox")
    assert proto.startswith("#VRML_SIM R2025a utf8")
    assert "PROTO MyBox [" in proto
    assert "field SFVec3f translation 1 2 3" in proto
    assert 'field SFString name "box"' in proto
    assert "translation IS translation" in proto and "name IS name" in proto
    assert "DEF BOX" not in proto
    try:
        au.wrap_proto(ns, "9bad")
        assert False
    except ValueError:
        pass


def test_wrap_proto_injects_missing_fields():
    ns = 'Solid { children [ Shape { geometry Sphere { radius 0.3 } } ] }'
    proto = au.wrap_proto(ns, "Ball")
    assert "translation IS translation" in proto and "name IS name" in proto


def test_records_csv_and_summary():
    rows = [{"sim_time": 0.0, "node": "ball", "pos": [0, 0, 1.0], "speed": 0.1},
            {"sim_time": 0.1, "node": "ball", "pos": [0, 0, 0.5], "speed": 0.8}]
    csv = au.records_to_csv(rows, ["sim_time", "node", "pos", "speed"])
    assert csv.splitlines()[0] == "sim_time,node,pos,speed"
    assert "0 0 1.0" in csv and "0 0 0.5" in csv
    s = au.summarize_series(rows, "speed")
    assert s["max"] == 0.8 and s["count"] == 2


def test_cad_shape_node_string():
    s = au.cad_shape_node_string("meshes/part.obj", name="part",
                                 physics=True, bounding_box=[1, 2, 3])
    assert s.startswith("Solid {") and 'CadShape { url [ "meshes/part.obj" ]' in s
    assert "boundingObject Box { size 1 2 3 }" in s and "physics Physics { }" in s
    assert "physics" not in au.cad_shape_node_string("a.dae")


def test_supervisor_scaffold():
    sc = au.supervisor_script_scaffold("demo", "print(sup.getTime())")
    assert "from controller import Supervisor" in sc
    assert "def node(ref)" in sc and "print(sup.getTime())" in sc


def test_extern_controller_command():
    cmd = au.extern_controller_command("/wc.exe", "/ctrl.py", robot="bot",
                                       protocol="tcp", port=1234, extra_args=["--x"])
    assert "--robot-name=bot" in cmd and "--protocol=tcp" in cmd
    assert "--port=1234" in cmd and cmd[-2] == "/ctrl.py" and cmd[-1] == "--x"
    assert au.extern_controller_command("/wc.exe", "/ctrl.py") == ["/wc.exe", "/ctrl.py"]
