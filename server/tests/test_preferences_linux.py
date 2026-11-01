"""Linux Webots preferences: Qt's INI file is case-sensitive and keeps
ungrouped keys in '%General'."""

import sys
from pathlib import Path

import pytest

from tools import assets

pytestmark = pytest.mark.skipif(sys.platform == "win32",
                                reason="Windows preferences live in the registry")

CONF = """[%General]
checkWebotsUpdateOnStartup=true
checkwebotsupdateonstartup=true
startupMode=Real-time

[MainWindow]
position=@Point(326 360)

[View3d]
hideAllCameraOverlays=false
"""


@pytest.fixture
def conf(tmp_path, monkeypatch, fake_mcp, fake_bridge):
    path = tmp_path / ".config" / "Cyberbotics" / "Webots-R2025a.conf"
    path.parent.mkdir(parents=True)
    path.write_text(CONF, encoding="utf-8")
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    assets.register(fake_mcp, fake_bridge)
    return path, fake_mcp.tools


def test_set_keeps_key_case_and_reads_back(conf):
    path, tools = conf
    tools["set_webots_preference"]("View3d/hideAllCameraOverlays", "true")
    text = path.read_text(encoding="utf-8")
    assert "hideAllCameraOverlays=true" in text
    assert "hideallcameraoverlays" not in text
    # The case-duplicate an older lower-casing write left no longer breaks a read.
    prefs = tools["get_webots_preferences"]()["preferences"]
    assert prefs["View3d/hideAllCameraOverlays"] == "true"
    assert prefs["MainWindow/position"] == "@Point(326 360)"


def test_general_group_is_qt_percent_general(conf):
    path, tools = conf
    tools["set_webots_preference"]("General/startupMode", "Pause")
    text = path.read_text(encoding="utf-8")
    assert "[General]" not in text
    assert "startupMode=Pause" in text
    assert tools["get_webots_preferences"]()["preferences"]["General/startupMode"] == "Pause"
