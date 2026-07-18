"""Extern controllers & asset import (P4.2, P10.2, P4.7, P4.3).

Runs the user's own controller / supervisor code as an external process against
the running Webots (non-destructive: the world file is untouched), and imports
CAD meshes / flattens PROTOs. Command building and node/script generation live in
the bridge's Webots-free `authoring` module; the subprocess is managed here.
"""

import collections
import os
import subprocess
import sys
import threading
from typing import Optional

from tools.app import MCP_ROOT
from webots_home import find_webots_home

_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import authoring  # noqa: E402

_CONTROLLERS = os.path.join(str(MCP_ROOT), "controllers")

# process state; `popen` is injectable so tests can supply a fake process.
_state = {"proc": None, "log": collections.deque(maxlen=2000), "robot": None,
          "popen": subprocess.Popen}


def _webots_controller_exe():
    home = find_webots_home()
    for rel in ("msys64/mingw64/bin/webots-controller.exe",
                "webots-controller.exe", "webots-controller"):
        p = home / rel
        if p.exists():
            return p
    return home / "webots-controller"


def _pump(stream):
    for line in iter(stream.readline, ""):
        if not line:
            break
        _state["log"].append(line.rstrip())


def register(mcp, bridge):
    @mcp.tool()
    def run_extern_controller(controller_path: str, robot: str,
                              protocol: Optional[str] = None,
                              port: Optional[int] = None,
                              args: Optional[list] = None) -> dict:
        """Run any controller file as an EXTERNAL process against the running
        Webots — the controller-development loop: edit the user's controller, run
        it, read its prints, iterate, without touching the world file. Sets the
        robot's controller field to '<extern>' (via the supervisor), launches
        webots-controller, and captures its stdout/stderr. Stop with
        stop_extern_controller."""
        if _state["proc"] is not None and _state["proc"].poll() is None:
            return {"error": "an extern controller is already running; "
                             "stop_extern_controller first"}
        bridge.command("set_node_field",
                       {"node": robot, "field": "controller", "value": "<extern>"})
        exe = _webots_controller_exe()
        cmd = authoring.extern_controller_command(exe, controller_path, robot=robot,
                                                  protocol=protocol, port=port,
                                                  extra_args=args)
        _state["log"].clear()
        env = os.environ.copy()
        env["WEBOTS_HOME"] = str(find_webots_home())
        proc = _state["popen"](cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, env=env, encoding="utf-8", errors="replace")
        _state["proc"] = proc
        _state["robot"] = robot
        if proc.stdout is not None:
            threading.Thread(target=_pump, args=(proc.stdout,), daemon=True).start()
        return {"started": True, "robot": robot, "command": cmd,
                "pid": getattr(proc, "pid", None)}

    @mcp.tool()
    def get_extern_controller_output(lines: int = 50) -> dict:
        """Read the running (or last) extern controller's captured output — the
        controller's prints and any traceback."""
        log = list(_state["log"])
        proc = _state["proc"]
        running = proc is not None and proc.poll() is None
        return {"running": running, "robot": _state["robot"],
                "total_lines": len(log), "lines": log[-lines:]}

    @mcp.tool()
    def stop_extern_controller() -> dict:
        """Stop the extern controller process (does not restore the robot's
        original controller — set it back with set_node_field if needed)."""
        proc = _state["proc"]
        if proc is None or proc.poll() is not None:
            return {"stopped": False, "reason": "no extern controller running"}
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            proc.kill()
        _state["proc"] = None
        return {"stopped": True, "robot": _state["robot"]}

    @mcp.tool()
    def create_supervisor_script(name: str, code: str) -> dict:
        """Scaffold a standalone Supervisor automation script (with robust
        node/field helpers) into controllers/<name>/, ready to run at full speed
        with run_supervisor_script."""
        text = authoring.supervisor_script_scaffold(name, code)
        d = os.path.join(_CONTROLLERS, name)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}.py")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return {"created": path}

    @mcp.tool()
    def run_supervisor_script(name: str, robot: str) -> dict:
        """Run a script created by create_supervisor_script as an extern controller
        on 'robot' — for automation that must run at full speed inside Webots
        (e.g. regenerate + evaluate many environments)."""
        path = os.path.join(_CONTROLLERS, name, f"{name}.py")
        if not os.path.exists(path):
            return {"error": f"no supervisor script named {name!r} "
                             "(create_supervisor_script first)"}
        return run_extern_controller(path, robot)

    @mcp.tool()
    def import_cad_model(url: str, name: str = "cad", physics: bool = False,
                         bounding_box: Optional[list] = None,
                         position: Optional[list] = None) -> dict:
        """Drop a CAD mesh (.obj/.dae) into the scene as a CadShape Solid. Optional
        Box bounding_box=[x,y,z] + physics make it a dynamic collidable object."""
        node_string = authoring.cad_shape_node_string(
            url, name=name, physics=physics, bounding_box=bounding_box,
            position=tuple(position) if position else (0, 0, 0))
        result = bridge.command("spawn_node", {"node_string": node_string})
        result["node_string"] = node_string
        return result

    @mcp.tool()
    def convert_proto(proto_file: str, out_path: Optional[str] = None) -> dict:
        """Convert a PROTO file to URDF via 'webots convert' (R2025a's single-task
        converter writes the URDF to stdout). Returns the URDF and the file it was
        written to. Runs headless."""
        home = find_webots_home()
        exe = home / "webots"
        for cand in (home / "webots", home / "msys64/mingw64/bin/webots.exe"):
            if cand.exists():
                exe = cand
                break
        out = out_path or (os.path.splitext(proto_file)[0] + ".urdf")
        try:
            # 'webots convert <proto>' emits the URDF on stdout (no output-file arg).
            r = subprocess.run([str(exe), "convert", proto_file],
                               capture_output=True, text=True, timeout=120)
            urdf = r.stdout
            ok = "<robot" in urdf
            if ok:
                with open(out, "w", encoding="utf-8") as fh:
                    fh.write(urdf)
            return {"converted": ok, "out_path": out if ok else None,
                    "urdf": urdf[-4000:],
                    "stderr": r.stderr[-2000:]}
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}
