"""Webots application management: launch, quit, console capture, docs search."""

import collections
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from webots_home import find_webots_executable, find_webots_home

MCP_ROOT = Path(__file__).resolve().parents[2]

_state = {"proc": None, "log": collections.deque(maxlen=2000)}


def _pump(stream):
    for line in iter(stream.readline, ""):
        _state["log"].append(line.rstrip())
    stream.close()


def _hide_camera_overlays(world: Path):
    """Hide device overlays (camera windows) in the world's .wbproj so they don't
    cover the 3D view in screenshots. Format: 'robot:device;visible;scale;x;y'."""
    proj = world.parent / f".{world.stem}.wbproj"
    if not proj.exists():
        return
    try:
        lines = proj.read_text(encoding="utf-8", errors="replace").splitlines()
        for i, line in enumerate(lines):
            if line.startswith("renderingDevicePerspectives:"):
                prefix, _, devices = line.partition(":")
                # second field per device is overlay visibility: 'dev;1;scale;x;y'
                lines[i] = prefix + ":" + re.sub(r";1;", ";0;", devices, count=1)
        if sys.platform == "win32":
            # .wbproj files are hidden on Windows; hidden files can't be overwritten
            import ctypes
            ctypes.windll.kernel32.SetFileAttributesW(str(proj), 0x80)  # NORMAL
            proj.write_text("\n".join(lines) + "\n", encoding="utf-8")
            ctypes.windll.kernel32.SetFileAttributesW(str(proj), 0x02)  # HIDDEN
        else:
            proj.write_text("\n".join(lines) + "\n", encoding="utf-8")
    except OSError:
        pass  # cosmetic only — never block launching over overlay hiding


def _maximize_webots_window(timeout_s: float = 20.0):
    """Maximize the Webots window once it appears (bigger window = higher-resolution
    exportImage screenshots). Windows-only; a no-op elsewhere."""
    if sys.platform != "win32":
        return
    import ctypes
    user32 = ctypes.windll.user32

    def worker():
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hwnd = user32.FindWindowW(None, None)
            found = []

            @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
            def enum_cb(h, _):
                buf = ctypes.create_unicode_buffer(256)
                user32.GetWindowTextW(h, buf, 256)
                if "Webots R" in buf.value and user32.IsWindowVisible(h):
                    found.append(h)
                return True

            user32.EnumWindows(enum_cb, None)
            if found:
                user32.ShowWindow(found[0], 3)  # SW_MAXIMIZE
                return
            time.sleep(1.0)

    threading.Thread(target=worker, daemon=True).start()


def register(mcp, bridge):
    @mcp.tool()
    def launch_webots(world_path: Optional[str] = None, mode: str = "realtime",
                      auto_install_bridge: bool = True, minimized: bool = False,
                      fullscreen: bool = False, no_rendering: bool = False,
                      batch: bool = True) -> dict:
        """Launch Webots with ANY world file (defaults to the bundled demo world).
        If the world doesn't contain the MCP bridge, it is installed automatically
        (backup created) so the MCP has full access — works with sample worlds and
        foreign projects alike. mode: 'pause', 'realtime', or 'fast'.
        no_rendering runs headless-style (fast batch simulation); batch suppresses
        blocking GUI dialogs. Console output is captured — read it with
        get_webots_console. Wait a few seconds before sending bridge commands."""
        if _state["proc"] is not None and _state["proc"].poll() is None:
            return {"error": "Webots already running (launched by this server). Use quit_webots first."}
        exe = find_webots_executable(console=True)
        world = world_path or str(MCP_ROOT / "worlds" / "demo.wbt")
        bridge_installed = None
        if auto_install_bridge and '"mcp_bridge"' not in Path(world).read_text(encoding="utf-8", errors="replace"):
            from tools.world_files import install_bridge  # local import avoids cycle at load
            bridge_installed = install_bridge(Path(world))
        _hide_camera_overlays(Path(world))
        args = [str(exe), f"--mode={mode}", "--stdout", "--stderr"]
        if minimized:
            args.append("--minimize")
        if fullscreen:
            args.append("--fullscreen")
        if no_rendering:
            args.append("--no-rendering")
        if batch:
            args.append("--batch")
        args.append(world)
        env = os.environ.copy()
        env["WEBOTS_HOME"] = str(find_webots_home())
        # let Webots find our controllers even for foreign worlds
        extra = str(MCP_ROOT)
        prev = env.get("WEBOTS_EXTRA_PROJECT_PATH", "")
        env["WEBOTS_EXTRA_PROJECT_PATH"] = f"{extra};{prev}" if prev else extra
        _state["log"].clear()
        proc = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                encoding="utf-8", errors="replace")
        _state["proc"] = proc
        threading.Thread(target=_pump, args=(proc.stdout,), daemon=True).start()
        if not (minimized or fullscreen or no_rendering):
            _maximize_webots_window()
        return {"launched": True, "pid": proc.pid, "world": world, "mode": mode,
                "bridge_installed": bridge_installed,
                "note": "wait ~5-10s for the world to load, then use get_simulation_state"}

    @mcp.tool()
    def is_webots_running() -> dict:
        """Check whether the Webots process launched by this server is still running,
        and whether the MCP bridge inside it is reachable."""
        proc = _state["proc"]
        running = proc is not None and proc.poll() is None
        bridge_ok = False
        try:
            bridge.command("ping", timeout=5.0)
            bridge_ok = True
        except Exception:  # noqa: BLE001
            pass
        return {"process_running": running,
                "pid": proc.pid if running else None,
                "bridge_reachable": bridge_ok,
                "note": None if running or bridge_ok else
                "Webots may still be running if it was started manually; "
                "bridge_reachable is the authoritative check."}

    @mcp.tool()
    def quit_webots(force: bool = False) -> dict:
        """Quit the Webots process launched by this server (force=True kills it)."""
        proc = _state["proc"]
        if proc is None or proc.poll() is not None:
            return {"quit": False, "reason": "no Webots process launched by this server"}
        if force:
            proc.kill()
        else:
            try:
                bridge.command("execute_code", {"code": "supervisor.simulationQuit(0)"},
                               timeout=10.0)
            except Exception:  # noqa: BLE001
                proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
        _state["proc"] = None
        bridge.close()
        return {"quit": True}

    @mcp.tool()
    def get_webots_console(lines: int = 50, errors_only: bool = False,
                           filter_text: Optional[str] = None) -> dict:
        """Read the Webots console output: controller prints, physics/parse WARNINGs,
        ERRORs, crashed-controller notices. errors_only filters to WARNING/ERROR/
        exception lines — check this whenever something behaves unexpectedly.
        Only available for Webots instances launched via launch_webots (a manually
        started Webots prints to its own GUI console instead)."""
        log = list(_state["log"])
        if errors_only:
            pat = re.compile(r"WARNING|ERROR|CRITICAL|Traceback|Exception|"
                             r"exited with status|failed", re.IGNORECASE)
            log = [l for l in log if pat.search(l)]
        if filter_text:
            log = [l for l in log if filter_text.lower() in l.lower()]
        return {"total_lines": len(log), "lines": log[-lines:]}

    @mcp.tool()
    def search_webots_docs(query: str, max_results: int = 8) -> dict:
        """Search the local Webots reference manual / user guide for API details,
        node fields, or PROTO documentation. Returns matching snippets."""
        docs_dir = find_webots_home() / "docs"
        if not docs_dir.exists():
            return {"error": f"docs not found at {docs_dir}"}
        pattern = re.compile(re.escape(query), re.IGNORECASE)
        hits = []
        for path in docs_dir.glob("**/*.md"):
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for m in pattern.finditer(text):
                start = max(0, m.start() - 150)
                end = min(len(text), m.end() + 250)
                hits.append({"file": str(path.relative_to(docs_dir)),
                             "snippet": text[start:end].strip()})
                if len(hits) >= max_results:
                    return {"query": query, "results": hits}
                break  # one hit per file, keep results diverse
        return {"query": query, "results": hits}
