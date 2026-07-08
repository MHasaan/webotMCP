"""Preflight diagnostics: one call that tells you what's healthy and what to fix."""

import socket
import time


def _port_open(host, port, timeout=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def register(mcp, bridge):
    @mcp.tool()
    def preflight() -> dict:
        """Health-check the whole stack in one call: Webots process, bridge TCP
        port, bridge round-trip latency, simulation state, and robot agents.
        Run this FIRST when anything misbehaves — every failing check comes with
        the fix."""
        checks = []

        def check(name, ok, detail, fix=None):
            entry = {"check": name, "ok": bool(ok), "detail": detail}
            if not ok and fix:
                entry["fix"] = fix
            checks.append(entry)
            return ok

        # 1. Webots process launched by this server
        from tools.app import _state
        proc = _state.get("proc")
        proc_running = proc is not None and proc.poll() is None
        check("webots_process", True,
              f"pid {proc.pid}" if proc_running else
              "not launched by this server (may still be running externally)")

        # 2. bridge port listening
        port_ok = check(
            "bridge_port", _port_open(bridge.host, bridge.port),
            f"{bridge.host}:{bridge.port}",
            fix="launch_webots, or install_bridge_into_world + open the world in "
                "Webots; if Webots was force-killed, kill stale 'python mcp_bridge.py' "
                "processes holding the port")

        # 3. bridge round-trip
        state = None
        if port_ok:
            t0 = time.time()
            try:
                bridge.command("ping", timeout=5.0)
                latency = round((time.time() - t0) * 1000, 1)
                check("bridge_roundtrip", True, f"{latency} ms")
                state = bridge.command("get_simulation_state", timeout=10.0)
            except Exception as exc:  # noqa: BLE001
                check("bridge_roundtrip", False, str(exc),
                      fix="the world may be paused from the Webots GUI (blocks the "
                          "bridge) — resume it, or reload the world")
        else:
            check("bridge_roundtrip", False, "skipped (port closed)")

        # 4. simulation state + agents
        if state:
            check("simulation", True,
                  f"t={state.get('time')}s mode={state.get('mode')} "
                  f"step={state.get('basic_time_step')}ms "
                  f"world={state.get('world')}")
            agents = state.get("agents") or state.get("mcp_agents") or []
            check("robot_agents", True,
                  f"{len(agents)} attached: {agents}" if agents else
                  "none attached (use attach_mcp_controller to control robots)")
            if state.get("mode") == "pause":
                check("pause_warning", True,
                      "simulation is paused: per-robot commands may time out — "
                      "step_simulation or set_simulation_mode('realtime') first")
        ok = all(c["ok"] for c in checks)
        return {"healthy": ok, "checks": checks}
