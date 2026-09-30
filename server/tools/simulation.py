"""Simulation control tools."""

from typing import Optional


def register(mcp, bridge):
    @mcp.tool()
    def world_reload() -> dict:
        """Reload the current world from its .wbt file, discarding runtime changes.
        All controllers (including the MCP bridge) restart — expect a brief
        disconnect. If the reply is lost, an unknown-outcome error is returned;
        the next command reconnects. Inspect state before retrying the reload."""
        return bridge.command("world_reload")

    @mcp.tool()
    def get_simulation_state() -> dict:
        """Get the current Webots simulation state: time, mode (pause/realtime/fast),
        basic time step, loaded world file, and which robots have MCP agents attached."""
        return bridge.command("get_simulation_state")

    @mcp.tool()
    def set_simulation_mode(mode: str) -> dict:
        """Set the simulation run mode. mode: 'pause', 'realtime', or 'fast'
        (fast = run as fast as possible). While paused, scene/simulation tools keep
        working and step_simulation advances time deterministically, but per-robot
        commands (set_motor, get_camera_image, ...) may time out — step or resume
        first. The Webots GUI shows a paused sim as 'running at 0.00x'."""
        return bridge.command("set_simulation_mode", {"mode": mode})

    @mcp.tool()
    def step_simulation(steps: int = 1) -> dict:
        """Advance the simulation by N basic time steps (useful while paused to
        move time forward deterministically). N must be an integer in 0..100000;
        invalid counts are rejected before stepping."""
        return bridge.command("step_simulation", {"steps": steps}, timeout=300.0)

    @mcp.tool()
    def reset_simulation(reload_world: bool = False) -> dict:
        """Reset the simulation to its initial state. reload_world=True does a full
        world reload (restarts all controllers, including the bridge — expect a
        brief disconnect)."""
        return bridge.command("reset_simulation", {"reload_world": reload_world})

    @mcp.tool()
    def save_world(path: Optional[str] = None) -> dict:
        """Save the current world (with all runtime modifications) to its .wbt file,
        or to a different path if given."""
        return bridge.command("save_world", {"path": path})

    @mcp.tool()
    def load_world(path: str) -> dict:
        """Load a different .wbt world file in Webots. The bridge reconnects only if
        the new world also contains the mcp_bridge robot."""
        return bridge.command("load_world", {"path": path})
