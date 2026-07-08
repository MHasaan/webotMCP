"""Dynamic scene observation: object motion tracking, interaction (contact)
detection, and multi-frame visual capture of the running simulation."""

from typing import Optional

from tools.sensing import _to_image


def register(mcp, bridge):
    @mcp.tool()
    def watch_simulation(duration_s: float = 3.0, sample_every: int = 2,
                         max_points: int = 40, nodes: Optional[list] = None) -> dict:
        """Watch the simulation run for duration_s seconds (sim time) and return a
        motion digest: for every dynamic object (or the given nodes) its trajectory
        (positions over time), displacement, path length, top speed — plus all
        interactions (contact_start/contact_end events: who touched whom, when,
        where). THE tool for understanding how the scene behaves in motion.
        Set up the action first (e.g. set_motor to drive a robot), then watch."""
        state = bridge.command("get_simulation_state")
        steps = max(1, int(duration_s * 1000 / state["basic_time_step"]))
        bridge.command("start_tracking", {"nodes": nodes, "sample_every": sample_every})
        bridge.command("step_simulation", {"steps": steps}, timeout=max(120.0, duration_s * 20))
        return bridge.command("stop_tracking", {"max_points": max_points}, timeout=60.0)

    @mcp.tool()
    def start_tracking(nodes: Optional[list] = None, sample_every: int = 2) -> dict:
        """Start recording object positions/velocities and contacts continuously
        (defaults to all dynamic objects and robots). Use when you want to track
        motion across your own sequence of commands (driving, forces, motions...),
        then read results with get_object_trajectories / get_interactions."""
        return bridge.command("start_tracking",
                              {"nodes": nodes, "sample_every": sample_every})

    @mcp.tool()
    def stop_tracking(max_points: int = 40) -> dict:
        """Stop tracking and return the final motion digest (trajectories + interactions)."""
        return bridge.command("stop_tracking", {"max_points": max_points}, timeout=60.0)

    @mcp.tool()
    def get_object_trajectories(node: Optional[str] = None, max_points: int = 50) -> dict:
        """Get recorded trajectories so far (tracking keeps running). Optionally for
        one object. Includes displacement, path length, speeds, and downsampled
        position history with timestamps."""
        return bridge.command("get_tracking", {"node": node, "max_points": max_points},
                              timeout=60.0)

    @mcp.tool()
    def get_interactions() -> dict:
        """Get just the interaction log recorded so far: contact_start / contact_end
        events between objects (and with the static environment), with sim time and
        contact position."""
        r = bridge.command("get_tracking", {"max_points": 1}, timeout=60.0)
        return {"interactions": r["interactions"], "sim_time": r["sim_time"]}

    @mcp.tool()
    def capture_sequence(steps: int = 100, frames: int = 5, max_dim: int = 768,
                         quality: int = 85, follow: Optional[str] = None) -> list:
        """Advance the simulation and capture evenly-spaced screenshots of the 3D
        view — SEE the motion as a filmstrip (returned as inline images, each
        preceded by its sim-time caption). Max 10 frames. Pass follow=<robot/object
        name> to make the camera track it so moving objects stay in frame. Runs
        alongside tracking if start_tracking is active."""
        if follow:
            bridge.command("set_viewpoint", {"follow": follow})
        result = bridge.command("capture_sequence",
                                {"steps": steps, "frames": frames, "quality": quality},
                                timeout=300.0)
        out = []
        for fr in result["frames"]:
            out.append(f"t={fr['t']}s:")
            out.append(_to_image(fr["base64"], max_dim))
        return out
