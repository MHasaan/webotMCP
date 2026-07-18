"""High-level robot behaviors (P1.1, P1.3, P1.5): one tool call = one behavior.

These close the loop over existing MCP commands (get_devices, set_motor,
step_simulation, get_node_pose, get_lidar_summary) using the bridge's Webots-free
`kinematics` module, so the LLM doesn't micro-manage wheel velocities.
"""

import math
import os
import sys
from typing import Optional

from tools.app import MCP_ROOT

_BRIDGE_DIR = os.path.join(str(MCP_ROOT), "controllers", "mcp_bridge")
if _BRIDGE_DIR not in sys.path:
    sys.path.insert(0, _BRIDGE_DIR)
import kinematics  # noqa: E402


def register(mcp, bridge):
    def _resolve_wheels(robot, left_motor, right_motor):
        if left_motor and right_motor:
            return left_motor, right_motor, None
        devs = bridge.robot_command(robot, "get_devices")
        devlist = devs.get("devices") if isinstance(devs, dict) else devs
        left, right = kinematics.pick_wheel_motors(devlist or [])
        return left_motor or left, right_motor or right, devs

    def _set_wheels(robot, l_name, wl, r_name, wr):
        bridge.robot_command(robot, "set_motor",
                             {"motors": {l_name: {"velocity": wl},
                                         r_name: {"velocity": wr}}})

    @mcp.tool()
    def drive_robot(robot: str, linear: float = 0.0, angular: float = 0.0,
                    duration_s: float = 2.0, wheel_radius: float = 0.05,
                    track_width: float = 0.3, left_motor: Optional[str] = None,
                    right_motor: Optional[str] = None,
                    max_wheel_speed: float = 0.0) -> dict:
        """Differential-drive convenience: drive at linear (m/s) + angular (rad/s)
        for duration_s of sim time, then stop. Wheel motors are auto-paired from
        the device list (override with left_motor/right_motor; tune wheel_radius/
        track_width for your robot). Returns start/end pose and distance travelled."""
        l_name, r_name, devs = _resolve_wheels(robot, left_motor, right_motor)
        if not (l_name and r_name):
            return {"error": "could not identify wheel motors; pass left_motor and "
                             "right_motor", "devices": devs}
        wl, wr = kinematics.diff_drive_wheel_speeds(linear, angular, wheel_radius,
                                                    track_width)
        if max_wheel_speed > 0:
            wl, wr = kinematics.clamp_wheel_speeds(wl, wr, max_wheel_speed)
        start = bridge.command("get_node_pose", {"node": robot})
        _set_wheels(robot, l_name, wl, r_name, wr)
        state = bridge.command("get_simulation_state")
        steps = max(1, int(duration_s * 1000 / state["basic_time_step"]))
        bridge.command("step_simulation", {"steps": steps},
                       timeout=max(120.0, duration_s * 20))
        _set_wheels(robot, l_name, 0.0, r_name, 0.0)
        end = bridge.command("get_node_pose", {"node": robot})
        dist = None
        if start.get("position") and end.get("position"):
            dist = math.dist(start["position"][:2], end["position"][:2])
        return {"robot": robot,
                "wheel_speeds": {l_name: round(wl, 4), r_name: round(wr, 4)},
                "start": start.get("position"), "end": end.get("position"),
                "distance_m": round(dist, 4) if dist is not None else None}

    @mcp.tool()
    def move_robot_to(robot: str, target: list, tolerance: float = 0.15,
                      max_duration_s: float = 15.0, wheel_radius: float = 0.05,
                      track_width: float = 0.3, v_max: float = 0.3,
                      left_motor: Optional[str] = None,
                      right_motor: Optional[str] = None, leg_steps: int = 5) -> dict:
        """Closed-loop go-to-point: a heading P-controller drives the robot toward
        target=[x,y] using ground-truth pose each tick, until within 'tolerance' or
        'max_duration_s'. Straight-line reactive drive — NOT an obstacle-aware
        planner (use get_lidar_summary / build_occupancy_grid between legs for
        that). Returns whether it arrived, the final pose and steps taken."""
        l_name, r_name, devs = _resolve_wheels(robot, left_motor, right_motor)
        if not (l_name and r_name):
            return {"error": "could not identify wheel motors; pass left_motor and "
                             "right_motor", "devices": devs}
        state = bridge.command("get_simulation_state")
        ts = state["basic_time_step"]
        max_steps = max(1, int(max_duration_s * 1000 / ts))
        done = False
        steps = 0
        pose = bridge.command("get_node_pose", {"node": robot})
        while steps < max_steps:
            pose = bridge.command("get_node_pose", {"node": robot})
            pos = pose.get("position") or [0, 0, 0]
            yaw = kinematics.yaw_from_matrix(pose.get("orientation")
                                             or (1, 0, 0, 0, 1, 0, 0, 0, 1))
            lin, ang, done = kinematics.heading_control(
                pos[:2], yaw, target, v_max=v_max, tolerance=tolerance)
            if done:
                break
            wl, wr = kinematics.diff_drive_wheel_speeds(lin, ang, wheel_radius,
                                                        track_width)
            _set_wheels(robot, l_name, wl, r_name, wr)
            bridge.command("step_simulation", {"steps": leg_steps},
                           timeout=max(60.0, max_duration_s * 5))
            steps += leg_steps
        _set_wheels(robot, l_name, 0.0, r_name, 0.0)
        final = bridge.command("get_node_pose", {"node": robot})
        return {"reached": done, "final": final.get("position"), "target": target,
                "steps": steps}

    @mcp.tool()
    def build_occupancy_grid(robot: str, resolution: float = 0.1, size: float = 4.0,
                             lidar: Optional[str] = None) -> dict:
        """Build a 2D occupancy grid from one lidar scan + the robot's pose:
        returns an ASCII map ('#' = obstacle, '.' = free, north up) and an obstacle
        list in world coordinates. Server-side math over get_lidar_summary — no
        bridge changes. Accumulate multiple calls as the robot moves for coverage."""
        summary = bridge.robot_command(robot, "get_lidar_summary",
                                       {"include_point_cloud": True, "lidar": lidar})
        pts = (summary.get("point_cloud") or summary.get("points") or []
               if isinstance(summary, dict) else [])
        pose = bridge.command("get_node_pose", {"node": robot})
        origin = tuple((pose.get("position") or [0, 0])[:2])
        xy = [(p[0], p[1]) for p in pts if isinstance(p, (list, tuple)) and len(p) >= 2]
        grid = kinematics.occupancy_grid(xy, resolution=resolution, size=size,
                                         origin=origin)
        grid["robot"] = robot
        grid["points"] = len(xy)
        grid["ascii"] = "\n".join(grid["grid"])
        return grid
