"""Pure robot-behavior math: differential-drive kinematics, a go-to-point heading
controller, wheel-motor pairing heuristics, and occupancy-grid rasterization.

No Webots (`controller`) imports — unit-testable standalone. The behavior tools
orchestrate existing MCP commands over this math.
"""

import math


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# Differential drive (1.1)
# ---------------------------------------------------------------------------

def diff_drive_wheel_speeds(linear, angular, wheel_radius, track_width):
    """(v m/s, ω rad/s) → (left, right) wheel angular velocities (rad/s).
    v = r(ωL+ωR)/2,  ω = r(ωR-ωL)/track."""
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be > 0")
    left = (linear - angular * track_width / 2.0) / wheel_radius
    right = (linear + angular * track_width / 2.0) / wheel_radius
    return left, right


def clamp_wheel_speeds(left, right, max_speed):
    """Scale a (left,right) pair down uniformly so neither exceeds max_speed
    (preserves the turn ratio)."""
    if max_speed <= 0:
        return left, right
    peak = max(abs(left), abs(right))
    if peak > max_speed:
        s = max_speed / peak
        return left * s, right * s
    return left, right


# ---------------------------------------------------------------------------
# Go-to-point heading controller (1.3)
# ---------------------------------------------------------------------------

def heading_control(pos_xy, yaw, target_xy, kp=3.0, v_max=0.3, w_max=2.0,
                    tolerance=0.1):
    """Reactive straight-line drive toward a point. Returns
    (linear, angular, done). Slows forward speed when the heading error is large
    so the robot turns in place before driving."""
    dx = target_xy[0] - pos_xy[0]
    dy = target_xy[1] - pos_xy[1]
    dist = math.hypot(dx, dy)
    if dist <= tolerance:
        return 0.0, 0.0, True
    err = wrap_angle(math.atan2(dy, dx) - yaw)
    angular = _clamp(kp * err, -w_max, w_max)
    # forward speed gated by how well we're facing the target
    facing = max(0.0, math.cos(err))
    linear = _clamp(v_max * facing, 0.0, v_max)
    return linear, angular, False


def yaw_from_matrix(orientation):
    """Yaw about the vertical (z) axis from a row-major 3x3 rotation matrix."""
    return math.atan2(orientation[3], orientation[0])


# ---------------------------------------------------------------------------
# Wheel-motor pairing heuristic (1.1)
# ---------------------------------------------------------------------------

def pick_wheel_motors(devices):
    """From a device list ([{"name","type"}]), guess the (left, right) wheel-motor
    names. Prefers RotationalMotors whose names contain left/right + wheel/motor;
    falls back to exactly two rotational motors. Returns (left, right) or
    (None, None)."""
    motors = [d for d in devices
              if "motor" in (d.get("type", "").lower())
              or "motor" in (d.get("name", "").lower())]
    if not motors:
        motors = [d for d in devices if "Rotational" in d.get("type", "")]

    def has(d, *words):
        n = d.get("name", "").lower()
        return all(w in n for w in words)

    left = next((d["name"] for d in motors if "left" in d.get("name", "").lower()), None)
    right = next((d["name"] for d in motors if "right" in d.get("name", "").lower()), None)
    if left and right:
        return left, right
    rot = [d for d in devices if "Rotational" in d.get("type", "")
           or "motor" in d.get("name", "").lower()]
    if len(rot) == 2:
        return rot[0]["name"], rot[1]["name"]
    return None, None


# ---------------------------------------------------------------------------
# Occupancy grid (1.5)
# ---------------------------------------------------------------------------

def occupancy_grid(points_xy, resolution=0.1, size=4.0, origin=(0.0, 0.0)):
    """Rasterize 2D obstacle points into a square occupancy grid centered at
    origin. Returns {"grid": [rows of '#'/'.'], "cells": n, "resolution",
    "obstacles": [[cx,cy], ...]} (world-centered cell centers)."""
    n = max(1, int(round(size / resolution)))
    grid = [[0] * n for _ in range(n)]
    half = size / 2.0
    ox, oy = origin
    for x, y in points_xy:
        gx = int((x - ox + half) / resolution)
        gy = int((y - oy + half) / resolution)
        if 0 <= gx < n and 0 <= gy < n:
            grid[gy][gx] = 1
    obstacles = []
    rows = []
    for gy in range(n - 1, -1, -1):  # north (+y) at top
        row = grid[gy]
        rows.append("".join("#" if c else "." for c in row))
        for gx in range(n):
            if row[gx]:
                cx = ox - half + (gx + 0.5) * resolution
                cy = oy - half + (gy + 0.5) * resolution
                obstacles.append([round(cx, 3), round(cy, 3)])
    return {"grid": rows, "cells": n, "resolution": resolution,
            "size": size, "origin": [ox, oy], "obstacles": obstacles}
