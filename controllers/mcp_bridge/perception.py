"""Pure perception / projection helpers: axis-angle rotation, pinhole projection
of world points into the current view, viewport label planning, and world-file
header parsing. No Webots (`controller`) imports — unit-testable standalone.
"""

import math
import re


# ---------------------------------------------------------------------------
# Rotation
# ---------------------------------------------------------------------------

def axis_angle_to_matrix(axis, angle):
    """3x3 rotation matrix (row-major, as 9-tuple) from axis-angle."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return (1, 0, 0, 0, 1, 0, 0, 0, 1)
    x, y, z = x / n, y / n, z / n
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    return (t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
            t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
            t * x * z - s * y, t * y * z + s * x, t * z * z + c)


def _matvec(m, v):
    return [m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]]


def camera_basis(orientation_matrix):
    """Forward / up unit vectors of a Webots-style viewpoint from its rotation
    matrix. Webots (ENU/FLU) cameras look along their local +X axis with +Z up
    (matches the bridge's _look_at_orientation, verified against sample worlds)."""
    fwd = _matvec(orientation_matrix, [1, 0, 0])
    up = _matvec(orientation_matrix, [0, 0, 1])
    return fwd, up


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------

def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return [a[0] / n, a[1] / n, a[2] / n] if n > 1e-12 else [0, 0, 0]


def project_point(point, cam_pos, forward, up, fov_v, width, height):
    """Pinhole-project a world point into image pixels. fov_v = vertical field of
    view (rad). Returns {px, py, depth, visible}. depth is distance along the view
    axis; visible means in front of the camera and inside the frame."""
    rel = _sub(point, cam_pos)
    f = _norm(forward)
    depth = _dot(rel, f)
    if depth <= 1e-6:
        return {"px": None, "py": None, "depth": round(depth, 5), "visible": False}
    right = _norm(_cross(f, up))
    tup = _cross(right, f)  # true up
    x = _dot(rel, right)
    y = _dot(rel, tup)
    focal = (height / 2.0) / math.tan(fov_v / 2.0)
    px = width / 2.0 + focal * x / depth
    py = height / 2.0 - focal * y / depth
    visible = (0 <= px <= width) and (0 <= py <= height)
    return {"px": round(px, 1), "py": round(py, 1), "depth": round(depth, 4),
            "visible": bool(visible)}


def viewport_labels(objects, cam_pos, forward, up, fov_v, width, height):
    """Project a list of ``{"name", "position"}`` into the view; return the
    visible ones sorted near→far with pixel coordinates (occlusion-unaware)."""
    out = []
    for o in objects:
        if not o.get("position"):
            continue
        pr = project_point(o["position"], cam_pos, forward, up, fov_v, width, height)
        if pr["visible"]:
            out.append({"name": o.get("name"), "px": pr["px"], "py": pr["py"],
                        "depth": pr["depth"]})
    out.sort(key=lambda r: r["depth"])
    return out


# ---------------------------------------------------------------------------
# World-file header parsing (5.4)
# ---------------------------------------------------------------------------

def parse_world_header(text):
    """Summarize a .wbt: Webots version, WorldInfo title/info, basicTimeStep,
    coordinateSystem, and the robots present with their controllers."""
    out = {}
    m = re.search(r"#VRML_SIM (\S+)", text)
    if m:
        out["webots_version"] = m.group(1)
    title = re.search(r'title\s+"([^"]*)"', text)
    if title:
        out["title"] = title.group(1)
    info = re.search(r"info\s+\[\s*\"([^\"]*)\"", text)
    if info:
        out["info"] = info.group(1)
    bts = re.search(r"basicTimeStep\s+([\d.]+)", text)
    if bts:
        out["basic_time_step"] = float(bts.group(1))
    cs = re.search(r'coordinateSystem\s+"([^"]*)"', text)
    out["coordinate_system"] = cs.group(1) if cs else "ENU"
    robots = []
    for m in re.finditer(r"\b(\w+)\s*\{[^{}]*?controller\s+\"([^\"]*)\"", text,
                         re.DOTALL):
        robots.append({"type": m.group(1), "controller": m.group(2)})
    out["robots"] = robots
    out["robot_count"] = len(robots)
    return out
