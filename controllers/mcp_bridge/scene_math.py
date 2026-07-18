"""Pure geometry / placement / validation math for the MCP bridge.

Deliberately free of any Webots (`controller`) imports so it can be unit-tested
without a running simulator. The bridge builds plain-data catalogs (lists of
dicts, AABBs as ``(min3, max3)`` tuples) and delegates the math here.

Conventions: an AABB is ``([minx,miny,minz], [maxx,maxy,maxz])``. ``up`` is the
index of the vertical axis (2 = z for ENU, 1 = y for NUE). All lengths in meters.
"""

import math
import random

Vec3 = list


# ---------------------------------------------------------------------------
# AABB primitives
# ---------------------------------------------------------------------------

def center_of(aabb):
    return [(aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3)]


def size_of(aabb):
    return [aabb[1][i] - aabb[0][i] for i in range(3)]


def aabb_from_center_size(center, size):
    h = [s / 2.0 for s in size]
    return ([center[i] - h[i] for i in range(3)],
            [center[i] + h[i] for i in range(3)])


def aabb_overlap(a, b, eps=0.0):
    return all(a[0][i] <= b[1][i] - eps and b[0][i] <= a[1][i] - eps
               for i in range(3))


def aabb_contains(outer, inner):
    return all(outer[0][i] <= inner[0][i] and inner[1][i] <= outer[1][i]
               for i in range(3))


def _horizontal_axes(up):
    return [i for i in range(3) if i != up]


# ---------------------------------------------------------------------------
# Structured placement (8.1)
# ---------------------------------------------------------------------------

def place_on_position(mover_size, target_aabb, offset=(0.0, 0.0), up=2, gap=0.0):
    """Center a mover of ``mover_size`` on top of ``target_aabb`` (+ horizontal
    ``offset``), resting its bottom on the target's top (+ ``gap``)."""
    tc = center_of(target_aabb)
    ax = _horizontal_axes(up)
    pos = [0.0, 0.0, 0.0]
    pos[ax[0]] = tc[ax[0]] + offset[0]
    pos[ax[1]] = tc[ax[1]] + offset[1]
    pos[up] = target_aabb[1][up] + mover_size[up] / 2.0 + gap
    return pos


def drop_position(mover_center, mover_size, supports, floor=0.0, up=2, gap=0.0):
    """Lower a mover so its bottom rests on the highest support whose horizontal
    footprint overlaps it, else on the floor plane. ``supports`` are AABBs."""
    ax = _horizontal_axes(up)
    mh = [mover_center[a] for a in ax]
    hh = [mover_size[a] / 2.0 for a in ax]
    top = floor
    for s in supports:
        if all(s[0][a] <= mh[k] + hh[k] and mh[k] - hh[k] <= s[1][a]
               for k, a in enumerate(ax)):
            top = max(top, s[1][up])
    pos = list(mover_center)
    pos[up] = top + mover_size[up] / 2.0 + gap
    return pos


def align_positions(items, axis, mode="center"):
    """Give every item the same coordinate on ``axis``. ``items`` = list of
    ``{"id", "position":[x,y,z]}``. Returns ``{id: new_position}``."""
    if not items:
        return {}
    vals = [it["position"][axis] for it in items]
    ref = {"min": min(vals), "max": max(vals)}.get(mode, sum(vals) / len(vals))
    out = {}
    for it in items:
        p = list(it["position"])
        p[axis] = ref
        out[it["id"]] = p
    return out


def distribute_positions(items, axis, spacing=None, extent=None):
    """Evenly space items along ``axis`` (ordered by current coordinate). Give
    ``spacing`` for a fixed gap, ``extent`` to spread across a total span, or
    neither to spread across the current first..last range."""
    if not items:
        return {}
    ordered = sorted(items, key=lambda it: it["position"][axis])
    n = len(ordered)
    start = ordered[0]["position"][axis]
    if spacing is None:
        end = start + extent if extent is not None else ordered[-1]["position"][axis]
        spacing = (end - start) / (n - 1) if n > 1 else 0.0
    out = {}
    for i, it in enumerate(ordered):
        p = list(it["position"])
        p[axis] = start + i * spacing
        out[it["id"]] = p
    return out


def row_positions(start, step, count):
    return [[start[i] + step[i] * k for i in range(3)] for k in range(count)]


def grid_positions(start, step_row, step_col, rows, cols):
    out = []
    for r in range(rows):
        for c in range(cols):
            out.append([start[i] + step_row[i] * r + step_col[i] * c
                        for i in range(3)])
    return out


# ---------------------------------------------------------------------------
# Collision-aware placement & scatter (8.2)
# ---------------------------------------------------------------------------

def find_free_space(size, region, occupied, up=2, near=None, step=None):
    """First center where an AABB of ``size`` fits inside ``region`` without
    overlapping any ``occupied`` AABB. Searches a grid; if ``near`` is given the
    grid is tried nearest-first. Returns a center [x,y,z] or None."""
    ax = _horizontal_axes(up)
    up_val = region[0][up] + size[up] / 2.0
    half = {a: size[a] / 2.0 for a in ax}
    if step is None:
        step = {a: max(size[a], 0.1) for a in ax}
    else:
        step = {ax[0]: step[0], ax[1]: step[1]}
    lo = {a: region[0][a] + half[a] for a in ax}
    hi = {a: region[1][a] - half[a] for a in ax}
    cands = []
    a0 = lo[ax[0]]
    while a0 <= hi[ax[0]] + 1e-9:
        a1 = lo[ax[1]]
        while a1 <= hi[ax[1]] + 1e-9:
            cands.append((a0, a1))
            a1 += step[ax[1]]
        a0 += step[ax[0]]
    if near is not None:
        cands.sort(key=lambda c: (c[0] - near[ax[0]]) ** 2 + (c[1] - near[ax[1]]) ** 2)
    for a0, a1 in cands:
        center = [0.0, 0.0, 0.0]
        center[ax[0]], center[ax[1]], center[up] = a0, a1, up_val
        box = aabb_from_center_size(center, size)
        if not any(aabb_overlap(box, o) for o in occupied):
            return center
    return None


def scatter_poses(count, size, region, occupied=None, min_spacing=0.0, seed=None,
                  random_yaw=True, up=2, max_tries=200):
    """Randomly place ``count`` AABBs of ``size`` in ``region`` by rejection
    sampling: no overlap with ``occupied`` or each other, honoring ``min_spacing``
    between centers. Deterministic given ``seed``. Returns a list of
    ``{"position", "yaw"}`` (None where placement failed)."""
    rng = random.Random(seed)
    occupied = list(occupied or [])
    ax = _horizontal_axes(up)
    up_val = region[0][up] + size[up] / 2.0
    placed_centers = []
    poses = []
    for _ in range(count):
        pose = None
        for _t in range(max_tries):
            center = [0.0, 0.0, 0.0]
            center[up] = up_val
            for a in ax:
                center[a] = rng.uniform(region[0][a] + size[a] / 2.0,
                                        region[1][a] - size[a] / 2.0)
            box = aabb_from_center_size(center, size)
            if any(aabb_overlap(box, o) for o in occupied):
                continue
            if min_spacing > 0 and any(
                    math.dist([center[a] for a in ax], [pc[a] for a in ax]) < min_spacing
                    for pc in placed_centers):
                continue
            yaw = rng.uniform(-math.pi, math.pi) if random_yaw else 0.0
            occupied.append(box)
            placed_centers.append(center)
            pose = {"position": [round(v, 5) for v in center], "yaw": round(yaw, 5)}
            break
        poses.append(pose)
    return poses


# ---------------------------------------------------------------------------
# World validation (8.3)
# ---------------------------------------------------------------------------

def _entry_aabb(e):
    if e.get("position") is None or e.get("size") is None:
        return None
    return aabb_from_center_size(e["position"], e["size"])


def validate_world(entries, floor=0.0, up=2, coordinate_system="ENU",
                   float_tol=0.02, arena=None):
    """Static lint over a catalog. ``entries`` are dicts with position, size,
    kind ('dynamic'|'static'), def, name, has_physics, has_bounding, base_type.
    Returns a list of ``{severity, code, message, nodes}``."""
    issues = []

    def add(sev, code, msg, nodes):
        issues.append({"severity": sev, "code": code, "message": msg, "nodes": nodes})

    if coordinate_system != "ENU":
        add("info", "non_enu",
            f"World coordinate system is {coordinate_system}, not ENU (up axis is "
            f"index {up}); verify spatial assumptions.", [])

    # duplicate DEF names
    defs = {}
    for e in entries:
        d = e.get("def")
        if d:
            defs.setdefault(d, []).append(_label(e))
    for d, who in defs.items():
        if len(who) > 1:
            add("warning", "duplicate_def", f"DEF name '{d}' used {len(who)} times", who)

    boxes = []
    for e in entries:
        box = _entry_aabb(e)
        label = _label(e)

        if e.get("kind") == "dynamic" and e.get("has_bounding") is False:
            add("error", "missing_bounding",
                "dynamic object has Physics but no boundingObject", [label])
        if e.get("has_bounding") and e.get("has_physics") is False \
                and e.get("kind") == "static" and e.get("base_type") == "Solid" \
                and e.get("expect_dynamic"):
            add("info", "static_with_bounding",
                "object has a boundingObject but no Physics (stays static)", [label])

        if box is not None:
            bottom = box[0][up]
            if bottom < floor - 1e-4:
                add("warning", "below_floor",
                    f"object extends {round(floor - bottom, 4)} m below the floor",
                    [label])
            if arena is not None and e.get("position") is not None:
                pos = e["position"]
                if not all(arena[0][i] <= pos[i] <= arena[1][i] for i in range(3)):
                    add("warning", "outside_arena",
                        "object is outside the arena region", [label])
            boxes.append((e, box, label))

    # floating dynamic objects: bottom above floor with no support beneath
    for e, box, label in boxes:
        if e.get("kind") != "dynamic":
            continue
        bottom = box[0][up]
        if bottom <= floor + float_tol:
            continue
        supported = False
        ax = _horizontal_axes(up)
        for e2, box2, _ in boxes:
            if e2 is e:
                continue
            if box2[1][up] <= bottom + float_tol and box2[1][up] >= bottom - float_tol \
                    and all(box2[0][a] <= box[1][a] and box[0][a] <= box2[1][a] for a in ax):
                supported = True
                break
        if not supported:
            add("warning", "floating",
                f"dynamic object floats {round(bottom - floor, 4)} m above the floor "
                f"with no support beneath", [label])

    # overlapping pairs
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if aabb_overlap(boxes[i][1], boxes[j][1], eps=1e-4):
                add("warning", "overlap",
                    "objects have overlapping bounding boxes",
                    [boxes[i][2], boxes[j][2]])

    order = {"error": 0, "warning": 1, "info": 2}
    issues.sort(key=lambda x: order.get(x["severity"], 3))
    return issues


def _label(e):
    return e.get("name") or e.get("def") or e.get("id")


# ---------------------------------------------------------------------------
# Snapshot diff (7.6)
# ---------------------------------------------------------------------------

def diff_snapshots(a, b, move_tol=1e-3, rot_tol=1e-3):
    """Diff two snapshots keyed by id -> {name, position, yaw}. Reports added,
    removed, moved (displacement > move_tol) and rotated (|Δyaw| > rot_tol)."""
    ids_a, ids_b = set(a), set(b)
    added = [{"id": i, "name": b[i].get("name")} for i in sorted(ids_b - ids_a)]
    removed = [{"id": i, "name": a[i].get("name")} for i in sorted(ids_a - ids_b)]
    moved, rotated = [], []
    for i in sorted(ids_a & ids_b):
        pa, pb = a[i].get("position"), b[i].get("position")
        if pa and pb:
            d = math.dist(pa, pb)
            if d > move_tol:
                moved.append({"id": i, "name": b[i].get("name"),
                              "displacement": round(d, 5),
                              "from": pa, "to": pb})
        ya, yb = a[i].get("yaw"), b[i].get("yaw")
        if ya is not None and yb is not None and abs(yb - ya) > rot_tol:
            rotated.append({"id": i, "name": b[i].get("name"),
                            "delta_yaw": round(yb - ya, 5)})
    return {"added": added, "removed": removed, "moved": moved, "rotated": rotated,
            "summary": {"added": len(added), "removed": len(removed),
                        "moved": len(moved), "rotated": len(rotated)}}


# ---------------------------------------------------------------------------
# Top-down vector map (7.4)
# ---------------------------------------------------------------------------

def svg_scene_map(entries, up=2, width=640, height=640, pad=24):
    """Render a top-down SVG map of the catalog: AABB footprints + labels,
    projecting onto the two horizontal axes. Pure string output (no rendering)."""
    ax = _horizontal_axes(up)
    foots = []
    for e in entries:
        box = _entry_aabb(e)
        if box is None:
            continue
        foots.append((e, box))
    if not foots:
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
                f'height="{height}"><text x="{pad}" y="{pad}">empty scene</text></svg>')
    xs = [b[0][ax[0]] for _, b in foots] + [b[1][ax[0]] for _, b in foots]
    ys = [b[0][ax[1]] for _, b in foots] + [b[1][ax[1]] for _, b in foots]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    spanx = max(maxx - minx, 1e-6)
    spany = max(maxy - miny, 1e-6)
    scale = min((width - 2 * pad) / spanx, (height - 2 * pad) / spany)

    def sx(x):
        return pad + (x - minx) * scale

    def sy(y):  # invert so +north is up
        return height - pad - (y - miny) * scale

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
             f'height="{height}" font-family="sans-serif">',
             f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8f8f8"/>']
    for e, box in foots:
        x0, x1 = sx(box[0][ax[0]]), sx(box[1][ax[0]])
        y0, y1 = sy(box[1][ax[1]]), sy(box[0][ax[1]])
        w = max(x1 - x0, 2)
        h = max(y1 - y0, 2)
        col = e.get("color")
        fill = ("rgb(%d,%d,%d)" % tuple(int(max(0, min(1, c)) * 255) for c in col)
                if col and len(col) == 3 else "#8ab")
        parts.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{w:.1f}" '
                     f'height="{h:.1f}" fill="{fill}" fill-opacity="0.5" '
                     f'stroke="#334" stroke-width="1"/>')
        parts.append(f'<text x="{(x0 + x1) / 2:.1f}" y="{(y0 + y1) / 2:.1f}" '
                     f'font-size="11" text-anchor="middle">{_esc(_label(e))}</text>')
    parts.append('</svg>')
    return "\n".join(parts)


def _esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))
