"""World-building tools (P8) and map/diff (P7.4/7.6).

Reliable authoring: placement that respects object sizes (no more objects spawned
intersecting or floating), collision-aware scatter, a static world validator, a
top-down map, and scene snapshot/diff. All geometry math runs in the bridge's
pure scene_math module; these tools are thin, well-documented entry points.
"""

from typing import Optional


def register(mcp, bridge):
    @mcp.tool()
    def drop_to_ground(node: str, floor: float = 0.0, gap: float = 0.0,
                       up: int = 2) -> dict:
        """Lower a node so its bounding-box bottom rests on the highest support
        beneath it (another object's top, else the floor plane at 'floor'). Fixes
        floating or interpenetrating placements. up = vertical axis index
        (2=z ENU default, 1=y NUE)."""
        return bridge.command("drop_to_ground",
                              {"node": node, "floor": floor, "gap": gap, "up": up})

    @mcp.tool()
    def place_on(node: str, target: str, offset: Optional[list] = None,
                 gap: float = 0.0, up: int = 2) -> dict:
        """Place 'node' centered on top of 'target', resting on its top face.
        offset=[dx,dy] shifts it horizontally; gap adds clearance."""
        return bridge.command("place_on", {"node": node, "target": target,
                                            "offset": offset, "gap": gap, "up": up})

    @mcp.tool()
    def align_objects(nodes: list, axis: int, mode: str = "center") -> dict:
        """Give every listed node the same coordinate on 'axis' (0=x,1=y,2=z).
        mode='min'|'center'|'max' picks the reference from the group's extents."""
        return bridge.command("align_objects",
                              {"nodes": nodes, "axis": axis, "mode": mode})

    @mcp.tool()
    def distribute_objects(nodes: list, axis: int, spacing: Optional[float] = None,
                           extent: Optional[float] = None) -> dict:
        """Evenly space nodes along 'axis' (ordered by current coordinate). Give
        'spacing' for a fixed gap, 'extent' to spread over a total span, or
        neither to spread across the current first..last range."""
        return bridge.command("distribute_objects",
                              {"nodes": nodes, "axis": axis, "spacing": spacing,
                               "extent": extent})

    @mcp.tool()
    def place_row(node_string: str, start: list, step: list, count: int) -> dict:
        """Spawn 'count' copies of a Webots node string in a line: copy k goes at
        start + k*step (both [x,y,z])."""
        return bridge.command("place_row", {"node_string": node_string,
                                            "start": start, "step": step,
                                            "count": count})

    @mcp.tool()
    def place_grid(node_string: str, start: list, step_row: list, step_col: list,
                   rows: int, cols: int) -> dict:
        """Spawn a rows x cols grid of a Webots node string: cell (r,c) at
        start + r*step_row + c*step_col."""
        return bridge.command("place_grid", {"node_string": node_string,
                                             "start": start, "step_row": step_row,
                                             "step_col": step_col, "rows": rows,
                                             "cols": cols})

    @mcp.tool()
    def find_free_space(size: list, region: dict, near: Optional[list] = None,
                        up: int = 2) -> dict:
        """Find the first pose where an axis-aligned box of 'size'=[x,y,z] fits
        inside region={'min':[x,y,z],'max':[x,y,z]} without overlapping any
        existing object. 'near'=[x,y,z] prefers spots nearest that point."""
        return bridge.command("find_free_space",
                              {"size": size, "region": region, "near": near,
                               "up": up})

    @mcp.tool()
    def scatter_objects(count: int, size: list, region: dict,
                        node_string: Optional[str] = None, min_spacing: float = 0.0,
                        seed: Optional[int] = None, random_yaw: bool = True,
                        up: int = 2) -> dict:
        """Randomly place 'count' boxes of 'size' in 'region' without overlaps,
        honoring 'min_spacing' between centers. With 'node_string' it spawns a
        copy at each pose; without it just returns the poses. A fixed 'seed'
        makes the layout reproducible (domain randomization). Returns per-object
        poses (null where placement failed)."""
        return bridge.command("scatter_objects",
                              {"count": count, "size": size, "region": region,
                               "node_string": node_string, "min_spacing": min_spacing,
                               "seed": seed, "random_yaw": random_yaw, "up": up})

    @mcp.tool()
    def validate_world(floor: float = 0.0, arena: Optional[dict] = None,
                       up: int = 2) -> dict:
        """Static lint over the world before running it: flags overlapping
        bounding boxes, objects below the floor or floating with no support,
        dynamic objects missing a boundingObject, duplicate DEF names, non-ENU
        coordinate systems, and (with 'arena'={'min','max'}) objects outside the
        arena. Returns issues sorted with errors first."""
        return bridge.command("validate_world",
                              {"floor": floor, "arena": arena, "up": up})

    @mcp.tool()
    def get_scene_map(width: int = 640, height: int = 640, up: int = 2) -> dict:
        """Top-down vector SVG map of the world: every object's bounding-box
        footprint drawn to scale and labeled. Crisp, tiny and unambiguous way to
        grasp a whole layout without a screenshot."""
        return bridge.command("get_scene_map",
                              {"width": width, "height": height, "up": up})

    @mcp.tool()
    def snapshot_scene(name: str = "default") -> dict:
        """Store the current object layout (poses + sizes) under 'name' so you can
        later diff_scene against it — 'what did my last edit/run actually change?'.
        Complements save_checkpoint, which restores state but doesn't explain it."""
        return bridge.command("snapshot_scene", {"name": name})

    @mcp.tool()
    def diff_scene(name_a: str, name_b: str = "now") -> dict:
        """Report what changed between two snapshots (name_b='now' = live scene):
        objects added, removed, moved (with displacement) and rotated."""
        return bridge.command("diff_scene", {"name_a": name_a, "name_b": name_b})
