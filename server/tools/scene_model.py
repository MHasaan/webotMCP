"""Semantic scene-model tools (P7) and contact/physics primitives (P2).

These give the LLM a ready-made semantic picture of the world — an object
inventory with sizes/masses/colors, deep per-object properties, and spatial
relationships — instead of reconstructing it from raw get_scene_tree fields.
"""

from typing import Optional


def register(mcp, bridge):
    @mcp.tool()
    def get_object_catalog(max_depth: int = 3, page_size: Optional[int] = None,
                           cursor: int = 0) -> dict:
        """Inventory of every Solid/Robot in the world: one row per object with
        id, name, DEF, type, position, yaw, size (AABB extents from its
        boundingObject), mass, static|dynamic, color and parent id. The best
        first call for scene understanding and script generation. Sizes/masses
        are best-effort (null when not derivable). Page big worlds with page_size
        (+ cursor); next_cursor is returned while more remain."""
        return bridge.command("get_object_catalog",
                              {"max_depth": max_depth, "page_size": page_size,
                               "cursor": cursor})

    @mcp.tool()
    def get_object_properties(node: str) -> dict:
        """Deep single-object composite: everything from get_object_catalog plus
        full world AABB (min/max), velocity, center of mass, static balance,
        contact partners, and (for robots) the device list. Replaces several
        get_node_* calls with one. 'node' = DEF name, id, or name field."""
        return bridge.command("get_object_properties", {"node": node})

    @mcp.tool()
    def get_contact_points(node: str, include_descendants: bool = False) -> dict:
        """What is this node touching right now: list of contact points
        [{point:[x,y,z], other_node_id, other_name}]. include_descendants folds
        in contacts of the node's children (e.g. a robot's whole body)."""
        return bridge.command("get_contact_points",
                              {"node": node,
                               "include_descendants": include_descendants})

    @mcp.tool()
    def reset_node_physics(node: str) -> dict:
        """Zero a single object's linear+angular velocity (e.g. stop a runaway
        ball) without resetting the whole simulation."""
        return bridge.command("reset_node_physics", {"node": node})

    @mcp.tool()
    def find_nodes_near(radius: float, node: Optional[str] = None,
                        point: Optional[list] = None) -> dict:
        """Proximity search: all catalog objects within 'radius' meters of a
        point [x,y,z] or of another node's position (that node is excluded).
        Returns nodes sorted by distance."""
        return bridge.command("find_nodes_near",
                              {"radius": radius, "node": node, "point": point})

    @mcp.tool()
    def objects_in_region(min: list, max: list) -> dict:
        """All catalog objects whose position falls inside the axis-aligned box
        from min=[x,y,z] to max=[x,y,z]."""
        return bridge.command("objects_in_region", {"min": min, "max": max})

    @mcp.tool()
    def check_overlap(node_a: str, node_b: str) -> dict:
        """Approximate axis-aligned bounding-box intersection test between two
        nodes — catches accidentally intersecting placements. Returns both AABBs
        and an overlap boolean."""
        return bridge.command("check_overlap",
                              {"node_a": node_a, "node_b": node_b})

    @mcp.tool()
    def find_overlapping_pairs() -> dict:
        """Scan the whole catalog for pairs of objects whose AABBs intersect
        (approximate) — a fast lint for objects spawned inside each other."""
        return bridge.command("find_overlapping_pairs")

    @mcp.tool()
    def get_spatial_relations(node: Optional[str] = None,
                              near_threshold: float = 0.5) -> dict:
        """Relation graph over the catalog: touching (contacts), on_top_of
        (contact + higher AABB), inside (AABB containment) and near (within
        near_threshold meters). Pass 'node' to get only its relations; omit for
        the whole scene. Answers 'what is on the table?' in one call."""
        return bridge.command("get_spatial_relations",
                              {"node": node, "near_threshold": near_threshold})
