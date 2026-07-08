"""Scene-tree inspection and editing tools."""

from typing import Optional


def register(mcp, bridge):
    @mcp.tool()
    def get_scene_tree(max_depth: int = 3, include_fields: bool = False,
                       parent: Optional[str] = None, page_size: Optional[int] = None,
                       cursor: int = 0) -> dict:
        """Get the Webots scene tree (all nodes in the world). Returns node type,
        DEF name, robot name, translation and children. Start with the default
        summary; set include_fields=True or use get_node_details for more.
        For LARGE worlds, page instead of dumping everything: pass page_size (and
        optionally parent + cursor) to list direct children one level at a time;
        the response includes next_cursor while more remain."""
        return bridge.command("get_scene_tree",
                              {"max_depth": max_depth, "include_fields": include_fields,
                               "parent": parent, "page_size": page_size, "cursor": cursor})

    @mcp.tool()
    def find_nodes(query: str = "", base_type: Optional[str] = None,
                   max_results: int = 20) -> dict:
        """Search the scene for nodes by case-insensitive substring against DEF name,
        'name' field and type. Optional base_type filter ('Robot', 'Solid', ...).
        Returns node summaries with world positions."""
        return bridge.command("find_nodes", {"query": query, "base_type": base_type,
                                             "max_results": max_results})

    @mcp.tool()
    def get_scene_bounds() -> dict:
        """Get the center and radius of the interesting part of the scene (dynamic
        objects and robots) — useful for framing viewpoints and captures."""
        return bridge.command("get_scene_bounds")

    @mcp.tool()
    def get_node_details(node: str) -> dict:
        """Get every field of a node plus its world position/orientation.
        'node' can be a DEF name, a numeric node id, or a robot's name field."""
        return bridge.command("get_node_details", {"node": node})

    @mcp.tool()
    def set_node_field(node: str, field: str, value, index: Optional[int] = None) -> dict:
        """Set any field of any node. Value type must match the field: bool, int,
        float, string, or list of floats for vec2/vec3/rotation/color.
        For multi-valued (MF) fields, pass 'index'."""
        return bridge.command("set_node_field",
                              {"node": node, "field": field, "value": value, "index": index})

    @mcp.tool()
    def get_node_field(node: str, field: str, max_items: int = 1000) -> dict:
        """Read one field of a node in full (no truncation) — use for long MF fields
        like coordinate arrays. Errors list the node's available fields."""
        return bridge.command("get_node_field",
                              {"node": node, "field": field, "max_items": max_items})

    @mcp.tool()
    def set_velocity(node: str, linear: Optional[list] = None,
                     angular: Optional[list] = None) -> dict:
        """Set a physics-enabled node's velocity: linear [vx,vy,vz] m/s and/or
        angular [wx,wy,wz] rad/s (world frame)."""
        return bridge.command("set_velocity",
                              {"node": node, "linear": linear, "angular": angular})

    @mcp.tool()
    def apply_force(node: str, force: Optional[list] = None,
                    torque: Optional[list] = None, offset: Optional[list] = None,
                    relative: bool = False, duration_steps: int = 1) -> dict:
        """Apply a force [fx,fy,fz] (N) and/or torque (N·m) to a physics-enabled node.
        A force lasts ONE physics step (~16-32ms); use duration_steps to sustain it
        (advances the simulation while applying). For an instant impulse-like shove
        prefer set_velocity. offset = application point in the node's frame;
        relative=True interprets vectors in the node's frame instead of world."""
        return bridge.command("apply_force", {"node": node, "force": force,
                                              "torque": torque, "offset": offset,
                                              "relative": relative,
                                              "duration_steps": duration_steps},
                              timeout=120.0)

    @mcp.tool()
    def get_node_pose(node: str, include_velocity: bool = False,
                      relative_to: Optional[str] = None,
                      include_center_of_mass: bool = False) -> dict:
        """Get a node's world-frame position (x,y,z) and orientation (3x3 rotation
        matrix, row-major), optionally with linear+angular velocity.
        relative_to = another node: also returns the 4x4 pose of 'node' expressed in
        that node's frame (e.g. cup relative to gripper). include_center_of_mass adds
        the physics center of mass and whether the object is statically balanced."""
        return bridge.command("get_node_pose",
                              {"node": node, "include_velocity": include_velocity,
                               "relative_to": relative_to,
                               "include_center_of_mass": include_center_of_mass})

    @mcp.tool()
    def get_node_string(node: str) -> dict:
        """Export a node (with ALL current field values) as a Webots node string —
        the node's 'source code'. Feed it to spawn_node to copy it elsewhere, or
        edit it and respawn."""
        return bridge.command("get_node_string", {"node": node})

    @mcp.tool()
    def clone_node(node: str, new_def: Optional[str] = None,
                   position: Optional[list] = None, parent: Optional[str] = None) -> dict:
        """Duplicate a node with all its current field values. new_def = DEF name for
        the copy; position = [x,y,z] for the copy (defaults to a small offset so it
        doesn't overlap the original); parent = insert under a different node."""
        return bridge.command("clone_node", {"node": node, "new_def": new_def,
                                             "position": position, "parent": parent})

    @mcp.tool()
    def insert_field_item(node: str, field: str, value, index: int = -1) -> dict:
        """Insert a value into a multi-valued (MF) field at index (-1 = append).
        For MF node fields (like 'children'), value is a Webots node string."""
        return bridge.command("insert_field_item",
                              {"node": node, "field": field, "value": value,
                               "index": index})

    @mcp.tool()
    def remove_field_item(node: str, field: str, index: Optional[int] = None) -> dict:
        """Remove item at index from a multi-valued (MF) field, or clear a
        single-node (SF) field when index is omitted."""
        return bridge.command("remove_field_item",
                              {"node": node, "field": field, "index": index})

    @mcp.tool()
    def save_checkpoint(name: str = "default", nodes: Optional[list] = None) -> dict:
        """Save the pose+physics state of all dynamic objects (or the given nodes)
        under a named checkpoint. Try an action, then rewind with restore_checkpoint
        — the simulation equivalent of undo. Checkpoints live until world reload."""
        return bridge.command("save_checkpoint", {"name": name, "nodes": nodes})

    @mcp.tool()
    def restore_checkpoint(name: str = "default") -> dict:
        """Rewind objects to a named checkpoint saved with save_checkpoint
        (positions, rotations and physics state; sim time keeps advancing)."""
        return bridge.command("restore_checkpoint", {"name": name})

    @mcp.tool()
    def set_joint_position(node: str, position: float, index: int = 1) -> dict:
        """Pose an articulated joint DIRECTLY through the supervisor — no motor or
        controller needed (e.g. pose a robot arm for a screenshot). 'node' must be
        the joint node itself (HingeJoint/SliderJoint/...; find via get_scene_tree).
        index: 1 (default), 2 for Hinge2Joint axis 2, 2-3 for BallJoint."""
        return bridge.command("set_joint_position",
                              {"node": node, "position": position, "index": index})

    @mcp.tool()
    def set_node_visibility(node: str, visible: bool = True,
                            from_node: Optional[str] = None) -> dict:
        """Hide/show a node for a specific viewer: the main Viewpoint by default,
        or a camera node via from_node. Useful for decluttering screenshots or
        occlusion experiments — physics is unaffected."""
        return bridge.command("set_node_visibility",
                              {"node": node, "visible": visible,
                               "from_node": from_node})

    @mcp.tool()
    def get_node_proto(node: str) -> dict:
        """Introspect a PROTO instance: parameter names/types/values and its
        derivation chain. (For the PROTO *library* docs use get_proto_info.)"""
        return bridge.command("get_node_proto", {"node": node})

    @mcp.tool()
    def frame_node(node: str) -> dict:
        """Move the 3D Viewpoint to frame a node — Webots' built-in 'move viewpoint
        to object'. Fastest way to look at something; combine with
        get_viewport_screenshot."""
        return bridge.command("frame_node", {"node": node})

    @mcp.tool()
    def get_selected_node() -> dict:
        """Get the node the USER currently has selected in the Webots GUI (scene tree
        or 3D view click) — lets a human point at an object for you."""
        return bridge.command("get_selected_node")

    @mcp.tool()
    def move_node(node: str, position: Optional[list] = None,
                  rotation: Optional[list] = None, reset_physics: bool = True) -> dict:
        """Teleport a node. position = [x, y, z]; rotation = axis-angle [x, y, z, angle].
        Physics is reset by default so the object doesn't keep old momentum."""
        return bridge.command("move_node", {"node": node, "position": position,
                                            "rotation": rotation, "reset_physics": reset_physics})

    @mcp.tool()
    def spawn_node(node_string: str, parent: Optional[str] = None,
                   field: str = "children", position: int = -1) -> dict:
        """Spawn a new node into the world from a Webots node string, e.g.
        'DEF MY_BOX Solid { translation 0 0 0.5 children [ Shape { appearance PBRAppearance { baseColor 1 0 0 } geometry Box { size 0.2 0.2 0.2 } } ] boundingObject Box { size 0.2 0.2 0.2 } physics Physics { } }'
        PROTO instances work too if declared via EXTERNPROTO in the world, e.g. 'E-puck { translation 0 0 0 }'.
        By default appends to the scene root; pass parent/field to insert elsewhere."""
        return bridge.command("spawn_node", {"node_string": node_string, "parent": parent,
                                             "field": field, "position": position})

    @mcp.tool()
    def delete_node(node: str) -> dict:
        """Delete a node from the world (by DEF name, id, or name field)."""
        return bridge.command("delete_node", {"node": node})

    @mcp.tool()
    def set_viewpoint(position: Optional[list] = None, orientation: Optional[list] = None,
                      look_at: Optional[list] = None, follow: Optional[str] = None) -> dict:
        """Move the 3D view camera. position = [x,y,z]; look_at = [x,y,z] target the
        camera should point at (orientation computed automatically — prefer this over
        raw axis-angle orientation); follow = object name to track while it moves
        (empty string stops following). Combine with get_viewport_screenshot to look
        around the scene."""
        return bridge.command("set_viewpoint", {"position": position,
                                                "orientation": orientation,
                                                "look_at": look_at, "follow": follow})
