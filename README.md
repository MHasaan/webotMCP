# Webots MCP

Full-access MCP server for the [Webots](https://cyberbotics.com) robot simulator —
modeled on Unity MCP. Lets an AI assistant see, understand, and modify a running
simulation: scene-tree inspection/editing, node spawning/deletion, viewport
screenshots, simulation control, generic control of **any** robot (motors, sensors,
cameras, LEDs, motions), and arbitrary code execution inside Webots.

## Architecture

```
Claude (MCP client)
   │ stdio (MCP protocol)
Python MCP server (server/main.py, FastMCP)
   │ TCP JSON frames, localhost:10022
mcp_bridge — Supervisor controller inside Webots (full scene/simulation API)
   │ localhost:10023
mcp_robot  — generic agent controller, attachable to any robot (auto-discovers devices)
```

## Setup

1. `pip install -r requirements.txt`
2. Register the server with Claude Code (use this repo's actual path):
   ```
   claude mcp add webots -- python <path-to-this-repo>/server/main.py
   ```
   or add to `.mcp.json`:
   ```json
   { "mcpServers": { "webots": { "command": "python", "args": ["<path-to-this-repo>/server/main.py"] } } }
   ```
3. Get a world with the bridge running — either:
   - ask for `launch_webots` (opens the bundled `worlds/demo.wbt`), or
   - for your own project: `install_bridge_into_world("path\\to\\your.wbt")`, then open it in Webots.

## Whole-software control

Beyond the loaded world, the MCP controls the Webots application itself:

- **Any world, any project**: `launch_webots` auto-installs the bridge into worlds
  that don't have it (backup created). Works on Webots' own sample worlds too
  (`list_sample_worlds`).
- **Full asset library**: `search_protos` / `get_proto_info` index all ~850 official
  PROTOs (robots, furniture, environments, appearances); `add_proto_to_world`
  declares one so `spawn_node('Nao { }')` etc. works after a reload.
- **Projects**: `create_project` scaffolds a new Webots project; `create_controller`
  writes robot controllers.
- **Recording**: `start/stop_movie_recording` (.mp4), `start/stop_animation_recording`
  (interactive HTML5), `export_screenshot` to file.
- **Application settings**: `get_webots_preferences` / `set_webots_preference`
  (registry-backed: startup mode, python command, rendering options...).
- **Process control**: launch (windowed/fullscreen/minimized/no-rendering), quit,
  console capture, docs search.

## Dynamic scene observation

The MCP sees the scene **in motion**, not just as snapshots:

- `watch_simulation(duration_s)` — run the sim and get a motion digest: every
  object's trajectory (positions over time), displacement, path length, top speed,
  plus **interaction events** (contact_start/contact_end: who touched whom, when,
  where — attributed by matching contact points across objects).
- `start_tracking` / `get_object_trajectories` / `get_interactions` / `stop_tracking`
  — record continuously while you drive robots or apply forces.
- `capture_sequence(steps, frames, follow=...)` — filmstrip of the 3D view as the
  sim advances; `follow` keeps a moving object in frame.
- `set_viewpoint(look_at=[x,y,z])` — aim the camera at any point (orientation is
  computed; Webots cameras look along +x with +z up).

## Scene understanding (Unity-MCP-style)

- `screenshot_multiview(target=..., batch='surround'|'orbit')` — capture 6 canonical
  views (or an azimuth × elevation orbit grid) around any node or the whole scene in
  one call, each returned inline with its angle caption; viewpoint restored after.
- `get_viewport_screenshot(view_target=..., view_position=[x,y,z])` — positioned
  one-shot capture: aim at a node (auto-framed from scene bounds) or from an exact
  position, without permanently moving the camera.
- `find_nodes(query, base_type)` — search the scene by name/DEF/type substring;
  returns summaries with world positions.
- `get_scene_tree(page_size=..., parent=..., cursor=...)` — paged one-level listing
  for large worlds (returns `next_cursor`), in addition to the recursive summary.
- `get_scene_bounds()` — center + radius of the dynamic part of the scene.
- `batch_execute([{action, params}, ...])` — run many scene/simulation commands in
  one call (bulk spawning, mass field edits) with per-command results and
  `stop_on_error` control.
- **MCP resources** — read-only live state at `webots://simulation`,
  `webots://scene`, `webots://robots`, and `webots://scene/{node}`.

## Ground-truth perception

Webots cameras can report *what they see* — no ML needed:

- `get_camera_recognition(robot)` — labeled detections from a robot camera: model
  name, node id, 3D pose relative to the camera, physical size, pixel bounding box,
  colors. Run `enable_camera_recognition(robot)` once first (adds the Recognition
  node via the supervisor). Objects need a non-empty `recognitionColors` field.
- `get_segmentation_image(robot)` — per-object color mask image
  (`enable_camera_recognition(robot, segmentation=True)` first).
- `get_depth_image(robot)` — RangeFinder depth map as an image + distance stats.
- `get_radar_targets(robot)` — radar detections (distance, azimuth, speed, power).

## Scene authoring extras

- `get_node_string(node)` — export any node's full "source" (all field values);
  `clone_node(node, new_def, position)` — duplicate it in one call.
- `get_node_pose(node, relative_to=...)` — pose in another node's frame (e.g. cup
  relative to gripper); `include_center_of_mass=True` adds CoM + static balance.
- `get_selected_node()` — the node the user clicked in the Webots GUI: lets a human
  point at an object for the assistant.
- `world_reload()` / `get_recording_status()` — reload world; poll movie encoding.
- `save_checkpoint(name)` / `restore_checkpoint(name)` — snapshot all dynamic
  objects and rewind: try an action, undo, try again.
- `set_joint_position(joint, pos)` — pose articulated joints through the supervisor,
  no motors/controllers needed (e.g. pose an arm for a screenshot).
- `insert_field_item` / `remove_field_item` — full MF-field editing (append points
  to coordinate arrays, remove children by index...).
- `frame_node(node)` — Webots' built-in "move viewpoint to object" fast path.

## Robot control extras

- `configure_motor` — acceleration limits, available force/torque, PID gains
  `[kp, ki, kd]`, or direct force/torque actuation (bypass position control).
- `get_motor_state(..., include_feedback=True)` — measured force/torque feedback.
- `get_lidar_summary` — now includes a polar occupancy digest (nearest obstacle per
  sector, degrees, null = clear) and optional 3D point cloud.
- `export_urdf(robot)` — the robot's kinematic model as URDF.
- `send_message` / `get_messages` — inter-robot radio (Emitter/Receiver), with
  signal strength and direction to sender.
- `set_connector(lock=...)` / `vacuum_gripper(on=...)` — docking and suction grasping.
- `speak(text)` — robot text-to-speech; `set_brake(damping)`; `display_draw(commands)`
  — draw text/shapes on robot Display devices; `get_battery`; `robot_custom_data`.
- `set_node_visibility` — hide/show objects per viewer (declutter screenshots).
- `get_node_proto` — introspect a PROTO instance's parameters and derivation chain.

## Diagnostics, tool groups & tests

- `preflight()` — one call that health-checks the stack (Webots process, bridge
  port, round-trip latency, sim state, agents) and tells you the fix for anything
  failing. Run it first when something misbehaves.
- `list_tool_groups()` / `manage_tool_groups(group, enabled)` — disable tool groups
  you don't need (`observe`, `app`, `assets`) to keep the tool list lean; `core`
  stays on.
- Tests: `pip install pytest`, then `cd server && python -m pytest tests -q`
  (fake-bridge unit tests for every tool module + TCP frame-protocol tests;
  no Webots required).

## Typical workflow

1. `get_simulation_state` — confirm the bridge is connected
2. `get_scene_tree` (paged for big worlds) + `get_viewport_screenshot` /
   `screenshot_multiview` — see and understand the scene; `find_nodes` to search it
3. Edit: `spawn_node`, `move_node`, `set_node_field`, `delete_node`, `set_viewpoint`
   (`batch_execute` for bulk edits)
4. Control a robot: `list_robots` → `attach_mcp_controller(robot)` →
   `get_robot_devices` → `set_motor` / `get_camera_image` / `get_sensor_values`
5. Drive time: `set_simulation_mode` (pause/realtime/fast), `step_simulation`,
   `reset_simulation`
6. Anything else: `execute_supervisor_code` / `execute_robot_code`

## Notes & limitations

- The Webots installation is auto-detected (WEBOTS_HOME env var → Windows registry →
  `webots` on PATH → standard install locations on Windows/Linux/macOS). Set
  `WEBOTS_HOME` explicitly only for unusual install paths.
- Ports are configurable: `WEBOTS_MCP_PORT` (default 10022) and
  `WEBOTS_MCP_AGENT_PORT` (default 10023).
- Pause is implemented by the bridge not stepping (the GUI shows "0.00x"). While
  paused, scene/simulation tools work and `step_simulation` is deterministic, but
  per-robot commands may time out — step or resume first.
- Pausing from the **Webots GUI** blocks the bridge until resumed; prefer the
  `set_simulation_mode` tool.
- Viewport screenshots capture the 3D view as rendered — keep the Webots window
  reasonably large and unobstructed for useful images.
- `attach_mcp_controller` replaces the robot's own controller (the previous name is
  returned; restore it with `set_node_field(robot, "controller", old_name)` +
  `execute_supervisor_code` restartController if needed).
- If Webots was force-killed, stale `mcp_bridge.py` python processes can hold port
  10022 and freeze new sessions — kill leftover `python.exe mcp_bridge.py` processes.

## Layout

- `server/` — FastMCP server (`main.py`), TCP client (`connection.py`), tool modules (`tools/`)
- `controllers/mcp_bridge/` — supervisor bridge controller (runs inside Webots)
- `controllers/mcp_robot/` — generic robot agent controller
- `worlds/demo.wbt` — demo world with bridge + a simple camera/lidar-less wheeled robot
