# Webots MCP

Full-access MCP server for the [Webots](https://cyberbotics.com) robot simulator.
Lets an AI assistant see, understand, and modify a running
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

### DEF names and device names

`Node.getFromProtoDef()` searches DEF declarations inside a PROTO. A device's
`name` field is a different identifier. Check the model before using that API;
do not repeatedly probe nonexistent DEFs during scene observation.

For example, a PR2 vacuum integration queried `r_vacuum_gripper` and
`l_vacuum_gripper` as DEFs even though the model declared only device names.
Both returned null, but together took about 47–52 seconds per scene read on
the measured Webots R2025a setup. Omitting those unavailable optional lookups
reduced scene-code execution to approximately 0.04 seconds. Preserve required
measurements and validate the result; substituting wrist coordinates would
misrepresent the vacuum-head pose.

Use known node handles where available. Any handles cached by integration code
must be refreshed after world reload or relevant PROTO regeneration.

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
  `webots://scene`, `webots://robots`, `webots://scene/{node}`, and
  `webots://conventions` (units + this world's up-axis / coordinate system).

## Semantic scene model (`scene_model` group)

A ready-made semantic picture of the world instead of reconstructing it field by
field:

- `get_object_catalog()` — one inventory row per Solid/Robot: position, yaw, size
  (from the boundingObject), mass, static|dynamic, color, parent. Paged.
- `get_object_properties(node)` — deep single-object composite: full world AABB,
  velocity, center of mass, static balance, contact partners, devices.
- `get_contact_points(node)` / richer `get_node_details` (velocity, contact count,
  CoM) / `reset_node_physics(node)` — stop a single runaway object.
- Spatial queries: `find_nodes_near`, `objects_in_region`, `check_overlap`,
  `find_overlapping_pairs`, `get_spatial_relations` (touching / on_top_of / inside /
  near) — answer "what's on the table?" in one call.

## World building (`world_build` group)

Reliable authoring — no more objects spawned floating or intersecting:

- `drop_to_ground(node)` / `place_on(node, target)` — rest an object on the support
  beneath it or centered on another's top face.
- `align_objects` / `distribute_objects` / `place_row` / `place_grid` — bulk layout.
- `find_free_space(size, region)` / `scatter_objects(count, size, region, seed=...)`
  — collision-aware placement + reproducible domain randomization.
- `validate_world()` — static lint: overlaps, below-floor / floating objects,
  dynamic nodes missing a boundingObject, duplicate DEFs, non-ENU warning.
- `get_scene_map()` — top-down labeled vector SVG of the whole layout.
- `snapshot_scene(name)` / `diff_scene(a, b='now')` — "what did my last edit change?"
  (added / removed / moved / rotated).

## Run & understand (`analyze` group)

Turn "run it and see" into one structured, deterministic call:

- `wait_until(condition, timeout_s)` — step until an event fires instead of
  guessing a duration. Condition DSL: `distance`, `contact`, `speed`, `position`,
  `sim_time`, plus `any`/`all` combinators. Returns when/where it fired.
- `run_experiment(duration_s, watch=, until=, restore=)` — one iteration in one
  call: checkpoint → track → run (for a duration or `until` a condition) → return a
  run report (per-object motion + displacement, interaction timeline, scene diff vs
  start, anomalies) → rewind (`auto` / `keep` / `on_anomaly`). Optional final image.
- `detect_anomalies()` — over the currently-tracked motion: NaN/inf blow-ups,
  teleports, runaway velocity, below-floor / out-of-arena — each with a fix hint.
- `get_console_diagnostics()` / `get_controller_logs(robot)` — classify the Webots
  console (ODE/physics, controller tracebacks, missing assets, parse warnings) with
  fixes, and split logs per controller.

## Authoring & automation (`authoring` group)

Session → reusable assets and reproducible code:

- `generate_world_script(format='python'|'json'|'wbt')` — export the current scene
  as a standalone Supervisor script, a declarative scenario, or a clean world file.
- `extract_proto_from_node(node, proto_name)` — turn a tuned node into a reusable
  PROTO (exposes translation/rotation/name) written to the project protos/.
- `create_world(name, template='empty'|'indoor_room'|'outdoor_flat')` — scaffold a
  ready .wbt (base nodes only); `backup_world` / `list_world_backups` /
  `restore_world_backup` — managed edit history.
- `list_appearances` / `set_appearance(node, base_color=)` / `set_recognition_colors`
  — style shapes and make objects visible to camera recognition in one call.
- `configure_lighting(preset='indoor'|'outdoor'|'studio'|'night')` — fix the top
  cause of useless screenshots and failed recognition.
- `record_states(duration_s, nodes=, fields=)` — record motion to a CSV file (with
  summary stats) for offline analysis and regression tests.

## Experiments & scenarios (`experiments` group)

- `configure_physics(recipe=|gravity=|basic_time_step=|random_seed=|fps=)` — set
  WorldInfo without field paths; recipes: earth/moon/mars/zero_g/slow_motion/
  high_fidelity. Set `random_seed` (then reset) for reproducible physics.
- `profile_simulation(duration_s)` — achieved sim/wall real-time factor;
  `profile_from_log(path)` parses a `--log-performance` file.
- `compare_runs(report_a, report_b)` — per-object end-position divergence between
  two run_experiment reports ("it fails one time in five").
- `save_scenario(name)` / `load_scenario(file, seed=)` / `run_scenario(file, runs=, seeds=)` — capture the world as a versionable scenario.json (objects + physics +
  wait_until conditions + duration), rebuild it deterministically, and batch
  build→run→report across seeds with a divergence comparison.

## Developer experience (`dx` group)

- `get_viewport_labels()` — project every object into the CURRENT 3D view and get
  pixel-space labels (near→far, in-frame); overlay them on get_viewport_screenshot.
- `describe_sample(path)` / `list_sample_worlds(query)` — summarize any world file
  (version, timestep, coordinate system, robots + controllers) and browse the
  installed Webots sample/benchmark worlds.
- `update_world_file(path)` / `clear_webots_cache()` — migrate old worlds and clear
  the asset cache via the Webots CLI.
- **Workflow prompts** — `inspect_scene`, `robot_bringup`, `record_demo`,
  `experiment_loop`: proven multi-step recipes the client can invoke directly.
- **Resource** `webots://console` — the last 50 lines of the Webots console.

## Extern controllers & asset import (`extern` group)

Debug the user's real code and pull in assets without touching the world file:

- `run_extern_controller(controller_path, robot)` / `get_extern_controller_output`
  / `stop_extern_controller` — set the robot's controller to `<extern>` and run any
  controller file as an external process, capturing its stdout: the
  edit→run→read→iterate controller-development loop.
- `create_supervisor_script(name, code)` / `run_supervisor_script(name, robot)` —
  scaffold and run full-speed Supervisor automation as an extern controller.
- `import_cad_model(url, physics=, bounding_box=)` — drop an .obj/.dae mesh into the
  scene as a CadShape Solid.
- `convert_proto(proto_file)` — flatten an opaque PROTO to base nodes (headless).

## High-level robot behaviors (`behavior` group)

One tool call = one closed-loop behavior (no micro-managing wheel velocities):

- `drive_robot(robot, linear, angular, duration_s)` — differential-drive
  convenience; auto-pairs wheel motors, converts (v, ω) → wheel speeds, runs and
  stops, returns start/end pose + distance.
- `move_robot_to(robot, target, tolerance)` — closed-loop go-to-point via a
  heading controller over ground-truth pose (straight-line reactive, not a
  planner).
- `build_occupancy_grid(robot, resolution, size)` — rasterize a lidar scan + pose
  into an ASCII occupancy map + world-frame obstacle list.

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

- Commands are never replayed automatically after a lost reply. An “outcome
  unknown” error means the operation may already have executed; timeout does not
  cancel it. The next command reconnects. See [transport reliability and audit](docs/transport-reliability.md)
  for deadline, reload, agent, and step-count behavior.
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
  reasonably large and unobstructed for useful images, may need to change view port
  camera to a better position manually for better scene context and understanding.
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
