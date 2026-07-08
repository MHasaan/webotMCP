# webotMCP Enhancement Plan

Based on a full survey of the Webots R2025a installation (`I:\Webots`): the Python controller
API (`lib/controller/python/controller/` — 40+ device/API modules), the Supervisor/Node/Field/Proto
API surface, bundled assets (`projects/` — robots, objects, appearances, samples, vehicles,
osm_importer), and docs — compared against every tool currently exposed by webotMCP.

Legend: **[S]** supervisor bridge change · **[A]** mcp_robot agent change · **[M]** MCP server tool ·
effort ≈ S/M/L.

---

## Priority 1 — Perception: ground-truth vision (highest value)

The single biggest untapped capability. Webots cameras have built-in **Recognition**:
the simulator itself reports what objects a camera sees — no ML needed.

### 1.1 `get_camera_recognition(robot, camera)` — [A][M] · M
`Camera.getRecognitionObjects()` returns, per visible object: node id, model name,
world position/orientation relative to camera, position & size **on the image** (bounding
box), and colors. Enable with `camera.recognitionEnable(ts)`; requires the camera PROTO to
have a `Recognition` node — the tool should auto-add one via the supervisor if missing.
Return a JSON detection list; optionally overlay boxes on the camera image (Pillow).
This gives the model *labeled* vision: "what does the robot see and where".

### 1.2 `get_segmentation_image(robot, camera)` — [A][M] · S
`camera.enableRecognitionSegmentation()` + `getRecognitionSegmentationImage()` — a
per-object color mask image. Return inline like `get_camera_image`.

### 1.3 `get_depth_image(robot, rangefinder)` — [A][M] · M
RangeFinder devices are currently read as raw values only via generic sensors. Add:
colorized depth map rendered to JPEG (near=white/far=black), plus min/max/mean stats.
The agent already whitelists RangeFinder in `SENSOR_TYPES` but skips it in
`get_sensor_values` — this fills the hole.

### 1.4 `get_radar_targets(robot, radar)` — [A][M] · S
`radar.getTargets()` → distance/azimuth/speed per target. Same skip-hole as above.

### 1.5 Lidar upgrade: point cloud + occupancy summary — [A][M] · M
Current tool returns a downsampled 1D range list. Add `lidar.getPointCloud()` support:
downsampled 3D points, and a compact polar occupancy summary ("obstacle at 0.4m,
bearing 30°..45°") — much more LLM-friendly than raw ranges.

## Priority 2 — Scene authoring & state management

### 2.1 `clone_node` / `get_node_string` — [S][M] · S
`node.exportString()` serializes any node (with all field values) to a Webots string;
feeding it back through `spawn_node` = copy/paste/duplicate. Also useful as
"show me this node's source". Trivial to add, big authoring win.

### 2.2 Checkpoints: `save_checkpoint` / `restore_checkpoint` — [S][M] · M
`node.saveState(name)` / `node.loadState(name)` per node + `simulationResetPhysics()`.
Bridge iterates dynamic nodes, saves all states under a named checkpoint, restores on
demand. Enables "try an action, rewind, try again" experimentation loops — the sim
equivalent of undo.

### 2.3 `set_joint_position` — [S][M] · S
`node.setJointPosition(pos, index)` poses articulated joints directly through the
supervisor — pose a robot arm for a screenshot without attaching a controller or motors.

### 2.4 MF-field editing: `insert_field_item` / `remove_field_item` — [S][M] · S
Field API has `insertMF*/removeMF/removeSF/importSFNodeFromString`; webotMCP can currently
only set existing indexes. Rounds out full scene-tree editing (e.g. append a point to a
coordinate array, remove one child by index).

### 2.5 `set_node_visibility` — [S][M] · S
`node.setVisibility(fromNode, visible)` — hide/show objects per-viewpoint/camera
(occlusion experiments, decluttering screenshots).

### 2.6 Relative poses — [S][M] · S
`node.getPose(fromNode)` — "where is the cup relative to the gripper" without math.
Add `relative_to` param on `get_node_pose`. Also expose `getCenterOfMass` and
`getStaticBalance` (is this stack of boxes stable?).

### 2.7 PROTO introspection — [S][M] · M
`node.isProto()`, `getProto()` → parameter names/values, `getFromProtoDef()` to reach
inside PROTO internals. Today PROTO instances are semi-opaque; this lets the model
inspect and tune e.g. a Nao's hidden internals or a parameterized furniture PROTO.

### 2.8 `get_selected_node` — [S][M] · S
`supervisor.getSelected()` — the node the USER clicked in the Webots GUI. Great
human-in-the-loop affordance: "make *this one* (points and clicks) red".

## Priority 3 — Robot capabilities

### 3.1 Motor control depth — [A][M] · S
Expose `setAvailableTorque/Force`, `setAcceleration`, `setControlPID`, and torque reading
(`enableTorqueFeedback`/`getTorqueFeedback`) via a `configure_motor` tool + richer
`get_motor_state`.

### 3.2 Inter-robot messaging: `send_message` / `get_messages` — [A][M] · M
Emitter/Receiver devices — send/receive on channels. Enables multi-robot coordination
scenarios driven by the model.

### 3.3 Actuator odds & ends — [A][M] · S each
- `set_connector(lock=True/False)` — docking/magnetic gripping (Connector device)
- `vacuum_gripper(on/off)` — VacuumGripper turnOn/Off + isOn
- `speaker_speak(text)` — Speaker TTS (`speaker.speak()`) and sound playback
- `set_brake(damping)` — Brake device
- `display_draw(...)` — draw text/shapes/images on robot Display devices
- battery: report via `get_sensor_values` when `batterySensorEnable`d

### 3.4 `export_urdf(robot)` — [A][M] · S
`robot.getUrdf()` — hand the robot's kinematic model to the LLM (or to external tools).
Excellent context for planning motions of arbitrary robots.

### 3.5 `get_custom_data` / `set_custom_data` — [A][M] · S
Robot custom-data channel — lightweight blackboard between the MCP and native controllers.

## Priority 4 — App-level & content

### 4.1 Movie status polling — [S][M] · S
`movieIsReady()` / `movieFailed()` → `get_recording_status` tool (stop_movie_recording
currently blocks blind).

### 4.2 `world_reload` — [S][M] · S
`supervisor.worldReload()` — cleaner than load_world(current).

### 4.3 Asset browsing upgrades — [M] · M
`projects/` contains more than PROTOs: `samples/` (demos, howto, tutorials, robotbenchmark),
`guided_tour.txt`, appearances, `resources/osm_importer` (OpenStreetMap → world generator),
`projects/vehicles` (SUMO traffic co-simulation). Add: `list_sample_worlds` category
filters (already partial), `describe_sample(path)` (parse world + its controllers),
and document osm_importer/vehicle sims in the README as advanced workflows.

### 4.4 Viewpoint niceties — [S][M] · S
`node.moveViewpoint()` is Webots' built-in "frame this node" — simpler and more robust
than the manual look_at math for the common case; add `frame_node(node)` fast path to
`set_viewpoint`.

## Priority 5 — Infrastructure hardening (Unity-MCP parity)

### 5.1 Test suite — · L
There are NO tests today. Add `server/tests/` (pytest): fake-bridge unit tests for every
tool module (the batch_execute mock pattern already proved out), frame-protocol tests for
`connection.py`, and an optional live smoke leg (`--live`) that launches the demo world
headless (`--no-rendering --minimize`) and runs ping → scene tree → screenshot → spawn →
watch. Mirrors unity-mcp's `Server/tests` + local harness.

### 5.2 Preflight & error ergonomics — · M
A `preflight` check tool (bridge reachable? sim paused? agents alive? controller stale?)
plus consistent error hints (nearest-name suggestions on node/device misses — partially
present).

### 5.3 Tool groups — · M
~45 tools and growing. Split registration into core / recording / assets / advanced
groups with a `manage_tools` enable/disable toggle (unity-mcp pattern) to keep default
context lean.

### 5.4 HTTP transport (multi-client) — · L, only if needed
Current stdio server is single-client (each client would fight over one bridge TCP port).
A shared FastMCP HTTP server with session routing would allow Claude Desktop + Code
concurrently. Defer unless multi-agent workflows materialize.

---

## Suggested implementation order

| Phase | Items | Rationale |
|-------|-------|-----------|
| 1 | 1.1, 1.2, 1.3, 2.1, 2.6, 2.8, 4.1, 4.2 | Max value / small effort; perception first |
| 2 | 2.2, 2.3, 2.4, 3.1, 3.4, 1.5, 4.4 | Authoring + robot depth |
| 3 | 3.2, 3.3, 2.5, 2.7, 1.4, 3.5, 4.3 | Breadth |
| 4 | 5.1, 5.2, 5.3 (5.4 deferred) | Hardening |

Every new agent-side feature must degrade gracefully when a device is absent
(clear error listing available devices), and every image-returning tool follows the
existing `_to_image` inline-JPEG pattern with `max_dim` control.
