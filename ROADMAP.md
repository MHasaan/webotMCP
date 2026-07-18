# webotMCP Roadmap v2

> **v2.1** adds the *environment-engineering track* (Priorities 7–10): semantic scene
> understanding, world building/management, iteration analysis, and script/scenario
> generation — for workflows where the LLM **builds and manages the environment**
> rather than driving robots.

Everything in `PLAN.md` has shipped. This plan is based on a fresh survey of the
**Webots R2025a source tree** (`D:\temp\webots-master`: `lib/controller/python/controller/`
— full Python API; `src/webots/gui/` — CLI flags, streaming servers, single-task mode;
`src/webots/nodes/` — all 100+ node types; `projects/`, `scripts/`, `docs/`) and the
**installed R2025a** (`C:\Program Files\Webots`, incl. `webots-controller.exe` and
`resources/web/streaming_viewer/`), compared against the current ~80 MCP tools
(12 modules), the bridge's 50 supervisor commands and the agent's 28 robot commands.

Legend: **[S]** bridge (supervisor controller) · **[A]** mcp_robot agent · **[M]** MCP
server tool · **[X]** external process/CLI. Effort ≈ S/M/L. Verified API names are from
the R2025a source.

---

## Priority 1 — High-level robot behaviors (biggest usability win)

Raw `set_motor` forces the LLM to micro-manage wheel velocities and poll sensors step
by step — slow, chatty, error-prone. These composite tools close the loop *inside* the
bridge/agent, so one tool call = one behavior.

### 1.1 `drive_robot(robot, linear, angular, duration_s)` — [A][M] · M
Differential-drive convenience. Implementation: agent inspects motor devices; heuristic
pairing of wheel motors (names containing `left`/`right` + `wheel`/`motor`, or exactly
two `RotationalMotor`s with `position: inf` support). Wheel radius/track width read via
supervisor from the robot's `HingeJoint`/`Solid` children when available, else expose
`left_motor/right_motor/wheel_radius/track_width` overrides. Converts (v, ω) →
wheel speeds, runs for `duration_s` sim time inside the agent loop, then stops motors
and returns start/end pose (from supervisor) + distance travelled. Degrade gracefully:
if pairing fails, error lists candidate motors.

### 1.2 `wait_until(condition, timeout_s)` — [S][M] · M
Event-driven stepping: the bridge steps the sim until a condition fires, then reports
what happened and when. Condition mini-DSL (JSON):
`{"type":"distance","a":"BOX","b":"ROBOT","op":"<","value":0.3}`,
`{"type":"contact","a":"BOX","b":"FLOOR"}` (via `node.getContactPoints()`),
`{"type":"speed","node":"BALL","op":"<","value":0.01}` (via `node.getVelocity()`),
`{"type":"sim_time","op":">","value":12.5}`, plus `any`/`all` combinators.
Returns `{fired, condition_index, sim_time, poses_of_involved_nodes}`. This is the
missing primitive for "push the box until it reaches the wall" workflows and makes
experiments deterministic instead of guess-the-duration.

### 1.3 `move_robot_to(robot, target=[x,y], tolerance, max_duration_s)` — [S][A][M] · M
Closed-loop go-to-point: supervisor supplies ground-truth pose each control tick; a
simple heading P-controller feeds 1.1's wheel mapping. Runs paused-world-safe (bridge
steps). Returns success/timeout, final pose, path summary (reuses tracking
infrastructure). Explicitly *not* a planner — document as straight-line reactive drive;
obstacle-aware navigation stays with the LLM (using `get_lidar_summary` between legs).

### 1.4 `solve_arm_ik(robot, target_position, target_orientation=None)` — [M] · L (experimental)
Server-side IK using `ikpy` built from `export_urdf` output (already implemented).
Returns joint name → angle map; user then applies via `set_motor` batch or
`set_joint_position`. Optional dependency (`pip install ikpy`), tool self-reports if
missing. Gate behind the `advanced` tool group.

### 1.5 `build_occupancy_grid(robot, resolution, size)` — [M] · M
Accumulate lidar scans + supervisor poses while tracking is active into a 2D grid;
return as compact ASCII/PNG + obstacle list. Pure server-side math over existing
`get_lidar_summary(point_cloud=True)` + `get_node_pose` calls — no bridge changes.

## Priority 2 — Native tracking, contacts & physics state

The observe stack currently polls positions every step and infers interactions by
matching contact points manually. R2025a has purpose-built streaming APIs.

### 2.1 Switch trackers to native streaming — [S] · M
`node.enablePoseTracking(ms)` / `disablePoseTracking`,
`node.enableContactPointsTracking(ms)`, `field.enableSFTracking(ms)` (all confirmed in
`node.py`/`field.py`). Rework `start_tracking`/`watch_simulation` to enable trackers
once and read cached values — lower overhead per step, exact sampling intervals, and
scales to many objects. Keep the old path as fallback for nodes where tracking fails.

### 2.2 `get_contact_points(node, include_descendants=False)` — [S][M] · S
Direct exposure of `node.getContactPoints(includeDescendants)` → list of
`{point, other_node_id/name}`. Today contacts are only visible through the tracking
digest; an instant "what is this touching right now" query is a much cheaper primitive.

### 2.3 Richer `get_node_details` — [S] · S
Add `velocity` (`node.getVelocity()` → linear+angular), `contact_count`, and for
physics nodes `center_of_mass`/`static_balance` (APIs already used elsewhere). One call
for full dynamic state instead of three.

### 2.4 `reset_node_physics(node)` — [S][M] · S
`node.resetPhysics()` — zero a single object's velocities (stop a runaway ball) without
resetting the whole simulation.

## Priority 3 — Perception & actuation depth

### 3.1 `configure_camera` + intrinsics — [A][M] · S
`camera.setFov` (within `minFov`/`maxFov`), `setExposure`, `setFocalDistance` (zoom &
focus, confirmed in `camera.py`). New `configure_camera(robot, camera, fov=, exposure=,
focal_distance=)`; extend `get_camera_image` metadata with intrinsics
(`fov, focal_length, near, width, height`) so the LLM can do pixel → ray math with
recognition bounding boxes.

### 3.2 Annotated recognition image — [M] · S
`get_camera_recognition(annotate=True)`: server draws bounding boxes + model labels on
the camera JPEG with Pillow (positions come from `getPositionOnImage`/`getSizeOnImage`,
already returned). One image that shows *and* names everything — ideal for the LLM and
for human review.

### 3.3 Speaker sound playback — [A][M] · S
`speak()` exists; add `play_sound(robot, wav_path, volume, loop)` via
`Speaker.playSound(left, right, sound, volume, pitch, balance, loop)` and
`is_speaking`/`stop` state. Sound files can come from `projects/default/worlds/sounds`.

### 3.4 Pen trails — [A][S][M] · S
`set_pen(robot, write=True, ink_color=[r,g,b], density)` (Pen device: `write`,
`setInkColor`). Plus supervisor-side `draw_trajectory(node, color)`: replay a tracked
path as an `IndexedLineSet` spawned into the scene — visualizes any object's motion in
screenshots, not just pen-equipped robots.

### 3.5 Keyboard/joystick passthrough — [A][M] · S (nice-to-have)
Agent-side `robot.getKeyboard()` polling exposed as `get_user_keys()`: lets a human
teleop-nudge while the MCP observes. Low cost since the agent already runs a step loop.

## Priority 4 — Whole-application: streaming, extern controllers, converters

These use app features found in `src/webots/gui/` — no controller API involved.

### 4.1 Web streaming: `start_web_stream(mode='w3d'|'mjpeg', stream_port)` — [X][M] · M
Webots ships two streaming servers (`WbW3dStreamingServer`, `WbMultimediaStreamingServer`)
enabled by `--stream[=w3d|mjpeg] --port=N`, plus a ready browser client at
`resources/web/streaming_viewer/index.html`. Tool relaunches (or launches) Webots with
the flags and returns the viewer URL. Payoff: live remote viewing of the sim in a
browser, independent of the Webots window — including a usable video feed when the
main window is minimized. Caveat to document: requires launch-time flag, so changing it
means a relaunch.

### 4.2 Extern controllers: debug the user's real code — [X][M] · M
Today `attach_mcp_controller` *replaces* a robot's controller. R2025a supports
`controller "<extern>"` + the `webots-controller.exe` launcher (confirmed at
`msys64/mingw64/bin/`), which runs any controller file as an external process
(`--robot-name=`, `--protocol=`, remote TCP possible). New tools:
- `run_extern_controller(controller_path, robot, env=, args=)` — set the robot's
  `controller` field to `<extern>` (supervisor), spawn `webots-controller.exe <path>`,
  pump stdout/stderr into the existing console-log deque.
- `get_extern_controller_output()` / `stop_extern_controller()`.
This turns webotMCP into a *controller development* loop: the LLM edits the user's
controller file, runs it externally, reads its prints, iterates — without touching the
world file or losing the original controller binding.

### 4.3 `convert_proto(proto_file, out_path)` — [X][M] · S
`webots convert` single-task mode (`WbSingleTaskApplication::convertProto`) flattens a
PROTO to base nodes. Use case: "open up" an opaque PROTO so `set_node_field` /
`get_node_string` can reach everything inside. Runs headless, no bridge needed.

### 4.4 World-file maintenance — [X][M] · S
- `update_world_file(path)` → `webots --update-world` (batch-migrate old worlds to
  R2025a format; useful before `install_bridge_into_world` on downloaded worlds).
- `clear_webots_cache()` → `--clear-cache` (fixes corrupted-asset weirdness).
- `preflight()` gains `--sysinfo` output (GPU/driver info) in its report.

### 4.5 Heartbeat watchdog — [M] · S
Launch with `--heartbeat=<ms>`; the launcher thread watches for missing heartbeats in
stdout and flags "Webots GUI frozen" in `preflight` instead of tools timing out blind.

### 4.6 `import_urdf_robot(urdf_path, name)` — [M] · M
Via the official `urdf2webots` pip package (optional dep): convert URDF → PROTO, drop
it in the project's `protos/`, register with `add_proto_to_world`. Closes the loop with
`export_urdf` (ROS ecosystems both ways).

### 4.7 `import_cad_model(path_or_url, physics=False, scale)` — [S][M] · S
Webots's `CadShape` node renders `.obj`/`.dae` directly (`WbCadShape` in source).
Tool wraps it: spawn `Solid { children [ CadShape { url [...] } ] }`, optional box
bounding object + Physics. Instant "put my mesh in the scene".

## Priority 5 — Content domains & world authoring

### 5.1 Animated humans (Skin API) — [A][S][M] · M
`skin.py` is fully unused: `getBoneCount/getBoneName`, `setBoneOrientation/Position`,
`getBonePosition/Orientation`. `projects/humans/` ships pedestrian PROTOs with a
`pedestrian.py` controller. Add `list_bones(robot, skin)` + `set_bone_pose(...)` on the
agent, and a `spawn_pedestrian(position, trajectory=...)` convenience that reuses the
shipped controller. Unlocks human-robot interaction scenes.

### 5.2 `configure_physics(gravity=, timestep=, fps=, contact_properties=...)` — [S][M] · S
Convenience wrapper over `WorldInfo` fields (all editable via existing field API, but
the LLM shouldn't need to know field paths). Include common recipes in the docstring
(slow motion, moon gravity, higher-fidelity contacts via `basicTimeStep`).

### 5.3 `generate_terrain(heightmap|noise_params, size)` — [S][M] · M
Build an `ElevationGrid` from a height array (or simple Perlin noise generated
server-side) and spawn it as a textured floor. Cheap procedural outdoor worlds.

### 5.4 Sample/benchmark discovery — [M] · S
`list_sample_worlds` exists; add `describe_sample(path)` (parse the world header +
its controllers' docstrings) and surface `projects/guided_tour.txt` and
`projects/samples/robotbenchmark/` as curated lists. Document the
`projects/vehicles` SUMO co-simulation workflow in the README (works today via
`launch_webots` on those worlds; no new code).

## Priority 6 — Server infrastructure & DX

### 6.1 Progress notifications — [M] · S
FastMCP `Context.report_progress()` during `watch_simulation`, `screenshot_multiview`,
movie encoding waits, and extern-controller runs. Long calls stop looking hung.

### 6.2 MCP prompts — [M] · S
Register 3–4 FastMCP prompts encoding proven workflows: *inspect-scene* (state → tree →
multiview), *robot-bringup* (list → attach → devices → sensors), *record-demo*
(viewpoint → movie → actions → stop), *experiment-loop* (checkpoint → act → watch →
restore). Prompts are cheap documentation that clients can invoke directly.

### 6.3 Robustness fixes — [M][S] · M
- `preflight(fix=True)`: auto-kill stale `mcp_bridge.py` processes holding port 10022
  (the known freeze cause in README), retry connect.
- `connection.py`: transparent reconnect-once on dropped socket before failing.
- Port collision: if 10022 busy at bridge start, try +1..+5 and write the port into a
  discovery file the server reads (removes the manual env-var dance).
- Windows: `quit_webots` escalation path (WM_CLOSE → terminate → taskkill tree).

### 6.4 Live smoke-test leg — · M
`pytest --live`: launch `worlds/demo.wbt` with `--no-rendering --minimize --batch`,
then ping → scene tree → spawn → step → watch → robot attach → sensor read → quit.
Existing fake-bridge tests cover logic; this catches real-Webots regressions
(API renames, timing, Windows quirks). Mark `@pytest.mark.live`, skipped by default.

### 6.5 New MCP resources — [M] · S
`webots://console` (last N console lines), `webots://preferences`,
`webots://protos/{name}` (proto info as a resource). Read-only state belongs in
resources, keeping tool calls for actions.

### 6.6 HTTP transport option — [M] · L (deferred)
`fastmcp` supports streamable-HTTP; would allow Claude Desktop + Code concurrently with
a session lock on the single bridge. Still deferred — revisit only when a second
concurrent client is actually needed.

---

# Environment-engineering track (v2.1)

Focus: the LLM as **world builder and experiment analyst**. Verified against source:
`WorldInfo` fields (`randomSeed`, `coordinateSystem`, `basicTimeStep` — `docs/reference/worldinfo.md`),
`--log-performance=<file>` (`WbGuiApplication.cpp`), 70 appearance PROTOs
(`projects/appearances/protos/`), 40+ object categories (`projects/objects/` incl. a
`create_wall` generator), `node.exportString()`, contact/pose tracking APIs.

## Priority 7 — Semantic scene model: *where everything is and what it is*

Today the LLM reconstructs the scene from `get_scene_tree` + `get_node_details` calls
(one per object, raw fields). These tools give it a ready-made semantic model.

### 7.1 `get_object_catalog()` — [S][M] · M
The single most useful scene-understanding call: one table with every Solid-derived
node — `name/DEF, type, proto, position, rotation(yaw), size, mass, dynamic|static|kinematic,
color, parent`. Implementation: bridge walks the tree once; **size** computed from
`boundingObject` introspection (Box `size`, Cylinder `radius/height`, Sphere `radius`,
Capsule, Mesh → AABB of transformed primitives; fallback: geometry fields of the first
Shape child; else `null`); **mass** from `Physics.mass` (or `density × volume` when
mass is -1); **color** from `PbrAppearance.baseColor`/`Appearance.material.diffuseColor`
or `recognitionColors[0]`. Paged like `get_scene_tree` for big worlds. This is the
"inventory sheet" a script generator needs.

### 7.2 `get_object_properties(node)` — [S][M] · S
Deep single-object composite: everything from 7.1 plus full AABB (min/max corners),
velocity, center of mass, contact partners (2.2), joint summary for articulated nodes,
PROTO parameters (existing `get_node_proto`), device list if robot, and the
`boundingObject` shape description. Replaces 3–5 calls with one.

### 7.3 Spatial queries — [S][M] · M
Server/bridge-side geometry over the catalog (no new Webots API needed):
- `find_nodes_near(point_or_node, radius)` — proximity search;
- `objects_in_region(min=[x,y,z], max=[x,y,z])` — box query;
- `check_overlap(node_a, node_b)` / `find_overlapping_pairs()` — AABB intersection
  test (approximate, documented as such) — catches accidentally intersecting placements;
- `get_spatial_relations(node?)` — relation graph: `on_top_of` (contact + higher AABB),
  `touching` (contacts), `inside` (AABB containment), `near` (< threshold), with
  distances. Lets the LLM answer "what is on the table?" in one call.

### 7.4 `get_scene_map(region=None, annotate=True)` — [S][M] · M
Top-down annotated map: move viewpoint overhead (restore after, like multiview),
capture, then server-side draw labels/arrows on the image (Pillow) by projecting each
catalog object's position into the image (overhead projection is trivial: linear in x,y).
Optionally pure-vector SVG mode (no screenshot) rendering AABB footprints + names —
crisp, tiny, and unambiguous for an LLM. The fastest way to grasp a whole layout.

### 7.5 Annotated viewport screenshots — [M] · M
`get_viewport_screenshot(..., annotate=True)`: project 3D object positions into the
current view (Viewpoint `position`, `orientation`, `fieldOfView` are all readable) and
draw name labels on visible objects (depth-sorted, occlusion-unaware — documented).
Turns any screenshot into a labeled diagram.

### 7.6 Scene snapshots & diff — [S][M] · M
`snapshot_scene(name)` — store the catalog (poses + key fields) server-side;
`diff_scene(name_a, name_b='now')` — report added/removed nodes, moved objects
(with displacement), rotated, field changes. Complements `save_checkpoint` (which
*restores* state but doesn't *explain* what changed). Core primitive for "what did my
last edit/run actually do?".

### 7.7 `webots://conventions` resource + world config awareness — [M] · S
Static resource documenting units (meters/kg/s), axis convention, and the world's
actual `WorldInfo.coordinateSystem` (ENU default; some sample worlds are NUE!),
`basicTimeStep`, `gravity`. `preflight` and `get_simulation_state` gain a
`coordinate_system` field so spatial reasoning never silently assumes the wrong axes.

## Priority 8 — World building & management

### 8.1 Structured placement — [S][M] · M
Building on the catalog's AABBs (7.1):
- `drop_to_ground(node)` — set z so the AABB bottom rests on the floor/support below
  (find support via downward AABB sweep against other objects, else floor plane);
- `place_on(node, target, offset=[dx,dy])` — put A on top of B, centered + offset;
- `align_objects(nodes, axis, mode='min|center|max')` and
  `distribute_objects(nodes, axis, spacing|extent)`;
- `place_row/place_grid(proto_or_node, count/rows×cols, start, step)` — bulk layout
  (wraps `spawn_node` + placement math; honors `batch_execute` semantics).
No new Webots API — pure bridge math + existing spawn/move. Eliminates the #1 authoring
failure mode: objects spawned intersecting or floating.

### 8.2 Collision-aware placement & scatter — [S][M] · M
- `find_free_space(size, region, near=None)` — first pose where an AABB of `size` fits
  without overlapping catalog AABBs;
- `scatter_objects(node_string|proto, count, region, min_spacing, random_yaw=True,
  seed=None)` — randomized placement with rejection sampling; `seed` + reporting the
  chosen poses makes generated environments **reproducible** (pairs with
  `WorldInfo.randomSeed`, 8.8). This is the domain-randomization primitive for
  automated environment generation.

### 8.3 `validate_world()` — [S][M] · M
Static lint pass over the world; returns a problem list with severities and fixes:
overlapping AABBs; objects below the floor or floating in air; dynamic Solids missing
`boundingObject`/`Physics` (or vice-versa: huge meshes used as bounding objects);
duplicate DEF names; robots whose `controller` doesn't exist in the project;
`EXTERNPROTO` declared but unused / used but undeclared (parse the .wbt header vs node
types); `basicTimeStep` too large for small/fast objects (heuristic); non-ENU
`coordinateSystem` warning. Run it after building, before running — catches the
problems that otherwise surface as confusing mid-run physics chaos.

### 8.4 Appearance & material tools — [S][M] · S
`list_appearances()` — index the 70 appearance PROTOs (Asphalt, BrushedSteel,
Cardboard...) like `search_protos`; `set_appearance(node, base_color=|texture=|
appearance_proto=)` — rewrite a Shape's appearance in one call (today requires knowing
exact nested field paths); `set_recognition_colors(node, colors)` — make any object
visible to camera recognition (currently a manual field edit users forget).

### 8.5 Lighting & background presets — [S][M] · S
`configure_lighting(preset='indoor'|'outdoor'|'studio'|'night', intensity=)` — set
`Background`/`TexturedBackground(Light)`, add/adjust `DirectionalLight`/`PointLight`s.
Bad lighting is the top cause of useless screenshots and failed camera recognition.

### 8.6 World templates & file management — [M] · M
`create_world(name, template='empty'|'indoor_room'|'outdoor_flat', size=, from_world=)`
— scaffold a ready .wbt (floor + lighting + viewpoint + bridge pre-installed;
`from_world` clones an existing/sample world into the project). Plus
`backup_world()` / `list_world_backups()` / `restore_world_backup(n)` — timestamped
copies (install_bridge already makes one-off backups; make it a managed history).

### 8.7 PROTO authoring: reusable components — [S][M] · M
`extract_proto_from_node(node, proto_name, params=[...])` — take a tuned scene node,
`node.exportString()` it, wrap in a PROTO header exposing chosen fields as parameters,
write to project `protos/`, ready for `spawn_node("MyShelf { height 2 }")`. Reuse
`scripts/proto_formatter` conventions. Turns one-off builds into a reusable parts
library — the compounding asset for automated environment construction.

### 8.8 Determinism & physics config — [S][M] · S
Fold into `configure_physics` (5.2): `random_seed` (`WorldInfo.randomSeed`; 0 = time-based,
set explicit for reproducible runs), `coordinate_system` (read-only report), `fps`,
`optimal_thread_count`. Document: identical seeds + same timestep ⇒ reproducible physics.

## Priority 9 — Iteration understanding: *what happened and what went wrong*

### 9.1 `run_experiment(duration_s, watch=[...], options)` — [S][M] · M
The "full iteration in one call" composite: `save_checkpoint` → enable tracking +
contact tracking + console capture → run `duration_s` (or until a `wait_until`
condition, 1.2) → return a structured **run report**: per-object trajectories &
displacement, interaction timeline, scene diff vs start (7.6), console
warnings/errors (9.3), anomalies (9.2), final annotated screenshot, and
`restore='auto'|'keep'|'on_anomaly'`. This *is* the automated-environment iteration
loop: build → run → read one report → adjust → repeat.

### 9.2 Anomaly detection — [S] · M
During `watch_simulation`/`run_experiment`, flag per step (cheap checks on tracked
data): NaN/inf positions (physics blow-up), teleports (> Xm in one step), runaway
velocity, objects below the floor plane or outside the arena region, persistent deep
interpenetration, contact jitter (contact pairs toggling every step — classic unstable
resting contact), and robots whose controller died (console). Each anomaly reported
with time, nodes, and a hint ("reduce basicTimeStep", "add damping", "check
boundingObject size"). Converts silent physics weirdness into actionable diagnostics.

### 9.3 Console diagnostics & per-controller logs — [M] · S
`get_console_diagnostics()` — parse the captured Webots console: classify ODE/physics
warnings, controller tracebacks (Python), missing-asset errors, and map the known
message patterns to fixes; `get_controller_logs(robot)` — Webots prefixes console lines
with `[controller_name]`, so split per robot. Answers "why did my run misbehave"
without the LLM eyeballing 2000 raw log lines.

### 9.4 Time-series export: `record_states` — [S][M] · M
`record_states(nodes=[...], fields=[...], interval_ms)` during a run → tidy table
(sim_time × node × position/rotation/velocity/custom fields), returned as CSV file +
compact summary stats. Uses native SF-tracking (2.1) where possible. This is the raw
material for offline analysis, plotting, and regression-testing environments
("assert the box never exceeds 0.5 m/s").

### 9.5 `profile_simulation(duration_s)` — [X][M] · S
Relaunch/launch with `--log-performance=<file>` (confirmed flag), run, parse the
report: real-time factor, time spent in physics vs rendering vs controllers. Plus a
zero-restart variant: measure achieved sim-time/wall-time ratio from the bridge.
Tells you *why* a world is slow (usually mesh bounding objects or tiny timestep).

### 9.6 Reproducible runs — [S][M] · S
`run_experiment(..., seed=N)` sets `WorldInfo.randomSeed`, resets, runs — same inputs,
same outcome. Optional `compare_runs(report_a, report_b)`: trajectory divergence
point, differing events. Debugging tool for "it fails one time in five".

## Priority 10 — Script & scenario generation: from session to automation

### 10.1 `generate_world_script(format='wbt'|'python'|'json')` — [S][M] · M
Export the *current* scene as regenerable code: `wbt` = clean world file (exists via
`save_world`); `python` = a standalone supervisor script of `importMFNodeFromString`
calls that rebuilds the dynamic scene (uses `get_node_string` per spawned node);
`json` = declarative scenario file (10.3). The LLM builds interactively via MCP, then
hands the user a script that reproduces it without MCP. Directly answers "so a better
automation or script can be written".

### 10.2 Automation scripts as extern supervisors — [M] · M
Builds on 4.2: `create_supervisor_script(name, code)` (scaffold with the same
robust node/field helpers the bridge uses) + `run_supervisor_script(name)` — execute as
an extern controller with its own console capture. For automation that must run at
full speed inside Webots (e.g., regenerate + evaluate 100 environments), instead of
round-tripping every step through MCP.

### 10.3 Declarative scenarios — [M] · L
`scenario.json` schema: object list (proto/node-string, pose or placement rule from
8.1/8.2 with ranges + seed), physics config, success/failure conditions (the
`wait_until` DSL, 1.2), duration, metrics to record (9.4).
Tools: `save_scenario(name)` (capture current world as scenario),
`load_scenario(file)` (build it), `run_scenario(file, runs=N, seeds=[...])` — batch
build→run→report (9.1) per variation, aggregate results table. This is the full
"automated environment" workflow as data, versionable in git.

### 10.4 Environment regression tests — [M] · S
Thin wrapper: `run_scenario` reports vs stored baseline → pass/fail per condition.
Lets a CI job answer "does the warehouse world still behave after my edits?".

---

## Improvements to existing tools (small, do alongside)

| Tool | Change |
|------|--------|
| `get_node_details` | + velocity, contacts, CoM (2.3) |
| `launch_webots` | + `stream=`, `heartbeat=`, `extern_urls=` flags (4.1/4.5) |
| `watch_simulation` / `start_tracking` | native pose/contact trackers (2.1); progress (6.1) |
| `preflight` | stale-process auto-fix, `--sysinfo`, heartbeat status (4.4/4.5/6.3) |
| `get_camera_image` | + intrinsics metadata (3.1) |
| `get_camera_recognition` | + `annotate=True` overlay (3.2) |
| `attach_mcp_controller` | docstring: point to extern-controller flow as the non-destructive alternative (4.2) |
| `search_webots_docs` | index `docs/guide` + `docs/reference` from source checkout when installed docs lack a page |
| `get_viewport_screenshot` / `screenshot_multiview` | + `annotate=True` labels (7.5) |
| `watch_simulation` | + `detect_anomalies=True` (9.2); reuse in `run_experiment` (9.1) |
| `get_simulation_state` | + `coordinate_system`, `random_seed`, real-time factor (7.7/9.5) |
| `spawn_node` / `move_node` | + `drop_to_ground=True` convenience flag (8.1) |
| `find_nodes` | + `near=`/`region=` filters once 7.3 lands |

## Suggested order

Environment-engineering track (P7–P10) is prioritized first per project goals: the LLM
builds and analyzes environments; robot-behavior tools (P1) come later.

| Phase | Items | Rationale |
|-------|-------|-----------|
| 1 | **7.1, 7.2, 7.3, 7.7**, 2.2, 2.3 | Semantic scene model — the foundation everything else reads |
| 2 | **8.1, 8.2, 8.3**, 7.4, 7.6 | Reliable building: placement, scatter, validation, map, diff |
| 3 | **9.1, 9.2, 9.3**, 1.2, 2.1 | Run → understand: experiment report, anomalies, console; `wait_until` |
| 4 | **10.1, 8.4, 8.5, 8.6, 8.7**, 7.5, 9.4 | Authoring depth + script export |
| 5 | **10.2, 10.3**, 4.2, 9.5, 9.6, 8.8 | Scenarios, extern-supervisor automation, profiling |
| 6 | 3.1, 3.2, 4.1, 4.3–4.7, 5.2–5.4, 6.1–6.5, 10.4 | Perception/app breadth + DX hardening |
| 7 | 1.1, 1.3, 1.5, 3.3, 3.4, 5.1 | Robot-behavior track (when needed) |
| — | 1.4 (IK), 3.5, 6.6 (HTTP) | Experimental / deferred |

## Ground rules (carried over)

- Every agent-side feature degrades gracefully when a device is absent (error lists
  available devices).
- Every image tool follows the `_to_image` inline-JPEG pattern with `max_dim`.
- Every new tool gets a fake-bridge pytest; behavior tools (P1) also get live-leg
  coverage (6.4).
- New tool groups: P1 behaviors → `behavior` group; converters/imports → `assets`;
  keep `core` lean.
