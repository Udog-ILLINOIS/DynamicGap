# Imitation Learning Data Collection — Plan and Issue Register

One file holding both the plan (Part 1) and every known issue (Part 2).
It merges and supersedes `IL_DATA_COLLECTION_PLAN.md` (original issue list),
`IL_DATA_COLLECTION_DESIGN.md` (first fix design) and
`IL_DATA_COLLECTION_REVIEW.md` (review of both).

**Status (2026-10-07).** Phase 0 code checks are done; results are in 1.11
and in each CHECK issue below. Decisions D1–D7 are still open. No source
file has been modified yet.

**Basis.** Line references are pinned to these commits (R-34):

| Repo | Commit | Working tree |
|---|---|---|
| `planners/dynamic_gap` | `f27f357bb9804e7d57b0115e60001c6b577c0fd0` | `rviz/arena.rviz` modified |
| `arena/arena-rosnav` | `8088929cbd2fd6df50ffd5474580a6c579285e4a` | config files modified; RViz tool clean |

The DG issues and their file/line evidence come from the original plan and
were re-checked against these commits. Code sketches and member names in the
fixes are still unverified until implemented.

**Changes from the first design.** This plan differs from
`IL_DATA_COLLECTION_DESIGN.md` in six ways, each driven by the review:

1. The goal is added to the sample.
2. The cursor is carried in a custom `CursorState` message.
3. A dead cursor is flagged, not acted on. The robot's behaviour changes only
   at episode reset. The `HELD` mode is dropped.
4. The autonomous planner's pick and cost are dropped from version 1.
5. Cursor liveness is judged in wall time.
6. Build order starts with a thin end-to-end slice and a pilot.

---

# Part 1 — The plan

## 1.1 Intent

Collect demonstrations of a human choosing which gap the robot should take,
to train a model that predicts that choice. One training sample is:

```
(scene observation, goal, variable-length candidate gap set)  ->  index into that set
```

The model does not detect gaps and does not produce trajectories. The
candidate set is part of the observation and must be recorded exactly as the
human saw it.

### What good data means

1. **Complete.** Each sample holds the options, the scene, the goal and the choice.
2. **Consistent.** The set the human saw is the set the label indexes into.
3. **Honest.** A label counts only when the human was really choosing.
4. **Self-contained.** A sample can be read without re-joining streams offline.
5. **Re-derivable.** Enough raw data is kept to recompute labels under another rule.
6. **Matched.** The human decides from the same information the model will get.

### Principles

- The sample is defined at the 5 Hz planning tick and stamped in-process.
- Record facts and flags at collection time. Filter and shift offline.
- Do not change robot behaviour unless the data cannot be made honest otherwise.
- Every new line in the planner is guarded by `manualSelectionMode_`.

## 1.2 Decisions still open

Defaults below are what the plan assumes until you say otherwise.

| # | Decision | Default assumed here |
|---|---|---|
| D1 | Does a still mouse mean a map spot or a direction from the robot? | Whatever is under the cursor on screen (see 1.5, T2). The RViz view then decides which it is. With the shipped view (top-down, following `rto/base_link`) a still mouse means a fixed offset from the robot. |
| D2 | Hover-nearest, or latch a gap until the operator points elsewhere? | Hover-nearest for version 1; revisit after the pilot. |
| D3 | On a dead cursor: change robot behaviour, or only flag? | Flag only. |
| D4 | Include the autonomous planner's pick? | Not in version 1. |
| D5 | What is the model given as input? | Scan, robot velocity, goal, candidate set. RViz is cut down to match. |
| D6 | Gaps only, or also "go straight" and "wait"? | Gaps only; forced-choice ticks are flagged. |
| D7 | Where bags are stored (`data_root`). | Not set. |

## 1.3 Fix A — one candidate list per tick

Build one `std::vector<ManualCandidate>` once per tick in `runPlanningLoop`.
`publishManualCandidateMarkers` and `getCursorSelectedTrajectory` both read it.

- Every tube is listed, including infeasible ones, in `traj_idx` order.
- The recorded label is the position in this array.
- The dense training index is derived offline by masking on `shown`.
- Gap identity over time is `(left_model_id, right_model_id)`. IDs are
  unique for the planner's lifetime (1.11, R-21), so `episode_id` is not
  needed for uniqueness but is kept for segmentation. Gap bearing is
  recorded as a second continuity cue.
- Selection runs first, then markers are published, and the highlight is
  driven by the cursor selection, chosen by identity.

Closes DG-13, DG-14, DG-15, R-19.

## 1.4 Fix B — one sample message per tick

Messages live in a small separate package so offline tools do not need to
build the planner. `tick_seq` increments on every tick and a sample is
published on every exit path of the planning loop, with an empty candidate
set if need be. There are three early returns before candidates exist
(not ready, colliding, no gaps; 1.11 R-28).

### `CursorState.msg` (published by the RViz tool)

```
Header  header                 # sim stamp; frame_id = RViz fixed frame
time    wall_stamp             # wall clock at publish
bool    tool_active            # false in the one message sent on deactivate
bool    pick_ok                # the current screen position hits the ground plane
geometry_msgs/Point point      # in header.frame_id; valid only if pick_ok
int32   screen_x
int32   screen_y
time    last_move_wall         # wall clock of the last real mouse move
```

### `ManualGapCandidate.msg`

```
int32   traj_idx               # index into gapTrajs / gapTubes (sparse)
int32   display_id             # number shown to the human (dense); -1 if not shown
int32   left_model_id
int32   right_model_id
bool    feasible               # non-empty trajectory
bool    shown                  # drawn as selectable this tick
uint16  tube_len               # number of gaps in the tube
# gap geometry: robot frame at the gap snapshot stamp
float32 left_range
float32 left_theta
float32 right_range
float32 right_theta
float32 bearing                # gap centre bearing
# models: robot frame at the gap snapshot stamp
geometry_msgs/Vector3 left_model_pos
geometry_msgs/Vector3 left_model_vel
geometry_msgs/Vector3 right_model_pos
geometry_msgs/Vector3 right_model_vel
# odom frame at the tick
geometry_msgs/Point   goal_pos
geometry_msgs/Pose    terminal_pose
geometry_msgs/Point[] path     # downsampled candidate trajectory
float32 cursor_dist            # cursor to terminal pose; NaN if none
float32 cursor_bearing_diff    # bearing(cursor) - bearing(terminal), from robot; NaN if none
```

### `ManualGapSample.msg` (published by the planner)

```
uint8 SOURCE_NONE          = 0   # no cursor has ever arrived, or no candidates
uint8 SOURCE_GAP_CURSOR    = 1   # a gap was selected from the cursor
uint8 SOURCE_CLICK_TO_MOVE = 2   # following a click-created trajectory
uint8 SOURCE_MODEL         = 3   # reserved: a learned model chose

uint16  schema_version
Header  header                   # planning tick, sim time
time    wall_stamp
uint32  tick_seq
int32   episode_id               # scenario_reset counter
bool    in_reset_window          # tick is close to an episode reset

# observation
sensor_msgs/LaserScan scan       # copied with the gaps under gapMutex_
time    gap_snapshot_stamp
time    odom_stamp
geometry_msgs/Pose    robot_pose_odom
geometry_msgs/Twist   robot_vel
geometry_msgs/Point   goal_robot_frame
geometry_msgs/Point   local_waypoint_robot_frame
ManualGapCandidate[]  candidates
bool    ungap_would_apply        # forced-choice tick (D6)
bool    idle_would_apply

# cursor
CursorState cursor               # latest message received, verbatim
time    cursor_rx_wall           # planner wall clock at receipt
geometry_msgs/Point cursor_odom  # the point actually used for selection
bool    cursor_live              # see 1.5, P2

# choice
uint8   source
int32   selected_idx             # position in candidates[]; -1 if none
int32   selected_left_model_id
int32   selected_right_model_id
geometry_msgs/Point click_point_odom   # for CLICK_TO_MOVE
```

**A tick is a usable expert label when**
`source == GAP_CURSOR && cursor_live && !in_reset_window`.

The manual branch also sets the planner's current left/right model IDs, as
the autonomous path does.

Closes DG-01, DG-02, DG-03, DG-08; partly DG-04; R-06, R-10, R-12, R-14,
R-15, R-16, R-28.

## 1.5 Fix C — cursor validity

The tool's last-known-position publisher stays. A still mouse is a deliberate
hold. Nothing here requires the operator to keep moving the mouse.

### Tool side

- **T1. Publish `CursorState`** at 10 Hz while active, and one final message
  with `tool_active = false` on `deactivate()`.
- **T2. Re-project every publish.** Store the mouse's screen position and
  viewport, not the 3D point. On each timer tick, intersect the mouse ray
  with the ground plane:

  ```cpp
  #include <rviz/geometry.h>

  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  Ogre::Vector3 hit;
  msg.pick_ok = rviz::getPointOnPlaneFromWindowXY(viewport_, ground,
                                                  screenX_, screenY_, hit);
  ```

  What is published is then always what is under the cursor on screen, even
  when the camera follows the robot. This is required, not optional: the
  shipped view follows the robot (1.11, R-17).
- **T3. Pointer leaves the window:** set `pick_ok = false`.
- **T4. Camera control:** the tool never forwards events to the view
  controller, so the camera cannot be moved while it is active, and
  switching tools calls `deactivate()` (1.11, R-22). With the following
  top-down view this is acceptable; forwarding wheel/zoom events is optional.

### Planner side

- **P1. Transform properly.** Use the message's own `frame_id` and stamp
  through the tf buffer; reject and warn on failure. Record the transformed
  point.
- **P2. Compute `cursor_live`**, without changing behaviour:

  ```
  cursor_live = tool_active && pick_ok
             && (wall_now - cursor_rx_wall) < cursorTimeout   // ~0.5 s wall
             && last_move_wall > last_reset_wall
  ```
- **P3. Clock check.** If the cursor's sim stamp and the planner clock differ
  by seconds, log an error (RViz is not on sim time).
- **P4. Reset — the one behaviour change.** On `scenario_reset`, clear
  `hasCursorPos_`, `cursorPosOdomFrame_`, `selectedManualCandidateId_`,
  `hasTrajCreationPoint_`, `isTrackingCreatedTraj_`, record
  `last_reset_wall`, and ignore the cursor until `last_move_wall` is later
  than it. Subscribe to `scenario_reset` directly: `Planner::reset()` is
  never called on an episode reset (1.11, R-04). Also clear gap estimator
  state, which does survive the teleport (1.11, R-26).
- **P5. No distance gate in the planner.** `cursor_dist` and
  `cursor_bearing_diff` are recorded per candidate; gating is done offline
  with thresholds set from the pilot.

Closes DG-05 (offline), DG-06, DG-07, DG-10, DG-11, DG-12, DG-20, R-02,
R-03, R-05, R-17, R-20, R-24.

## 1.6 Fix D — recording

**Format.** rosbag, via a new `record_il.launch`.

**Topics.** The sample topic, the `CursorState` topic, `scan`, `odom`, `acc`,
`cmd_vel`, `/tf`, `/tf_static`, the goal, the global plan, the static map,
`scenario_reset`, per-episode task-generator output, the pedsim topic,
collision/bumper output, `/traj_creation_point`. No marker topics.

**Options.** lz4 compression; split by size; latched topics repeated into
each split (check that your `rosbag record` supports `--repeat-latched`).

**Layout.**

```
<data_root>/il_gap_selection/
  2026-10-05_session01/
    session_0.bag, session_1.bag, ...
    params.yaml        all rosparams at start
    versions.yaml      git hash and git diff of each repo
    view.rviz          the RViz config used
    notes.txt          operator ID, instruction given, anything odd
```

`data_root` is a launch argument and sits outside the git repo (D7).

**Automatic checks, run on every bag.**

- `tick_seq` is contiguous.
- Recomputing nearest-endpoint from the recorded cursor and candidates
  reproduces `selected_idx` on every `GAP_CURSOR` tick.
- All sim stamps are on one clock.
- Every episode has an outcome (success, collision or timeout).
- No `.bag.active` files remain.

**During sessions.** Watch free disk space; the recorder stops quietly when
it runs low.

Closes DG-16, DG-17, DG-18, DG-19, R-07, R-08, R-09, R-13, R-36.

## 1.7 Offline processing

The bags are the raw record. A script reads them and writes the training set.

- **Label filter.** Keep ticks that pass the rule in 1.4. Apply distance,
  bearing and near-tie thresholds chosen from the pilot.
- **Reaction lag (DG-09).** For lag *k* (set in wall time, converted with the
  recorded real-time factor): take the selected identity at tick *t*, find
  the same identity in tick *t − k*, pair that index with the observation
  from *t − k*. Drop the sample if the identity does not exist there.
- **Episode filter.** Drop or separately tag episodes that end in collision.
- **Splits.** By episode, ideally by scenario or map. Never by tick.
- **Reporting.** Per session: ticks, usable ticks, decisions (identity
  switches backed by mouse movement), ticks with more than one candidate.

## 1.8 Collection protocol (to be written before collecting)

One page covering: the instruction given to operators (what to optimise);
who operates and a warm-up period; session length and breaks; the maps,
scenarios and pedestrian densities to cover; the target in decisions, not
ticks; the fixed RViz config.

## 1.9 Order of work

| Phase | Work | Done when |
|---|---|---|
| 0 | Code checks: R-04, R-17, R-19, R-21, R-22, R-25, R-26, R-27. Pin repo commits. Settle D1–D7. | **Code checks done (1.11).** R-25, R-27 need a running sim. D1–D7 open. |
| 1 | Thin slice: message package, single candidate list (A), minimal sample (B), `record_il.launch`, offline loader. | A bag loads and the automatic checks run. |
| 2 | Pilot: ten minutes of collection. Measure tie rate, switch rate, trivial-tick fraction, identity lifetimes, cursor liveness. | Thresholds and D2 are decided from data. |
| 3 | Cursor work (C): tool and planner. | The checklist in 1.10 passes. |
| 4 | Recording hardening, metadata, replay viewer. Regression run of the autonomous planner. | Benchmark unchanged with manual mode off. |
| 5 | Write the protocol; collect. | — |
| 6 | Offline processing and dataset build. | — |

## 1.10 Checks before a long session

- [ ] Samples arrive at 5 Hz with non-empty `candidates` and a goal.
- [ ] Cursor sim stamp and sample stamp are on the same clock.
- [ ] `cursor.header.frame_id` is `map`.
- [ ] `selected_idx` matches the gap the robot visibly follows.
- [ ] Mouse held still for 10 s: `cursor_live` stays true.
- [ ] Tool deactivated: `cursor_live` goes false within 0.5 s.
- [ ] Pointer moved out of the window: `cursor_live` goes false.
- [ ] After a reset: `episode_id` increments; `cursor_live` is false until
      the mouse moves.
- [ ] A click-to-move produces `CLICK_TO_MOVE` ticks.
- [ ] The automatic bag checks in 1.6 pass on the test bag.

## 1.11 Phase 0 results (2026-10-07)

Static reading of the code at the pinned commits. "Runtime" means it needs a
running sim to settle.

| ID | Result | Finding |
|---|---|---|
| R-04 | **Confirmed, worse** | `Planner::reset()` is called only from `recordAndCheckVel` (`Planner.cpp:2681`) on a planning failure. The call in `DynamicGapPlanner.cpp:105` is commented out. Nothing in the planner reacts to an episode reset. |
| R-17 | **Confirmed** | `arena.rviz:823-833`: view is `rviz/TopDownOrtho` with `Target Frame: rto/base_link`, so the camera follows the robot. The tool stores a 3D map point (`continuous_gap_selection_tool.cpp:83-91`), so under a still mouse the published point stays put in the map while the screen scrolls. |
| R-19 | **Confirmed, different** | Markers are published before selection (`Planner.cpp:2138-2152`). The green highlight compares `display_id` with `selectedManualCandidateId_`, which only the old numeric topic `/manual_traj_selection` sets (`:706-711`). In cursor mode the highlight never reflects the cursor pick at all. |
| R-21 | **Confirmed, mostly good** | IDs come from `currentModelIdx_` (`Planner.h:550`), a counter that only increments (`GapPointAssociator.cpp:174`). IDs are never reused and never reset, so they are unique for the planner's lifetime, across episodes. Copies, manipulation and propagation keep the ID (`RotatingFrameCartesianKalmanFilter.cpp:95,140`). Association is per point, nearest within `assoc_thresh` (1.0 m), so a split or merge changes the pair, and a point can inherit a nearby edge's ID. Raw and simplified gaps draw from the same counter. |
| R-22 | **Confirmed** | `processMouseEvent` returns `0` and never calls the view controller, so the camera cannot move while the tool is active. Switching tools calls `deactivate()`, which stops publishing; the planner keeps using the last point (DG-06). |
| R-26 | **Confirmed** | Nothing clears `prevRawGaps_` / `prevSimplifiedGaps_` or their models on a reset. After a teleport, any new gap point within 1.0 m (robot frame) of an old one inherits the old model, velocity estimate included. |
| R-28 | **Confirmed** | `runPlanningLoop` returns before candidates exist in three places: not ready (`:1973`), colliding (`:1980`), no gaps (`:2002`). `laserScanCB` also returns early when colliding (`:294`), leaving gaps from the previous scan. |
| DG-10 | **Low risk, keep the check** | `arena.rviz:798` sets `Fixed Frame: map`. The planner still assumes it. |
| DG-12 | **Low risk, keep the check** | `use_sim_time` (`start_trials.launch:77`) and RViz (`:172`) are in the same launch; roslaunch sets params before starting nodes. Only a separately started RViz would break it. |
| R-25 | Runtime | Needs a sim run to see whether a tick can mix pre- and post-reset data. |
| R-27 | Runtime | Needs a sim run to see whether the sim clock jumps backwards at reset. |
| R-34 | **Done** | Commits pinned at the top of this file. |

**New observation.** The tool sets `lastKnownPos_.header.stamp` only on a
mouse move and the timer re-sends that message unchanged, so the current
`/gap_selection_point` stamp is already "sim time of last move". Not a bug,
but any code reading it as "time of publish" is wrong.

---

# Part 2 — Issue register

**IDs.** `DG-xx` are from the original plan (DG-20 was added during design).
`R-xx` are from the review.
**Severity.** BLOCKER: good data is impossible without a change. HIGH: data
will be silently wrong or unusable. MED: degrades quality; workaround exists.
LOW: annoyance or future-proofing.
**Tag.** DOC: follows from the documents. CHECK: needed a code or setup
check; the result is in 1.11.
**Status.** All issues are OPEN. "Plan" gives the section that addresses it.

## Index

| ID | Sev | Tag | Title | Plan |
|---|---|---|---|---|
| DG-01 | BLOCKER | DOC | Selection never leaves `getCursorSelectedTrajectory` | 1.4 |
| DG-02 | BLOCKER | DOC | Candidates exist only as markers, then are deleted | 1.4 |
| DG-03 | BLOCKER | DOC | Detection, selection and cursor run at three rates | 1.4 |
| R-10 | BLOCKER | DOC | The goal is not in the sample | 1.4 |
| DG-04 | HIGH | DOC | Nearest-endpoint is a proxy for intent; ties invisible | 1.4, 1.7 |
| DG-05 | HIGH | DOC | No abstain and no gate on the nearest match | 1.5 P5, 1.7 |
| DG-06 | HIGH | DOC | Cursor state never cleared; leaks across resets | 1.5 P2, P4 |
| DG-07 | HIGH | DOC | No staleness check; failed picks hold the old point | 1.5 T2, P2 |
| DG-08 | HIGH | DOC | Click-to-move looks like a gap selection | 1.4 |
| DG-09 | HIGH | DOC | Human reaction lag unmodelled | 1.7 |
| DG-10 | HIGH | DOC | Cursor frame assumed to be `map` | 1.5 P1 |
| DG-13 | HIGH | DOC | Two definitions of the candidate set | 1.3 |
| DG-14 | HIGH | DOC | Dense `display_id` vs sparse trajectory index | 1.3 |
| DG-15 | HIGH | DOC | Stable gap identity not recorded | 1.3 |
| DG-20 | HIGH | DOC | Tool republishes the pre-reset point after a reset | 1.5 P4 |
| R-01 | HIGH | DOC | Autonomous pick and cost are not free to record | 1.2 D4 |
| R-04 | HIGH | CHECK ✔ | Reset is never called on an episode reset | 1.5 P4 |
| R-05 | HIGH | DOC | First design changed robot behaviour needlessly | 1.5 |
| R-07 | HIGH | DOC | Record list was incomplete | 1.6 |
| R-11 | HIGH | CHECK | The human sees more than the model will | 1.2 D5, 1.8 |
| R-13 | HIGH | DOC | No episode outcome recorded | 1.6, 1.7 |
| R-17 | HIGH | CHECK ✔ | A still mouse does not point where the operator sees it | 1.5 T2 |
| R-18 | HIGH | DOC | Choice is matched against a set the human has not seen | 1.2 D2, 1.7 |
| R-21 | HIGH | CHECK ✔ | The model-ID pair: unique, but changes on split/merge | 1.3 |
| R-25 | HIGH | CHECK (runtime) | Samples straddle the reset | 1.4 |
| R-26 | HIGH | CHECK ✔ | Gap estimators survive the teleport | 1.5 P4 |
| R-29 | HIGH | DOC | Ticks are not independent samples | 1.7, 1.8 |
| R-32 | HIGH | DOC | No collection protocol | 1.8 |
| R-36 | HIGH | DOC | No automatic consistency checks | 1.6 |
| DG-11 | MED | DOC | Cursor transformed at latest time, not its stamp | 1.5 P1 |
| DG-12 | MED | DOC | RViz sim-time assumed; fails silently | 1.5 P3 |
| DG-16 | MED | DOC | Planner topics are namespaced, cursor topics global | 1.6 |
| DG-17 | MED | DOC | No rosbag infrastructure in the repo | 1.6 |
| DG-18 | MED | DOC | Episode boundaries and metadata not wired in | 1.4, 1.6 |
| R-02 | MED | DOC | Ground-plane pick can still fail | 1.5 T2, T3 |
| R-03 | MED | DOC | Cursor timeout should use wall time | 1.5 P2 |
| R-06 | MED | DOC | Message fields had no frames or times | 1.4 |
| R-08 | MED | DOC | Session-level metadata is not enough | 1.6 |
| R-09 | MED | CHECK | rosbag details glossed over | 1.6 |
| R-12 | MED | DOC | Candidate paths not recorded | 1.4 |
| R-14 | MED | DOC | No wall time or real-time factor | 1.4 |
| R-15 | MED | DOC | No schema version; messages inside the planner package | 1.4 |
| R-16 | MED | DOC | Action space differs from the deployed planner | 1.2 D6, 1.4 |
| R-19 | MED | CHECK ✔ | Highlight ignores the cursor pick entirely | 1.3 |
| R-20 | MED | DOC | A distance gate conflicts with pointing in a direction | 1.5 P5 |
| R-22 | MED | CHECK ✔ | Camera cannot move while the tool is active | 1.5 T4 |
| R-24 | MED | DOC | A bare `PointStamped` forces workarounds | 1.4, 1.5 T1 |
| R-27 | MED | CHECK (runtime) | Sim clock behaviour at reset unknown | Phase 0 |
| R-30 | MED | DOC | Expert-only data and drift | 1.4 |
| R-31 | MED | DOC | No split rule | 1.7 |
| R-33 | MED | DOC | No regression check on the autonomous planner | 1.9 Phase 4 |
| R-35 | MED | DOC | Build order hid problems until late | 1.9 |
| R-37 | MED | DOC | No pilot | 1.9 Phase 2 |
| R-38 | MED | DOC | No way to look at the labels | 1.9 Phase 4 |
| DG-19 | LOW | DOC | Bag volume over long sessions | 1.6 |
| R-23 | LOW | DOC | Near-ties measurable but no rule set | 1.7 |
| R-28 | LOW | CHECK ✔ | Ticks with no sample | 1.4 |
| R-34 | LOW | DOC ✔ | Line references are not pinned to a commit | Header |

✔ = checked against the code; see 1.11.

## 2.1 The label is lost

**DG-01 — Selection never leaves `getCursorSelectedTrajectory`** (BLOCKER)
- *Problem.* The function (`Planner.cpp:854`) picks the nearest trajectory
  and sets it current, but never assigns `lowestCostTrajIdx`. The manual
  branch returns early at `:2183`, so the current left/right model IDs (set
  only at `:1720`, `:1724`, `:1744`, all non-manual) are never updated.
- *Effect.* The label exists only as a local variable. No recording of
  existing topics recovers it.
- *Fix.* Publish it in `ManualGapSample`; set the model IDs in the manual branch.

**DG-02 — Candidates exist only as markers, then are deleted** (BLOCKER)
- *Problem.* Gap geometry, models, goals and feasibility go out only as
  `visualization_msgs` markers. The `Gap *` / `GapTube *` objects are deleted
  at `Planner.cpp:2186-2190`.
- *Effect.* The model's input is destroyed every tick; markers carry no IDs
  or flags.
- *Fix.* Structured `candidates[]` copied before the delete.

**DG-03 — Three rates** (BLOCKER)
- *Problem.* Gap detection runs in `laserScanCB` at 25 Hz, selection in the
  5 Hz planning loop, the cursor at 10 Hz, on different threads.
- *Effect.* A sample is only well defined at the planning tick; anything
  else needs an offline three-way join.
- *Fix.* Define and stamp the sample at the tick; copy scan and gaps
  together under `gapMutex_`.

**DG-08 — Click-to-move looks like a gap selection** (HIGH)
- *Problem.* `tryHandleTrajCreationPoint` returns `GAP` while tracking a
  click-created trajectory (`Planner.cpp:49-103`, called at `:862`).
- *Effect.* Those ticks are identical to real selections at the output.
- *Fix.* `source = CLICK_TO_MOVE`; excluded from labels.

## 2.2 Candidate set and identity

**DG-13 — Two definitions of the candidate set** (HIGH)
- *Problem.* The marker publisher (`:735`) and the selector each build their
  own list, with separate skip rules and hardcoded flags (`:740-741`).
- *Effect.* They agree by coincidence; drift mislabels the whole dataset.
- *Fix.* One list per tick, read by both.

**DG-14 — Dense `display_id` vs sparse trajectory index** (HIGH)
- *Problem.* `gapTrajs[i]` maps to `gapTubes[i]` (`:1230`) with empty
  entries for infeasible tubes; `display_id` skips those holes.
- *Effect.* The two indices differ whenever a candidate is infeasible.
- *Fix.* Array position is canonical; both indices are recorded.

**DG-15 — Stable gap identity not recorded** (HIGH)
- *Problem.* Per-point model IDs exist (`GapPointAssociator.cpp:125-139`)
  but are not saved; raw indices reshuffle every tick.
- *Effect.* "Held one gap for 2 s" and "switched ten times" look the same.
- *Fix.* Record the ID pair and gap bearing per candidate.

**R-21 — The model-ID pair** (HIGH, CHECK ✔)
- *Finding.* IDs come from a lifetime counter: never reused, never reset
  between episodes, kept through copy, manipulation and propagation.
  Association is per point within 1.0 m, so splits and merges change the
  pair for the same opening, and a point can inherit a neighbour's ID.
- *Effect.* A pair change is not proof the human switched. The lag shift
  (DG-09) depends on this.
- *Fix.* Key on the pair; use bearing as a second cue; count switches only
  when backed by mouse movement.

**R-19 — Highlight ignores the cursor pick** (MED, CHECK ✔)
- *Finding.* Markers are published before selection, and the highlight is
  driven by `selectedManualCandidateId_`, which only `/manual_traj_selection`
  sets. In cursor mode the operator never sees which gap is selected except
  through `curr_exec_dg_traj`.
- *Fix.* Select first, then publish; highlight the cursor pick by identity.

## 2.3 Cursor validity

**DG-06 — Cursor state never cleared** (HIGH)
- *Problem.* `hasCursorPos_` is set at `:722` and never reset.
  `Planner::reset()` (`:2611`) does not clear any manual-mode state.
- *Effect.* The planner uses the last cursor forever after the tool is
  switched off, and across episode resets.
- *Fix.* `cursor_live` flag; clear on `scenario_reset`.

**DG-07 — No staleness check; failed picks hold the old point** (HIGH)
- *Problem.* The tool updates its stored point only on a `MouseMove` whose
  `get3DPoint` succeeded (`continuous_gap_selection_tool.cpp:83`).
- *Effect.* Hovering over empty space silently keeps the old point.
- *Fix.* Ground-plane pick re-projected every publish; `pick_ok` flag.

**DG-20 — Tool republishes the pre-reset point** (HIGH)
- *Problem.* After a reset the tool resends the old point within 100 ms,
  undoing any clearing in the planner.
- *Fix.* Ignore the cursor until `last_move_wall` is later than the reset.

**DG-05 — No abstain and no gate** (HIGH)
- *Problem.* The nearest-candidate search (`:869-890`) has no threshold.
- *Effect.* A cursor far from everything still yields a confident label.
- *Fix.* Record distance and bearing difference; gate offline.

**R-17 — A still mouse does not point where the operator sees it** (HIGH, CHECK ✔)
- *Finding.* The shipped view follows `rto/base_link`. The tool stores a 3D
  map point, so as the robot drives the screen scrolls under a still mouse
  but the published point stays fixed in the map.
- *Fix.* Store the screen position; re-project each publish.

**R-02 — Ground-plane pick can still fail** (MED)
- *Problem.* The ray misses the plane above the horizon, and no event
  arrives once the pointer leaves the window.
- *Fix.* `pick_ok = false` in both cases.

**R-03 — Cursor timeout should use wall time** (MED)
- *Problem.* A sim-time timeout flaps if the sim does not run at real time.
- *Fix.* Wall clock for liveness.

**R-20 — A distance gate conflicts with pointing in a direction** (MED)
- *Problem.* Pointing well past a gap along its bearing is a clear choice
  that a metric gate rejects.
- *Fix.* Record both distance and bearing difference; decide offline.

**R-22 — Camera cannot move while the tool is active** (MED, CHECK ✔)
- *Finding.* The tool never forwards mouse events to the view controller.
  Switching tools to move the camera deactivates it.
- *Fix.* With the following top-down view, accept it. Optionally forward
  wheel events for zoom.

**R-24 — A bare `PointStamped` forces workarounds** (MED)
- *Problem.* Tool state had to be inferred from timeouts and point equality.
- *Fix.* `CursorState` message.

**R-05 — First design changed robot behaviour needlessly** (HIGH)
- *Problem.* Timeout, distance gate and `HELD` all altered control, against
  the "record facts, filter offline" principle, and `HELD` relied on
  unverified ID stability.
- *Fix.* Flag only; behaviour changes at reset alone.

## 2.4 What the label means

**DG-04 — Nearest-endpoint is a proxy for intent** (HIGH)
- *Problem.* The cursor is matched to a trajectory endpoint
  (`:869-890`), which depends on the goal placer and trajectory generator.
- *Effect.* Changing the pipeline shifts label meaning; near-ties are
  indistinguishable from confident picks.
- *Fix.* Record the cursor, all distances and full geometry so labels can be
  recomputed.

**R-18 — Choice is matched against a set the human has not seen** (HIGH)
- *Problem.* Tick *t*'s cursor is matched to tick *t*'s candidates before
  they are drawn. A jumping endpoint flips the selection with no input.
- *Options.* Latch by identity; hysteresis; or keep and filter switches with
  no mouse movement behind them (possible via `last_move_wall`).
- *Fix.* Version 1 keeps hover-nearest and filters offline; decide D2 after
  the pilot.

**DG-09 — Reaction lag unmodelled** (HIGH)
- *Problem.* The cursor reflects an earlier rendered frame plus human
  reaction time.
- *Effect.* Pairing observation(t) with action(t) leaks the future.
- *Fix.* Offline shift by identity (1.7).

**R-23 — Near-ties: no rule set** (LOW)
- *Fix.* Threshold chosen from pilot data.

## 2.5 What the model needs to see

**R-10 — The goal is not in the sample** (BLOCKER)
- *Problem.* Neither earlier document records where the robot is going.
- *Effect.* The choice cannot be predicted without it.
- *Fix.* Goal and local waypoint in the sample.

**R-11 — The human sees more than the model will** (HIGH, CHECK)
- *Problem.* RViz may show the map, global plan and pedestrians outside
  sensor view.
- *Effect.* Choices driven by that become noise.
- *Fix.* Fix the model's inputs (D5); cut RViz to match; save the config.

**R-12 — Candidate paths not recorded** (MED)
- *Fix.* Downsampled path and tube length per candidate.

**R-16 — Action space differs from the deployed planner** (MED)
- *Problem.* "Go straight" and "wait" are disabled, so the human must always
  pick a gap.
- *Fix.* Flag ticks where either would have applied (D6).

**R-01 — Autonomous pick and cost are not free** (HIGH)
- *Problem.* Manual mode returns before costs are computed; the scoring
  block has side effects.
- *Fix.* Dropped from version 1 (D4). Needs a side-effect-free scoring
  function if wanted later.

## 2.6 Frames and time

**DG-10 — Cursor frame assumed to be `map`** (HIGH)
- *Problem.* The tool stamps with RViz's fixed frame; the planner always
  applies map-to-odom (`:721`). The shipped config uses `map`.
- *Fix.* Transform from the message's own frame.

**DG-11 — Transform taken at latest time** (MED)
- *Problem.* `tfCB` (`:690`) looks up at `ros::Time(0)`.
- *Fix.* Look up at the cursor's stamp; record the result.

**DG-12 — RViz sim time assumed** (MED)
- *Problem.* If RViz starts before `use_sim_time` is set
  (`start_trials.launch:77`), cursor stamps are wall time. Low risk when
  RViz is started by that launch file.
- *Fix.* Clock check in the planner; test-bag check.

**R-06 — Message fields had no frames or times** (MED)
- *Fix.* Stated per field in 1.4.

**R-14 — No wall time or real-time factor** (MED)
- *Problem.* Human lag is in wall time; all stamps were sim time.
- *Fix.* Wall stamp in each sample.

## 2.7 Reset and episodes

**R-04 — Reset is never called on an episode reset** (HIGH, CHECK ✔)
- *Finding.* `Planner::reset()` runs only after a planning failure. The
  call from the plugin is commented out.
- *Fix.* Subscribe to `scenario_reset` directly.

**R-25 — Samples straddle the reset** (HIGH, CHECK, runtime)
- *Problem.* A tick can mix a pre-reset scan with a post-reset pose.
- *Fix.* `in_reset_window` flag; excluded from labels.

**R-26 — Gap estimators survive the teleport** (HIGH, CHECK ✔)
- *Finding.* Previous gaps and their models are never cleared on reset. Any
  new point within 1.0 m of an old one inherits its model.
- *Fix.* Clear previous gaps on `scenario_reset` (manual mode only, so the
  benchmark path is unchanged).

**R-27 — Sim clock at reset** (MED, CHECK, runtime)
- *Problem.* A backwards jump clears tf buffers and breaks bag ordering.

**DG-18 — Episode boundaries and metadata** (MED)
- *Problem.* `scenario_reset` (`task_generator_node.py:116`, latched
  `std_msgs/Int16`) and run parameters are not captured.
- *Fix.* `episode_id` in the sample; metadata files per session.

**R-13 — No episode outcome** (HIGH)
- *Fix.* Record collision and goal-reached events.

**R-28 — Ticks with no sample** (LOW, CHECK ✔)
- *Finding.* Three early returns before candidates exist.
- *Fix.* Publish on every exit path; `tick_seq` always increments.

## 2.8 Recording

**DG-17 — No rosbag infrastructure** (MED)
- *Problem.* The only recorder is a CSV logger in `arena_evaluation`.
- *Fix.* `record_il.launch`.

**DG-16 — Namespacing** (MED)
- *Problem.* Planner topics are robot-relative (`Planner.h:486`); cursor
  topics are global (`:187`, `:189`).
- *Fix.* Record list names both; cursor topic names become parameters.

**DG-19 — Bag volume** (LOW)
- *Fix.* No markers; compression; splitting.

**R-07 — Record list was incomplete** (HIGH)
- *Fix.* Goal, global plan, `acc`, map, collisions, task info added.

**R-08 — Session-level metadata is not enough** (MED)
- *Fix.* Per-episode task info in the bag; `git diff` saved.

**R-09 — rosbag details** (MED, CHECK)
- *Problem.* Split needs a size; latched topics land only in the first
  split; crashes leave `.bag.active`; low disk stops recording quietly; a
  slow recorder drops messages.
- *Fix.* Options and checks in 1.6.

**R-15 — No schema version** (MED)
- *Fix.* `schema_version`; separate message package.

## 2.9 Learning-side

**R-29 — Ticks are not independent samples** (HIGH)
- *Problem.* One held gap produces many near-identical ticks; many ticks
  have a single candidate.
- *Fix.* Targets and reports in decisions.

**R-32 — No collection protocol** (HIGH)
- *Problem.* No fixed operator instruction, coverage or targets.
- *Fix.* Protocol in 1.8.

**R-30 — Expert-only data and drift** (MED)
- *Fix.* `SOURCE_MODEL` reserved for later corrective collection.

**R-31 — No split rule** (MED)
- *Fix.* Split by episode.

**R-33 — No regression check on the autonomous planner** (MED)
- *Fix.* Guard new code; benchmark before and after.

## 2.10 Process

**R-34 — Line references not pinned** (LOW) — done, see header.

**R-35 — Build order hid problems** (MED)
- *Fix.* Thin slice first.

**R-36 — No automatic consistency checks** (HIGH)
- *Fix.* Checks in 1.6.

**R-37 — No pilot** (MED)
- *Fix.* Phase 2.

**R-38 — No way to look at the labels** (MED)
- *Fix.* Replay viewer in Phase 4.

---

# System map (for reference, at the pinned commits)

**Producer — `arena/arena-rosnav/utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.cpp`**

- Shortcut key `g`. On `activate()`, a timer publishes the last mouse-hover
  3D point as `geometry_msgs/PointStamped` on `/gap_selection_point` at 10 Hz
  (line 29), stamped in RViz's fixed frame at the time of the last move.
- Left-click also publishes one point on `/traj_creation_point`
  (click-to-move, dynamic_gap commit `f27f357`).

**Consumer — `planners/dynamic_gap/src/Planner.cpp`**

- `/man_gap` rosparam gates `manualSelectionMode_` (`:118`; set at
  `arena_bringup/launch/start_trials.launch:95`).
- `rvizPublishPointSubsciberCB` (`:715`) transforms the cursor map to odom.
- Gap detection, association and estimation run in `laserScanCB` (`:272`).
- `runPlanningLoop` (`:1967`) builds candidates, publishes markers (`:2140`),
  then selects (`:2151`). The manual branch returns at `:2183`.

**Rates (robot `rto`)**

| Stream | Rate | Source |
|---|---|---|
| Laser scan | 25 Hz | `rto/model_params.yaml:39` |
| Planning loop | 5 Hz | `configs/mbf/local/dgap_local_planner_params.yaml:5` |
| Cursor | 10 Hz | `continuous_gap_selection_tool.cpp:29` |

Scan: 512 beams, ±pi, `max_range` 5.0 m.

---

# Unfiled / new issues

_Append below. Give an ID and move into a section once triaged._
