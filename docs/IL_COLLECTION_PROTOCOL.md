# IL collection protocol (template)

Plan section 1.8 of `IL_DATA_COLLECTION_PLAN.md`. Fill in every **TODO**
before the first real session; the tooling below already exists.

## 1. Decisions to fill in

| Item | Value |
|---|---|
| Instruction given to operators (what to optimise: reach the goal quickly? safely? like a person would?) | **TODO** |
| Operators, and their warm-up (e.g. 5 min unrecorded on `map_empty`) | **TODO** |
| Session length and breaks | **TODO** (suggest ≤ 20 min, then a break) |
| Maps, scenarios and pedestrian densities to cover | **TODO** |
| Target size, in decisions (not ticks) per map/density | **TODO** |
| RViz config (`view.rviz` is saved with each session; plan R-11, PL-26) | **TODO**: shipped `arena.rviz`, or a cut-down view matching the model inputs (D5) |
| `data_root` (outside every git repo, D7) | **TODO** |
| Thresholds and reaction lag (from the pilot, plan Phase 2) | **TODO** |

Plan defaults in effect until changed: D1 what is under the cursor on screen
(re-projected every publish); D2 hover-nearest; D3 flag only; D4 no autonomous
pick; D6 gaps only.

## 2. Running a session

```
source <workspace>/devel/setup.bash
roslaunch arena_bringup start_trials.launch man_gap:=true \
    record_il:=true il_data_root:=<data_root> il_operator:=<id> \
    [il_session:=<name>] [map_file:=... other usual args]
```

In RViz press `g` (ContinuousGapSelectionTool). Hover the mouse over the gap
to take; the selected number turns green. Left-click drives to a point
(click-to-move; those ticks are not labels). Switching tools stops the robot.
Do not use other RViz tools during a session.

After the session, write anything odd into
`<data_root>/il_gap_selection/<session>/notes.txt`.

## 3. Checks before a long session (plan 1.10)

Run a 2-minute test session, then:

```
il_tools/il_check_session <data_root>/il_gap_selection/<session>
```

- [ ] All automatic checks pass.
- [ ] `usable_ticks` is most of `cursor_ticks`; `ticks_with_choice` > 0.
- [ ] Playback with `il_tools/il_replay_viewer` shows the selected path where
      the robot went.
- [ ] Mouse held still for 10 s: the selection stays (no hold-still ticks).
- [ ] Tool switched off: the robot stops within 0.5 s.
- [ ] Pointer moved out of the RViz window: the robot keeps the last
      selection; those ticks have `cursor_live = false`.
- [ ] After an episode reset, `episode_id` increments and ticks within 1 s
      of the reset have `in_reset_window = true`.
- [ ] A click-to-move produces `SOURCE_CLICK_TO_MOVE` ticks.
- [ ] `cursor.header.frame_id` is `map` and no clock warning is logged by the
      planner ("is RViz on sim time?").

## 4. After collection

```
il_tools/il_check_session <sessions...> --yaml checks.yaml
il_tools/il_build_dataset --out <dataset_dir> --lag <s> \
    --max-cursor-dist <m> --max-bearing-diff <rad> --min-tie-margin <m> <sessions...>
```

Keep the session folders unchanged; the dataset can always be rebuilt from
them with other thresholds.
