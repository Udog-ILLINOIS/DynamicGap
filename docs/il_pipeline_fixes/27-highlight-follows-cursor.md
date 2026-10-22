# 27 — Highlight followed the numeric selector, not the cursor pick

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`
**Behaviour change:** Manual mode only: the green number is the gap the cursor selected this tick.

## Problem
Markers were published before selection (`Planner.cpp:2140-2152` before the change), and the green highlight compared `display_id` with `selectedManualCandidateId_`, which only `/manual_traj_selection` sets. In cursor mode no number was ever green.

## Why it mattered for data collection
The operator could not see which gap the planner took from the cursor, so they could not correct a wrong pick.

## What changed
`runPlanningLoop` (manual mode) runs `selectManualTrajectory` first and then `publishManualCandidateMarkers(selectedIdx)`, both on the same candidate list (issue A/B doc). The selected candidate is green. If there is no cursor pick, a candidate whose number matches `/manual_traj_selection` is green, as before.

## Effects
- Autonomous mode: no change.
- Manual mode: the highlight shows the cursor pick.

## Test
Covered by the sim smoke test and code review.
