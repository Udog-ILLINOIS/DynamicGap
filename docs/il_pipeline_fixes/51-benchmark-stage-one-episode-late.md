# 51 — Benchmark stages on the same map ran one episode late

**Status:** Fixed · **Repo(s):** arena-rosnav · **Files:** `task_generator/task_generator/tasks/modules/benchmark.py`, `task_generator/task_generator/task_config.py` (new), `task_generator/scripts/filewatcher.py`, `task_generator/task_generator/tasks/task_factory.py`, `task_generator/task_generator/task_generator_node.py`, `task_generator/test/test_benchmark_stage_timing.py` (new), `test_double_resume.py`, `test_benchmark_il_record.py`, `test_filewatcher_config_file.py`
**Behaviour change:** With `tm_modules:=benchmark`, a stage's config is used from the stage's first episode on, a run no longer starts one extra reset after the last episode, and a change of `tm_robots`/`tm_obstacles` between stages relaunches. Every launch: at each reset the task modes get the server's current config if it changed.

## Problem
A reset runs `before_reset` of the modules, places robots and obstacles from the task config, then runs `after_reset` (`task_factory._reset_task`). The benchmark counted episodes in `after_reset`. When a stage's episodes were used up it switched to the next stage and wrote that stage's config, but the reset that starts the new stage had already placed everything from the old config. Stages that change the map relaunch and were not affected; stages on the same map ran one episode late: the first stage got one extra episode, every following stage ran in its predecessor's slot, and the last stage's scenario never ran.

Seen on 2026-10-26, `il_dataset_small_v1` (CS1–CS4, one episode each, `gru_learn`): the episodes were CS1, CS1, CS2, CS3 (start, goal, pedestrian spawn points and static obstacles in the scan all match those files), the benchmark log reported stages 1–4. Every suite with several stages on one map is affected (50ped_daniel, the IL suites), including all sessions marked preliminary.

A second, smaller gap: the written file reaches the task modes asynchronously (filewatcher → `task_generator_server` → reconfigure callback of each task mode). Moving the count alone would leave a window in which a reset still reads the old scenario.

## What changed
- `Mod_Benchmark.before_reset` counts the episode (and so switches stage and writes its config) before the reset places anything; `after_reset` does nothing.
- The stage's config is overlaid on the backup and written once (`_taskgen_write(..., base=)`, which returns it), instead of restoring the backup and then writing.
- Without a relaunch, `_reincarnate` then sets that whole config on `task_generator_server` itself with a blocking `update_configuration` (`_apply_task_config`), so the server holds it before the reset continues; the filewatcher applies the same file later. A failure is logged as an error.
- The config-to-parameter mapping moved from the filewatcher into `task_generator/task_config.py` (`task_config_params`), used by both; lists of numbers are now encoded too (`;`-joined strings).
- `CombinedTask._reset_task`, after the modules' `before_reset`, gives the robot and obstacle task modes the server's current config (`_sync_task_config`) when it changed since the last reset: their reconfigure callbacks are asynchronous. This holds for every task mode. A task mode that fails to load it logs an error and keeps its config, as when the callback fails.
- When a module ends or restarts the run in `before_reset` (benchmark completed, relaunch), no further module runs, nothing is placed, and the node publishes no `/scenario_reset`, so no episode is started in the old stage's recordings.
- A change of `tm_robots` or `tm_obstacles` between stages now relaunches, like a map or robot change: task modes are launch parameters, and without a relaunch the old one kept running.

## Effects
- Each stage runs its own scenario, with its episode count. The benchmark log (`S`/`E` lines) now matches what ran.
- A run no longer starts one more episode after the last one before it ends.
- Each reset reads the server's parameters once (one master call).
- Not changed: the `RANDOM_seed` reseeding of the global RNG (`constants._cb_reconfigure`) is still asynchronous; the config file is still written in place (not atomically), as before.

## Test
`task_generator/test/test_benchmark_stage_timing.py`: the scenario active at each reset for stages of 1/1/1/1 and 2/1/3 episodes (fails on the old code with `0.json` twice); the config written once onto the base; the whole written config (base values included) applied with `update_configuration`, a failure reported; a task-mode change requires a relaunch; `task_config_params` (names, lists, dicts); `_sync_task_config` (applies a changed config once, a failing task mode keeps its config, runs after the modules and before placement, no further module and no placement after a module ended the run). `test_double_resume.py`: no `/scenario_reset` when the run ended in the reset.

End to end (2026-10-26, master 11312, no display, planner driving): suite CS1 ×1, CS2 ×2, then a stage with `tm_obstacles: random` (4 pedestrians, 2 static). At each `/scenario_reset` the server held CS1, CS2, CS2 and the pedestrians spawned at those files' positions; the random stage relaunched ("task mode change requires restart") and ran 4 random pedestrians; no "task config not applied" error; the run completed after the fourth episode.

```
cd src/arena/arena-rosnav/task_generator/test && python3 -m unittest discover -p 'test_*.py'
```
