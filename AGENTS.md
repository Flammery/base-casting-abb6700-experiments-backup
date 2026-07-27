# Base Casting ABB 6700 Agent Notes

This file is for Codex/agents taking over this experiment. It explains the
active architecture, script responsibilities, and hard boundaries. For algorithm
details, read `PRINCIPLES.md`. For how to run the workflow, read `README.md`.
Before changing partition architecture, read `DECISION_LOG.md`; for persisted
fields read `MANIFEST_SCHEMA.md`; for known failures and acceptance checks read
`TROUBLESHOOTING.md` and `VALIDATION.md`. Read `COORDINATE_SYSTEMS.md` before
changing placement, workobject, tooldata, transforms, or RAPID coordinates.

## Active Architecture

Use `src/` as the reusable software library and this experiment directory as the
project-specific layer.

```text
software export
  -> inputs/latest_script_test.rsp.json
  -> optional region partition preprocessing
  -> inputs/latest_partitioned.rsp.json
  -> window/conf RAPID export
  -> optional optimal Y-position selection
  -> results/
```

Default input priority for export scripts:

```text
inputs/latest_partitioned.rsp.json
inputs/latest_script_test.rsp.json
project/test-0704-selected.rsp.json
```

Do not change `src/` or the UI/project schema for experiment-only behavior. The
project still stores only `selected_path_face_regions`; labels such as `6.1` and
`6.2` belong in the partition manifest.

Manual partitioning is implemented in `ui/manual_partition_dialog.py` and
`scripts/manual_region_partitioning.py`. Keep the three modes distinct:
`boundary` and `slab` use drawn barriers, while `pick` uses rectangle or polygon
clip areas. The dialog writes manifest version 2 with per-region barriers and
picked polygons. The `.rsp.json` file remains schema-compatible and must not be
used to persist the drawn geometry.

When changing manual partitioning, verify both the preview and the exporter:
the preview should show the same raster-domain clip that
`window_conf_export.py` applies to generated samples. Keep holes from the mesh
boundary as exclusions, and keep each picked polygon as an independent output
patch.

## Script Responsibilities

- `scripts/region_partition_preprocess.py`
  CLI for stage 2 preprocessing. Reads `latest_script_test.rsp.json` by default,
  partitions only explicitly requested 1-based selected region numbers, writes
  `latest_partitioned.rsp.json`, and writes `latest_partitioned_manifest.json`.

- `scripts/region_partitioning.py`
  Reusable experiment-layer partition algorithm. It builds face adjacency, splits
  curved and near-planar patches, applies planar neck/bottleneck splitting, and
  returns plain face-id regions plus manifest records. Keep algorithm details
  documented in `PRINCIPLES.md`.

- `scripts/window_conf_export.py`
  Core phase-1 window/conf exporter. It reads selected regions, scans workpiece
  poses and turntable angles, filters each region by the base machining window,
  generates boundary-UV raster paths, and exports per-region ABB RAPID modules,
  point CSVs, and summary tables. Each new RAPID module carries a
  `RSP_EXPERIMENT_META_V1` comment so the Qt importer can restore and verify the
  scene model placement without confusing it with wobjdata.

- `scripts/raster_domain.py`
  Experiment-owned manual-v2 planner. It builds the 2D chart domain, derives
  per-patch scan axes, subtracts hole intervals, ray-projects samples onto the
  selected STL cells, and returns XYZ plus facet normals. Do not replace this
  with STL cell coloring or mesh clipping.

- `experimental_algorithms/hole_aware_raster.py`
  Planner used selectively by default UI/runner `auto` mode and forced only by
  CLI `--planner hole-aware`. It groups raster runs into cells and creates
  deterministic complete-cell paths; the exporter retracts and transfers above
  the surface between cells. It accepts manual-v2 chart samples and projected
  face-id samples. Ordinary auto patches stay on the normal raster fast path.

- `scripts/optimal_y_selection.py`
  Lightweight selector for dual-robot rail experiments. Ordinary and hole-aware
  regions use `max(abs(world_y))` over processing waypoints. Internally
  validated avoidance rows instead minimize absolute TCP local-Z roll and then
  maximize sampled robot clearance. Unresolved avoidance rows remain available
  for RobotStudio diagnosis but do not enter optimal output.

- `scripts/optimal_y_score_configurable.py`
  Stable configurable CLI/UI runner. It scans the user-supplied installation
  range and turntable angles, dispatches the reusable planners and optional
  robot-pose avoidance, and writes candidate/optimal/diagnostic reports.

- `scripts/robot_config_override.py`
  Loads a main-application `robot_studio_mechanism_config` export for avoidance
  trials. It validates six-axis MDH and nonzero per-link envelopes, then applies
  the exported robot configuration/seed and enables configured segment radii.

- `scripts/region_selectors.py` and `experimental_algorithms/robot_pose_avoidance.py`
  Region selectors and the five-entry TCP-Z roll library are active only for
  user-selected avoidance regions. Each sampled waypoint uses the previous
  successful joint solution as its next IK seed; confdata quadrant changes are
  diagnostic only, while actual joint jumps, J5 margin, FK collision, and
  clearance decide acceptance. Do not present this sampled result as ABB
  interference validation.

- `experimental_algorithms/support_surface_growth.py`
  Recovers an avoidance patch's full near-planar support from final path
  `face_id` seeds. The Avoidance Settings dialog persists one model-coordinate
  UVN volume per resolved planning label; only non-support cells intersecting
  that volume enter the experimental wall mesh. Keep the sidecar separate from
  `.rsp.json`, and preserve the yellow machining / green support / red wall /
  translucent gray volume preview whenever thresholds or bounds change.

- `scripts/robotstudio_package.py`
  Packages selected Optimal-Y RAPID into one job per region/patch. It separates
  exported tooldata/wobjdata into CalibData, preserves their original names and
  values, and writes a RobotStudio job manifest. It does not perform robot
  validation.

- `robotstudio_addin/`
  RobotStudio 6.08 add-in that recognizes a generated station after RobotStudio
  opens it normally, waits for its virtual controller, and loads the matching
  CalibData/path sidecars. Generated stations reuse the calibrated template
  controller sequentially; validate one station at a time.

- `scripts/runs/`
  Historical and concrete fixed-parameter experiment entries. They import the
  reusable scripts above and preserve earlier standalone batches. New UI and
  configurable runs use `scripts/optimal_y_score_configurable.py`.

- root-level `window_conf_export_*.py` wrappers
  Historical compatibility entry points. Keep real implementations under
  `scripts/runs/`.

## Hard Rules

- Keep project-specific scan loops, partition experiments, result naming, and
  batch analysis under `experiments/base_casting_abb6700/`.
- Generated RAPID/CSV/JSON reports go under `results/`; script-test inputs and
  partitioned inputs go under `inputs/`.
- Do not merge all selected faces into one long path. Each selected or
  preprocessed region is planned and exported independently.
- Use the base machining window before exporting a region.
- Use region boundary-UV raster axes. PCA is only a fallback when boundary axes
  are unavailable.
- Use `base_y_aligned` TCP orientation.
- Use fixed confdata by base Y and emit `ConfL \Off;`.
- Do not make full per-waypoint IK the main batch loop.
- Avoidance selector `N` means source region N (including its patches), while
  `N-M`/`N_M`/`N.M` means one patch. Never interpret a selector as a hole-aware
  cell. Unselected planning regions must retain the former auto/base-y path.
- Do not suppress a geometrically valid candidate because the experimental
  numerical IK did not converge. Keep the baseline path with an explicit
  `ik-unresolved`/diagnostic status, but permit only `baseline-validated` or
  `alternative-validated` avoidance rows to enter `optimal_paths`.
- Risky or exploratory algorithms should start in `experimental_algorithms/`
  unless they are already part of this experiment's reusable script layer.
- Hole-aware accepts either a complete manual-v2 `raster_chart + clip_polygon`
  domain or a raw projected face-id region with no explicit polygon metadata.
  Never interpret explicit excludes without their chart/clip, bridge a gap with
  processing motion, or silently fall back to legacy. Cell-to-cell lifted
  motion is not collision, clearance, reachability, or IK validation.
- For manual manifest v2, the raster polygon owns the machining boundary. STL
  cells only provide ray hits and normals. Preview colors use a raster texture,
  never cell-centroid classification or clipped replacement geometry.
- Inherit tool and workobject names, TCP geometry, picked origin, and base
  placement data from the exported project. Experiment X/Y/Z/RZ must update
  model placement and wobj together through `placement_for()`. Use world/base
  coordinates for machining-window checks and wobj coordinates for robtargets.
- Never change tool TCP to compensate orientation, never rotate picked origin
  twice, and never treat the fallback tool load as calibrated data.
- RobotStudio scene component identity/model placement and RAPID wobj
  identity/data are separate concerns. Never derive one name from the other.
- RobotStudio export must preserve both plain region labels (`2`) and partition
  labels (`1_2`), explicitly include RZ in station filenames, write stations
  under `optimal_paths/<region_label>/`, and never overwrite the template.
- Keep each generated `.rsstn` with its `.mod` files and matching
  `.robotstudio_job.json`; opening a station is what triggers its RAPID switch.

## 中文说明

本文件给接手实验的 Codex/agent 看，重点是架构、脚本职责和边界。

当前流程是：

```text
软件导出选面
  -> inputs/latest_script_test.rsp.json
  -> 可选：指定 region 分区预处理
  -> inputs/latest_partitioned.rsp.json
  -> window_conf_export.py 导出 RAPID
  -> 可选：optimal_y_selection.py 选择导轨 Y 位置
  -> results/
```

脚本分工：

- `region_partition_preprocess.py`：分区预处理入口，只处理用户指定的
  selected region，例如 `--regions 6`。
- `region_partitioning.py`：分区算法模块，负责曲面/近似平面划分和平面窄通道分解。
- `window_conf_export.py`：核心 RAPID 导出脚本。
- `optimal_y_selection.py`：导轨 Y 位置轻量选择器。
- `scripts/runs/`：具体参数实验入口。

不要为了实验方便修改 `src/` 或项目保存格式。`.rsp.json` 里仍然只保存
`selected_path_face_regions`；`6.1`、`6.2` 等子区标签只写在 manifest 里。
