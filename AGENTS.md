# Base Casting ABB 6700 Agent Notes

This file is for Codex/agents taking over this experiment. It explains the
active architecture, script responsibilities, and hard boundaries. For algorithm
details, read `PRINCIPLES.md`. For how to run the workflow, read `README.md`.

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
  point CSVs, and summary tables.

- `scripts/optimal_y_selection.py`
  Lightweight selector for dual-robot rail experiments. It chooses one candidate
  per region by `max(abs(world_y))` over processing waypoints only. Do not add
  per-waypoint IK optimization here.

- `scripts/runs/`
  Concrete parameter-run entry points. They import the reusable scripts above
  and set model X/Y/Z, angle lists, output directories, and feed variants for
  one experiment batch.

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
- Risky or exploratory algorithms should start in `experimental_algorithms/`
  unless they are already part of this experiment's reusable script layer.

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
