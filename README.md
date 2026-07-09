# Base Casting ABB 6700 Experiment

This is the project-specific offline workspace for the large base-casting
polishing study. For agent handoff rules, read `AGENTS.md`. For algorithm
principles, read `PRINCIPLES.md`.

## 中文说明

这是底座大型结构件 ABB 6700 抛光实验目录。这里的脚本用于离线读取软件导出的
`.rsp.json` 选面快照，必要时先做区域分区，然后按窗口/固定构型策略导出 ABB
RAPID 路径，最后可选地做导轨 Y 位置选择。

## 当前流程

推荐按四步走：

1. 在软件中选择抛光面，并导出脚本测试输入。
   输出文件是：

```text
experiments/base_casting_abb6700/inputs/latest_script_test.rsp.json
```

2. 如果某个 selected region 需要分区，运行分区预处理。例如只分第 1 个 region：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py --regions 1
```

分区结果写到：

```text
experiments/base_casting_abb6700/inputs/latest_partitioned.rsp.json
experiments/base_casting_abb6700/inputs/latest_partitioned_manifest.json
```

3. 运行窗口/构型 RAPID 导出：

```powershell
python experiments\base_casting_abb6700\scripts\window_conf_export.py
```

4. 如果是双机器人导轨批量实验，再运行 `scripts/runs/` 中对应的 optimal-y
runner。

导出脚本的输入优先级是：

```text
inputs/latest_partitioned.rsp.json
inputs/latest_script_test.rsp.json
project/test-0704-selected.rsp.json
```

所以只要第 2 步生成了 `latest_partitioned.rsp.json`，第 3 步会自动读取分区后的
输入。

## 常用命令

查看分区结果但不写文件：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py --regions 1 --dry-run
```

输出完整 manifest，包括 face id 列表：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py --regions 1 --dry-run --dump-manifest
```

同时指定多个 selected region：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py --regions 1,6,8
```

重新导出 RAPID：

```powershell
python experiments\base_casting_abb6700\scripts\window_conf_export.py
```

运行某个具体参数实验：

```powershell
python experiments\base_casting_abb6700\scripts\runs\optimal_y_score_x3500_z440.py
```

## 目录职责

- `inputs/`
  软件导出的脚本测试输入、分区后的输入、分区 manifest。

- `scripts/`
  可复用实验脚本和策略模块。

- `scripts/runs/`
  具体参数批跑入口，例如固定 X/Z、扫描 Y、扫描转台角度、optimal-y 选择。

- `configs/`
  可选参数记录。

- `results/`
  RAPID、TXT、点位 CSV、汇总 CSV、summary JSON、实验报告。

- `experimental_algorithms/`
  更激进或尚未稳定的算法原型预留区。

## 脚本说明

- `scripts/region_partition_preprocess.py`
  分区预处理 CLI。默认读取 `inputs/latest_script_test.rsp.json`，只处理
  `--regions` 指定的 selected region，输出 `latest_partitioned.rsp.json` 和
  `latest_partitioned_manifest.json`。

- `scripts/region_partitioning.py`
  分区算法模块。实现 face 邻接、局部近似平面/曲面划分、平面窄通道检测和小碎片合并。
  具体算法原则见 `PRINCIPLES.md`。

- `scripts/window_conf_export.py`
  核心 RAPID 导出脚本。负责读取 selected regions、扫描工件位姿和转台角度、窗口筛选、
  生成 boundary-UV raster 路径、写 RAPID/TXT/CSV/summary。

- `scripts/optimal_y_selection.py`
  双机器人导轨 Y 位置选择模块。按 processing waypoint 的
  `max(abs(world_y))` 为每个 region 选择候选路径。

- `scripts/runs/*.py`
  具体实验入口。它们导入上面的可复用模块，设置 X/Y/Z、角度、输出目录和 feed variant。

## 分区结果怎么看

分区不会修改项目 schema。`.rsp.json` 里仍然只有 `selected_path_face_regions`。
子区标签写在 manifest 中，例如：

```text
1.1 planar
1.2 planar
1.3 curved
```

当前 `latest_partitioned_manifest.json` 会记录每个子区的：

- 原始 region 编号；
- 子区标签；
- `planar` 或 `curved`；
- face 数；
- 面积；
- face id 列表。

后续 `window_conf_export.py` 不理解 `1.1` 这种标签，它只看到多个独立 region 并逐个导出。

## 当前策略摘要

- 分区预处理只处理用户指定的 region，其它 region 不动。
- 先做局部曲率/近似平面分区。
- 只在 planar patch 内做窄通道检测。
- 窄通道基于 scanline interval 连通关系，不使用一条全局 XYZ/UV 直线贯穿切面。
- 每个输出 region 后续独立规划和导出。
- RAPID 导出使用 base window、boundary-UV raster、`base_y_aligned` 姿态、固定
  confdata 和 `ConfL \Off;`。

## STEP/CAD precise-surface experiment entry

The existing STL/VTK workflow remains unchanged. STEP/CAD v1 is an independent
experimental path through `scripts/step_cad_pipeline.py`; it reads SolidWorks
`STEP AP242` with OpenCascade, indexes CAD faces with geometry signatures, and
samples path points directly from B-Rep surfaces instead of interpolating STL
triangles.

Basic flow:

```powershell
python experiments\base_casting_abb6700\scripts\step_cad_pipeline.py index --step part.step --manifest part.step_manifest.json
python experiments\base_casting_abb6700\scripts\step_cad_pipeline.py pick --manifest part.step_manifest.json --regions 1 --polygons-json picks.json --output part.step_pick_manifest.json
python experiments\base_casting_abb6700\scripts\step_cad_pipeline.py sample --step part.step --manifest part.step_pick_manifest.json --output samples.json --spacing 20 --point-step 20
```

`index` writes `source_step`, CAD face signatures, and empty selection fields.
`pick` attaches `selected_cad_face_regions` and `n_1/n_2` clip patches. `sample`
uses OpenCascade trimmed-face classification, so holes are excluded by CAD
topology and the generated points include model-space position, normal, tangent,
and UV.

## 测试

运行本实验目录测试：

```powershell
.venv\Scripts\python.exe -m pytest experiments/base_casting_abb6700/tests -q
```

当前测试覆盖：

- 输入优先级；
- 分区预处理输出可被项目 loader 读取；
- 窄通道会分区；
- 没有窄通道的连续区域不会被全局线误切；
- 曲面/近似平面阶段能分出不同 patch；
- 未指定 region 保持原样。
