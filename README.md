# Base Casting ABB 6700 Experiment

This is the project-specific offline workspace for the large base-casting
polishing study. For agent handoff rules, read `AGENTS.md`. For algorithm
principles, read `PRINCIPLES.md`.

设计决策见 `DECISION_LOG.md`，manifest 字段见 `MANIFEST_SCHEMA.md`，已知故障见
`TROUBLESHOOTING.md`，坐标同步见 `COORDINATE_SYSTEMS.md`，提交前验收见
`VALIDATION.md`。带孔连续路径的行为和限制见 `HOLE_AWARE_PLANNER.md`。

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

## 手动 UV 分区

需要按实际加工区域圈选时，在实验面板中执行“手动区域划分”，选择目标
`region` 后选择一种模式：

- **面边界式**：使用拉线，在局部 UV 域内切开边界两侧的区域；
- **贯穿式**：使用拉线，按整条分割带切开区域；
- **圈选区域式**：使用矩形或多边形，只保留圈出的加工区域，可连续圈选多个区域。

圈选模式下，矩形由拖拽生成，多边形靠近起点会自动闭合；可使用“上一个/下一个”
切换多个 selected region。点击“应用”后会写出项目文件和同名 manifest，例如：

```text
inputs/latest_partitioned.rsp.json
inputs/latest_partitioned_manifest.json
```

manifest v2 会记录 `partition_mode`、`barriers_by_region`、
`picked_polygons_by_region` 以及每个 patch 的 `clip_space` 和 `raster_chart`。
`.rsp.json` 仍只保存原有的 `selected_path_face_regions`；圈选边界只由导出器读取
manifest 后作为 raster-domain clip 使用，不改变项目 schema。

交互规则：

- 关闭绘制工具后，左键拖动旋转二维加工域；
- 滚轮缩放；
- “旋转90°”用于快速调整观察方向，“翻面”用于修正操作视图镜像；
- 矩形始终在屏幕坐标中横平竖直，松开后再转换为 raster UV；
- 多边形可靠近起点闭合，也可右键结束。

点击“快速预览路径”会调用正式导出的同一 `plan_region_uv()`，只在内存中生成并
显示路径，不创建整批 Optimal-Y 输出。手动 v2 的颜色区域由二维 RGBA mask 作为
texture 显示；STL 不会被切割或重建。

### “开始”与“开始-1”

- 第一行“开始”使用默认 `auto` planner：先快速判断孔 polygon 是否与当前 patch 相交；
  无孔 patch 使用普通 raster，有孔 patch 才进入 cell/绕孔规划。普通采样后若发现同一
  scanline 被拆成多个 run，也会安全升级为 hole-aware；
- 第二行“开始-1”强制所有 patch 使用 `hole-aware`，用于对照和排障；
- `auto` 的两类路径都只保留每个 patch 的首尾安全位置；
- 新策略结果目录追加 `_hole_aware`，不会覆盖同日 legacy 输出；
- “快速预览路径”目前仍是 legacy 预览，不能用它判断 hole-aware 的最终顺序。

新策略要求 manual v2 patch 中同时存在 `raster_chart` 和 `clip_polygon`。完整限制、
失败行为和 RobotStudio 验收要求见 `HOLE_AWARE_PLANNER.md`。

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

从命令行运行与“开始-1”相同的新策略：

```powershell
python experiments\base_casting_abb6700\scripts\runs\optimal_y_score_configurable.py --project experiments\base_casting_abb6700\inputs\latest_partitioned.rsp.json --planner hole-aware --model-x=3700 --model-y=-1900,100,1900 --model-z=440 --angles 0,180
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
  更激进或尚未稳定的算法原型；当前包含 UI“开始-1”调用的
  `hole_aware_raster.py`。

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

- `scripts/raster_domain.py`
  手动 manifest v2 的二维路径核心：计算 patch 扫描轴、生成 polygon scanline、扣除
  孔洞 interval，并沿 chart normal 射线投射到选中 STL，返回 XYZ 和三角面法向。

- `experimental_algorithms/hole_aware_raster.py`
  对 raster runs 做 cell 分解和同侧优先排序，必要时在有效二维域内规划绕孔 connector。
  它已通过 `--planner hole-aware` 接入 runner，但仍是实验策略。

- `scripts/optimal_y_selection.py`
  双机器人导轨 Y 位置选择模块。按 processing waypoint 的
  `max(abs(world_y))` 为每个 region 选择候选路径。

- `scripts/runs/*.py`
  具体实验入口。它们导入上面的可复用模块，设置 X/Y/Z、角度、输出目录和 feed variant。

## 分区结果怎么看

自动分区和手动分区是两种不同数据：

- 自动预处理会把 face-id regions 写入 `.rsp.json`，manifest 记录
  `planar/curved`、面积和 face ids；
- 手动 v2 不改写源 face region，manifest 记录 `raster_chart`、二维 patch polygon、
  holes 和 `1_1/1_2` 等标签，exporter 会理解这些标签并逐 patch 规划。

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

上述 `1.1 planar` 说明只适用于自动 face-id 分区。手动 v2 标签会进入输出目录和汇总表。

## 当前策略摘要

- 分区预处理只处理用户指定的 region，其它 region 不动。
- 先做局部曲率/近似平面分区。
- 只在 planar patch 内做窄通道检测。
- 窄通道基于 scanline interval 连通关系，不使用一条全局 XYZ/UV 直线贯穿切面。
- 每个输出 region 后续独立规划和导出。
- RAPID 导出使用 base window、boundary-UV raster、`base_y_aligned` 姿态、固定
  confdata 和 `ConfL \Off;`。
- 默认 `auto` 按 patch 在普通 raster 与 hole-aware 之间分流；`legacy` 仅保留为 CLI
  回退策略，“开始-1”强制 hole-aware。

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
- manifest v1/v2 不混读；
- 手动 patch 独立扫描轴；
- raster-domain 射线 XYZ 与 facet normal；
- 孔洞扣除和不连续 segment；
- hole-aware cell 分解、禁止同一扫描线跨孔和仅首尾安全点；
- 输出目录日期及路径长度。
