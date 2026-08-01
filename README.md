# Base Casting ABB 6700 Experiment

This is the project-specific offline workspace for the large base-casting
polishing study. For agent handoff rules, read `AGENTS.md`. For algorithm
principles, read `PRINCIPLES.md`.

## Current avoidance and result behavior (2026-07-22)

- Turntable input accepts arbitrary comma-separated angles; each value is used
  as the actual model/workobject RZ.
- Only user-selected avoidance regions run the experimental sampled IK/FK and
  robot-link/workpiece collision screen. Ordinary regions remain on the normal
  export path and are validated by ABB/RobotStudio.
- Avoidance IK continuity is selected by seeding each point from the previous
  successful joint solution and checking the real joint-angle jump. Confdata
  quadrant changes are diagnostic and are not a rejection condition.
- `all_candidates.csv` contains only generated non-empty paths. Unresolved
  avoidance paths remain available for diagnosis but do not enter
  `optimal_paths`; the compact `robot_avoidance_trials.csv` states the selected
  roll, sampled interference result, clearance, joint jump, and reason.
- After a successful UI run, the generated result directory opens
  automatically. A folder-open failure does not change the calculation result.

Optimal-Y 完成后，可在实验 UI 点击“导入 RobotStudio 验证”，选择该次实验结果目录，
按 region/patch 生成 RobotStudio 6.08 人工验证工作站。安装、目录选择、命名规则、操作流程和
安全边界见 `ROBOTSTUDIO_EXPORT.md`；生成工作站不等于碰撞、可达性或姿态验证通过。

设计决策见 `DECISION_LOG.md`，manifest 字段见 `MANIFEST_SCHEMA.md`，已知故障见
`TROUBLESHOOTING.md`，坐标同步见 `COORDINATE_SYSTEMS.md`，提交前验收见
`VALIDATION.md`。带孔连续路径的行为和限制见 `HOLE_AWARE_PLANNER.md`。指定
region/patch 的机械臂替代姿态试验见 `docs/ROBOT_ARM_AVOIDANCE_WORKFLOW.md`。

## 中文说明

这是底座大型结构件 ABB 6700 抛光实验目录。这里的脚本用于离线读取软件导出的
`.rsp.json` 选面快照，必要时先做区域分区，然后按窗口/固定构型策略导出 ABB
RAPID 路径，最后可选地做导轨 Y 位置选择。

## 环境依赖

本仓库不提交虚拟环境，运行时复用同一工作区中的 `src` 主程序仓库。三个仓库应保持
下面的相对目录结构：

```text
p1/
├─ src/
└─ experiments/
   └─ base_casting_abb6700/
```

首次克隆或更换电脑后，在 `p1` 目录创建虚拟环境，然后进入本实验仓库安装依赖：

```powershell
py -3.12 -m venv .venv
cd experiments\base_casting_abb6700
..\..\.venv\Scripts\python.exe -m pip install --upgrade pip
..\..\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`requirements.txt` 会以 editable 模式安装 `../../src`，并安装实验测试和静态检查所需的
`pytest`、`ruff`。`.venv/`、PyCharm 的 `.idea/` 和 VS Code 的 `.vscode/` 都只保留在
本机，不提交到 Git。

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

点击“快速预览路径”会调用正式导出的同一 `plan_region_uv_auto()`，只在内存中生成并
显示路径，不创建整批 Optimal-Y 输出。手动 v2 的颜色区域由二维 RGBA mask 作为
texture 显示；STL 不会被切割或重建。

### “开始”的自动路径策略

- 第一行“开始”使用默认 `auto` planner：内部孔和与边界有面积重叠的 exclude 进入
  cell 规划；仅接触边界不算。普通采样后若同一 scanline 被拆成多个 run，也会升级；
- hole-aware 完整加工每个 cell，并在 cell 之间法向抬刀转场；普通面保留快速 raster；
- 新策略类型写入 `summary.json`；结果目录使用日期和当日递增编号，运行之间不会覆盖；
- “快速预览路径”使用同一自动判定，并在状态栏显示 cell 抬刀数量与触发原因。

新策略同时支持 manual-v2 patch 和未划分的原始 face-id region。manual-v2 的显式
exclude 仍必须与 `raster_chart + clip_polygon` 成套存在；原始 region 在普通投影发现
split-scanline 后直接复用投影扫描轴建立 cells。完整限制见 `HOLE_AWARE_PLANNER.md`。

### 避障设置

第一行点击“避障设置”打开三维弹窗。区域可混合填写 `1-1，2，3-2`：带连字符的是
指定 patch，裸数字是源 region；英文逗号、中文逗号、顿号和分号均可分隔。裸 region
会解析为其全部最终 patches，每个 patch 独立设置。U/V 在同一行输入支撑面总宽度的
扩大百分比，`30%` 表示最终宽度为 `130%`；N+/N- 在下一行输入沿局部法向两侧的
毫米高度。

解析后范围自动显示。“隐藏范围/显示范围”只切换三维叠加显示，参数仍保留；
“清除选择”只清空弹窗中的临时选择，便于重新输入，不会删除已经保存的设置。
点击“应用”才写入输入项目旁边的同名 `*_avoidance.json`，具体路径显示在弹窗底部；
不会修改 `.rsp.json` 或分区 manifest。保存文件只是可复用设置，不会在下次启动 UI
时自动启用；只有本次在弹窗中点击“应用”，下一次 runner 才会收到避障参数。只解析、
预览、取消或关闭弹窗均不启用。runner 结束后自动解除启用，但保留 JSON 供下次编辑。

弹窗中黄色为打磨面、绿色为恢复支撑面、红色为 UVN 范围内墙体、半透明灰色体为
覆盖范围。完整支撑面的所有顶点先投影到 UV 平面并生成一个二维凸包，凸包会有意
覆盖内孔、边缘缺口、窄连接和凹入区域，再按 U/V 比例扩大并沿 N+/N- 拉伸；曲面也
先投影再生成凸包。STL 三角边不作为范围边界，红色墙体预览也由同一凸多边形裁切。
未分区的 `1/2/3` 会把整个所选加工面直接作为支撑面；`1-1/3-2` 等 patch 会从路径
命中点恢复更大的支撑面，同时强制保留其源加工面。黄色加工 cell 不允许同时进入
红色墙体集合。
范围外模型不进入本轮墙体网格，不按朝上/朝下过滤。正式 runner 仍执行现有姿态库、
数值 IK/FK、抽样碰撞和间隙筛查；本次墙体范围功能没有改变这些规则。

转台角度不再使用固定预设。实验 UI 的“转台”输入框可填写单个角度 `270`，也可填写
多个角度 `0,180` 或范围 `0-30-330`；逗号列表按输入顺序运行，范围格式依次为
`start-step-stop` 且包含终点，负角度和 360 会规范到 `0..359` 并去重。

默认结果目录使用紧凑的
`x位置-y位置-z位置-angle角度-YYYYMMDD-编号` 规则。例如：
`x3500-ym1900,100,1900-z440-angle0,30,330-20260801-01`。坐标范围按
`起点,步长,终点` 表示，负数以 `m`、小数点以 `p` 表示；编号在同一天全局递增。
planner、窗口模式和是否启用避障等完整参数记录在 `summary.json`，不再放进目录名。

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

从命令行强制所有 patch 使用 hole-aware（仅用于排障/对照）：

```powershell
python experiments\base_casting_abb6700\scripts\optimal_y_score_configurable.py --project experiments\base_casting_abb6700\inputs\latest_partitioned.rsp.json --planner hole-aware --model-x=3700 --model-y=-1900,100,1900 --model-z=440 --angles 0,180
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
  更激进或尚未稳定的算法原型；当前包含 auto 按需调用的
  `hole_aware_raster.py`，以及只对指定区域调用的 `robot_pose_avoidance.py`。

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
  对 raster runs 做 cell 分解和稳定扫描排序；cell 内完整加工，cell 间由导出器抬刀转场。
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
- 默认 `auto` 按 patch 在普通 raster 与 hole-aware 之间分流；`legacy` 和强制
  `hole-aware` 仅保留为 CLI 排障/回退策略。

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
