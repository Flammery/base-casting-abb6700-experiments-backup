# 底座打磨自动 selected-region 分区算法

> 说明：本文只描述自动 face-id 分区算法。手动 boundary/slab/pick 属于独立的
> manifest v2 raster-domain 流程，操作说明见 `README.md`，字段见
> `MANIFEST_SCHEMA.md`，约束与历史决策见 `PRINCIPLES.md` 和 `DECISION_LOG.md`。

本文说明 `scripts/region_partitioning.py` 当前的 selected region 预处理逻辑。它只属于
`experiments/base_casting_abb6700` 实验层，不修改 `src/` 软件库，也不改变项目 `.rsp.json`
schema。

## 1. 问题背景

当前软件导出的 `latest_script_test.rsp.json` 可能只有一个很大的
`selected_path_face_regions`。这个大区域会跨过柱子两侧和中间区域，如果直接按曲率或
平面性分区，后续路径可能把不同机器人姿态才能加工的面混在一起。

因此 v2 分区顺序改为：

```text
selected region
  -> 转角/柱子分区
  -> 主平面/斜面分类
  -> planar 窄口分区
  -> 机器人 base 窗口尺寸切分
  -> 小碎片合并
```

最终 `.rsp.json` 仍然只写普通 face-id region 列表；人能读懂的标签和诊断信息写入
`latest_partitioned_manifest.json`。

## 2. 转角/柱子分区

柱子把工件分成左侧、中间、右侧三类可达区域。默认策略是在模型 XY 的 X 方向上做
面积直方图，寻找两个面积低谷作为柱子分界：

- `left`：左侧柱子外侧区域；
- `center`：中间区域；
- `right`：右侧柱子外侧区域。

默认角度组为：

```text
left   -> [0]
center -> [270]
right  -> [180]
```

这些角度只是分区元数据和尺寸切分依据；后续 exporter 是否按 manifest 过滤角度可以单独接入。
如果现场发现 0 度附近需要微调，可以用 CLI 传入角度组，例如：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py `
  --regions 1 `
  --left-angles 345,0,15 `
  --center-angles 255,270,285 `
  --right-angles 165,180,195
```

如果自动柱子分界不稳定，可以手动覆盖：

```powershell
python experiments\base_casting_abb6700\scripts\region_partition_preprocess.py `
  --regions 1 `
  --left-cut-x -600 `
  --right-cut-x 600
```

## 3. 主平面/斜面分类

旧 manifest 使用 `planar` / `curved`。为了兼容后续脚本，v2 仍保留这个字段，但实际语义变为：

- `kind = planar`, `surface_class = main_plane`：主平面；
- `kind = curved`, `surface_class = slope`：斜面或过渡面。

分类在每个 turn zone 内单独进行。算法用区域的面积加权法向和面积加权中心点形成参考平面，
再按两个条件判断 face 是否属于主平面：

- face normal 与参考法向夹角不超过 `planar_normal_deg`；
- face centroid 到参考平面的距离不超过 `planar_rms_mm`。

这样做的目的是避免把左侧、中间、右侧因姿态限制不同的面先混成一个“大平面”。

## 4. planar 窄口分区

主平面内部仍保留旧的窄口检测逻辑：

- 从 patch 边界推导局部 UV 轴；
- 沿 scanline 统计 interval 宽度；
- 当连续 scanline 宽度明显低于局部中值时，把该位置当成瓶颈；
- 只切断 interval 图的局部连通关系，不用一条全局直线硬切整个区域。

这能避免 L 形或宽连接区域被全局轴误切，同时保留真实窄口把两块大平面分开的能力。

## 5. 机器人窗口尺寸切分

ABB 6700 当前实验窗口按 base 坐标约束：

```text
base X span <= 1000 mm
base Y span <= 2000 mm
```

v2 会使用 patch 的 nominal angle，把 patch 顶点旋转到对应的 base XY 方向后检查跨度。
若超限，则沿超限比例最大的 base 轴切分，并在切分后重新按 mesh 邻接拆连通分量。

例如一个 `800 x 2400` 的面，如果 nominal angle 下只有 base Y 超过 `2000`，会优先沿
base Y 切成两个约 `800 x 1200` 的小区。

## 6. 自动分区路径的孔洞和边界

孔洞和边界外不在分区脚本里重复判断。后续路径生成使用
`src/robot_studio_qt/path_planning/mesh_raster.py`：

- scanline 只从真实投影三角面生成 interval；
- 采样点必须通过 barycentric 判断落回某个三角面；
- 孔洞没有三角面，外边界外也没有三角面，因此不会生成路径点。

这部分属于自动/legacy face-id region 的通用路径规划库。

手动 manifest v2 不使用本节方法决定加工边界。它由
`scripts/raster_domain.py` 直接在二维 `clip_polygon` 内生成 scanline，从 interval
中减去 `exclude_polygons`，再沿 `raster_chart.normal` 射线命中选中 STL。STL 在该
流程中只提供 XYZ、face id 和 facet normal。两条路径不能混写成同一个算法说明。

## 7. 干涉检测预留

干涉检测本次不实现。manifest 中每个 patch 记录：

```json
"collision_status": "not_evaluated"
```

后续可以新增独立脚本读取 manifest、分区 face id 和角度组，对每个 patch 做机器人/工件/柱子干涉评估，
再把结果回写到新的报告或 manifest 扩展字段。
