# Hole-Aware 分区光栅与抬刀转场策略

## 当前状态

2026-07-15 起，带孔规划不再要求不同 cell 之间存在连续贴面的 A*/connector 路径。
当前策略是：**cell 内完成完整光栅，cell 间法向退刀、离面转移、再法向接近**。

- UI“开始”和 CLI 默认使用 `auto`；
- UI“快速预览路径”与正式运行共用 `plan_region_uv_auto()`；
- CLI `--planner hole-aware` 可强制使用 cell 抬刀策略；
- manual-v2 patch 和未划分的原始 face-id region 都能建立 cells；
- `legacy` 仅作为 CLI 排障/对照选项。

## 自动带孔/缺口判定

`auto` 按以下顺序选择：

1. 判断 manifest 中的 `exclude_polygons` 与当前 `clip_polygon` 是否有正面积重叠；
2. hole 完全位于 clip 内时，直接属于真正的内部孔；
3. hole 穿过 clip 边界时也属于相关 exclude，因为它会切掉边缘加工域；
4. 仅在点或边上接触、没有面积重叠时不切换 planner；
5. 未发现相关 exclude 时先生成普通 raster；若同一扫描线出现多个 run，则说明 STL
   ray hit 支撑域存在缺口，切换到相同的 cell 抬刀策略。

诊断原因写为：

- `exclude-overlap`：显式 exclude 与 clip 有面积重叠；
- `split-scanline`：普通采样发现同一扫描线存在多个有效 run；
- `regular-raster`：保持普通光栅。

`summary.json` 记录 `auto_hole_aware_path_count`、`auto_raster_path_count` 和
`auto_planner_reason_counts`；候选表记录 `planner_reason` 与 `motion_strategy`。

## Cell 路径和转场

1. 二维 scanline 先扣除所有 exclude intervals；
2. ray miss 会立即结束当前 run，禁止 processing motion 跨过无表面区域；
3. 相邻扫描线中 U 区间连续重叠的 runs 组成一个 Boustrophedon cell；
4. cell 按首次出现的扫描线、横向位置和稳定 cell id 排序；
5. 每个 cell 保留自身完整的双向往复光栅，不因下一个 cell 的位置改变顺序；
6. 每个 cell 起点前和终点后沿局部表面法向偏移 `SAFE_DISTANCE`，当前为 150 mm；
7. RAPID 在加工点与安全点之间使用 MoveL，在两个 cell 的安全点之间使用 MoveJ。

因此，空中转场的二维投影可以越过 exclude；加工光栅本身仍不会进入 exclude 或 ray-miss
区域。当前没有贪心重排，也没有 A*，cell 顺序不会造成“无 connector”而整面 deferred。

## 限制与验收

1. manual-v2 使用自身 `raster_chart + clip_polygon`；原始 face-id region 直接使用普通
   mesh raster 的投影原点、U 轴和已拆分 runs，不要求额外 manifest。
2. 如果输入带显式 clip/exclude 元数据，则 chart 和 clip 必须成套存在；不能在坐标系
   不明确时解释 exclude polygon。
3. `SAFE_DISTANCE=150 mm` 是沿每个端点局部法向的 TCP 偏移，不等于统一世界 Z 安全平面。
4. cell 间使用 MoveJ，TCP 实际轨迹不保证是两个安全点之间的笛卡尔直线。
5. 未做机器人可达性、逐点 IK、构型连续性、工具扫掠体或碰撞认证。
6. exclude 上方可转场只表示 processing path 不接触该二维域；砂轮、主轴、法兰和机器人
   仍必须在 RobotStudio 中低速/单步验证。
7. 每个 patch/region 独立规划，不跨区域合并路径。

## 失败条件

当前 cell 规划只在以下情况下失败：

- 手动 polygon 元数据不完整（chart 与 clip 没有成套提供）；
- 当前区域没有可投射到选中 STL 的有效 raster samples。

旧版 `No free-domain connector between raster cells` 已不再是当前算法的失败条件。
A*、cell 图遍历和连续贴面 connector 仅保留在学习文档中作为历史方案和后续研究资料。
