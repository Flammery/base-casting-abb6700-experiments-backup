# Hole-Aware 连续光栅策略

## 状态与入口

该策略是 2026-07-13 加入的带孔 patch 连续贴面加工方案，已经接入实验 UI 和真实
RAPID/CSV/Optimal-Y 导出。默认“开始”采用自动混合策略，不要求所有面都运行绕孔算法。

- UI 第一行“开始”：`auto`，逐 patch 快速分流；
- UI 第二行“开始-1”：强制 `hole-aware`，用于对照和排障；
- CLI：`scripts/runs/optimal_y_score_configurable.py --planner hole-aware`；
- CLI 默认 planner 为 `auto`；显式强制新策略时输出目录追加 `_hole_aware`；
- 当前“快速预览路径”仍调用 legacy `plan_region_uv()`，不会预览 hole-aware 顺序。

## 解决的问题

legacy raster-domain 会把每条扫描 run 编成独立 segment，`build_motion()` 再为每个
segment 添加法向进退刀点，因此会在扫描线边缘反复抬起。孔洞把同一扫描线切成多个
interval 后，逐行排序还会造成孔洞两侧频繁切换。

hole-aware 策略改为：

1. 继续在二维 raster chart 中扣除 `exclude_polygons`；
2. 根据相邻扫描线 interval 的重叠关系建立 Boustrophedon cells；
3. 一个 cell 内连续往复，遇边界直接在有效表面折返；
4. 先完成当前孔侧 cell，再进入相邻 cell；
5. cell 间直线穿孔时，在二维有效域内用确定性栅格 A* 查找绕孔路线；
6. connector 重新采样并逐点 ray lift 到选中 STL；
7. 整个 patch 只添加全局起点和终点安全位置，中间全部使用 MoveL。

## Auto 快速分流

`auto` 不会先对 20 个面全部运行 cell/A*。每个 patch 的顺序是：

1. 用 polygon 包围盒、点包含和边相交测试，判断 `exclude_polygons` 是否真的与当前
   `clip_polygon` 相交；manifest 列出的孔在 patch 外时直接忽略；
2. 命中相关孔时直接调用 hole-aware；
3. 未命中时运行一次普通 raster；
4. 线性扫描生成的 waypoint，若发现同一 base scanline 属于多个 segment，说明存在
   未被快速 polygon 检查捕获的缺口，再升级为 hole-aware；
5. 普通 raster 不构建 cells、不运行 A*，因此多数无孔面的新增开销只是一遍低成本
   polygon 检查和 waypoint 线性检查。

`summary.json` 会记录 `auto_hole_aware_path_count` 和 `auto_raster_path_count`。注意计数
按候选路径（位姿 × 角度 × patch × feed variant），不是模型的唯一面数。

安全位置在 base/world 坐标中定义：

```text
safe_x = endpoint_world_x - 100 mm
safe_y = endpoint_world_y
safe_z = endpoint_world_z + 100 mm
```

偏移后再转换为 `position_wobj` 写入 robtarget。安全点姿态继承对应首末加工点姿态。

## 当前限制

1. **只支持 manual manifest v2 路径域。** 目标 patch 必须同时具有
   `raster_chart` 和 `clip_polygon`；缺失时新 planner 返回 deferred，不回退 legacy。
2. **显式孔洞以 `exclude_polygons` 为准。** ray miss 会形成独立 run，connector 若
   无法逐点 lift 会失败，但当前不会从任意 STL 缺口自动重建精确孔轮廓。
3. **没有机器人碰撞和逐点 IK 验证。** `valid` 只表示二维不穿孔且 connector 能投射到
   表面，不代表 ABB 机器人、工具、法兰或工件无碰撞、可达或构型连续。
4. **没有刀具扫掠体补偿。** `boundary_margin` 控制 raster 采样余量；connector 的
   合法性仍按 TCP 点判断，不等价于砂轮半径、刀盘外形或安全包络的 Minkowski offset。
5. **绕孔不是全局最优。** cell 访问采用“相邻优先、距离次优”的确定性贪心顺序；
   connector 使用有限分辨率栅格 A*，目标是先稳定避孔，不保证全局最短总路径。
6. **窄通道可能判定失败。** A* 网格会限制最大节点数并自适应放大分辨率；小于网格
   分辨率的可通行间隙可能被视为不可行，此时路径进入 deferred，不允许穿孔直连。
7. **每个 patch 独立。** 不跨 patch 合并路径；每个成功 patch 都有自己的首尾安全点。
8. **安全点本身未做碰撞验证。** `x-100/z+100` 是当前约定，不保证所有安装位姿下都
   是实际无碰撞位置，必须在 RobotStudio 低速/单步验证。
9. **快速预览尚未切换。** UI 中看到的“快速预览路径”仍是 legacy；判断新策略必须看
   `_hole_aware` 结果中的点 CSV、RAPID 和 RobotStudio 轨迹。

## 失败策略

出现以下任一情况时，新策略不得生成穿孔回退路径：

- 缺少 raster chart 或 clip polygon；
- cell 之间找不到二维有效 connector；
- connector 任一点 ray lift 失败；
- 当前区域没有有效 raster samples。

失败区域写入现有 `deferred_paths.csv`，原因来自 `PathResult.message`。

## 替换 legacy 的条件

默认 runner 已使用 `auto`，但强制 hole-aware 的“开始-1”仍应保留到真实模型完成
`VALIDATION.md` 验收。确认自动分类、碰撞、可达性、工具包络和安全位置后，可删除
“开始-1”；`legacy` CLI 回退是否保留再单独决定。
