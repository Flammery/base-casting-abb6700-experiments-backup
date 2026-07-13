# Decision Log

本文件记录已经确认的架构决策、被否决方案和后续约束。新实现与这里冲突时，必须先更新决策状态，不能直接重试旧方案。

## D001 手动分区属于二维光栅域

- 日期：2026-07-13
- 状态：Accepted
- 决定：手动 boundary/slab/pick 分区定义 `raster_chart` 中的二维加工区域，不切割 STL，也不生成新的 mesh face id。
- 原因：加工边界和规则光栅不应受 STL 三角剖分影响。
- 约束：`clip_polygon`、`exclude_polygons` 和 scanline 必须在同一 raster chart 中计算。

## D002 STL 只负责路径落点和法向

- 日期：2026-07-13
- 状态：Accepted
- 决定：二维采样点沿 chart 法向投射到选中 STL；命中三角形提供 XYZ、face id 和 facet normal。
- 约束：不能用三角形重心或三角形边界决定手动分区形状。

## D003 禁止用 cell color 表达规则手动分区

- 日期：2026-07-13
- 状态：Accepted
- 被否决方案：按 triangle centroid 给整个 VTK cell 上色。
- 原因：必然产生锯齿、尖片和碎三角形边界。
- 当前方案：PySide/QPainter 生成二维 RGBA mask，VTK 将其作为 UV texture 覆盖到不变的 STL 上。

## D004 禁止通过几何裁剪重建手动分区面

- 日期：2026-07-13
- 状态：Rejected
- 被否决方案：使用 VTK implicit loop、空间挤出或逐三角形几何裁剪生成彩色 patch mesh。
- 原因：曲面、多层投影和孔洞会造成黑面、重叠、扇形碎片和拓扑不稳定。

## D005 每个 patch 独立规划

- 日期：2026-07-13
- 状态：Accepted
- 决定：每个 `records[].patches[]` 根据自身二维边界计算长短边、生成扫描线并独立导出。
- 约束：不能复用源 region 的单一扫描轴后再仅删点。

## D006 孔洞在二维 interval 阶段扣除

- 日期：2026-07-13
- 状态：Accepted
- 决定：从外轮廓 scanline intervals 中减去 `exclude_polygons`，射线未命中也必须断开 processing segment。
- 约束：孔洞两侧不能共享连续加工运动。

## D007 manifest v1/v2 必须按版本解释

- 日期：2026-07-13
- 状态：Accepted
- 决定：v1 表示已经写入 `.rsp.json` 的 face-id regions；v2 表示保留源 region、由 manifest 保存 raster patches。
- 约束：读取器必须同时校验 schema 和 version，不能只校验 schema。

## D008 快速预览与正式导出共用规划入口

- 日期：2026-07-13
- 状态：Accepted
- 决定：实验 UI 快速预览调用 `window_conf_export.plan_region_uv()`，不得维护另一套预览算法。

## D009 实验层与 src 的边界

- 日期：2026-07-13
- 状态：Accepted
- 决定：`scripts/raster_domain.py`、手动分区、窗口策略、批跑和 ABB 导出属于实验层；MeshTriangle、mesh reader、PathResult、Waypoint 和通用变换继续复用 `src`。
- 约束：不要复制整个 `src/path_planning` 到实验目录。

## D010 工具与工件坐标继承和同步

- 日期：2026-07-13
- 状态：Accepted
- 决定：tool/wobj 名称、TCP、picked origin 和基础 placement 来自软件导出的项目；实验只覆盖扫描要求的 model X/Y/Z/RZ，并使用同一 RZ 和旋转后的 picked origin 同步更新 wobj。
- 原因：模型预览、base 加工窗口和 ABB robtarget 必须描述同一个物理安装位姿。
- 约束：不能只移动模型不移动 wobj；不能直接把 world position 写入 robtarget；不能修改 tool TCP 补偿 quaternion。
- 例外：缺失工具载荷时允许写 RobotStudio 兼容占位值，但必须视为未标定数据。
- 详细公式：见 `COORDINATE_SYSTEMS.md`。

## 新决策模板

```text
## Dxxx 标题
- 日期：YYYY-MM-DD
- 状态：Proposed / Accepted / Rejected / Superseded
- 背景：
- 决定：
- 被否决方案：
- 原因：
- 约束与影响：
- 相关代码/测试：
```
