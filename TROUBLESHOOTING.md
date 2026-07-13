# Troubleshooting

## 快速预览提示 `plan_region_uv()` 参数数量不匹配

- 原因：运行中的 Python/Qt 进程缓存了旧模块，但 UI 已加载新调用接口。
- 处理：完全退出实验软件，确认进程结束后重新启动。
- 不要：通过继续增加可选位置参数掩盖新旧模块混用。

## 分区后模型黑掉或只剩碎片

- 检查是否错误清空基础 mesh actor。
- 检查是否回退到 VTK 几何裁剪 overlay。
- 当前正确方案是原 STL + 透明 raster texture，不生成裁剪 mesh。

## 分区边界出现密集三角形

- 原因通常是按 cell centroid 整片上色或显示 triangle edges。
- 手动 v2 应使用 QPainter mask/UV texture；分区窗口不显示 STL 内部三角边。

## 新分区没有走 raster-domain 路径

- 检查同名 manifest 是否存在。
- 检查 `version == 2`。
- 检查 record 和 patch 是否有 `raster_chart`。
- 旧 manifest 需要在当前软件中重新执行“区域划分 → 应用”。

## 路径穿过孔洞

- 检查孔洞是否写入 `exclude_polygons`。
- 检查 `raster_domain.raster_samples()` 是否在二维 interval 阶段扣孔。
- 检查射线未命中后是否开启了新 segment。
- 不要仅删除孔内点后继续连接孔洞两侧 waypoint。

## 多边形点击没有反应

- 旋转视图后检查吸附半径计算，必须使用变换矩阵列向量长度，不能只用 `m11/m22`。
- 可右键结束或靠近起点闭合；至少需要三个非重复点。

## 矩形随画布旋转而倾斜

- 矩形 rubber band 必须绘制在 viewport 坐标中，松开后再用 `mapToScene()` 转成 UV。
- 不要把 scene-axis-aligned polygon 直接作为屏幕矩形预览。

## 分区面显示镜像

- 检查 `v_axis = normal × u_axis` 和 `u_axis × v_axis = normal`。
- 可使用“翻面”修正操作视图；翻面不修改 STL facet normal。

## 结果混入上一次实验

- 默认目录末尾应有运行日期 `%m%d`。
- 显式传入 `--output-dir` 时由用户保证目录隔离。

## 软件预览正确但 RobotStudio 路径整体偏移

- 检查是否只修改了 `model_x/y/z/rz` 而没有同步 `wobj_x/y/z/rz`。
- 检查 `picked_origin` 是否按当前 RZ 旋转，或是否被旋转了两次。
- 检查 robtarget 是否使用 `position_wobj`，而不是 `position_world`。
- 对照 `COORDINATE_SYSTEMS.md` 中的 placement 公式。

## RobotStudio 中工件或路径重复旋转

- 检查 `model_rz`、`wobj_rz` 和 world-to-wobj quaternion 是否分别只应用一次。
- 禁止在路径点位置中预旋转后又通过 wobj 重复旋转。

## 工具方向错误但路径位置正确

- 检查 waypoint quaternion 和 world-to-wobj 姿态转换。
- 不要修改 tool TCP 或 flange-to-TCP 几何来补偿方向。
- 检查使用的 tool 名称是否仍来自输入项目。

## RAPID 工具载荷看似有效但未标定

- 若输入 `mass_kg <= 0`，实验可能写入 1 kg/占位 load。
- 占位值只解决 RobotStudio 格式接受问题，投入真实运行前必须替换为实测工具载荷。

## Git 显示 detached HEAD

- commit 仍会成功创建，但不会自动属于某个本地分支。
- 在清理或切换前记录 commit hash，并由用户决定 cherry-pick 或建立分支。
- `inputs/latest_*` 和时间戳快照是实验数据，不应与代码修复一起提交。
