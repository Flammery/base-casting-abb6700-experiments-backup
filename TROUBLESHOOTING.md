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

## CLI 强制 hole-aware 后全部进入 deferred

- 原始 face-id region 不再要求 manifest；检查是否确实生成了 projected raster samples。
- manual-v2 输入若带 clip/exclude，检查 `raster_chart` 与 `clip_polygon` 是否成套存在。
- 查看 `deferred_paths.csv`：当前主要失败条件是手动域元数据不完整或没有有效 sample；
  旧版 free-domain connector 失败已不再适用。

## “开始”把无孔面也分到 hole-aware

- `auto` 判断的是孔 polygon 是否与当前 patch 有正面积重叠，不是仅检查
  `exclude_polygons` 是否非空；内部孔和穿过边界的 exclude 都会触发，仅边界接触不会。
- 普通 raster 后如果同一 scanline 出现多个 segment，auto 也会主动升级，防止未记录
  孔洞造成直线跨越；查看实际 mesh/ray miss 是否形成了缺口。
- `summary.json` 的两个 auto path count 按候选路径计数，不等于唯一面数。

## 快速预览与正式 RAPID 的抬刀显示不同

- 快速预览与“开始”都调用 auto，但预览只绘制 processing raster，不绘制安全进退刀点。
- 状态栏的 `cell抬刀` 和 `判定` 可确认是否进入 cell 策略。
- 完整 MoveL/MoveJ 顺序仍需检查点 CSV、RAPID 或 RobotStudio。

## Cell 间抬刀路径可能靠近孔边或工件

- 当前法向安全点和 MoveJ 转场没有砂轮/刀盘扫掠体补偿或三维碰撞验证。
- `boundary_margin` 不是完整工具包络或碰撞模型。
- 在 RobotStudio 按真实工具尺寸检查；需要更大余量时应增大分区/边缘余量，不能把
  二维不穿孔当成实际无碰撞。

## 旧结果出现 `No free-domain connector between raster cells`

- 这是 2026-07-15 之前连续贴面 A* 方案的结果，不是当前 cell 抬刀策略的失败条件。
- 使用当前代码重新生成输出；不要直接混用旧候选目录和新 summary。

## Auto 和 CLI 强制 hole-aware 结果互相覆盖

- 默认情况下新策略目录会追加 `_hole_aware`。
- 如果显式传入相同的 `--output-dir`，目录隔离由调用者负责。
- 检查 `summary.json` 的 `planner` 字段，避免把 legacy 和 hole-aware 结果混用。

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

## RobotStudio 导出提示没有最优记录

- 必须选择一次实验结果目录，即同时包含 `summary.json`、`optimal_selection.csv` 和
  `optimal_paths` 的那一级；不要选择 `results` 总目录或某个单独面目录。
- 检查 `optimal_records.json` 的 `records` 是否非空，或 `optimal_selection.csv` 是否包含
  `BEST` 行。若 `candidate_count=0`，应先解决路径规划/deferred 问题，RobotStudio 打包器
  不会替代 Optimal-Y 计算。
- 每个最优标签必须存在对应的 `optimal_paths/<region_label>/<region_label>.txt`。

## RobotStudio 打开工作站后没有自动加载 RAPID

- 确认目标版本为 RobotStudio 6.08.01，并在插件安装或更新后完整重启 RobotStudio。
- 确认 `.rsstn` 旁边存在同名 `.robotstudio_job.json`，且其中引用的 `CalibData.mod` 和路径
  `.mod` 都存在。
- 查看 `%LOCALAPPDATA%\ABB6700RobotStudioBridge\addin.log` 和 `last_error.txt`。
- 确认模板虚拟控制器已启动、任务名与配置中的 `controller_task` 一致，默认是 `T_ROB1`。
- 不要同时打开多个生成工作站；这些文件按顺序复用模板虚拟控制器。

## RobotStudio 提示找不到场景工件组件

- `workpiece_component_name` 必须是模板工作站树中场景模型的精确名称，不是 RAPID wobj 名称。
- 工件模型安装位置与 RAPID 工件坐标相互独立；禁止通过把组件改名为 `wobj1` 来绕过错误。
- 若更换了模板工作站，同步更新 `configs/robotstudio_export.json` 后重新生成工作站。

## Git 显示 detached HEAD

- commit 仍会成功创建，但不会自动属于某个本地分支。
- 在清理或切换前记录 commit hash，并由用户决定 cherry-pick 或建立分支。
- `inputs/latest_*` 和时间戳快照是实验数据，不应与代码修复一起提交。
