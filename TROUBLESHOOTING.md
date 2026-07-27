# Troubleshooting

## 避障设置应用后红色墙体为空或范围不正确

- 确认主界面当前输入与 `*_avoidance.json` 中的 `input_project` 完全一致；重新分区或切换输入后应重新打开“避障设置”并应用。
- 实际保存位置显示在弹窗底部。只有“应用”会写文件；“清除选择”只重置弹窗临时状态，“隐藏范围”只关闭三维叠加。
- `U/V=30%` 表示最终总宽度为支撑面宽度的 130%，不是左右两边各增加 30%。
- `N+`、`N-` 使用毫米，并沿当前加工面的局部法向，不是固定的模型 Z。
- 红色只包含与灰色 UVN 长方体相交且不属于绿色支撑面的 cells；范围外模型保持灰色。
- 不按三角形法向区分上表面、下表面。若墙体缺失，应调整 U/V 或 N+/N-，不要增加朝向过滤。
- 手动 v2 patch 的黄色边界必须由 raster texture 显示；不能用三角形重心着色替代。

> 2026-07-22 以下新增条目覆盖文档后部旧的 `fallback-unverified -> deferred`
> 说明：当前非空诊断路径保留为候选，但仍不进入避障最优结果。

## 避障报告显示 IK unresolved，但姿态和位置看起来可加工

- 先看 `robot_avoidance_trials.csv` 的 `status`、`max_joint_jump_deg` 和
  `reason`，不要把 `ik-unresolved` 直接解释成 ABB 不可达或发生干涉。
- 当前避障层不再锁定 J1/J4/J6 的 confdata 分区；J6 从正小角度连续跨到
  负小角度不会仅因配置编号变化而失败。
- 若仍为 `ik-unresolved`，表示当前数值求解器、seed 和最多 7 个代表点的
  筛查没有得到完整结果。路径仍保留在 `all_candidates.csv` 供
  RobotStudio 复核，但不会进入 `optimal_paths`。
- `joint-discontinuous` 表示求得了解与上一成功点的实际最大关节差超过
  40°；这与 confdata 编号变化不同。
- `validated-interference` 表示已求得 IK 且抽样 FK 连杆检测到工件碰撞；
  `clearance-insufficient` 表示未碰撞但抽样最小间隙低于要求。
- 所有状态都只是抽样实验结果；完整 MoveL/MoveJ、工具、环境、自碰撞和
  ABB 控制器构型仍需在 RobotStudio 验证。

## 避障候选存在，但 optimal_paths 没有对应区域

- 这是预期的安全分层：非空几何路径会写入候选，方便定位和外部验证；
  只有 `baseline-validated` 或 `alternative-validated` 才能参与避障最优选择。
- 查看 `robot_avoidance_trials.csv` 和 `summary.json.avoidance_status_counts`
  判断是 IK 未解析、关节不连续、间隙不足还是检测到干涉。
- 普通区域不受这个过滤影响，仍按 processing waypoint 的
  `max(abs(world_y))` 选择。

## 导入实验 RAPID 后工件坐标系没有跟随模型

- 自动同步只在实验安装元数据、当前输入/CAD身份以及 RAPID wobj 校验
  全部通过后执行；先查看程序导入日志中的校验信息。
- 同步结果应满足：模型使用实验 X/Y/Z/RZ，wobj XYZ 使用旋转后的
  `picked_origin`，wobj RX/RY 保留输入项目标定值，wobj RZ 使用实验角度。
- 如果只移动模型而 W 坐标系不动，确认主程序调用的是
  `apply_verified_experiment_installation()`，不要重新加入只写
  `model_transform` 的旧逻辑。
- 未通过校验时禁止为了显示对齐而强制移动 W 坐标系；应先修复输入项目、
  picked origin 或 RAPID wobj 不一致。

## 计算成功但结果文件夹没有自动打开

- 自动打开只发生在进程 exit code 为 0、成功定位 `summary.json` 且
  `output_dir`/summary 所在目录实际存在时。
- 失败只影响 Explorer 启动，UI 状态会追加“结果文件夹未能自动打开”，
  不会把已完成的实验改成失败。
- 检查 Windows 文件关联、目录权限以及安全软件是否阻止 Explorer；结果
  仍可从状态栏显示的 `output=` 路径手动打开。

## 快速预览提示 `plan_region_uv()` 参数数量不匹配

- 原因：运行中的 Python/Qt 进程缓存了旧模块，但 UI 已加载新调用接口。
- 处理：完全退出实验软件，确认进程结束后重新启动。
- 不要：通过继续增加可选位置参数掩盖新旧模块混用。

## 分区后模型黑掉或只剩碎片

- 检查是否错误清空基础 mesh actor。
- 检查是否回退到 VTK 几何裁剪 overlay。
- 当前正确方案是原 STL + 透明 raster texture，不生成裁剪 mesh。

## 分区后未划分 region 不再高亮

- 手动 v2 只在 manifest 中记录被划分的源 region；`.rsp.json` 仍保留全部原始 regions。
- 预览必须把被划分的源 region 替换为其 patch，并把没有 manifest record 的 regions 原样透传。
- 例如 `1 -> 1_1` 时，三个源 regions 的最终显示标签应为 `1_1, 2, 3`，不能只显示 `1_1`。

## 分区 texture 只在特定观察角度可见

- 原因通常是 texture overlay 与原 STL 完全共面，透明渲染时发生 Z-buffer 深度竞争。
- overlay mapper 必须启用 polygon offset，并设置非零的负向 relative polygon offset；
  只开启 offset 模式但保留默认 `(0, 0)` 仍会随视角消失。
- 不要通过移动 STL、修改 facet normal、开启单面剔除或重建裁剪 mesh 规避该问题。

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

## 避障区域提示不存在

- `1-1` 表示 patch，内部会规范为 `1_1`；`1` 表示源 region 并覆盖其全部 patches。
- cell 没有可输入标签。查看错误中列出的当前 planning labels，并确认 `.rsp.json` 与同名
  manifest 配套且未被移动或改名。

## 避障结果是 `fallback-unverified`

- 表示五个姿态都未通过内部代表点筛查。快速预览可显示原 base-y 供诊断；正式 runner
  会将该位置 deferred，不生成 candidate/optimal 避障路线。
- 查看 `robot_avoidance_trials.csv` 的 IK、collision、configuration、joint jump 和 J5 字段。
- 当前不含工具/环境/自碰撞/扫掠体；即使显示 validated，也必须进入 RobotStudio 验证。

## 支撑面生长错误或绿色区域进入墙体

- 黄色是最终路径实际命中的 `face_id` 种子；若黄色不在目标 patch，先检查路径规划和
  manifest，而不是调整生长阈值。
- 绿色只应覆盖 patch 所在近共面支撑面。若越过圆角进入红色墙体，减小参考法向角或
  参考平面距离；若过早停止，再小幅增加阈值，并保存前后截图和 summary。
- `support-surface-failed` 会进入 deferred，不能回退到完整工件或 5 mm 模型假装成功。
- 当前默认是近似平面恢复；明显自由曲面需要单独的曲面模式，不能把平面阈值无限放宽。
