# Decision Log

## D020 Avoidance walls use operator-configured UVN volumes

- Date: 2026-07-27
- Status: Accepted for experiment; supersedes the global-obstacle sampling part
  of D016
- Background: D016 removed the recovered support but still treated the rest of
  the complete workpiece as obstacle candidates. A support-near 800 mm box only
  changed sampling priority, so distant geometry remained involved while dense
  nearby walls could still be reduced by the 6000-triangle budget.
- Decision: replace the main-panel avoidance text field with an Avoidance
  Settings dialog. Resolve selectors to final planning labels and persist one
  model-coordinate UVN rectangular volume per label. U/V are independently
  adjustable total-width expansion percentages; N+/N- are independent
  millimetre heights. Only non-support cells intersecting the volume enter the
  experimental wall mesh.
- UI: show the machining area in yellow, recovered support in green, included
  walls in red, and the UVN volume as a highly transparent gray solid. Parsing
  automatically displays the result. The dialog has Clear Selection, a
  Show/Hide toggle, Apply, and Cancel; it does not provide Apply-to-all. Clear
  Selection only resets the temporary dialog state and does not delete a saved
  sidecar.
- Persistence: write a versioned sibling `*_avoidance.json` sidecar and do not
  modify `.rsp.json` or the partition manifest. Show the exact sidecar path in
  the dialog.
- Rejected for this stage: filtering lower/downward faces, changing the TCP-roll
  library, changing IK/FK, or changing collision/clearance acceptance.
- Related code: `ui/avoidance_settings_dialog.py`, `ui/region_viewer.py`,
  `experimental_algorithms/support_surface_growth.py`,
  `scripts/optimal_y_score_configurable.py`.

## D019 Successful experiment runs open their result directory

- Date: 2026-07-22
- Status: Accepted
- Decision: after a successful configurable-runner process and a readable
  `summary.json`, the experiment UI opens the resolved output directory. Failed
  or cancelled processes do not open it; an Explorer launch failure is reported
  in UI status and does not change the calculation result.
- Related code: `ui/experiment_panel.py`.

## D018 Avoidance IK uses joint continuity instead of a hard confdata lock

- Date: 2026-07-22
- Status: Accepted for experiment; shared solver unchanged
- Background: sampled avoidance trials found numerically accurate solutions but
  rejected them when J6 crossed a confdata quadrant boundary, including small
  continuous motions across zero degrees.
- Decision: every sampled avoidance waypoint uses the previous successful joint
  solution as the next seed with `lock_configuration_to_seed=False`. Confdata
  tuples remain diagnostics. Acceptance uses actual joint jump, J5 margin, IK
  success, FK collision, and sampled clearance.
- Output policy: geometrically non-empty paths are retained in candidate output
  when internal IK is unresolved, but only `baseline-validated` and
  `alternative-validated` avoidance rows may enter `optimal_paths`.
- Rejected alternative: changing the shared `kinematics/solvers.py` before the
  experiment policy has been validated on ABB/RobotStudio results.
- Related code: `experimental_algorithms/robot_pose_avoidance.py`,
  `scripts/optimal_y_score_configurable.py`, and
  `scripts/optimal_y_selection.py`.

## D017 Verified RAPID imports synchronize model and displayed workobject

- Date: 2026-07-22
- Status: Accepted; extends D015
- Background: the imported experiment metadata moved only `model_transform`, so
  the workpiece coordinate actor could remain at an earlier pose even though the
  generated RAPID wobj had already passed verification.
- Decision: after current-project/CAD identity and RAPID wobj verification pass,
  apply the experiment X/Y/Z/RZ to the model and recompute the displayed
  workobject XYZ from the rotated `picked_origin`; preserve calibrated wobj
  RX/RY and set wobj RZ to the experiment angle.
- Constraint: RAPID robtargets continue to use the parsed program wobj; the scene
  must never derive model placement from an unverified wobj.
- Related code: `src/robot_studio_qt/path_planning/rapid_import.py` and
  `src/robot_studio_qt/ui/main_window.py`.

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
- 约束：孔洞两侧不能用一条直线 processing motion 直接相连；允许在端点法向退刀后，
  通过离面运动转移到下一个 cell 的安全接近点。

## D007 manifest v1/v2 必须按版本解释

- 日期：2026-07-13
- 状态：Accepted
- 决定：v1 表示已经写入 `.rsp.json` 的 face-id regions；v2 表示保留源 region、由 manifest 保存 raster patches。
- 约束：读取器必须同时校验 schema 和 version，不能只校验 schema。

## D008 快速预览与正式导出共用规划入口

- 日期：2026-07-13
- 状态：Accepted
- 决定：实验 UI 快速预览调用正式运行使用的 `window_conf_export.plan_region_uv_auto()`，不得维护另一套预览算法。

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

## D011 Auto/Hole-aware UI 策略

- 日期：2026-07-13
- 状态：Superseded by D013
- 背景：legacy 按 raster segment 插入进退刀，导致扫描线边缘反复抬起；带孔扫描线
  按行处理还会在孔洞两侧频繁切换。
- 决定：“开始”和 configurable runner 默认运行 `auto`：先快速判断与当前 patch 相关的
  hole polygon，无孔时使用普通 raster，有孔或普通采样后同一 scanline 出现多个 run 时
  使用 `experimental_algorithms/hole_aware_raster.py`。UI 只保留一个自动“开始”按钮；
  强制 hole-aware 仅通过 CLI 提供。
  auto 的两类路径都只保留首尾安全位置。
- 约束与影响：目标必须有 `raster_chart + clip_polygon`；connector 失败必须 deferred；
  快速预览仍为 legacy；新输出目录追加 `_hole_aware`；不得把二维避孔视为碰撞/IK认证。
- 后续：真实 RobotStudio 验证完成后，另行决定是否保留强制 hole-aware 和 legacy CLI 回退。
- 详细限制：见 `HOLE_AWARE_PLANNER.md`。

## D013 Cell 内完整光栅、Cell 间抬刀转场

- 日期：2026-07-15
- 状态：Accepted for experiment
- 背景：连续贴面 connector 把 Boustrophedon cell 图当成移动机器人覆盖问题；即使
  `1→0→2` 在二维/表面上可连接，也不符合打磨中“一个区域完整走完后再离面转场”的工艺。
- 决定：保留 run/cell 分解，但删除 cell 图贪心遍历、二维 A* 和 connector ray-lift。
  cell 按原扫描发现顺序稳定排序，每个 cell 完整加工；cell 之间在终点法向退刀，离面
  MoveJ 到下一起点的法向安全点，再 MoveL 接近。
- 自动判定：hole 完全位于 clip 内或与 clip 有正面积边界重叠都算相关 exclude；仅边界
  接触不算。同一扫描线多个 run 仍作为 STL 支撑缺口触发 cell 策略。
- 输入兼容：manual-v2 使用 chart/clip 建立 runs；未划分的原始 face-id region 复用普通
  mesh raster 的投影原点、U 轴和 split runs 建立相同 cells，不要求用户补做 manifest。
- 约束与影响：离面转场可以在二维投影上越过 exclude，但这不代表砂轮、主轴、法兰和
  机器人无碰撞；`SAFE_DISTANCE`、MoveJ 轨迹、可达性和构型必须在 RobotStudio 验证。
- 相关代码/测试：`experimental_algorithms/hole_aware_raster.py`、
  `scripts/window_conf_export.py`、`tests/test_hole_aware_raster.py`、
  `tests/test_raster_segments.py`。

## D012 RobotStudio 独立验证工作站导出

- 日期：2026-07-13
- 状态：Accepted for experiment
- 背景：Optimal-Y 完成后，人工逐面复制 RAPID、修改场景工件安装位置并另存工作站非常繁琐。
- 决定：实验 UI 增加“导入 RobotStudio 验证”，读取用户选择的实验结果目录，为每个最优
  region/patch 生成一个独立 RobotStudio 工作站。每个工作站只包含该面的最优路径。
- 坐标职责：场景工件组件名称和 `model_x/model_y/model_z/model_rz` 只控制模型安装位置；
  RAPID 的 tool/wobj 名称及数据来自该次路径导出，二者不得混用或相互改名。
- RAPID 规则：从最优路径模块提取 `tooldata` 和 `wobjdata` 到该工作站自己的
  `CalibData`，路径模块继续引用原始 tool/wobj 名称。
- 区域规则：同时支持未分区标签 `2`、`3`、`4` 和分区标签 `1_1`、`1_2`；文件名显示时
  分区下划线转为短横线。
- 命名规则：显式包含 `rz0`、`rz180` 等姿态，例如
  `3600_m800_440_rz0_1-1.rsstn`。
- 输出规则：生成的 `.rsstn` 放在该实验结果的 `optimal_paths/<region_label>/` 中，不覆盖
  模板工作站。
- 控制器规则：生成文件顺序复用模板中已实验标定的虚拟控制器；每个 `.rsstn` 同目录保存
  RAPID sidecar，工作站由 RobotStudio 正常打开后，插件自动切换为当前面的 `CalibData` 和路径模块。
  一次只验证一个生成工作站，不宣称多个工作站可并发使用同一控制器。
- 验证边界：插件只准备场景和程序，不评价姿态、碰撞、可达性或路径质量。
- 详细合同：见 `ROBOTSTUDIO_EXPORT.md`。

## D014 指定 region/patch 的小型机械臂姿态库

- 日期：2026-07-17
- 状态：Accepted for experiment；碰撞网格和最优规则由 D016 扩展
- 背景：少数加工区可能因机械臂腕部或杆段构型靠近工件，需要在不修改工具 TCP、接触点
  和表面法向的前提下做局部姿态试验。
- 决定：UI/CLI 只对用户指定的源 region 或 patch 测试整路径 TCP local-Z roll
  `[0,+15,-15,+30,-30]`；未指定项继续原 auto/base-y 策略。裸 `N` 匹配源 region 及其
  patches，`N-M/N_M/N.M` 精确匹配 patch，cell 不参与选择。
- 筛查：最多 7 个代表点使用 ABB MDH、数值 IK、构型连续性、关节跳变、J5 余量和
  机械臂碰撞。可导入主程序杆系包络；未导入时保留 5 mm 半径中心杆。
  工具/环境/自碰撞/扫掠体不在范围内。
- 回退：第一个通过的候选生效；全部失败则标记 `fallback-unverified`。快速预览可保留
  base-y 供诊断，但正式 runner 必须 deferred，禁止生成 candidate/optimal 避障路线。
- 日志：每个候选写 `robot_avoidance_trials.csv`，选择范围/状态写 `summary.json`。
- 相关代码/测试：`scripts/region_selectors.py`、
  `experimental_algorithms/robot_pose_avoidance.py`、configurable runner 和相关 pytest。

## D015 RAPID 快速预览恢复实验安装位姿

- 日期：2026-07-18
- 状态：Accepted
- 背景：Qt 主程序可导入实验 RAPID 检查路径，但若场景模型仍处于另一安装位姿，路径与工件不会重合。
- 决定：新 RAPID 写入 `RSP_EXPERIMENT_META_V1` 注释；旧结果由 `optimal_records.json` 或
  `optimal_selection.csv` 提供模型 X/Y/Z/RX/RY/RZ。Qt 导入器用 RAPID wobj 与输入项目
  picked origin 校验元数据，只在当前 CAD/输入项目一致时更新共享 `model_transform`。
- 被否决方案：从目录名猜位置；把 RAPID wobj 直接写成模型安装位置；导入时自动打开并覆盖整个项目。
- 原因：目录名不是稳定数据合同，wobj 与场景模型安装是两套坐标职责，自动替换项目会覆盖用户当前状态。
- 约束与影响：`workobject_transform` 和 RAPID 声明保持不变；校验失败仍允许只读路径预览，
  但禁止自动移动模型。单独复制新 RAPID 文件时仍可依靠内嵌元数据复现位置。
- 相关代码/测试：`scripts/window_conf_export.py`、`src/robot_studio_qt/path_planning/rapid_import.py`、
  `tests/test_window_conf_export.py`、`src/tests/path_planning/test_rapid_import.py`。

## D016 避障 patch 的种子支撑面恢复与墙体距离

- 日期：2026-07-20
- 状态：Accepted for experiment
- 背景：小 patch 所在加工面必然靠近路径，若完整工件网格直接参加最小距离评分，
  加工面会掩盖真正需要绕开的腔体墙面。
- 决定：只对用户指定的避障 region/patch，从最终路径 `Waypoint.face_id` 出发做共享边
  区域生长。局部法向、相对种子法向和参考平面误差同时合格的三角形组成完整支撑面；
  支撑面从墙体距离网格排除，剩余工件表面全部作为障碍。
- 障碍采样：优先保留支撑面包围盒外扩 800 mm 内的障碍 cells，再用全局障碍补足
  最多 6000 triangles，避免原全局步长采样优先漏掉邻近墙体。
- 选择规则：加工窗口仍是硬条件；候选须达到最小墙体间隙阈值，再最小化 TCP local-Z
  滚转角绝对值，相同角度以更大墙体间隙消除并列，不使用 base-Y 评分。
- UI/日志：快速预览黄=路径种子、绿=排除支撑面、红=墙体障碍；summary 保存每个区域
  的生长阈值、种子/支撑/障碍数量、参考法向和误差。
- 边界：当前只恢复近似平面，不是自由曲面语义分割；机械臂仍为抽样点/简化包络，
  不含工具、环境、自碰撞和连续扫掠验证。
- 相关代码/测试：`experimental_algorithms/support_surface_growth.py`、
  `scripts/optimal_y_score_configurable.py`、`ui/region_viewer.py`、
  `tests/test_support_surface_growth.py`。

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
