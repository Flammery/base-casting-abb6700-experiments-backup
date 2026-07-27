# Robot Arm Avoidance Trial Workflow（实验粗筛）

## Current stage: 2026-07-22

The UI/configurable runner again executes the experimental avoidance pipeline
for explicitly selected regions only:

```text
final geometric path
  -> support-surface recovery and obstacle mesh
  -> TCP local-Z rolls [0,+15,-15,+30,-30]
  -> sampled numerical IK seeded from the previous successful solution
  -> actual joint-jump and J5 checks
  -> FK robot-link collision and sampled clearance
  -> validated roll selection
```

`lock_configuration_to_seed` is false for all avoidance samples. A confdata
quadrant change is diagnostic, not a failure; actual joint-angle change decides
continuity. The shared solver remains unchanged while this experiment policy is
evaluated.

Non-empty paths with unresolved experimental IK are retained in candidate
output for ABB/RobotStudio diagnosis, but only `baseline-validated` and
`alternative-validated` avoidance paths enter `optimal_paths`. The compact
`robot_avoidance_trials.csv` contains installation XYZ, turntable angle, region,
feed variant, tool roll, selected flag, status, sampled interference, minimum
clearance, maximum joint jump, and a short reason. This sampled screen is not a
complete robot safety validation.

## 当前阶段（2026-07-22）

UI 的“快速预览”只生成并显示几何路径，不执行 IK/FK 或碰撞筛查；状态栏会显示
“待正式运行 IK/FK”。点击正式运行后，configurable runner 只对“避障区域”命中的
region/patch 执行数值 IK、FK 杆系碰撞、最小间隙和 TCP-Z roll 枚举。未命中的区域仍走
原有几何路径流程。

几何路径即使在实验 IK 中 unresolved，也会保留在 `candidates/` 供 ABB/RobotStudio
诊断；但只有 `baseline-validated` 和 `alternative-validated` 能进入 `optimal_paths/`。
因此不能把 `candidates/` 中存在 RAPID 文件理解成“已经避障通过”。

本文件说明实验 UI 中“避障区域”的机械臂姿态试验。该功能是候选筛查，不能替代
RobotStudio 或真实设备的安全验证。

## 范围

- 默认工具 TCP 标定和工具结构正确；当前碰撞只分析 ABB 机械臂杆段与工件。
- 未导入独立杆系配置时，试验把每段机械臂简化为半径 `5 mm`、额外间隙 `0 mm`
  的细中心杆。通过实验 UI“导入杆系配置”选择主程序导出的 `.rsc.json` 后，IK/FK
  使用其 MDH、关节限位和 seed，碰撞使用每段配置包络的近似半径。
- 配置包络仍是围绕杆系线段的胶囊近似，不是 ABB CAD 外壳；这是实验检测模型，
  不是安全认证模型。
- 不包含工具实体、自碰撞、底座、地轨、转台、环境设备和运动段 swept volume。
- 工程内 ABB 6700 MDH/数值 IK 是诊断模型，不是 RobotWare 解析求解器。
- 未填写“避障区域”时，所有 region/patch 完全沿用既有 `auto` 路径策略。

## 选择语法

- `1,2,3`：选择源 region；若 region 1 已分成多个 patch，`1` 会选择其全部 patch。
- `1-1,1-2`：只选择指定 patch；`1_1`、`1.1` 也可输入并统一为 `1_1`。
- cell 是 hole-aware 的临时运行单元，不可用该输入框选择。
- 空项、0、负数、错误标签或不存在的区域在运行前直接报错。

## 姿态库与工作流

```text
region/patch selector
  -> 既有 auto planner 生成接触路径
  -> 以最终路径 waypoint.face_id 为种子恢复完整支撑面
  -> 从工件网格排除支撑面，剩余面作为墙体障碍网格
  -> 固定整条路径的 TCP local-Z roll 候选
     [0, +15, -15, +30, -30] deg
  -> 每个候选均匀抽取最多 7 个代表点
  -> 数值 IK + J1/J4/J6 构型连续性
  -> 最大关节跳变 <= 40 deg，min |J5| >= 6 deg
  -> 导入时使用配置包络；未导入时使用 ABB 5 mm 中心杆
  -> 与墙体障碍网格做碰撞和最小间隙粗筛
  -> 抽样最小间隙 >= 默认 5 mm 安全阈值
  -> 选择 abs(TCP local-Z roll) 最小的通过者
  -> 避障区域按相同规则比较安装位置，不再使用 base-Y 评分
```

局部 Z roll 不改变 TCP 位置或 `TCP +Z = -surface normal`，只改变 TCP X/Y 和
机械臂腕部绕法向的构型。若所有候选均失败，状态为 `fallback-unverified`：快速预览
和正式 runner 都可保留原 `base_y` 几何路径供诊断；正式 runner 仍会把它写入
`candidates/`，但会在选择最优路径前将其过滤，禁止进入 `optimal_paths/`。

支撑面恢复使用网格邻接区域生长。候选三角形必须同时满足局部法向夹角、相对种子
参考法向和参考平面距离阈值；墙面、明显圆角和台阶会停止生长。“避障设置”弹窗
为每个最终 planning label 保存一个模型坐标 UVN 长方体，U/V 使用独立扩大比例，
N+/N- 使用独立毫米高度。只有范围内非支撑 cells 进入墙体网格，不区分朝上或朝下。
预览颜色为：黄色打磨面、绿色支撑面、红色范围内墙体、半透明灰色 UVN 范围；
范围外模型保持灰色。范围内墙体仍按当前实验预算最多输出 `6000` 个三角形。

## 输出日志

启用避障后结果目录追加 `_robot_avoid`，并写：

- `robot_avoidance_trials.csv`：每个 pose/angle/region/feed/candidate 的 roll、IK 失败数、
  碰撞数、碰撞连杆名称、抽样最小间隙、要求间隙、构型数、最大关节跳变、
  最小 `|J5|`、选择状态和原因；
- `summary.json`：原始输入、规范化 selector、实际命中的 labels、姿态库、状态计数、
  trial 表路径、杆系配置来源、每段包络尺寸/碰撞半径、支撑面生长参数/数量和当前检测边界；
- 候选记录：`avoidance_selected/status/pose/roll_degrees/min_clearance_mm/required_clearance_mm`。

`baseline-validated` 和 `alternative-validated` 仅表示代表点通过内部粗筛。最终仍需在
RobotStudio 低速/单步检查完整 MoveL/MoveJ、真实工具、环境、自碰撞和控制器构型。

## 代码职责

- `scripts/region_selectors.py`：region/patch 选择语法和匹配规则；
- `scripts/robot_config_override.py`：读取并验证主程序 `.rsc.json`，应用 MDH/seed 和配置包络；
- `experimental_algorithms/support_surface_growth.py`：路径种子区域生长和墙体障碍网格；
- `experimental_algorithms/robot_pose_avoidance.py`：姿态库、IK/FK/碰撞筛查和回退；
- `scripts/optimal_y_score_configurable.py`：只对命中项调度试验并写日志；
- `src/robot_studio_qt/tools/reachability/collision.py`：通用杆段—工件碰撞几何。
