# Robot Arm Avoidance Trial Workflow

本文件说明实验 UI 中“避障区域”的机械臂姿态试验。该功能是候选筛查，不能替代
RobotStudio 或真实设备的安全验证。

## 范围

- 默认工具 TCP 标定和工具结构正确；当前碰撞只分析 ABB 机械臂杆段与工件。
- 当前试验故意把每段机械臂简化为半径 `5 mm`、额外间隙 `0 mm` 的细中心杆，
  不使用机器人配置中的真实连杆包络；这是算法连通性试验，不是安全碰撞模型。
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
  -> 固定整条路径的 TCP local-Z roll 候选
     [0, +15, -15, +30, -30] deg
  -> 每个候选均匀抽取最多 7 个代表点
  -> 数值 IK + J1/J4/J6 构型连续性
  -> 最大关节跳变 <= 40 deg，min |J5| >= 6 deg
  -> ABB 5 mm 中心杆与当前位姿工件网格碰撞粗筛
  -> 按库顺序选择第一个通过者
  -> RAPID/Optimal-Y 保持原流程
```

局部 Z roll 不改变 TCP 位置或 `TCP +Z = -surface normal`，只改变 TCP X/Y 和
机械臂腕部绕法向的构型。若所有候选均失败，状态为 `fallback-unverified`：快速预览
可保留原 `base_y` 供诊断，但正式 runner 必须把该位置写入 deferred，禁止生成
candidate/optimal 避障路线。

## 输出日志

启用避障后结果目录追加 `_robot_avoid`，并写：

- `robot_avoidance_trials.csv`：每个 pose/angle/region/feed/candidate 的 roll、IK 失败数、
  碰撞数、碰撞连杆名称、构型数、最大关节跳变、最小 `|J5|`、选择状态和原因；
- `summary.json`：原始输入、规范化 selector、实际命中的 labels、姿态库、状态计数、
  trial 表路径、5 mm 细杆参数和当前检测边界；
- 候选记录：`avoidance_selected/status/pose/roll_degrees`。

`baseline-validated` 和 `alternative-validated` 仅表示代表点通过内部粗筛。最终仍需在
RobotStudio 低速/单步检查完整 MoveL/MoveJ、真实工具、环境、自碰撞和控制器构型。

## 代码职责

- `scripts/region_selectors.py`：region/patch 选择语法和匹配规则；
- `experimental_algorithms/robot_pose_avoidance.py`：姿态库、IK/FK/碰撞筛查和回退；
- `scripts/runs/optimal_y_score_configurable.py`：只对命中项调度试验并写日志；
- `src/robot_studio_qt/tools/reachability/collision.py`：通用杆段—工件碰撞几何。
