# ABB 6700 实验平台文档地图

> 核对日期：2026-08-04  
> 范围：`experiments/base_casting_abb6700`，并补充与主程序 `p1/src` 的数据交接入口。  
> 用法：先按“我要做什么”定位，再按文末推荐路线阅读。涉及安全、坐标、manifest 或当前算法行为时，不要只看学习文档中的单独一段。

## 1. 文档可信度与阅读规则

| 等级 | 含义 | 使用方式 |
| --- | --- | --- |
| 当前合同 | 当前运行行为、数据格式或不可违反的约束 | 实现和 PPT 的主要依据 |
| 当前学习文档 | 面向理解的算法说明，已标注当前/历史内容 | 用于解释原理，仍需与代码核对 |
| 历史/研究资料 | 旧原型、被替代方案、未来方向 | 只能解释演进，不能当成当前调用链 |
| 通用学习笔记 | 机器人学基础概念 | 用于补背景，不作为平台行为依据 |

冲突时按下面顺序判断：

```text
当前代码与测试
  > DECISION_LOG.md 中 Accepted 且未被 supersede 的决策
  > 当前合同文档（README / PRINCIPLES / MANIFEST_SCHEMA / COORDINATE_SYSTEMS / HOLE_AWARE_PLANNER）
  > 带日期的学习文档
  > 旧原型和通用学习笔记
```

## 2. 按“我要做什么”快速定位

| 任务 | 首选文档 | 继续阅读 |
| --- | --- | --- |
| 先了解整个平台 | [README](../README.md) | [PRINCIPLES](../PRINCIPLES.md)、本文第 5 节 |
| 理解主程序如何把选面交给实验平台 | [README：当前流程](../README.md#当前流程) | 主程序交接代码见本文第 6 节 |
| 理解 `.rsp.json`、manifest、avoidance sidecar 的职责 | [MANIFEST_SCHEMA](../MANIFEST_SCHEMA.md) | [PRINCIPLES](../PRINCIPLES.md)、[DECISION_LOG](../DECISION_LOG.md) |
| 自动 face-id 分区 | [REGION_PARTITIONING_ALGORITHM](REGION_PARTITIONING_ALGORITHM.md) | `scripts/region_partitioning.py` |
| UI 手动 boundary/slab/pick 分区 | [README：手动 UV 分区](../README.md#手动-uv-分区) | [MANIFEST_SCHEMA](../MANIFEST_SCHEMA.md)、D001–D008 |
| 理解 UV、scanline、interval、run、cell | [HOLE_AWARE_ALGORITHM_LEARNING_GUIDE](HOLE_AWARE_ALGORITHM_LEARNING_GUIDE.md) | [HOLE_AWARE_PLANNER](../HOLE_AWARE_PLANNER.md) |
| 理解 auto 为什么判为带孔/不连续 | [AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE](AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE.md) | `plan_region_uv_auto()`、`polygon_has_relevant_holes()` |
| 确认当前避孔策略 | [HOLE_AWARE_PLANNER](../HOLE_AWARE_PLANNER.md) | [AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE](AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE.md) |
| 理解杆系避障粗筛 | [ROBOT_ARM_AVOIDANCE_WORKFLOW](ROBOT_ARM_AVOIDANCE_WORKFLOW.md) | [AVOIDANCE_FILE_MAP](AVOIDANCE_FILE_MAP.md)、`robot_pose_avoidance.py` |
| 理解障碍/墙体如何判定 | [WALL_SELECTION_LEARNING](WALL_SELECTION_LEARNING.md) | `support_surface_growth.py`、D020–D024 |
| 理解工件 X/Y/Z/RZ、world、wobj | [COORDINATE_SYSTEMS](../COORDINATE_SYSTEMS.md) | `placement_for()`、`WorkpieceTransform` |
| 理解 runner 批扫、候选与最优选择 | [README](../README.md) | `configurable_experiment_runner.py`、`optimal_y_selection.py` |
| 导入 RobotStudio 验证 | [ROBOTSTUDIO_EXPORT](../ROBOTSTUDIO_EXPORT.md) | [VALIDATION](../VALIDATION.md)、`robotstudio_package.py` |
| 排查失败 | [TROUBLESHOOTING](../TROUBLESHOOTING.md) | [VALIDATION](../VALIDATION.md)、结果目录 `summary.json` |
| 理解为何采用当前方案 | [DECISION_LOG](../DECISION_LOG.md) | 按 D 编号追踪被替代方案 |
| 学习机器人、URDF、路径/轨迹基础 | [学习.md](学习.md) | 仅作通用背景，不代表当前平台实现 |

## 3. `docs/` 学习文档逐项说明

| 文档 | 主要回答 | 当前性与注意事项 | 关键代码 |
| --- | --- | --- | --- |
| [AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE](AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE.md) | `auto` 怎样通过显式 exclude 与 split-scanline 两级判定进入 hole-aware | 当前分流说明可用；后半部 A*/connector 只作历史学习 | `window_conf_export.py::plan_region_uv_auto`、`hole_aware_raster.py::polygon_has_relevant_holes` |
| [HOLE_AWARE_ALGORITHM_LEARNING_GUIDE](HOLE_AWARE_ALGORITHM_LEARNING_GUIDE.md) | UV、scanline、interval、run、cell、ray-lift 与旧 A* 原型 | 当前实现只保留 run/cell；不运行 cell 图贪心、A* 或贴面 connector | `raster_domain.py`、`hole_aware_raster.py`、`window_conf_export.py` |
| [REGION_PARTITIONING_ALGORITHM](REGION_PARTITIONING_ALGORITHM.md) | 自动 face-id 分区：转角区、主平面/斜面、窄口、窗口尺寸切分 | 描述 CLI 自动预处理；实验 UI 当前主要暴露手动 UV 分区 | `region_partition_preprocess.py`、`region_partitioning.py` |
| [ROBOT_ARM_AVOIDANCE_WORKFLOW](ROBOT_ARM_AVOIDANCE_WORKFLOW.md) | 指定 region/patch 的 TCP-Z roll、IK/FK、碰撞和间隙粗筛 | 当前主流程可用；`validated` 仅表示最多 7 个代表点通过内部粗筛 | `region_selectors.py`、`robot_pose_avoidance.py`、`configurable_experiment_runner.py` |
| [WALL_SELECTION_LEARNING](WALL_SELECTION_LEARNING.md) | 支撑面、UVN 凸包棱柱与红色墙体 cell 的判定 | 当前墙体筛选简明入口；加工面与墙体必须互斥 | `window_conf_export.py::support_for_planning_region`、`support_surface_growth.py` |
| [AVOIDANCE_FILE_MAP](AVOIDANCE_FILE_MAP.md) | 避障相关文件、调用图和能力边界 | 适合找代码；核对日期为 2026-07-22，细节以 2026-08 代码/决策为准 | UI、runner、IK/FK、collision 全链路 |
| [学习.md](学习.md) | 刚体树、URDF/Xacro/SDF、路径与轨迹、常见规划方法 | 通用机器人学习笔记，不是实验平台设计合同 | 无直接运行入口 |

## 4. 根目录合同与运维文档

| 文档 | 职责 | 何时必须读 |
| --- | --- | --- |
| [README](../README.md) | 操作入口、当前 UI/CLI 流程、目录职责 | 第一次使用或准备演示平台时 |
| [AGENTS](../AGENTS.md) | 架构边界、脚本职责、硬规则 | 修改代码前 |
| [PRINCIPLES](../PRINCIPLES.md) | 分区、路径、避孔、避障、最优选择的设计原则 | 修改算法或解释设计时 |
| [MANIFEST_SCHEMA](../MANIFEST_SCHEMA.md) | `.rsp.json`、manifest v1/v2、`*_avoidance.json` 合同 | 修改持久化或移动输入文件时 |
| [COORDINATE_SYSTEMS](../COORDINATE_SYSTEMS.md) | model/world/wobj、picked origin、姿态与 tooldata | 修改 X/Y/Z/RZ、RAPID 或 RobotStudio 位姿时 |
| [HOLE_AWARE_PLANNER](../HOLE_AWARE_PLANNER.md) | 当前 auto/hole-aware 合同与限制 | 解释避孔、cell 抬刀和失败条件时 |
| [ROBOTSTUDIO_EXPORT](../ROBOTSTUDIO_EXPORT.md) | 打包、工作站、插件、文件命名与版本边界 | 导入 RobotStudio 前 |
| [DECISION_LOG](../DECISION_LOG.md) | Accepted/Superseded/Rejected 决策 | 准备恢复旧方案或发现文档冲突时 |
| [VALIDATION](../VALIDATION.md) | 自动测试与人工/RobotStudio 验收清单 | 修改后交付或正式验证前 |
| [TROUBLESHOOTING](../TROUBLESHOOTING.md) | 常见 UI、分区、避孔、坐标、避障、RobotStudio 故障 | 结果异常时按现象定位 |

补充算法说明：

- [experimental_algorithms/README](../experimental_algorithms/README.md)：实验算法目录的启用状态。
- [ROBOT_POSE_AVOIDANCE_PRINCIPLES](../experimental_algorithms/ROBOT_POSE_AVOIDANCE_PRINCIPLES.md)：姿态粗筛的详细原理；文末 2026-07-22 覆盖段优先于前面的旧“硬构型锁”描述。

## 5. 当前端到端流程与文档落点

```text
主程序
  选择加工面 + 项目/工具/工件/机器人状态
  ├─ 导出 latest_script_test.rsp.json
  └─ 可选保存独立 ABB 杆系配置 .rsc.json
        │
        ▼
实验 UI 读取 .rsp.json，必要时手动导入 .rsc.json
        │
        ├─ 需要分区：手动 UV boundary/slab/pick
        │              -> latest_partitioned.rsp.json
        │              -> latest_partitioned_manifest.json
        │
        ├─ 需要杆系避障：选择 region/patch，设置 UVN 范围并“应用”
        │                 -> *_avoidance.json
        │
        └─ 都不需要：直接进入安装位姿/加工窗口设置
                │
                ▼
设置 model X/Y/Z、转台 RZ、加工窗口、边缘余量
                │
                ▼
Runner 枚举安装位姿 × 转角 × planning region × 长/短边进给
                │
                ├─ 整个 region 必须落入 world/base 加工窗口
                ├─ 生成 UV raster，ray-lift 到 STL，姿态采用 base_y_aligned
                ├─ auto：exclude-overlap / split-scanline -> cell 抬刀；否则普通 raster
                ├─ 仅指定避障区域：支撑面 -> UVN 墙体 -> 13 个 roll -> 抽样 IK/FK/碰撞/间隙
                └─ 导出每个候选的 RAPID/诊断记录
                │
                ▼
按 region 选择 optimal
  ├─ 普通/避孔：min max|world Y|，并列再 min max|world X|
  └─ 杆系避障：仅内部 validated，最大化 sampled minimum clearance，不按 roll 排序
                │
                ▼
optimal_paths + optimal_records.json + summary/CSV
                │
                ▼
RobotStudio 打包
  每个 region/patch 独立生成 rsstn + CalibData.mod + VALIDATE_*.mod + sidecar
                │
                ▼
RobotStudio 6.08.01 逐站低速/单步验证完整机器人、工具、环境与连续运动
```

对应主文档顺序：

1. [README](../README.md)
2. [MANIFEST_SCHEMA](../MANIFEST_SCHEMA.md)
3. [COORDINATE_SYSTEMS](../COORDINATE_SYSTEMS.md)
4. [AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE](AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE.md)
5. [ROBOT_ARM_AVOIDANCE_WORKFLOW](ROBOT_ARM_AVOIDANCE_WORKFLOW.md)
6. [ROBOTSTUDIO_EXPORT](../ROBOTSTUDIO_EXPORT.md)
7. [VALIDATION](../VALIDATION.md)

## 6. 代码入口地图

### 6.1 主程序到实验平台

| 动作 | 代码 |
| --- | --- |
| 视口维护已选加工 regions | `../../../src/robot_studio_qt/ui/viewport.py` |
| 点击“导出到脚本测试” | `../../../src/robot_studio_qt/ui/main_window.py::_on_path_face_test_export_requested` |
| 写时间戳快照与 `latest_script_test.rsp.json` | `../../../src/robot_studio_qt/ui/script_test_export.py::export_script_test_project` |
| 保存独立 `.rsc.json` | `../../../src/robot_studio_qt/ui/main_window.py::_save_mechanism_configuration` |
| `.rsp.json` / `.rsc.json` 数据合同 | `../../../src/robot_studio_qt/project.py` |

### 6.2 实验 UI 与预处理

| 动作 | 代码 |
| --- | --- |
| 实验面板总入口 | `../ui/experiment_panel.py` |
| 参数解析与 runner 命令 | `../ui/experiment_config.py` |
| 手动 UV 分区 UI | `../ui/manual_partition_dialog.py` |
| 手动分区几何/manifest | `../scripts/manual_region_partitioning.py` |
| 自动 face-id 分区 CLI | `../scripts/region_partition_preprocess.py`、`../scripts/region_partitioning.py` |
| 避障范围 UI | `../ui/avoidance_settings_dialog.py` |

### 6.3 Runner、路径和筛选

| 动作 | 代码 |
| --- | --- |
| 正式批量编排 | `../scripts/configurable_experiment_runner.py::run_optimal_scan` |
| planning region/manifest 解析 | `../scripts/window_conf_export.py::manual_clip_regions` |
| 位姿与 wobj 同步 | `../scripts/window_conf_export.py::placement_for` |
| UV 轴、路径、姿态与 RAPID | `../scripts/window_conf_export.py` |
| 二维 interval、扣孔与 ray-lift | `../scripts/raster_domain.py` |
| auto 分流与 cell 组织 | `../experimental_algorithms/hole_aware_raster.py` |
| 支撑面、UVN 范围、墙体网格 | `../experimental_algorithms/support_surface_growth.py` |
| TCP-Z roll 与 IK/FK/碰撞粗筛 | `../experimental_algorithms/robot_pose_avoidance.py` |
| `.rsc.json` 校验/应用 | `../scripts/robot_config_override.py` |
| 按 region 选择最优 | `../scripts/optimal_y_selection.py` |

### 6.4 RobotStudio

| 动作 | 代码 |
| --- | --- |
| 读取 optimal、拆分 Calib/路径、复制模板 | `../scripts/robotstudio_package.py` |
| 启动与队列 | `../scripts/robotstudio_package.py::queue_manifest` |
| RobotStudio 6.08 插件 | `../robotstudio_addin/` |
| 模板/组件/程序路径配置 | `../configs/robotstudio_export.json` |

## 7. 关键数据文件

| 文件 | 谁写 | 谁读 | 含义 |
| --- | --- | --- | --- |
| `inputs/latest_script_test.rsp.json` | 主程序 | 实验 UI/runner | 完整项目快照，含选面、工件、工具、机器人配置/状态 |
| `*.rsc.json` | 主程序“保存杆系配置” | 实验 UI/runner | 可选的独立六轴 MDH、seed、杆件包络覆盖 |
| `inputs/latest_partitioned.rsp.json` | 分区 UI/CLI | 实验 UI/runner | 分区后的项目入口；手动 v2 仍保持项目 schema |
| `inputs/latest_partitioned_manifest.json` | 分区 UI/CLI | planning-region 解析器 | patch 标签、clip、exclude、chart 或自动分区元数据 |
| `inputs/*_avoidance.json` | 避障设置“应用” | 下一次正式 runner | 指定 labels 的 UVN 墙体范围；文件存在不等于已启用 |
| `results/.../summary.json` | runner | UI/人工审查 | 本次参数、分流、避障状态、输出路径和能力边界 |
| `all_candidates.csv` | runner | 诊断 | 所有非空几何候选，可能未通过内部避障粗筛 |
| `robot_avoidance_trials.csv` | runner | 诊断 | 每个 roll 的抽样 IK/FK、碰撞、间隙和原因 |
| `optimal_records.json` / `optimal_paths/` | runner | RobotStudio 打包器 | 每个 planning region 的最终入选记录与 RAPID |
| `robotstudio_jobs.json` | 打包器 | RobotStudio 插件/队列 | 每个 region/patch 的工作站生成任务 |

## 8. 当前实现特别容易误读的地方

1. `auto` 是分流器，不是第三套几何算法。
2. hole-aware 不等于“识别到圆孔”；ray miss、凹口或同一扫描线多个 run 也会触发。
3. 当前 hole-aware 不运行 A*；cell 间是法向退刀、离面 MoveJ、再法向接近。
4. 避孔只保护 processing path 不在无支撑域贴面直穿，不验证机器人杆件、工具或环境。
5. 杆系“避障”只枚举固定 TCP local-Z roll，不改 TCP XYZ，也不搜索空间绕行路径。
6. 快速预览只画几何 processing path，不执行 IK/FK，也不完整显示 RAPID 进退刀/MoveJ。
7. 非空但避障未验证的路径仍可进入 `candidates/` 供诊断；只有 `baseline-validated` 或 `alternative-validated` 可进入避障区域的 `optimal_paths/`。
8. `validated` 是内部最多 7 点、简化胶囊杆与局部墙体网格的粗筛结论，不是安全认证。
9. RobotStudio 工作站生成成功只表示模型和程序准备完成，最终仍需逐站人工验证。

## 9. 推荐阅读路线

### 9.1 十分钟了解平台

```text
本文第 5 节
  -> README“当前流程”
  -> HOLE_AWARE_PLANNER
  -> ROBOT_ARM_AVOIDANCE_WORKFLOW“范围”
  -> ROBOTSTUDIO_EXPORT“版本和边界”
```

### 9.2 准备修改路径算法

```text
AGENTS
  -> PRINCIPLES
  -> MANIFEST_SCHEMA
  -> COORDINATE_SYSTEMS
  -> HOLE_AWARE_PLANNER
  -> DECISION_LOG D001–D013
  -> VALIDATION
```

### 9.3 准备修改杆系避障

```text
ROBOT_ARM_AVOIDANCE_WORKFLOW
  -> WALL_SELECTION_LEARNING
  -> AVOIDANCE_FILE_MAP
  -> ROBOT_POSE_AVOIDANCE_PRINCIPLES 文末覆盖段
  -> DECISION_LOG D018、D020–D024、D027
  -> VALIDATION
```

### 9.4 准备排查 RobotStudio

```text
ROBOTSTUDIO_EXPORT
  -> COORDINATE_SYSTEMS
  -> TROUBLESHOOTING 的 RobotStudio/坐标条目
  -> DECISION_LOG D012、D015、D017、D029
  -> VALIDATION“RobotStudio 工作站导出验收”
```
