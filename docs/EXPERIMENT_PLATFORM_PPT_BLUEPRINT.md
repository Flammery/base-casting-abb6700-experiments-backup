# ABB 6700 打磨实验平台 PPT 页面蓝图

> 版本：2026-08-04  
> 目标：用一张总流程图建立全局认识，再逐页展开分区、位姿、轨迹、避孔、杆系避障、最优选择和 RobotStudio 验证。  
> 预计页数：13 页。  
> 画面比例：16:9，建议 1280 × 720。  
> 内容依据：当前项目代码、`docs/DOCS_MAP.md` 所列合同/学习文档，以及用户提供的工程流程图风格参考。

## 1. 沟通目标

面向实验平台开发、机器人应用和工艺人员，讲清楚：

> 主程序负责定义加工对象和机器人模型；实验平台负责生成、筛选和解释安装位姿与路径候选；RobotStudio 负责对完整机器人单元做最终验证。

整套 PPT 必须始终区分：

- **避孔/缺口处理**：保护 TCP processing path，避免在孔洞或无表面支撑区贴面直穿。
- **杆系避障粗筛**：改变整条路径的 TCP local-Z roll，用抽样 IK/FK 和简化杆件包络筛选机器人姿态。
- **最终安全验证**：由 RobotStudio/真实单元检查完整 MoveL/MoveJ、工具、环境、自碰撞和控制器构型。

## 2. 全局视觉规范

### 2.1 风格

沿用参考图的工程流程图语言：

- 白色背景，不使用照片背景、渐变或阴影。
- 黑色/深灰色细线，节点和模块边界清楚。
- 浅绿色平行四边形：输入、配置、项目文件。
- 浅橙色圆角大框：系统、规划器或算法模块。
- 白色矩形：普通处理步骤。
- 白色菱形：条件判断。
- 浅蓝色平行四边形：输出、报告、数据库或结果文件。
- 实线箭头：控制流、计算流。
- 虚线箭头：文件写入、旁路记录、诊断输出。
- 模块名称写在边框左上角，不做 UI 卡片或装饰性标签。

### 2.2 建议色值

| 语义 | 填充 | 边框 |
| --- | --- | --- |
| 输入/配置 | `#E7F2DF` | `#8FB27C` |
| 模块容器 | `#FFF0DC` | `#E4A12D` |
| 处理/判断 | `#FFFFFF` | `#333333` |
| 输出/结果 | `#E0ECFF` | `#8DAFE5` |
| 警告/能力边界 | `#FFF6E8` | `#D38B20` |
| 主文字 | `#202020` | — |
| 次文字 | `#5B5B5B` | — |

### 2.3 字体和线条

- 中文：微软雅黑；英文和数字可跟随微软雅黑或 Aptos。
- 封面标题：50–56 pt。
- 页面标题：35–40 pt。
- 流程节点：16–19 pt。
- 补充说明：16–18 pt。
- 模块边框：1.4–1.8 pt。
- 流程箭头：1.4–1.8 pt，统一小三角箭头。
- 所有流程图使用 PowerPoint 原生形状和连接线，保持可编辑。

## 3. 页面蓝图

---

## 第 1 页｜封面

### 页面任务

建立主题和边界，不在封面堆叠算法名。

### 页面标题

**ABB 6700 打磨实验平台流程**

### 副标题

**从主程序选面、路径候选生成到 RobotStudio 验证**

### 页脚文字

`experiments/base_casting_abb6700 · 当前实现说明`

### 布局

- 左上或居中放主标题。
- 标题下放副标题。
- 右下角用一条细线连接三个小型原生形状：绿色输入 → 橙色实验平台 → 蓝色 RobotStudio 输出。
- 不放完整流程图。

### 讲解要点

这不是机器人安全认证工具介绍，而是当前离线实验平台如何组织输入、规划、筛选、导出和验证的实现说明。

### 内容来源

- `README.md`
- `AGENTS.md`
- `docs/DOCS_MAP.md`

---

## 第 2 页｜总流程图

### 页面任务

让观众先获得端到端地图，后续各页都是这张图的局部放大。

### 页面标题

**一条主线把选面、候选生成和最终验证连接起来**

### 页面可见文字

主流程节点依次为：

1. `主程序：定义加工对象`
2. `导出项目与杆系配置`
3. `实验平台加载 planning regions`
4. `可选：分区 / 杆系避障设置`
5. `设置工件位姿与加工窗口`
6. `Runner 生成并筛选候选`
7. `按 region 选择 Optimal`
8. `生成 RobotStudio 工作站`
9. `逐站人工验证`

页面底部结论：

> **实验平台负责候选生成和内部粗筛；RobotStudio 负责完整机器人单元验证。**

### 流程关系

```text
主程序
  ↓
项目/杆系 JSON
  ↓
实验平台输入
  ↓
分区/避障设置（可选）
  ↓
工件位姿、角度、窗口
  ↓
Runner：几何路径 → 避孔 → 可选杆系粗筛
  ↓
Candidates → Optimal
  ↓
RobotStudio 工作站
  ↓
完整验证
```

### 布局

- 横向主流程，必要时分为上下两行。
- 输入节点用绿色；Runner 用橙色大框；结果和 RobotStudio 用蓝色。
- “分区”“避孔”“杆系避障”只显示名称，不在总图展开判断细节。
- 右上角可放小型页码导航：`输入 → 预处理 → 规划 → 筛选 → 验证`。

### 讲解要点

- 主程序和实验平台通过 JSON 交接，不直接修改主程序项目 schema。
- 每个最终 planning region 独立规划和导出。
- RobotStudio 是流程的必要终点，不是可选展示步骤。

### 内容来源

- `README.md`
- `PRINCIPLES.md`
- `ROBOTSTUDIO_EXPORT.md`

---

## 第 3 页｜主程序输入与数据交接

### 页面任务

解释 `.rsp.json` 与 `.rsc.json` 的区别，避免理解成“所有内容只在一个文件里”。

### 页面标题

**主程序通过项目快照和可选杆系覆盖向实验平台交接数据**

### 页面可见文字

绿色输入节点：

- `已选加工面 selected_path_face_regions`
- `工件、picked origin 与 wobj`
- `抛光工具与 TCP`
- `机器人配置与关节状态`

中央主程序模块：

- `导出到脚本测试`
- `保存杆系配置（可选）`

蓝色输出节点：

- `latest_script_test.rsp.json`
- `ABB 6700 Style.rsc.json（可选）`

右侧说明：

> `.rsp.json` 是完整项目快照；`.rsc.json` 只在需要独立覆盖 MDH、seed 和杆件包络时导入。

### 流程关系

```text
主程序 ProjectSession
  ├─ 已选加工 regions + 工件 + 工具 + 项目内机器人状态
  │    └─ 导出 latest_script_test.rsp.json
  └─ 当前机器人/杆系配置
       └─ 可选保存 .rsc.json

两路文件在实验 UI 汇合
```

### 布局

- 上方三个绿色平行四边形表示主程序状态来源。
- 中间浅橙色圆角框表示主程序导出模块。
- 下方两个浅蓝平行四边形表示两种文件。
- 右侧使用一段不超过三行的解释文字。

### 讲解要点

- `.rsp.json` 已经带有项目内 `robot_config` 和 `joint_state`。
- `.rsc.json` 是避障实验的可选覆盖；当前要求六轴、`kinematic_model=mdh`，并且每段包络尺寸有效。
- 选面以 region 列表保存，不把多个加工面合成一条路径。

### 内容来源

- `p1/src/robot_studio_qt/ui/main_window.py`
- `p1/src/robot_studio_qt/ui/script_test_export.py`
- `p1/src/robot_studio_qt/project.py`
- `scripts/robot_config_override.py`

---

## 第 4 页｜分区和避障设置是两条独立的可选支线

### 页面任务

说明“需要分区”“需要杆系避障”“都不需要”三个操作分支，以及三个 JSON 文件的职责边界。

### 页面标题

**预处理只在需要时介入，分区与杆系避障互不替代**

### 页面可见文字

判断 1：`加工区域是否需要拆成独立 patch？`

- 是：`手动 UV 分区：boundary / slab / pick`
- 输出：`latest_partitioned.rsp.json + latest_partitioned_manifest.json`
- 否：`沿用原始 selected region`

判断 2：`是否指定杆系避障 region / patch？`

- 是：`设置 U/V 扩大比例与 N+/N− 高度`
- 输出：`*_avoidance.json`
- 否：`避障关闭，直接进入位姿设置`

底部规则：

> **avoidance sidecar 只有在弹窗点击“应用”后才武装下一次 Runner；文件存在不等于已启用。**

### 流程关系

```text
.rsp.json
  ↓
◇ 需要分区？
  ├─ 是 → boundary/slab/pick → partitioned.rsp.json + manifest
  └─ 否 → 原始 regions
                 ↓
◇ 需要杆系避障？
  ├─ 是 → region/patch selector + UVN 范围 → Apply → avoidance.json
  └─ 否 → 不启用
                 ↓
工件位姿与窗口设置
```

### 布局

- 左半页放两个串联判断菱形。
- 右上放“手动 UV 分区”橙色模块，内含三种模式。
- 右下放“避障设置”橙色模块，内含 selector、UVN、Apply。
- 三个 JSON 输出用蓝色平行四边形。

### 讲解要点

- 手动分区定义二维加工域，不切割 STL。
- `.rsp.json` 继续保存原有项目字段；patch 边界写入 manifest。
- `1` 表示源 region 的全部最终 patches，`1-1` 表示指定 patch。
- cell 是运行时单位，不能作为避障 selector。

### 内容来源

- `MANIFEST_SCHEMA.md`
- `README.md#手动-uv-分区`
- `DECISION_LOG.md` D001–D008、D020、D027
- `ui/manual_partition_dialog.py`
- `ui/avoidance_settings_dialog.py`

---

## 第 5 页｜工件位姿与三套坐标

### 页面任务

说明 X/Y/Z/RZ 如何同时控制模型和 wobj，以及 model/world/wobj 的不同职责。

### 页面标题

**安装位姿必须同步模型与 wobj，窗口和 RAPID 使用不同坐标**

### 页面可见文字

设置输入：

- `Model X / Y / Z`
- `Turntable RZ`
- `加工窗口 X/Y/Z`
- `边缘余量 boundary_margin`

三套坐标：

| 坐标 | 用途 |
| --- | --- |
| `position_model` | UV、STL 命中、face id、表面法向 |
| `position_world/base` | 加工窗口、base Y、confdata、IK/FK |
| `position_wobj` | RAPID robtarget |

变换主线：

`model → world/base → wobj`

公式框：

```text
wobj XYZ = model XYZ + rotate_RZ(picked_origin)
wobj RZ  = experiment RZ
```

警示文字：

> picked origin 只旋转一次；禁止只移动模型而不更新 wobj。

### 布局

- 左侧放 X/Y/Z/RZ 四个绿色输入形状。
- 中部放三套坐标的横向变换箭头。
- 右侧放公式与不可违反规则。
- 用细虚线从 world/base 指向“窗口/IK”，从 wobj 指向“RAPID”。

### 讲解要点

- 窗口判断必须使用 world/base。
- RAPID 位置必须写 `position_wobj`。
- 姿态先在 world 下生成，再转换到 wobj 相对四元数。
- 工具 TCP 不能用来补偿路径方向。

### 内容来源

- `COORDINATE_SYSTEMS.md`
- `scripts/window_conf_export.py::placement_for`
- `scripts/window_conf_export.py::rapid_text`

---

## 第 6 页｜Runner 的批量循环与窗口门控

### 页面任务

解释点击 Runner 后首先发生的批量枚举、区域门控和路径策略分派。

### 页面标题

**Runner 先枚举安装候选，再让每个区域独立通过加工窗口**

### 页面可见文字

外层循环：

```text
安装 X × Y × Z
  × 转台角度 RZ
  × planning region / patch
  × long_side / short_side
```

窗口判断：

> **当前区域的全部有效几何必须落入 world/base 加工窗口，才生成候选。**

默认实验参数：

- `spacing = 50 mm`
- `point_step = 50 mm`
- `boundary_margin = 6 mm`
- 默认窗口：`X 1500–2500 mm`、`Y -1050–1050 mm`
- 路径方向：`long_side / short_side`

判断输出：

- 否：`跳过该 pose / angle / region`
- 是：`进入 UV 路径规划`

### 流程关系

```text
读取项目与 planning regions
  ↓
For each X/Y/Z
  ↓
For each RZ
  ↓
For each region/patch
  ↓
◇ 整个区域在加工窗口内？
  ├─ 否 → coverage 记录，跳过
  └─ 是 → long/short 两个 feed variant → 规划
```

### 布局

- 使用类似参考图“Run”模块的嵌套矩形。
- 外框标题 `Configurable Runner`。
- 内部依次嵌套 `For each pose`、`For each angle`、`For each region`。
- 中央菱形进行窗口判断。
- 右侧用小参数表展示默认值。

### 讲解要点

- 每个 region/patch 独立生成候选，不跨区连接路径。
- “整个区域落入窗口”是保守门控，不是逐点裁剪后继续加工。
- 结果目录按日期和当日编号隔离，完整参数写入 `summary.json`。

### 内容来源

- `scripts/configurable_experiment_runner.py::run_optimal_scan`
- `scripts/window_conf_export.py::region_inside_window`
- `DECISION_LOG.md` D025、D028

---

## 第 7 页｜UV 光栅、ray-lift 与加工姿态

### 页面任务

解释如何从二维加工域得到三维 TCP 点和方向。

### 页面标题

**路径先在二维 UV 中组织，再投射回 STL 得到三维加工点**

### 页面可见文字

几何链路：

1. `读取当前 region / patch 的 STL triangles`
2. `建立局部 raster chart：origin / U / V / normal`
3. `从 patch 边界确定长边和短边扫描轴`
4. `在 clip_polygon 内生成 scanlines`
5. `从 intervals 中扣除 exclude_polygons`
6. `沿 chart normal 对每个 UV 点做 ray-lift`
7. `命中三角形提供 XYZ、face_id 和 facet normal`
8. `生成 model / world / wobj waypoint`

姿态规则：

```text
TCP +Z = -surface normal
TCP +Y 尽量投影对齐 base +Y
```

页面结论：

> **二维 polygon 决定加工边界，STL 只提供三维落点和法向。**

### 流程关系

```text
三维选中曲面
  → raster UV chart
  → scanline intervals
  → clip / subtract holes
  → ray-lift
  → XYZ + normal
  → base_y_aligned quaternion
  → Waypoint
```

### 布局

- 从左至右展示二维到三维的转换。
- 中间用一个简化二维矩形域表示 scanlines 与孔洞断口。
- 右侧用三个叠放坐标框表示 model/world/wobj waypoint。
- 所有示意均使用原生线条和形状，不使用不可编辑截图。

### 讲解要点

- 手动 v2 patch 使用自己的 `raster_chart + clip_polygon`。
- 未分区 face-id region 使用边界 UV 轴，PCA 只作为兜底。
- ray miss 必须结束当前 run，不能删除孔内点后继续直线连接。
- 四元数 `q` 与 `-q` 通过点积保持表示连续。

### 内容来源

- `scripts/raster_domain.py`
- `scripts/window_conf_export.py::uv_axes_from_region`
- `scripts/window_conf_export.py::plan_region_uv`
- `COORDINATE_SYSTEMS.md`

---

## 第 8 页｜Auto 避孔判定与 Cell 抬刀（PPT 样张页）

### 页面任务

解释 `auto` 如何选择普通 raster 或 hole-aware，以及当前 cell 抬刀策略实际做了什么。

### 页面标题

**Auto 用两级判定发现不连续，再让每个 Cell 独立完成加工**

### 页面副标题

`显式 exclude 正面积重叠` 与 `实际 split-scanline` 任一命中都会启用 cell 抬刀。

### 页面可见文字

顶部绿色输入：

- `clip_polygon`
- `exclude_polygons`
- `selected STL cells`

左侧模块 `Auto 分流`：

1. `读取当前 planning region`
2. 判断：`exclude 与 clip 有正面积重叠？`
3. 否：`生成普通 raster`
4. 判断：`同一扫描线存在多个 run？`
5. 否：`保留普通 raster`
6. 是：进入 `Hole-aware Cell 策略`

右侧模块 `Hole-aware Cell 策略`：

1. `scanline 扣除 exclude intervals`
2. `ray miss 立即断开 run`
3. `相邻重叠 runs 组成 cell`
4. `每个 cell 完整往复加工`
5. `端点沿局部法向退刀 150 mm`
6. `离面端点之间 MoveJ 转场`
7. `下一 cell 沿法向接近`

底部蓝色输出：

- `Regular raster PathResult`
- `Cell-lift PathResult`
- `planner_reason：regular-raster / exclude-overlap / split-scanline`

页面警示：

> **当前实现不运行 A*，也不生成跨孔的贴面 connector。**

### 布局

- 顶部三只浅绿色平行四边形作为输入。
- 主体为浅橙色大容器 `Auto Planner`。
- 容器左侧是判定流程，右侧是细化的 Cell 策略。
- 两个判断使用菱形，“是/否”直接写在连接线附近。
- 底部蓝色平行四边形显示两个 PathResult 和 planner reason。
- 连接线置于节点后方；所有对象保持可编辑。

### 讲解要点

- `auto` 是分流器，不是第三套几何算法。
- exclude 完全位于 clip 内或穿过边界都可能命中；仅点/边接触不算。
- split-scanline 可以来自真实孔、凹口、ray miss 或选面缺口。
- cell 内保留完整 raster，cell 间才抬刀。
- `SAFE_DISTANCE=150 mm` 沿端点局部法向，不等于统一世界 Z 安全平面。

### 内容来源

- `HOLE_AWARE_PLANNER.md`
- `docs/AUTO_HOLE_CLASSIFICATION_LEARNING_GUIDE.md`
- `docs/HOLE_AWARE_ALGORITHM_LEARNING_GUIDE.md`
- `scripts/window_conf_export.py::plan_region_uv_auto`
- `experimental_algorithms/hole_aware_raster.py`

---

## 第 9 页｜障碍墙体如何判定

### 页面任务

解释杆系避障使用的墙体网格从哪里来，以及为什么加工面不能同时成为障碍。

### 页面标题

**障碍不是整个工件，而是局部 UVN 范围内的非支撑墙体**

### 页面可见文字

支撑面规则：

- `未分区 region：全部加工 face_ids 直接作为支撑面`
- `分区 patch：从路径 face_id 生长支撑面，再强制合并源加工面`

UVN 范围构建：

1. `把完整支撑面的所有顶点投影到局部 UV`
2. `计算一个二维凸包`
3. `围绕中心按 U/V 百分比扩大`
4. `沿 N+/N− 拉伸成封闭棱柱`
5. `筛选棱柱内部或穿过边界的工件 cells`

核心公式：

```text
墙体障碍 cells
= UVN 凸包棱柱内的工件 cells
− 支撑面 cells
```

颜色说明：

- `黄色：实际打磨面`
- `绿色：完整支撑面`
- `红色：局部墙体障碍`
- `半透明灰色：UVN 范围`

### 布局

- 左侧为支撑面确定流程。
- 中央为 UV 投影 → 凸包 → 拉伸的流程。
- 右侧用集合关系和颜色图例说明最终墙体。
- 底部放一条不变量：`加工面 cells ∩ 墙体 cells = 空集`。

### 讲解要点

- 凸包有意覆盖孔洞、凹入区和窄连接，换取稳定单一外边界。
- 支撑面从障碍集合中排除是数据规则，不只是预览颜色规则。
- 正式 collision mesh 最多约 6000 个墙体三角形。
- 墙体范围只决定本轮参与粗筛的工件表面，不等于完整环境模型。

### 内容来源

- `docs/WALL_SELECTION_LEARNING.md`
- `docs/ROBOT_ARM_AVOIDANCE_WORKFLOW.md`
- `experimental_algorithms/support_surface_growth.py`
- `DECISION_LOG.md` D020–D024

---

## 第 10 页｜杆系姿态避障粗筛

### 页面任务

说明当前“避障”能改变什么、如何检查以及接受条件。

### 页面标题

**杆系避障只改变 TCP 绕法向的滚转，并对代表点做内部粗筛**

### 页面可见文字

固定姿态库：

`0° → +15° → −15° → +30° → −30°`

保持不变：

- `TCP XYZ`
- `TCP +Z = −surface normal`
- `加工点顺序`
- `避孔与 Cell 拓扑`

每个候选姿态最多均匀抽取 7 个点，依次检查：

1. `所有抽样点 IK 成功`
2. `上一成功解作为下一点 seed`
3. `最大实际关节跳变 ≤ 40°`
4. `最小 |J5| ≥ 6°`
5. `FK 杆件不与墙体网格碰撞`
6. `抽样最小间隙 ≥ 5 mm`

最小间隙：

```text
clearance
= distance(segment centerline, triangle)
− link envelope radius
```

选择规则：

> 在所有 `validated-clear` 姿态中选择 sampled minimum clearance 最大者；不按 TCP roll 排序。

### 布局

- 左上放 13 个姿态输入形状（`-90°` 到 `+90°`，间隔 `15°`）。
- 中部放 `For each roll` 嵌套框。
- 内部依次放 IK、连续性、J5、FK 碰撞、间隙菱形或处理框。
- 右侧放“保持不变”和最小间隙公式。
- 底部输出 `baseline-validated / alternative-validated / fallback-unverified`。

### 讲解要点

- 导入 `.rsc.json` 时使用其 MDH、关节限位、seed 和每段包络半径。
- 显式导入 `.rsc.json` 时使用配置包络；未导入时使用统一 100 mm 半径回退。
- confdata 象限变化只作诊断；连续性由真实关节跳变判断。
- 当前不重新规划 TCP XYZ，不是 RRT/PRM 类型的空间绕障。

### 能力边界文字

> **内部 validated 仅表示最多 7 个离散姿态通过简化模型粗筛，不代表完整运动安全。**

### 内容来源

- `docs/ROBOT_ARM_AVOIDANCE_WORKFLOW.md`
- `docs/AVOIDANCE_FILE_MAP.md`
- `experimental_algorithms/ROBOT_POSE_AVOIDANCE_PRINCIPLES.md` 文末覆盖段
- `experimental_algorithms/robot_pose_avoidance.py`
- `scripts/robot_config_override.py`

---

## 第 11 页｜候选、诊断与 Optimal 选择

### 页面任务

解释为什么 `candidates/` 中可能有未验证路径，以及普通区域和避障区域采用不同的最优规则。

### 页面标题

**所有非空几何路径用于诊断，只有符合准入规则的候选才能进入 Optimal**

### 页面可见文字

候选输出：

- `all_candidates.csv`
- `candidates/<pose>/rz<angle>/<region>/<feed>/`
- `robot_avoidance_trials.csv`
- `deferred_paths.csv`
- `coverage_by_pose.csv`

普通/避孔区域的最优规则：

```text
1. 最小化 max(abs(world_y))
2. 并列时最小化 max(abs(world_x))
```

杆系避障区域的准入与排序：

```text
只允许 baseline-validated / alternative-validated
1. 最大化 sampled minimum clearance
2. 完全相同时保留稳定扫描顺序，不按 TCP roll 排序
```

关键说明：

> 非空但 IK unresolved、碰撞、间隙不足或其它内部粗筛失败的路径仍可保留在 `candidates/` 供 RobotStudio 诊断，但不会进入避障区域的 `optimal_paths/`。

最终输出：

- `optimal_paths/<region_label>/`
- `optimal_selection.csv`
- `optimal_records.json`
- `summary.json`

### 布局

- 左侧大框 `Candidates`，汇聚普通 raster、cell-lift、避障试验结果。
- 中间用两个并列的 `Select` 模块表示普通评分和避障评分。
- 右侧蓝色输出框 `Optimal by region`。
- 失败但可诊断路径用虚线回到 `candidates/`，不连接 `optimal_paths/`。

### 讲解要点

- `candidates/` 存在 RAPID 不等于避障通过。
- 每个 region/patch 独立选一个最优结果。
- `summary.json` 是解释某次运行使用了哪些策略和限制的首要证据。

### 内容来源

- `scripts/configurable_experiment_runner.py`
- `scripts/optimal_y_selection.py`
- `DECISION_LOG.md` D018、D026、D028

---

## 第 12 页｜RAPID 生成与 RobotStudio 打包

### 页面任务

解释从最优记录到独立工作站的文件生成与加载链路。

### 页面标题

**每个最优 region 或 patch 都生成一套独立 RobotStudio 验证工作站**

### 页面可见文字

RAPID 路径生成：

- `加工点与法向进退刀：MoveL`
- `Cell 的两个离面端点之间：MoveJ`
- `姿态：world quaternion → wobj quaternion`
- `confdata：按 world Y 正负写入`
- `ConfL \Off`
- `保留项目 tool 和 wobj 名称`

RobotStudio 打包：

1. `读取 optimal_records.json`
2. `从最优 RAPID 分离 tooldata / wobjdata`
3. `写 CalibData.mod`
4. `写 VALIDATE_<region>.mod`
5. `从模板复制 .rsstn`
6. `只修改场景工件组件的 X/Y/Z/RZ`
7. `写同名 .robotstudio_job.json`
8. `插件等待虚拟控制器并加载模块`

文件结构示例：

```text
optimal_paths/1_1/
  1_1.txt
  CalibData.mod
  VALIDATE_1_1.mod
  3600_m800_440_rz0_1-1.rsstn
  3600_m800_440_rz0_1-1.robotstudio_job.json
```

### 布局

- 左侧放 RAPID 生成模块。
- 中央蓝色 `optimal_records + optimal_paths`。
- 右侧放 RobotStudio Package 橙色模块和最终文件树。
- 场景安装位置与 RAPID wobj 用两条独立箭头表示，避免误认为同一数据。

### 讲解要点

- 场景组件名不是 RAPID wobj 名称。
- 模板工作站不会被覆盖。
- 多个生成工作站复用模板虚拟控制器状态，必须一次只打开一个。
- `RSP_EXPERIMENT_META_V1` 使用 ASCII JSON 转义以兼容 RobotWare 6。

### 内容来源

- `ROBOTSTUDIO_EXPORT.md`
- `COORDINATE_SYSTEMS.md`
- `scripts/window_conf_export.py::rapid_text`
- `scripts/robotstudio_package.py`
- `robotstudio_addin/`
- `DECISION_LOG.md` D012、D015、D017、D029

---

## 第 13 页｜最终验证和能力边界

### 页面任务

回扣开场，明确哪些事情平台做了、哪些必须由 RobotStudio 完成。

### 页面标题

**实验平台给出可解释候选，最终安全结论仍来自完整单元验证**

### 页面可见文字

实验平台已经完成：

- `选面与 patch 独立规划`
- `加工窗口筛选`
- `UV raster 与 STL ray-lift`
- `孔洞/缺口处 Cell 抬刀`
- `可选杆系姿态抽样粗筛`
- `候选与 Optimal 结果追溯`
- `独立 RobotStudio 工作站准备`

RobotStudio/真实单元仍需验证：

- `完整 MoveL / MoveJ 连续运动`
- `真实 ABB 外壳和工具实体`
- `自碰撞、底座、地轨、转台和环境`
- `可达性、构型连续、轴限位与奇异点`
- `孔边和工件间隙`
- `速度、区间、载荷和加工安全`

最终结论：

> **生成工作站不等于验证通过；内部 validated 也不等于机器人安全认证。**

### 布局

- 左右两列，不做卡片墙。
- 中间用一条竖直分界线。
- 左列标题 `实验平台：生成与粗筛`。
- 右列标题 `RobotStudio：完整验证`。
- 底部用一条从左到右的粗箭头收束到最终结论。

### 讲解要点

- 快速预览只显示 processing geometry，不执行避障 IK/FK，也不完整显示 RAPID 运动。
- 内部碰撞最多抽样 7 点，墙体网格也可能降采样。
- 未包含工具实体和连续 swept volume。
- 正式验收应结合 `VALIDATION.md` 保存输入、summary、CSV、RAPID 和 RobotStudio 截图。

### 内容来源

- `VALIDATION.md`
- `TROUBLESHOOTING.md`
- `HOLE_AWARE_PLANNER.md`
- `docs/AVOIDANCE_FILE_MAP.md`
- `ROBOTSTUDIO_EXPORT.md`

## 4. 制作与验收要求

完整 PPT 制作时应满足：

1. 所有流程图节点、文字、判断菱形、模块框和连接线均为可编辑 PowerPoint 对象。
2. 总流程图只保留阶段级节点，详细判断全部放到后续展开页。
3. 同一术语保持一致：`region/patch`、`run/cell`、`model/world/wobj`、`candidate/optimal`。
4. 不把旧 A*/connector 原型画进当前调用链。
5. 不把 hole-aware 描述成机器人杆系避障。
6. 不把内部 `validated` 描述成 RobotStudio 或真实设备安全认证。
7. 每页标题只表达一个主要结论。
8. 每页 speaker notes 加 `[Sources]`，列出对应项目文档和代码文件。
9. 完成后逐页渲染，检查连接线是否穿过节点、中文是否换行、对象是否重叠或超出画布。
