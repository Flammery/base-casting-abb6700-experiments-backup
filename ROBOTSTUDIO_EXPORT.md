# RobotStudio 验证工作站导出

本流程把 Optimal-Y 结果整理成 RobotStudio 6.08 人工验证场景。它只准备模型和程序，不判断姿态、碰撞、可达性或路径质量。

## 使用前提

- 先完成一次 Optimal-Y 正式运行；所选目录必须包含非空的 `optimal_records.json`，或兼容的
  `optimal_selection.csv`，并且每条最优记录都能找到对应的 `optimal_paths/<region_label>/*.txt`；
- 选择的是一次实验的结果目录（包含 `summary.json`、`optimal_paths` 的那一级），不是
  `results` 总目录，也不是单独某个面目录；
- `configs/robotstudio_export.json` 中的 RobotStudio 6.08 路径、模板工作站、任务名和场景
  工件组件名必须与本机一致；
- 模板工作站必须已经包含可启动的虚拟控制器、机器人、工具和工件模型。模板本身不会被覆盖；
- 首次使用或插件更新后，先关闭 RobotStudio，运行 `scripts/install_robotstudio_addin.ps1`，
  再重新启动 RobotStudio。

UI 中点击“导入 RobotStudio 验证”后，应选择例如
`results/x3600_..._hole_aware` 这一层。若最优记录为空，打包器会直接报错，不会生成空工作站。

## 两套互不替代的数据

场景模型安装位置：

- 用 `workpiece_component_name` 精确查找 RobotStudio 场景组件；
- 用最优记录的 `model_x/model_y/model_z/angle_deg` 写入该组件的 X/Y/Z/RZ；
- 组件名称不是 RAPID wobj 名称，模型安装位置也不参与重新计算 wobj。

RAPID tool/wobj：

- tooldata、wobjdata、名称、TCP 和坐标值来自该次路径导出；
- 两条声明原样移入 `CalibData.mod`；
- 路径模块删除重复声明，但 Move 指令继续引用原来的 tool/wobj 名称；
- 不生成 `Workobject_1`，也不把场景组件名改成 wobj 名。

## 每个面的文件

```text
optimal_paths/
  1_1/
    1_1.txt
    CalibData.mod
    VALIDATE_1_1.mod
    3600_m800_440_rz0_1-1.rsstn
    3600_m800_440_rz0_1-1.robotstudio_job.json
  2/
    2.txt
    CalibData.mod
    VALIDATE_2.mod
    3600_m600_440_rz180_2.rsstn
    3600_m600_440_rz180_2.robotstudio_job.json
robotstudio_jobs.json
robotstudio_status.json
```

每个 region/patch 生成一份 `.rsstn`，不把多个面合并到一个路径模块。

## 区域和命名

- 未分区标签保持 `2`、`3`、`4`；
- 分区标签在数据目录中保持 `1_1`、`1_2`，在 `.rsstn` 文件名中显示为 `1-1`、`1-2`；
- 负数用 `m`，小数点用 `p`；
- 姿态必须显式写出，例如 `rz0`、`rz180`；
- 格式为 `{x}_{y}_{z}_rz{angle}_{region}.rsstn`。

## RobotStudio 运行方式

- 打包器从模板复制 `.rsstn`，只修改指定场景组件的安装矩阵，不覆盖模板；
- 生成的工作站仍使用模板工作站中已经实验标定的虚拟控制器；
- 打开任一生成工作站后，插件等待控制器就绪，再读取同名 `.robotstudio_job.json`，自动把该面的 `CalibData.mod` 和路径模块装入 `T_ROB1`；
- 从一个面切换到另一个面时，插件先用临时 `RSBRIDGE` 例程移动程序指针，再删除旧路径、加载新路径并把指针设到新 `main`；`RSBRIDGE` 随后删除；
- 因为这些工作站顺序复用同一实验虚拟控制器，一次只打开并验证一个生成工作站。复制 `.rsstn` 时必须同时保留同目录 `.mod` 和 `.robotstudio_job.json`。

## 版本和边界

目标版本为 RobotStudio 6.08.01（程序集 `6.8.8307.1040`）。插件只引用该安装目录内的 RobotStudio/PC SDK 程序集，不使用 PC SDK 2025 或 RobotStudio 2026 程序集。

“工作站和 RAPID 已准备”不代表碰撞、可达性、姿态或加工安全验证通过，最终判断仍由人工在 RobotStudio 中完成。

当前限制：

- 只适配 RobotStudio 6.08.01 和配置中指定的模板结构；未验证 RobotStudio 2025/2026；
- 插件目标框架为 .NET Framework 4.6.1；建议安装对应 Developer Pack/Targeting Pack。
  缺少时旧版 MSBuild 可能从 GAC 解析程序集并给出 `MSB3644`，即使本机编译成功也不代表
  构建环境完全可复现；
- 只修改模板 `PIM.xml` 中目标组件的平移和绕 Z 轴旋转，不创建机器人、工具、几何体或控制器；
- 只支持每个路径模块恰好一条 `tooldata` 和一条 `wobjdata` 声明；数据原样移入
  `CalibData.mod`，不重新标定、不生成 `Workobject_1`；
- 插件会替换 `T_ROB1` 中已有的 `CalibData`，并删除带 `main` 的旧路径模块；因此只应在
  专用验证控制器上使用，不能指向保存有其他生产程序的控制器；
- 多个生成工作站复用模板控制器状态，必须逐个打开、逐个验证，不能并行打开；
- 自动切换依赖 `.rsstn`、同名 `.robotstudio_job.json`、`CalibData.mod` 和路径 `.mod`
  保持在同一面目录中；移动或复制时必须一起处理；
- 插件只在工作站打开且虚拟控制器就绪后加载 RAPID；若 RobotStudio 已在运行，安装或更新
  插件后必须重启才会生效；
- 未执行碰撞检测、逐点 IK、构型连续性、奇异点、轴限位、工具扫掠体或安全速度验证。
