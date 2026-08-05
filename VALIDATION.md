# Validation Checklist

## 2026-07-27 UVN 避障范围

1. 主界面只显示“避障设置”按钮和已配置摘要，不再显示自由文本避障区域输入框。
2. 混合输入源 region 与 patch（例如 `1-1，2，3-2`），确认中英文逗号、顿号和分号均可分隔，且源 region 的每个最终 patch 独立生成设置页。
3. 默认 U/V 为 30%，确认最终宽度为原支撑面宽度的 130%；分别修改 U、V 后只有对应方向改变。
4. 分别修改 N+、N-，确认灰色范围只沿局部 N 的对应方向改变。
5. 黄色为精确打磨区域、绿色为恢复支撑面、红色仅为范围内非支撑墙体、范围外模型保持灰色。
6. 旋转查看半透明灰色异形拉伸体，确认底面跟随完整支撑面的 UV 投影轮廓，而不是其 UV 包围矩形；凹口外但仍在包围矩形内的墙体不得变红。
7. 确认 U/V 同行、N+/N- 同行；解析后自动显示，不再需要“预览”按钮。
8. “隐藏范围/显示范围”只控制叠加显示且保留参数；“清除选择”只清空临时输入，不删除已保存 sidecar。
9. 弹窗底部显示的保存路径与输入项目同目录、同主文件名；点击“取消”不写文件，点击“应用”写该 `*_avoidance.json`，不修改 `.rsp.json` 和分区 manifest。
10. 正式运行的 `summary.json` 记录 `avoidance_settings_path` 和每个区域的 `avoidance_volume` 范围/cell 数。
11. 不按朝上、朝下或法向正反过滤任何墙体；本轮不改变姿态库、IK/FK或碰撞判定。

## 2026-07-22 避障、坐标与结果目录回归项

1. 分别输入转台角度 `23`、`271`、`300`，确认输出 snapshot、RAPID元数据、
   模型 RZ 和显示的 wobj RZ 都使用实际角度，而不是固定角度表。
2. 对非零 `picked_origin` 手算旋转平移后的 wobj XYZ，确认导入路径后模型和
   W 坐标系同步，wobj RX/RY仍保持输入项目值。
3. 构造 J6 从正小角度跨到负小角度的连续解，确认配置元组可以变化、实际
   关节跳变很小并得到 `validated-clear`。
4. 构造超过 40°的实际关节跳变，确认得到 `joint-discontinuous`；不要用
   confdata 元组变化替代此检查。
5. 避障区域确认从 `-90°` 到 `+90°`、间隔 `15°` 的 13 个 local-Z roll 都写入精简的
   `robot_avoidance_trials.csv`，且包含 selected/status/interference/clearance/
   joint-jump/reason。
6. 同一安装位置有多个 validated roll 时，确认选择抽样最小净间隙最大的 roll；不同
   安装位置之间也选择抽样最小净间隙最大的候选，不按 roll 或 world-Y/world-X 排序。
7. `ik-unresolved` 或支撑面恢复失败但几何非空的路径应保留在候选记录，且
   不得进入 `optimal_paths`；普通区域先按 world-Y、并列时再按 world-X 进入最优选择。
8. 检查抽样 IK/FK、连杆碰撞和最小间隙结果后，仍必须在 RobotStudio验证
   完整路径、工具、环境、自碰撞和 ABB控制器构型。
9. UI实验成功后应自动打开本次结果目录；失败进程不打开，Explorer 启动
   失败只写状态、不改变实验完成结果。

## 自动测试

```powershell
C:\Users\21093\Desktop\p1\.venv\Scripts\python.exe -m pytest tests -q
```

关键覆盖：

- manifest v1/v2 不混读；
- 每个 patch 使用自身长短边；
- raster-domain 射线得到正确 XYZ；
- waypoint normal 来自命中三角面；
- 孔洞内无采样点；
- 孔洞或射线缺口产生独立 segment；
- hole-aware 中央孔 cell 分解、稳定 cell 顺序和每个 cell 的进退刀点；
- 输出目录日期和长度；
- 项目输入优先级。
- region/patch selector 规范化、自动/手动 patch 标签恢复和 local-Z roll 轴保持；
- 导入 `.rsc.json` 时 summary 必须记录 `configured-link-envelopes` 和配置文件路径；
  未导入时必须记录 `uniform-radius-links`、统一半径 100 mm 和
  `use_configured_segment_radius=false`。

## 固定人工验收流程

每次修改手动分区、预览或路径算法后，用同一个基准 `.rsp.json` 完成：

1. 选择包含圆孔的单个加工面。
2. 旋转二维视图，确认关闭绘制工具后左键可旋转、滚轮可缩放。
3. 拖矩形，确认屏幕上始终横平竖直。
4. 旋转 90° 后连续点击多边形并闭合，确认无“点击没反应”。
5. 创建两个形状方向不同的 patch。
6. 应用后检查模型仍完整，texture 边界无三角碎片，孔洞透明；当只划分多个源 region
   中的一个时，确认该源 region 被 patch 标签替换，而所有未划分 region 仍保持高亮；
   连续旋转三维预览，确认 texture 不随观察角度闪烁或消失。
7. 点击快速预览，检查两个 patch 独立生成扫描方向。
8. 检查 auto 快速预览：孔洞内没有蓝色路径，状态栏显示正确的 cell 抬刀数量和判定原因。
9. 点击唯一“开始”运行 auto；如需对照，再从 CLI 强制 `--planner hole-aware`，确认输出
   目录具有不同的当日递增编号且互不覆盖，并在各自 `summary.json.planner` 中记录策略。
10. 检查 hole-aware 点 CSV：每个 cell 的光栅完整连续，cell 边界有独立安全进退刀点。
11. 检查 hole-aware RAPID：加工与法向进退刀使用 MoveL，两个 cell 的抬起点之间使用 MoveJ。
12. 手算法向安全位置：`safe = endpoint + normal × SAFE_DISTANCE`，并确认转换后的 wobj 坐标。
13. 运行一次正式导出并记录 waypoint/cell/transfer/patch 数、planner reason 和 deferred 原因。
14. 导入 RobotStudio，低速或单步检查可达性、构型连续、孔边间隙和实际碰撞。
15. 分别输入 `1` 与 `1-1`，确认前者命中源 region 的全部 patches，后者只命中一个
    patch；未输入的区域候选仍为 `avoidance_status=not-requested`。
16. 检查 `robot_avoidance_trials.csv` 13 个 roll 均有记录，`summary.json` 的 requested、
    resolved labels、status counts 一致；`fallback-unverified` 不得显示为已安全。
17. 快速预览确认黄色 seed 位于实际 patch，绿色支撑面覆盖 patch 所在完整面且没有越过
    圆角进入墙体，红色包含需要绕开的腔体墙；核对 status 中的支撑面 cell 数。
18. 检查 `summary.json.support_surface_growth.regions` 的 seed/support/obstacle 数、参考法向、
    最大法向角和最大平面误差；`support-surface-failed` 不得进入 candidate/optimal。
19. 对选中的替代姿态在 RobotStudio 检查完整路径、工具、环境、自碰撞和控制器构型。

注意：快速预览与正式运行共用自动判定，但它只显示 processing raster，不显示完整的
RAPID 抬刀/MoveJ 轨迹，因此不能代替 CSV/RAPID 和 RobotStudio 验收。

## 坐标与 RAPID 验收

至少验证 RZ = 0/90/180/270 四个姿态：

1. tool 名称、wobj 名称与输入项目一致；
2. 根据 `picked_origin` 手算 wobj XYZ，与输出快照及 RAPID `wobjdata` 一致；
3. model -> world -> wobj -> world 往返误差在数值容差内；
4. base 加工窗口判断使用 world/base 坐标；
5. robtarget 位置使用 `position_wobj`；
6. world quaternion 转 wobj quaternion 后，RobotStudio 中工具方向与快速预览一致；
7. 修改实验 X/Y/Z/RZ 后模型与路径同步移动，没有整体偏移或重复旋转；
8. tool TCP/flange 几何未被实验参数改写；
9. 若使用占位 load，summary/验收记录明确标记“未标定”。

## Hole-aware 限制验收

正式使用 auto 或 CLI 强制 hole-aware 输出前还必须确认：

1. `summary.json` 的 `planner` 为 `hole-aware`；
2. manual-v2 patch 的 `raster_chart`、`clip_polygon` 和 `exclude_polygons` 正确；同时用
   一个没有 manifest 的原始 face-id split-scanline region 验证 projected cell 分支；
3. `auto_planner_reason_counts` 与预期的内部孔、边界重叠和 split-scanline 一致；
4. processing raster 没有进入孔或无表面区域；
5. 每个 cell 的法向安全点与 cell 间 MoveJ 在当前工件位姿下无碰撞且可达；
6. RobotStudio 中检查机器人本体、法兰、工具、工件和外围设备碰撞；
7. 不得把二维 `valid` 或 pytest 通过解释为机器人级安全认证。

auto 运行时第 1 项改为确认 `planner=auto`，并核对
`auto_hole_aware_path_count/auto_raster_path_count` 与预期分流一致。

## 建议保存的基准证据

- 分区窗口截图；
- texture 分区截图；
- 快速路径截图；
- RobotStudio 路径截图；
- `summary.json`；
- patch 数、孔洞数、waypoint 数、segment 数；
- 使用的 commit hash 和输入文件名。

视觉结果未完成上述检查时，不能仅凭 pytest 通过认定 UI/几何修改完成。

## RobotStudio 工作站导出验收

1. 场景工件组件名称由 RobotStudio 配置指定，不得使用 wobj 名称代替。
2. 场景工件只应用最优记录的 `model_x/model_y/model_z/model_rz`；RAPID wobj 保持路径
   导出的名称和数值。
3. `tooldata`、`wobjdata` 只存在于该工作站的 `CalibData`，路径模块仍引用原名称。
4. 同时测试未分区标签 `2` 与分区标签 `1_1`。
5. 文件名显式包含 `rz0` 或其它实际角度，并位于
   `optimal_paths/<region_label>/`。
6. 顺序打开两个不同区域的 `.rsstn`，确认场景安装位置随文件改变，控制器路径模块由
   `VALIDATE_<前一区域>` 自动切换为 `VALIDATE_<当前区域>`，且 `CalibData` 保留导出名称和数据。
7. 模板工作站的哈希和最后修改时间不得改变。
8. 插件不得把“成功保存工作站”报告为碰撞、可达性或姿态验证通过。
9. 切换完成后控制器中不得残留 `RSBRIDGE`，程序指针必须位于当前路径模块的 `main`。
10. 用包含中文 CAD 文件名的实验生成 RAPID，确认 `RSP_EXPERIMENT_META_V1` 整行只含
    ASCII，且 `json.loads` 后恢复原始中文路径。
11. 用未修复的历史 `.txt` 重新打包，确认新的 `VALIDATE_Rx.mod` 已把中文转换成
    `\uXXXX`，且 robtarget/Move 指令数量和坐标不变。
12. 构造 `CalibData + RSBRIDGE +` 语法损坏 `VALIDATE_R5` 的失败状态，再打开区域 2
    工作站；确认插件自动恢复为 `CalibData + VALIDATE_R2`，没有残留桥接或旧路径模块。
13. 在真实 RobotStudio 6.08.01 中运行 Check Program，确认错误数为 0；同时核对安装目录
    DLL 与 `robotstudio_addin/bin/Release` DLL 的 SHA-256 一致。
