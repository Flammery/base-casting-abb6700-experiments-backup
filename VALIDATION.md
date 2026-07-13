# Validation Checklist

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
- hole-aware 中央孔 cell 分解、禁止横穿孔洞和全局首尾安全点；
- 输出目录日期和长度；
- 项目输入优先级。

## 固定人工验收流程

每次修改手动分区、预览或路径算法后，用同一个基准 `.rsp.json` 完成：

1. 选择包含圆孔的单个加工面。
2. 旋转二维视图，确认关闭绘制工具后左键可旋转、滚轮可缩放。
3. 拖矩形，确认屏幕上始终横平竖直。
4. 旋转 90° 后连续点击多边形并闭合，确认无“点击没反应”。
5. 创建两个形状方向不同的 patch。
6. 应用后检查模型仍完整，texture 边界无三角碎片，孔洞透明。
7. 点击快速预览，检查两个 patch 独立生成扫描方向。
8. 对 legacy 快速预览，检查孔洞内没有蓝色路径，孔洞两侧不被一条直线 MoveL 横跨。
9. 使用同一参数分别点击“开始”（auto）和“开始-1”（强制 hole-aware），确认输出目录
   分别为普通目录和 `_hole_aware` 目录，互不覆盖。
10. 检查 hole-aware 点 CSV：先完整覆盖一个 cell/孔侧，再通过有效表面 connector
    进入另一侧；任何相邻点连线均不得穿孔或越出 clip polygon。
11. 检查 hole-aware RAPID：每个 patch 只有第一个安全目标使用 MoveJ，中间 processing
    和 connector 全部使用 MoveL，最后 MoveL 到终点安全位置。
12. 手算首尾安全位置：对应端点 world/base 坐标 `x-100、y不变、z+100`；确认 RAPID
    中写入的是转换后的 wobj 坐标。
13. 运行一次正式导出并记录 waypoint/cell/connector/patch 数及 deferred 原因。
14. 导入 RobotStudio，低速或单步检查可达性、构型连续、孔边间隙和实际碰撞。

注意：“快速预览路径”当前仍是 legacy，不能用它代替 hole-aware 的 CSV/RAPID 和
RobotStudio 验收。

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

正式使用 auto 或“开始-1”输出前还必须确认：

1. `summary.json` 的 `planner` 为 `hole-aware`；
2. 目标 patch 具有 `raster_chart`、`clip_polygon`，孔洞正确写入
   `exclude_polygons`；
3. `deferred_paths.csv` 中没有被误忽略的 connector/ray-lift 失败；
4. connector 与孔边的实际距离满足刀具半径和工艺安全包络，不能只看 TCP 点不穿孔；
5. base/world 安全点在当前工件位姿下无碰撞且可达；
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
