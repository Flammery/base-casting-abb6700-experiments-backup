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
8. 检查孔洞内没有蓝色路径，孔洞两侧没有连续 MoveL。
9. 运行一次正式导出并记录 waypoint/segment/patch 数。
10. 导入 RobotStudio，对比快速预览与实际路径。

## 建议保存的基准证据

- 分区窗口截图；
- texture 分区截图；
- 快速路径截图；
- RobotStudio 路径截图；
- `summary.json`；
- patch 数、孔洞数、waypoint 数、segment 数；
- 使用的 commit hash 和输入文件名。

视觉结果未完成上述检查时，不能仅凭 pytest 通过认定 UI/几何修改完成。
