# 墙体筛选实现简要学习

## 目标

只让加工面附近、确实可能影响机械臂姿态的模型表面进入墙体障碍网格。

预览颜色约定：

- 黄色：实际打磨面；
- 绿色：完整支撑面；
- 红色：局部范围内的墙体；
- 半透明灰色：UVN 筛选范围。

## 实现流程

### 1. 确定支撑面

支撑面必须从墙体集合排除。

- 未分区 region（如 `1、2、3`）：全部加工 `face_ids` 直接作为支撑面。
- 分区 patch（如 `1-1`）：从路径命中的 cell 向外恢复支撑面，再强制合并源加工面。

因此必须满足：

```text
加工面 cells ∩ 墙体 cells = 空集
```

### 2. 建立局部 UVN 范围

将完整支撑面的顶点投影到局部 UV 平面，求一个二维凸包。凸包会主动覆盖孔洞、
边缘缺口、窄连接和凹入区域。随后按输入比例扩大 U/V，并沿 N+、N- 拉伸成封闭棱柱。

凸包边界来自 UV 二维点，不使用 STL 三角边作为范围边界。

### 3. 筛选墙体

先用 UVN 包围范围快速筛出候选 cell，再判断它们是否位于或穿过封闭凸包棱柱。
最后删除全部支撑面 cell，剩余部分才是墙体：

```text
墙体 = 凸包棱柱内的模型 cells - 支撑面 cells
```

正式 runner 只把这些红色 cell 转换成碰撞三角网格。

## 关键经验

以前所有区域都从稀疏路径点生长支撑面，部分黄色加工 cell 没有被生长到，于是又被
当成红色墙体。正确做法不是只遮住红色预览，而是让设置窗口和正式 runner 共用
`support_for_planning_region()`，从计算集合上保证加工面与墙体互斥。

## 主要代码

- `scripts/window_conf_export.py`：统一解析普通 region 与 patch 的支撑面。
- `experimental_algorithms/support_surface_growth.py`：支撑面、UV 凸包和墙体 cell 筛选。
- `ui/avoidance_settings_dialog.py`：参数设置和预览。
- `ui/region_viewer.py`：黄色、绿色、红色及灰色范围显示。
- `scripts/optimal_y_score_configurable.py`：正式避障 runner 使用筛选后的墙体网格。

参数应用后保存到输入项目旁的 `*_avoidance.json`，不会修改 `.rsp.json` 或分区
manifest。
