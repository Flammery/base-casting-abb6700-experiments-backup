# Tool, Workobject, and Placement Coordinates

## 导入实验路径时的场景同步

实验 RAPID 中的 wobj 与嵌入/结果元数据计算一致并通过校验后，主程序必须
同时更新当前场景的 `model_transform` 和 `workobject_transform`：

```text
model XYZ/RX/RY/RZ = verified experiment installation
wobj XYZ           = model XYZ + rotate_RZ(picked_origin)
wobj RX/RY          = inherited project calibration
wobj RZ             = verified experiment RZ
```

该同步只用于让当前软件场景中的工件模型、W 坐标系和已解析 RAPID 路径描述
同一个物理安装位姿。RAPID robtarget 仍以程序内已解析的 wobj 为准；未通过
wobj 校验时不得自动移动模型或显示的 W 坐标系。

本文固化软件项目与实验脚本之间的工具坐标、工件坐标和安装位姿同步规则。修改位置扫描、转台角度、RAPID 导出或预览变换前必须阅读。

## 数据来源

软件导出的 `.rsp.json` 是以下信息的来源：

- polishing tool 名称；
- flange-to-TCP 几何和工具 TCP；
- 工件坐标名称；
- 原始 `WorkpiecePlacement`；
- `picked_origin`；
- `wobj_rx/wobj_ry` 等已有姿态字段。

实验脚本不得重新发明这些名称或通过修改 TCP 补偿路径姿态。

## 实验覆盖的字段

实验设置 X/Y/Z/RZ 时，`scripts/window_conf_export.py::placement_for()` 先克隆软件导出的 placement，再覆盖：

```text
model_x = experiment X
model_y = experiment Y
model_z = experiment Z
model_rz = experiment RZ
wobj_rz  = experiment RZ
```

`wobj_rx/wobj_ry`、名称和其它未明确覆盖字段继续继承软件导出值。

## picked_origin 与 wobj 原点

`picked_origin = (px, py, pz)` 是软件中确定的工件参考点。实验转角变化时，wobj 原点必须随模型一起旋转和平移：

```text
wobj_x = model_x + cos(RZ) * px - sin(RZ) * py
wobj_y = model_y + sin(RZ) * px + cos(RZ) * py
wobj_z = model_z + pz
```

禁止只修改 `model_*` 而不更新 `wobj_*`，也禁止对 picked origin 重复旋转。

## 三套位置坐标

每个 waypoint 保留：

- `position_model`：模型文件自身坐标，用于 raster ray hit、face id 和表面法向；
- `position_world`：应用当前 experiment placement 后的基座/世界坐标，用于加工窗口、base Y 正负和 confdata；
- `position_wobj`：相对同步后工件坐标的坐标，用于 ABB robtarget。

变换方向：

```text
model -> world/base -> wobj
```

窗口判断不能使用 `position_wobj`，RAPID robtarget 不能直接写 `position_world`。

## Hole-aware 首尾安全位置

实验性 hole-aware planner 每个 patch 只创建两个安全位置。偏移必须先在
world/base 坐标中计算：

```text
start_safe_world = first_processing_world + (-100, 0, +100) mm
end_safe_world   = last_processing_world  + (-100, 0, +100) mm
```

随后使用当前同步后的 `WorkpieceTransform` 把安全位置转换为 `position_wobj`。禁止直接
在 wobj 或 model 坐标中套用 `x-100/z+100`。安全点姿态继承对应加工端点姿态。

该偏移只是当前实验约定，不代表已完成碰撞、可达性或安全空间认证；每个安装位姿仍需
在 RobotStudio 中验证。

## 姿态与 RAPID

路径姿态先在 world/base 下按表面法向生成。写 RAPID 时：

```text
q_wobj = quaternion(wobj_rx, wobj_ry, wobj_rz)
q_target_in_wobj = conjugate(q_wobj) * q_target_in_world
```

RAPID 中：

- `wobjdata` 使用同步后的 wobj 名称、原点和姿态；
- robtarget 位置使用 `position_wobj`；
- robtarget 姿态使用 world-to-wobj 转换后的四元数；
- Move 指令继续引用软件导出的 tool 名称和 wobj 名称。

## Tooldata 规则

- 工具名称、TCP 和 flange-to-TCP 几何来自软件项目。
- 不允许通过改变 tool TCP 修正路径朝向；朝向规则属于 waypoint quaternion。
- 工具实测载荷应来自真实标定。

当前存在一个兼容例外：若导出的 `mass_kg <= 0`，实验暂设 `1.0 kg`，并可能把完全未定义的 RAPID load 替换为占位载荷。这只为 RobotStudio 接受程序，不代表真实工具载荷。

## 输出项目快照

实验不会覆盖原始软件输入。批跑会在结果目录保存应用当前 X/Y/Z/RZ 后的 `.rsp.json` 快照，便于追溯该批 RAPID 使用的 placement 和 wobj。

每个新生成的 RAPID 模块还在 `MODULE` 后写入一条 `RSP_EXPERIMENT_META_V1` 注释，保存完整模型安装位姿、region、CAD 路径、picked origin 和继承的 wobj RX/RY。Qt 主程序可在文件离开结果目录后恢复模型安装位置；该注释不替代 RAPID `wobjdata`，导入时仍必须用二者做一致性校验。

## 不可违反的检查项

1. tool/wobj 名称来自输入项目。
2. model placement 与 wobj 同步更新。
3. picked origin 只旋转一次。
4. 世界窗口使用 world/base 坐标。
5. robtarget 使用 wobj 相对坐标。
6. 工具 TCP 不承担路径姿态补偿。
7. 占位载荷必须标注为未标定。
