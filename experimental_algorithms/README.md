# Experimental Algorithms

Reserved for later prototypes that should not affect the application source tree yet.

## Active prototype: hole-aware raster

`hole_aware_raster.py` is intentionally kept here while default `auto` mode
selects it only for relevant-hole patches; CLI can force it for troubleshooting.
It groups scanline runs into cells, visits one hole side at a time,
and routes on-surface connectors. Ordinary auto patches retain the normal raster
fast path. Its limitations are documented in `../HOLE_AWARE_PLANNER.md`.

Candidate topics:

- collision and clearance filtering;
- partition polishing;
- automatic turntable pose selection;
- path sequencing and safe retract/flip moves;
- alternative UV extraction;
- RobotStudio validation report import.

Keep phase-1 production experiments in `../scripts/` until an algorithm is stable enough to replace them.

## 中文说明

这里预留给后续激进算法原型，避免还没验证稳定的内容影响软件主体。

当前 `hole_aware_raster.py` 是活动原型。唯一“开始”按钮的 `auto` 只对相关带孔 patch
调用它，无孔 patch 保留普通 raster 快路径；CLI 可强制所有 patch 使用它。
适用输入、失败策略和安全限制见 `../HOLE_AWARE_PLANNER.md`。

候选方向包括：

- 碰撞和安全间隙筛选；
- 分区抛光；
- 自动选择转台位姿；
- 路径排序，以及安全抬起/翻腕动作插入；
- 替代的 UV 轴提取方法；
- RobotStudio 验证报告导入。

第一阶段稳定实验仍放在 `../scripts/`。只有当某个新算法足够稳定、能够替代当前流程时，再考虑从这里移出去。
