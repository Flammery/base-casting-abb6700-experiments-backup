# Base Casting ABB 6700 Principles

This file records the design principles and algorithm rules for the base-casting
ABB 6700 polishing experiment. Runtime instructions belong in `README.md`.
Agent handoff notes and script ownership belong in `AGENTS.md`.

## Architecture Principles

`src/` is the reusable software library. This experiment directory owns
project-specific assumptions, preprocessing, scan grids, result naming, and batch
analysis.

The experiment flow is:

```text
inputs/latest_script_test.rsp.json
  -> optional selected-region partition preprocessing
  -> inputs/latest_partitioned.rsp.json
  -> window/conf RAPID export
  -> optional optimal Y-position selection
```

The application layer owns:

- project file schema;
- selected-face persistence;
- mesh reading and legacy mesh-raster sampling primitives;
- shared geometry and transform math;
- generic RAPID formatting utilities.

The experiment layer owns:

- selected-region partition preprocessing;
- scan grids and naming;
- machining-window parameters;
- batch summaries;
- RAPID grouping;
- phase-specific assumptions.
- manual raster-domain scanlines, hole subtraction, and ray projection.

## Manual UV Partitioning

Manual partitioning is an experiment-layer clipping operation. It must not
rewrite the project schema or replace the selected face-id regions.

The UI supports three explicit modes:

- `boundary`: a drawn barrier is interpreted against the local raster boundary
  and produces clipped patches on either side;
- `slab`: a drawn barrier creates a through-cut across the local raster domain;
- `pick`: rectangles and free-form polygons define the exact areas to retain.

All three modes operate in the region's local raster UV chart. A picked polygon
is a clip polygon, not a new mesh region: the exporter samples only points inside
the polygon and outside any generated hole polygons. Multiple picked polygons
produce independent patches labelled from the source region, such as `1_1` and
`1_2`.

For manual v2, the polygon—not the STL triangulation—owns the machining
boundary. Scanlines are created directly in 2D, holes are subtracted as
intervals, and accepted samples are ray-projected to selected STL cells. The hit
triangle supplies XYZ and facet normal only. Each patch derives its own scan
axes from its own polygon.

The v2 manual manifest is the source of truth for these clips. It records the
mode and per-region input geometry, while each patch records `clip_space`,
`raster_chart`, `clip_polygon`, `exclude_polygons`, and source face ids. An
unchanged or unselected region must remain available to the normal exporter.

Manual selection is intentionally separate from automatic curvature and
bottleneck partitioning: use automatic partitioning for geometry-derived
boundaries, and manual UV clipping when the operator has a known machining
area. Do not introduce global XYZ split lines or mutate `src/` to support this
experiment.

Manual partition visualization uses a PySide-painted RGBA raster mask as a VTK
texture on the unchanged STL. Cell-centroid colors and replacement/clipped mesh
geometry are prohibited because they reintroduce triangle-shaped boundaries or
unstable topology.

## Partition Preprocessing Algorithm

The partition preprocessor is stage 2 of the workflow. It reads selected regions
from `latest_script_test.rsp.json`, partitions only user-specified 1-based region
numbers, and writes `latest_partitioned.rsp.json`. The `.rsp.json` schema is not
changed. Human labels such as `6.1` are written only to
`latest_partitioned_manifest.json`.

Algorithm rules:

- Build a face adjacency graph from mesh triangles. Two faces are adjacent when
  they share a quantized mesh edge.
- Preserve unspecified selected regions exactly as they came from the input.
- For specified regions, first split by local curvature/near-planarity:
  - use face normals, face area, and centroids;
  - hard-edge adjacency is limited by `hard_edge_angle_deg`;
  - local planar region growing accepts a neighbor only if it remains within
    `planar_normal_deg` and `planar_rms_mm`;
  - small planar seeds are rejected or merged so they do not create noisy output.
- Then apply bottleneck splitting only inside planar patches:
  - derive local boundary UV axes from the patch boundary;
  - build scanline intervals;
  - connect intervals on adjacent scanlines only when they overlap enough;
  - mark neck lines when scanline width drops below the local median by
    `neck_width_ratio` for at least `min_neck_lines`;
  - cut only interval graph connectivity across the neck, never a global XYZ or
    UV straight line through the whole selected face.
- Merge output patches smaller than `min_patch_faces` or
  `min_patch_area_ratio` into adjacent larger patches.

This avoids the known failure mode where a global split line cuts one continuous
large area into two pieces even though no real narrow channel separates them.

Current default intent:

- produce meaningful polishing regions rather than many tiny geometry fragments;
- keep two broad planar areas split when a bottleneck or curvature transition
  justifies it;
- keep curved/transition material as a separate region when it is large enough.

## Window/Conf Export Strategy

The phase-1 RAPID export strategy is "limited machining window + fixed
configuration", not global IK for every waypoint.

### Placement and coordinate inheritance

Tool and workobject names, tool TCP/flange geometry, picked origin, and the base
placement come from the software-exported project. Experiment X/Y/Z/RZ are
placement overrides, not replacement coordinate definitions. Updating a scan
pose must update model placement and wobj together using the rotated picked
origin defined in `COORDINATE_SYSTEMS.md`.

`position_model` is used for surface/ray geometry, `position_world` for base
window and confdata decisions, and `position_wobj` for ABB robtargets. Do not
write world positions directly as wobj targets, rotate picked origin twice, or
modify tool TCP to compensate an unwanted path quaternion. A fallback load used
when mass is missing is compatibility data, not tool calibration.

Current base machining window:

```text
base x in [1500, 2500] mm
base y in [-1050, 1050] mm
```

Only export a region for an angle when the whole selected or preprocessed region
falls inside this window after applying the tested workpiece pose.

Region path rules:

- Plan each selected or preprocessed region independently.
- Do not merge all selected faces into one long path.
- Generate raster paths from the region boundary UV axes.
- `long_side` and `short_side` are both valid variants; broad scans may run only
  `long_side`.
- PCA is only a fallback when boundary edges are unavailable.
- Do not use global XYZ scan axes or current path tangent as a substitute for
  region UV.

### Experimental hole-aware motion

The default UI `Start` and configurable runner use `auto`: patches with relevant
hole polygons (or multiple runs on one sampled scanline) use hole-aware order,
while ordinary patches use the normal raster sampler. Forced `hole-aware` and
`legacy` remain CLI-only troubleshooting modes:

- subtract holes before ordering motion;
- decompose scanline runs into boustrophedon cells;
- finish one cell/hole side before visiting another;
- preserve deterministic scan discovery order instead of greedy cell reordering;
- retract along the endpoint normal and transfer above the surface between cells.

Each cell has one approach and one departure point at `SAFE_DISTANCE` along the
local surface normal. Processing, approach, and departure use MoveL; transfer
between lifted endpoints uses MoveJ.

This planner does not establish collision freedom, robot reachability, IK
continuity, tool-envelope clearance, or global shortest motion. It accepts a
complete manual-v2 chart/clip domain or a raw projected face-id raster; explicit
exclude polygons still require their chart/clip. Processing motion must never
cross a hole or silently fall back to legacy motion. See
`HOLE_AWARE_PLANNER.md` for the complete contract.

TCP orientation:

```text
TCP +Z = -surface normal
TCP +Y = project(base +Y onto tangent plane)
TCP +X = TCP +Y cross TCP +Z
```

RAPID configuration:

```text
base y < 0   -> [-1,-1,0,1]
base y >= 0  -> [0,0,-1,1]
```

Generated modules must include:

```rapid
ConfL \Off;
```

Tooldata should reflect the real flange-to-TCP geometry. Do not change tool TCP
to compensate for an unwanted path orientation.

## Selected Robot-Arm Avoidance Trials

Robot-arm avoidance is opt-in by source region or patch label. It first creates
the same auto-planned contact path as an ordinary region, then tests a small
constant local TCP-Z roll library. Local Z roll preserves the contact point and
tool axis, so it does not weaken `TCP +Z = -surface normal`.

Only representative waypoints are screened with the project robot configuration,
initial joint state, numerical IK, joint continuity, J5 margin, FK collision, and
sampled clearance against the placed workpiece. The previous successful joint
solution is the next IK seed. A J1/J4/J6 confdata quadrant change is recorded but
does not reject a solution; the actual joint-angle jump is the continuity rule.
Unselected planning regions must not enter this loop. A geometrically valid path
whose IK remains unresolved is retained for RobotStudio diagnosis but cannot enter
the internally validated optimal output. No candidate result is a RobotStudio or
real-cell safety certificate; tool, environment, self-collision, complete robot
geometry, and swept motion remain outside this stage. Full rules are in
`docs/ROBOT_ARM_AVOIDANCE_WORKFLOW.md`.

### UVN obstacle volumes

Avoidance wall selection is configured separately from the project schema.
The experiment UI resolves source-region/patch selectors and recovers the
complete support. An unsplit source region uses all selected machining
`face_ids` directly as support. A derived patch grows beyond its path-hit
seeds, but its source machining cells are always unioned into the support
result. Consequently, a machining cell can never also be a wall candidate.
Every support vertex is projected into UV, and one
two-dimensional convex hull owns the avoidance boundary; STL triangle edges do
not. The hull deliberately encloses holes, edge defects, narrow connections,
small fragments, and concave bays, then extrudes in N for each final planning
label. Curved support is projected before the hull is built. U and V
independently scale the hull about the support UV centre; a value of 30 means a
final span of 130 percent. N+ and N- are independent millimetre heights.

Every non-support mesh cell inside or crossing the closed footprint prism is a
wall candidate. The UV bounding rectangle is only diagnostic and cannot select
cells outside the projected support shape. Cells outside the volume are
excluded from this experiment's wall mesh. Do not classify or discard cells by
upward/downward normal. Manual-v2 machining boundaries and avoidance-preview
boundaries remain raster textures; whole-cell colors do not replace their
authoritative UV masks.

Settings are stored in a versioned `*_avoidance.json` sidecar. The `.rsp.json`
schema remains unchanged. Existing TCP-roll, IK/FK, clearance, and optimal
selection behavior is intentionally unchanged by this wall-selection stage.

## Y-Position Selection

For dual-robot rail scans, first export feasible candidate paths with the
fixed-window strategy. Then choose one candidate per region using:

```text
score = max(abs(world_y))
```

Use only processing waypoints for the score. Approach/depart points are motion
padding and must not affect placement selection. Tie breakers are only for
deterministic output, not extra optimization metrics.

Human-facing CSV tables for these runs should stay minimal: position plus
covered region(s). Detailed file paths, point counts, and diagnostics belong in
result folders and `summary.json`.

## 中文原则

本文件记录算法和原则，不写具体运行教程。运行命令看 `README.md`，脚本职责看
`AGENTS.md`。

### 架构原则

`src/` 是可复用的软件库；`experiments/base_casting_abb6700/` 是底座项目的
实验层。实验层可以调用 `src/`，但不要为了临时实验修改软件项目格式或 UI。

整体流程：

```text
inputs/latest_script_test.rsp.json
  -> 可选：指定 selected region 分区预处理
  -> inputs/latest_partitioned.rsp.json
  -> window/conf RAPID 导出
  -> 可选：optimal-y 选择导轨 Y 位置
```

### 分区算法原则

分区预处理只处理用户指定的 region，例如 `--regions 6`。其它 region 原样保留。
`.rsp.json` 里仍然只是 `selected_path_face_regions`，子区标签如 `6.1`、`6.2`
只写入 `latest_partitioned_manifest.json`。

算法顺序：

1. 从 mesh 三角面建立 face 邻接图。
2. 对指定 region 做局部曲率/近似平面分区。
3. 只在 planar patch 内做窄通道检测。
4. 基于 scanline interval 连通关系断开瓶颈。
5. 小碎片并回相邻大 patch。

关键约束：

- 不能用一条全局 XYZ/UV 直线贯穿整个面来切分。
- 窄通道只是断开局部 interval 连通关系。
- 如果两块区域没有经过真正的瓶颈分离，即使被某条全局线穿过，也不能被误切。
- 输出应该是可实际抛光的少量区域，而不是大量几何碎片。

### 导出原则

每个 region 独立规划、独立导出 RAPID。导出前必须用 base 加工窗口筛选。
姿态使用 `base_y_aligned`，confdata 按 base Y 正负固定，并在 RAPID 中写
`ConfL \Off;`。

### Y 位置选择原则

Y 位置选择不做逐点 IK 搜索。先生成候选路径，再按 processing waypoint 的
`max(abs(world_y))` 为每个 region 选一个候选。
