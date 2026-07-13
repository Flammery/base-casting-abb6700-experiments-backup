# Partition Manifest Contract

## 项目文件与 manifest 的职责

`.rsp.json` 继续只保存软件项目字段和 `selected_path_face_regions`。手动画线、矩形、多边形、二维 chart 和 patch 标签不写入项目 schema，而是写入项目旁边的同名 `_manifest.json`。

## Version 1：实体 face-id 分区

v1 已经把输出 regions 写入 `.rsp.json`：

```json
{
  "schema": "base_casting_abb6700.manual_region_partition_manifest",
  "version": 1,
  "records": [{"patches": [{"face_ids": [1, 2, 3]}]}]
}
```

读取规则：直接使用项目中的 `selected_path_face_regions`，不得把 v1 patch 当成 v2 clip polygon。

## Version 2：二维 raster-domain patches

```json
{
  "schema": "base_casting_abb6700.manual_region_partition_manifest",
  "version": 2,
  "partition_mode": "boundary | slab | pick",
  "records": [
    {
      "original_region": 1,
      "clip_space": "raster_uv",
      "raster_chart": {
        "origin": [0, 0, 0],
        "u_axis": [1, 0, 0],
        "v_axis": [0, 1, 0],
        "normal": [0, 0, 1]
      },
      "patches": [
        {
          "label": "1_1",
          "source_region": 1,
          "clip_polygon": [[0, 0], [100, 0], [100, 50], [0, 50]],
          "exclude_polygons": [],
          "face_ids": [10, 11]
        }
      ]
    }
  ]
}
```

字段规则：

- `original_region`：1-based 源 selected region。
- `raster_chart.origin/u_axis/v_axis/normal`：模型坐标中的 chart frame，U/V/normal 必须保持右手关系。
- `clip_polygon`：raster UV 中的 patch 外轮廓。
- `exclude_polygons`：raster UV 中必须扣除的孔洞或侧区。
- `face_ids`：只用于限制射线可命中的 STL cells，不定义二维边界。
- `label`：人类可读及输出目录标签。

## 读取与回退

1. 同时检查 `schema` 和 `version`。
2. v2 patch 缺少有效 `clip_polygon` 时，不得静默丢掉源 region。
3. v2 缺少 `raster_chart` 时属于旧兼容数据，只能走 legacy mesh raster；要使用新算法必须重新分区。
4. 未被任何 v2 record 覆盖的源 region 保持正常导出。
5. manifest 中的 source region 必须在项目 region 数量范围内。

## 文件命名

```text
latest_partitioned.rsp.json
latest_partitioned_manifest.json
```

两个文件必须成对使用。复制或更名项目文件时必须同步复制或更名 manifest。
