"""Shared parsing and resolution for experiment region/patch selectors."""

from __future__ import annotations

import re


_SELECTOR_PATTERN = re.compile(r"^(\d+)(?:[-_.](\d+))?$")


def canonical_region_selector(raw: str) -> str:
    """Normalize ``1-1``, ``1_1`` and ``1.1`` to manifest label ``1_1``."""

    token = raw.strip()
    match = _SELECTOR_PATTERN.fullmatch(token)
    if match is None:
        raise ValueError(f"无效避障区域 {raw!r}；请使用 1、2 或 1-1、1-2")
    source_value = int(match.group(1))
    patch_value = int(match.group(2)) if match.group(2) is not None else None
    if source_value <= 0 or (patch_value is not None and patch_value <= 0):
        raise ValueError("避障 region/patch 编号必须是 1-based 正整数")
    source = str(source_value)
    patch = match.group(2)
    return source if patch is None else f"{source}_{patch_value}"


def parse_region_selectors(raw: str) -> list[str]:
    values: list[str] = []
    seen: set[str] = set()
    for item in re.split(r"[,，、;；]+", raw):
        if not item.strip():
            continue
        value = canonical_region_selector(item)
        if value not in seen:
            seen.add(value)
            values.append(value)
    return values


def selector_matches(selectors: set[str], region_label: str, source_region: int) -> bool:
    """A source-region selector applies to all of its derived patches."""

    label = canonical_region_selector(str(region_label))
    return label in selectors or str(int(source_region)) in selectors


def validate_selectors(selectors: list[str], planning_regions: list[dict]) -> None:
    unmatched = [
        selector
        for selector in selectors
        if not any(selector_matches({selector}, item["label"], int(item["source_region"])) for item in planning_regions)
    ]
    if unmatched:
        labels = ",".join(str(item["label"]).replace("_", "-").replace(".", "-") for item in planning_regions)
        raise ValueError(f"避障区域不存在: {unmatched}; 当前可用区域/patch: {labels}")
