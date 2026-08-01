from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import math
import zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = EXPERIMENT_DIR / "configs" / "robotstudio_export.json"
MANIFEST_SCHEMA = "base_casting_abb6700.robotstudio_jobs"
MANIFEST_VERSION = 1

_CALIB_DECLARATION = re.compile(
    r"(?im)^[ \t]*(?:(?:TASK|LOCAL)\s+)?PERS\s+(?:tooldata|wobjdata)\s+"
    r"[A-Za-z_][A-Za-z0-9_]*\s*:=.*?;[ \t]*(?:\r?\n)?"
)
_MODULE_HEADER = re.compile(r"(?im)^([ \t]*MODULE\s+)([A-Za-z_][A-Za-z0-9_]*)")
_EXPERIMENT_META_COMMENT = re.compile(
    r"(?m)^([ \t]*!\s*RSP_EXPERIMENT_META_V1)[ \t]+(\{[^\r\n]*\})[ \t]*(?=\r?$)"
)


@dataclass(frozen=True)
class OptimalRecord:
    region_label: str
    model_x: float
    model_y: float
    model_z: float
    angle_deg: int
    source_rapid: Path


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def format_number_token(value: float) -> str:
    rounded = round(float(value), 6)
    if abs(rounded - round(rounded)) < 1e-6:
        text = str(abs(int(round(rounded))))
    else:
        text = f"{abs(rounded):.6f}".rstrip("0").rstrip(".").replace(".", "p")
    return f"m{text}" if rounded < 0 else text


def filename_region_label(label: str) -> str:
    return str(label).strip().replace("_", "-").replace(".", "-")


def rapid_identifier(label: str) -> str:
    identifier = re.sub(r"[^A-Za-z0-9_]", "_", str(label).strip())
    if not identifier:
        raise ValueError("Empty region label")
    if identifier[0].isdigit():
        identifier = f"R{identifier}"
    return identifier.upper()


def station_filename(record: OptimalRecord) -> str:
    return (
        f"{format_number_token(record.model_x)}_"
        f"{format_number_token(record.model_y)}_"
        f"{format_number_token(record.model_z)}_"
        f"rz{record.angle_deg}_"
        f"{filename_region_label(record.region_label)}.rsstn"
    )


def _source_rapid_for(result_dir: Path, label: str, explicit: str | None = None) -> Path:
    if explicit:
        candidate = Path(explicit)
        if not candidate.is_absolute():
            candidate = result_dir / candidate
        if candidate.exists():
            return candidate.resolve()

    folder = result_dir / "optimal_paths" / label
    preferred = folder / f"{label}.txt"
    if preferred.exists():
        return preferred.resolve()
    files = sorted([*folder.glob("*.txt"), *folder.glob("*.mod")])
    if len(files) != 1:
        raise FileNotFoundError(f"Cannot identify one optimal RAPID file for region {label}: {folder}")
    return files[0].resolve()


def read_optimal_records(result_dir: Path) -> list[OptimalRecord]:
    records_path = result_dir / "optimal_records.json"
    raw_rows: list[dict[str, Any]]
    if records_path.exists():
        payload = load_json(records_path)
        rows = payload.get("records", [])
        if not isinstance(rows, list):
            raise ValueError(f"records must be a list: {records_path}")
        raw_rows = [row for row in rows if isinstance(row, dict)]
    else:
        selection_path = result_dir / "optimal_selection.csv"
        if not selection_path.exists():
            raise FileNotFoundError(f"Missing optimal_records.json and optimal_selection.csv: {result_dir}")
        with selection_path.open("r", encoding="utf-8-sig", newline="") as handle:
            raw_rows = list(csv.DictReader(handle))

    records: list[OptimalRecord] = []
    labels: set[str] = set()
    for row in raw_rows:
        label = str(row.get("region_label") or row.get("covered_region") or row.get("region") or "").strip()
        if not label:
            raise ValueError("Optimal record has no region label")
        if label in labels:
            raise ValueError(f"Duplicate optimal region label: {label}")
        labels.add(label)
        records.append(
            OptimalRecord(
                region_label=label,
                model_x=float(row["model_x"]),
                model_y=float(row["model_y"]),
                model_z=float(row["model_z"]),
                angle_deg=int(float(row["angle_deg"])),
                source_rapid=_source_rapid_for(
                    result_dir,
                    label,
                    str(row.get("txt") or row.get("source_rapid") or "") or None,
                ),
            )
        )
    if not records:
        raise ValueError(f"No optimal records found: {result_dir}")
    return records


def split_rapid_modules(source_text: str, region_label: str, calib_module_name: str, prefix: str) -> tuple[str, str]:
    declarations = [match.group(0).strip() for match in _CALIB_DECLARATION.finditer(source_text)]
    tool_count = sum(bool(re.search(r"(?i)\btooldata\b", item)) for item in declarations)
    wobj_count = sum(bool(re.search(r"(?i)\bwobjdata\b", item)) for item in declarations)
    if tool_count != 1 or wobj_count != 1:
        raise ValueError(f"Expected one tooldata and one wobjdata declaration, got tool={tool_count}, wobj={wobj_count}")

    path_text = _CALIB_DECLARATION.sub("", source_text)

    def ascii_metadata(match: re.Match[str]) -> str:
        try:
            metadata = json.loads(match.group(2))
        except json.JSONDecodeError as error:
            raise ValueError("Invalid RSP_EXPERIMENT_META_V1 JSON") from error
        return f"{match.group(1)} {json.dumps(metadata, ensure_ascii=True, separators=(',', ':'))}"

    path_text = _EXPERIMENT_META_COMMENT.sub(ascii_metadata, path_text)
    module_name = f"{rapid_identifier(prefix)}_{rapid_identifier(region_label)}"
    if len(module_name) > 32:
        module_name = module_name[:32]
    path_text, replacements = _MODULE_HEADER.subn(lambda match: f"{match.group(1)}{module_name}", path_text, count=1)
    if replacements != 1:
        raise ValueError("RAPID source has no MODULE header")
    if not path_text.endswith("\n"):
        path_text += "\n"

    calib_lines = [f"MODULE {calib_module_name}"]
    calib_lines.extend(f"    {item}" for item in declarations)
    calib_lines.extend(["ENDMODULE", ""])
    return path_text, "\n".join(calib_lines)


def create_station_copy(
    template: Path,
    output: Path,
    component_name: str,
    installation: dict[str, float],
) -> None:
    namespace = "urn:abb-robotics-pim"
    ET.register_namespace("", namespace)
    with zipfile.ZipFile(template, "r") as source:
        pim_bytes = source.read("PIM.xml")
        root = ET.fromstring(pim_bytes)
        component = None
        for candidate in root.iter(f"{{{namespace}}}ComponentInstance"):
            name = candidate.find(f"{{{namespace}}}Name")
            if name is not None and name.get("Value") == component_name:
                component = candidate
                break
        if component is None:
            raise ValueError(f"Scene component not found in template station: {component_name}")
        transform = component.find(f"{{{namespace}}}Transform")
        if transform is None:
            raise ValueError(f"Scene component has no Transform: {component_name}")

        radians = math.radians(float(installation["rz_deg"]))
        cosine, sine = math.cos(radians), math.sin(radians)
        values = {
            "RowX": {"X": cosine, "Y": sine, "Z": 0.0},
            "RowY": {"X": -sine, "Y": cosine, "Z": 0.0},
            "RowZ": {"X": 0.0, "Y": 0.0, "Z": 1.0},
            "RowT": {
                "X": float(installation["x"]) / 1000.0,
                "Y": float(installation["y"]) / 1000.0,
                "Z": float(installation["z"]) / 1000.0,
            },
        }
        for row_name, coordinates in values.items():
            row = transform.find(f"{{{namespace}}}{row_name}")
            if row is None:
                raise ValueError(f"Template Transform has no {row_name}")
            for axis, value in coordinates.items():
                node = row.find(f"{{{namespace}}}{axis}")
                if node is None:
                    raise ValueError(f"Template Transform has no {row_name}/{axis}")
                node.set("Value", format(value, ".17g"))

        replacement_pim = ET.tostring(root, encoding="utf-8", xml_declaration=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        with zipfile.ZipFile(temporary, "w") as destination:
            for entry in source.infolist():
                payload = replacement_pim if entry.filename == "PIM.xml" else source.read(entry.filename)
                destination.writestr(entry, payload)
    temporary.replace(output)


def build_package(result_dir: Path, config_path: Path = DEFAULT_CONFIG) -> Path:
    result_dir = result_dir.resolve()
    config = load_json(config_path.resolve())
    records = read_optimal_records(result_dir)
    template = Path(str(config["template_station"])).resolve()
    if not template.exists():
        raise FileNotFoundError(f"RobotStudio template station does not exist: {template}")
    jobs: list[dict[str, Any]] = []

    for record in records:
        region_dir = result_dir / "optimal_paths" / record.region_label
        region_dir.mkdir(parents=True, exist_ok=True)
        path_module = region_dir / f"{config.get('path_module_prefix', 'VALIDATE')}_{record.region_label}.mod"
        calib_module = region_dir / "CalibData.mod"
        output_station = region_dir / station_filename(record)
        source_text = record.source_rapid.read_text(encoding="utf-8-sig")
        path_text, calib_text = split_rapid_modules(
            source_text,
            record.region_label,
            str(config.get("calib_module_name", "CalibData")),
            str(config.get("path_module_prefix", "VALIDATE")),
        )
        path_module.write_text(path_text, encoding="utf-8")
        calib_module.write_text(calib_text, encoding="utf-8")
        installation = {
            "x": record.model_x,
            "y": record.model_y,
            "z": record.model_z,
            "rz_deg": record.angle_deg,
        }
        create_station_copy(
            template,
            output_station,
            str(config["workpiece_component_name"]),
            installation,
        )
        sidecar = region_dir / f"{output_station.stem}.robotstudio_job.json"
        sidecar.write_text(
            json.dumps(
                {
                    "schema": "base_casting_abb6700.robotstudio_station_job",
                    "version": 1,
                    "output_station": str(output_station.resolve()),
                    "controller_task": str(config.get("controller_task", "T_ROB1")),
                    "calib_module_name": str(config.get("calib_module_name", "CalibData")),
                    "calib_module": str(calib_module.resolve()),
                    "path_module": str(path_module.resolve()),
                    "path_module_name": _MODULE_HEADER.search(path_text).group(2),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        jobs.append(
            {
                "region_label": record.region_label,
                "model_installation": installation,
                "rapid_coordinates_are_independent": True,
                "source_rapid": str(record.source_rapid),
                "calib_module": str(calib_module.resolve()),
                "path_module": str(path_module.resolve()),
                "output_station": str(output_station.resolve()),
                "station_job": str(sidecar.resolve()),
            }
        )

    manifest = {
        "schema": MANIFEST_SCHEMA,
        "version": MANIFEST_VERSION,
        "result_dir": str(result_dir),
        "robotstudio_exe": str(config["robotstudio_exe"]),
        "sdk_bin": str(config["sdk_bin"]),
        "template_station": str(template),
        "controller_task": str(config.get("controller_task", "T_ROB1")),
        "calib_module_name": str(config.get("calib_module_name", "CalibData")),
        "workpiece_component_name": str(config["workpiece_component_name"]),
        "jobs": jobs,
    }
    manifest_path = result_dir / "robotstudio_jobs.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest_path


def bridge_directory() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if not local_app_data:
        raise EnvironmentError("LOCALAPPDATA is not defined")
    return Path(local_app_data) / "ABB6700RobotStudioBridge"


def queue_manifest(manifest_path: Path, launch: bool = True) -> Path:
    manifest_path = manifest_path.resolve()
    manifest = load_json(manifest_path)
    bridge = bridge_directory()
    bridge.mkdir(parents=True, exist_ok=True)
    pending_path = bridge / "pending.json"
    temporary_path = bridge / "pending.json.tmp"
    temporary_path.write_text(
        json.dumps({"manifest_path": str(manifest_path)}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary_path.replace(pending_path)
    status_path = manifest_path.parent / "robotstudio_status.json"
    if status_path.exists():
        status_path.unlink()
    if launch:
        executable = Path(str(manifest["robotstudio_exe"]))
        if not executable.exists():
            raise FileNotFoundError(f"RobotStudio executable does not exist: {executable}")
        jobs = manifest.get("jobs", [])
        first_station = Path(str(jobs[0]["output_station"])) if jobs else None
        command = [str(executable)]
        if first_station is not None:
            command.append(str(first_station))
        subprocess.Popen(command, close_fds=True)
    return pending_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Package Optimal-Y results for RobotStudio 6.08 station generation.")
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--queue", action="store_true", help="Queue the manifest for the RobotStudio 6.08 add-in.")
    args = parser.parse_args()
    manifest_path = build_package(args.result_dir, args.config)
    print(f"ROBOTSTUDIO_JOBS={manifest_path}")
    if args.queue:
        print(f"ROBOTSTUDIO_PENDING={queue_manifest(manifest_path)}")


if __name__ == "__main__":
    main()
