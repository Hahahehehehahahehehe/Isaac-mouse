"""Inspect GLB / mesh stats for Phase 2 quality gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import trimesh


def inspect(path: Path) -> dict:
    mesh = trimesh.load(path, force="mesh")
    boundary_edges = mesh.edges[trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1)]
    return {
        "path": str(path),
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "euler_number": int(mesh.euler_number),
        "bounds_min": mesh.bounds[0].tolist(),
        "bounds_max": mesh.bounds[1].tolist(),
        "extents_m": mesh.extents.tolist(),
        "max_extent_m": float(mesh.extents.max()),
        "has_uv": bool(getattr(mesh.visual, "uv", None) is not None),
        "boundary_edge_count": int(len(boundary_edges)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect mesh for Phase 2.")
    parser.add_argument(
        "path",
        nargs="?",
        default=Path(__file__).resolve().parent.parent / "assets" / "seed3d" / "mouse_static.glb",
        type=Path,
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "assets" / "usd" / "inspect_report.json",
    )
    args = parser.parse_args()

    report = inspect(args.path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nSaved → {args.output}")


if __name__ == "__main__":
    main()
