"""
Phase 2 fallback — headless mesh prep + USD export (no Blender required).

Produces a watertight, decimated sim mesh. UVs are not preserved by decimation;
use phase2_blender_prepare_mouse.py for PBR-preserving export.

Usage:
  python scripts/phase2_export_usd.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

PROJECT_ROOT = Path(__file__).resolve().parent.parent
INPUT_GLB = PROJECT_ROOT / "assets" / "seed3d" / "mouse_static.glb"
OUTPUT_USD = PROJECT_ROOT / "assets" / "usd" / "mouse_soft.usd"
TEXTURE_DIR = PROJECT_ROOT / "assets" / "usd" / "textures"
REPORT_PATH = PROJECT_ROOT / "assets" / "usd" / "phase2_mesh_report.json"

TARGET_BODY_LENGTH_M = 0.08
TARGET_FACE_COUNT = 10_000
VOXEL_PITCH_M = 0.0018  # tuned for ~5k–10k faces after watertight remesh


def y_up_to_z_up(points: np.ndarray) -> np.ndarray:
    """GLB Y-up → Isaac Sim / USD Z-up: rotate -90° about X."""
    x, y, z = points[:, 0].copy(), points[:, 1].copy(), points[:, 2].copy()
    rotated = np.empty_like(points)
    rotated[:, 0] = x
    rotated[:, 1] = -z
    rotated[:, 2] = y
    return rotated


def load_and_prepare() -> tuple[trimesh.Trimesh, Path | None]:
    mesh = trimesh.load(INPUT_GLB, force="mesh")
    texture_path: Path | None = None

    visual = mesh.visual
    if hasattr(visual, "material") and hasattr(visual.material, "baseColorTexture"):
        img = visual.material.baseColorTexture
        if img is not None:
            TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
            texture_path = TEXTURE_DIR / "base_color.png"
            img.save(texture_path)

    mesh.apply_scale(TARGET_BODY_LENGTH_M / mesh.extents.max())
    simplified = mesh.simplify_quadric_decimation(face_count=TARGET_FACE_COUNT)

    # Watertight voxel remesh for soft-body FEM input
    voxel = simplified.voxelized(pitch=VOXEL_PITCH_M)
    watertight = voxel.marching_cubes
    # marching_cubes returns voxel-grid coords; map back to world space
    watertight.apply_transform(voxel.transform)

    watertight.vertices = y_up_to_z_up(np.asarray(watertight.vertices, dtype=np.float64))
    return watertight, texture_path


def write_usd(mesh: trimesh.Trimesh, texture_path: Path | None) -> None:
    OUTPUT_USD.parent.mkdir(parents=True, exist_ok=True)
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/Mouse")
    mesh_prim = UsdGeom.Mesh.Define(stage, "/Mouse/Mesh")

    points = Vt.Vec3fArray([Gf.Vec3f(*v) for v in mesh.vertices])
    face_counts = Vt.IntArray([3] * len(mesh.faces))
    indices = Vt.IntArray(mesh.faces.flatten().tolist())

    mesh_prim.CreatePointsAttr(points)
    mesh_prim.CreateFaceVertexCountsAttr(face_counts)
    mesh_prim.CreateFaceVertexIndicesAttr(indices)
    mesh_prim.CreateSubdivisionSchemeAttr("none")

    if texture_path and texture_path.exists():
        rel_tex = Path("textures") / texture_path.name
        mat = UsdShade.Material.Define(stage, "/Mouse/Material")
        shader = UsdShade.Shader.Define(stage, "/Mouse/Material/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        reader = UsdShade.Shader.Define(stage, "/Mouse/Material/DiffuseTexture")
        reader.CreateIdAttr("UsdUVTexture")
        reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(rel_tex).replace("\\", "/"))
        reader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            reader.ConnectableAPI(), "rgb"
        )
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(mesh_prim).Bind(mat)

    stage.GetRootLayer().Save()


def main() -> None:
    if not INPUT_GLB.exists():
        raise FileNotFoundError(f"Missing input GLB: {INPUT_GLB}")

    mesh, texture_path = load_and_prepare()
    write_usd(mesh, texture_path)

    report = {
        "pipeline": "phase2_export_usd.py (headless fallback)",
        "target_body_length_m": TARGET_BODY_LENGTH_M,
        "target_face_count": TARGET_FACE_COUNT,
        "voxel_pitch_m": VOXEL_PITCH_M,
        "watertight": bool(mesh.is_watertight),
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "extents_m": mesh.extents.tolist(),
        "max_extent_m": float(mesh.extents.max()),
        "texture": str(texture_path) if texture_path else None,
        "usd_output": str(OUTPUT_USD),
        "note": "UVs not preserved. Prefer Blender script for PBR-quality export.",
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print(f"\nUSD → {OUTPUT_USD}")


if __name__ == "__main__":
    main()
