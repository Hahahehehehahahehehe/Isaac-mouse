"""
Phase 2 — Mesh prep & GLB → USD (run inside Blender)

Usage:
  1. Open mouse.blend in Blender (>= 3.6).
  2. Scripting workspace → Open this file → Run Script (Alt+P).

Output:
  assets/usd/mouse_soft.usd
  assets/usd/textures/   (exported PBR textures, if enabled)
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector

# ---------------------------------------------------------------------------
# Config — adjust if needed
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(bpy.data.filepath).resolve().parent if bpy.data.filepath else Path(__file__).resolve().parent.parent
TARGET_BODY_LENGTH_M = 0.08  # real-world longest axis (~8 cm)
TARGET_FACE_COUNT = 10_000
MERGE_DISTANCE_M = 0.0002
DECIMATE_FACE_COUNT = TARGET_FACE_COUNT

# If decimated mesh is not watertight, apply voxel Remesh (destroys UV detail on sim mesh).
FORCE_WATERTIGHT_REMEsh = True
REMEsh_VOXEL_SIZE_M = 0.0015  # ~1.5 mm; tune for ~8k–12k faces

EXPORT_USD = True
USD_OUTPUT = PROJECT_ROOT / "assets" / "usd" / "mouse_soft.usd"
TEXTURE_DIR = PROJECT_ROOT / "assets" / "usd" / "textures"

# Isaac Sim uses Z-up; GLB/Blender import is typically Y-up — rotate root for sim if needed.
APPLY_Z_UP_ROTATION = True  # rotate -90° about X so longest body axis aligns with +Y or +Z as you prefer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def log(msg: str) -> None:
    print(f"[phase2] {msg}")


def report_path() -> Path:
    path = PROJECT_ROOT / "assets" / "usd" / "phase2_mesh_report.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def get_mesh_objects() -> list[bpy.types.Object]:
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not meshes:
        raise RuntimeError("No mesh objects found in the scene.")

    # Drop Blender default primitives (e.g. Cube) that came with the file.
    substantial = [obj for obj in meshes if len(obj.data.polygons) >= 1000]
    if substantial:
        removed = [obj.name for obj in meshes if obj not in substantial]
        if removed:
            log(f"Ignoring small/default meshes: {removed}")
            bpy.ops.object.select_all(action="DESELECT")
            for obj in meshes:
                if obj not in substantial:
                    obj.select_set(True)
            if bpy.context.selected_objects:
                bpy.ops.object.delete()
        meshes = substantial

    meshes.sort(key=lambda o: len(o.data.polygons), reverse=True)
    if len(meshes) > 1:
        log(f"Found {len(meshes)} substantial meshes; joining into one.")
    return meshes


def mesh_stats(obj: bpy.types.Object) -> dict:
    mesh = obj.data
    bbox = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    mins = Vector((min(v.x for v in bbox), min(v.y for v in bbox), min(v.z for v in bbox)))
    maxs = Vector((max(v.x for v in bbox), max(v.y for v in bbox), max(v.z for v in bbox)))
    extents = maxs - mins
    return {
        "name": obj.name,
        "vertices": len(mesh.vertices),
        "faces": len(mesh.polygons),
        "bounds_min": [mins.x, mins.y, mins.z],
        "bounds_max": [maxs.x, maxs.y, maxs.z],
        "extents_m": [extents.x, extents.y, extents.z],
        "max_extent_m": float(max(extents)),
    }


def set_active(obj: bpy.types.Object) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def apply_transforms(obj: bpy.types.Object) -> None:
    set_active(obj)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def merge_and_clean(obj: bpy.types.Object) -> None:
    set_active(obj)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.remove_doubles(threshold=MERGE_DISTANCE_M)
    bpy.ops.mesh.delete_loose(use_verts=True, use_edges=True, use_faces=True)
    bpy.ops.object.mode_set(mode="OBJECT")


def count_boundary_edges(obj: bpy.types.Object) -> int:
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    boundary = sum(1 for e in bm.edges if e.is_boundary)
    non_manifold = sum(1 for e in bm.edges if not e.is_manifold)
    bm.free()
    return boundary, non_manifold


def is_watertight(obj: bpy.types.Object) -> bool:
    boundary, non_manifold = count_boundary_edges(obj)
    return boundary == 0 and non_manifold == 0


def scale_to_target_length(obj: bpy.types.Object, target_m: float) -> None:
    stats = mesh_stats(obj)
    current = stats["max_extent_m"]
    if current <= 1e-9:
        raise RuntimeError(f"Mesh {obj.name} has zero extent.")
    factor = target_m / current
    obj.scale = (obj.scale.x * factor, obj.scale.y * factor, obj.scale.z * factor)
    apply_transforms(obj)
    log(f"Scaled {obj.name} by {factor:.6f} → max extent {mesh_stats(obj)['max_extent_m']:.4f} m")


def decimate_to_face_count(obj: bpy.types.Object, target_faces: int) -> None:
    mesh = obj.data
    current = len(mesh.polygons)
    if current <= target_faces:
        log(f"{obj.name}: {current} faces — at or below target, skipping decimate.")
        return
    ratio = target_faces / current
    mod = obj.modifiers.new(name="Phase2_Decimate", type="DECIMATE")
    mod.decimate_type = "COLLAPSE"
    mod.ratio = ratio
    mod.use_collapse_triangulate = True
    set_active(obj)
    bpy.ops.object.modifier_apply(modifier=mod.name)
    log(f"{obj.name}: decimated {current} → {len(mesh.polygons)} faces (ratio={ratio:.4f})")


def remesh_watertight(obj: bpy.types.Object, voxel_size: float) -> None:
    mod = obj.modifiers.new(name="Phase2_Remesh", type="REMESH")
    mod.mode = "VOXEL"
    mod.voxel_size = voxel_size
    mod.adaptivity = 0.0
    set_active(obj)
    bpy.ops.object.modifier_apply(modifier=mod.name)
    log(f"{obj.name}: voxel remesh at {voxel_size} m → {len(obj.data.polygons)} faces")


def join_meshes(objects: list[bpy.types.Object]) -> bpy.types.Object:
    if len(objects) == 1:
        return objects[0]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    # Join into the highest-poly mesh (objects[0] after sort).
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    return bpy.context.view_layer.objects.active


def ensure_z_up(root: bpy.types.Object) -> None:
    """Rotate so the asset matches Isaac Sim Z-up convention (Blender is Z-up already).

    Seed3D GLB imports often arrive with the body length on Y. We rotate -90° X
    only when the longest bbox axis is Y (typical GLB orientation).
    """
    if not APPLY_Z_UP_ROTATION:
        return
    stats = mesh_stats(root)
    extents = stats["extents_m"]
    axis = extents.index(max(extents))
    if axis == 1:  # Y is longest → rotate to lay body along Z or keep; Isaac often expects Z-up ground
        root.rotation_euler[0] -= 1.5707963267948966  # -90° X
        apply_transforms(root)
        log("Applied -90° X rotation for Z-up / Isaac Sim convention.")


def cleanup_scene_for_export(root: bpy.types.Object) -> None:
    """Keep only the processed mouse mesh for a clean USD export."""
    root.name = "Mouse"
    root.parent = None

    bpy.ops.object.select_all(action="DESELECT")
    for obj in list(bpy.context.scene.objects):
        if obj != root:
            obj.select_set(True)
    if bpy.context.selected_objects:
        bpy.ops.object.delete()

    set_active(root)
    log(
        f"Export scene: {root.name} — "
        f"{len(root.data.vertices)} verts, {len(root.data.polygons)} faces, datablock={root.data.name}"
    )


def export_usd(root: bpy.types.Object, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)

    cleanup_scene_for_export(root)
    set_active(root)

    kwargs = dict(
        filepath=str(output_path),
        check_existing=False,
        selected_objects_only=True,
        export_animation=False,
        export_hair=False,
        export_uvmaps=True,
        export_normals=True,
        export_materials=True,
        generate_preview_surface=True,
        export_textures_mode="NEW",
        overwrite_textures=True,
        relative_paths=True,
        root_prim_path="/Mouse",
        convert_orientation=False,
        export_meshes=True,
        export_lights=False,
        export_cameras=False,
        export_curves=False,
        export_points=False,
        export_volumes=False,
    )

    valid = {p.identifier for p in bpy.ops.wm.usd_export.get_rna_type().properties}
    filtered = {k: v for k, v in kwargs.items() if k in valid}
    bpy.ops.wm.usd_export(**filtered)

    log(f"Exported USD → {output_path}")


def main() -> None:
    log(f"Project root: {PROJECT_ROOT}")
    USD_OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    meshes = get_mesh_objects()
    for obj in meshes:
        log(f"Initial {obj.name}: {json.dumps(mesh_stats(obj), indent=2)}")

    # Join multiple parts if Seed3D split materials into sub-meshes
    root = join_meshes(meshes)
    root.name = "Mouse"

    merge_and_clean(root)
    scale_to_target_length(root, TARGET_BODY_LENGTH_M)
    decimate_to_face_count(root, DECIMATE_FACE_COUNT)

    boundary, non_manifold = count_boundary_edges(root)
    watertight = is_watertight(root)
    log(f"After decimate — boundary edges: {boundary}, non-manifold: {non_manifold}, watertight: {watertight}")

    if not watertight and FORCE_WATERTIGHT_REMEsh:
        log("Mesh not watertight; applying voxel Remesh (UV layout may change).")
        remesh_watertight(root, REMEsh_VOXEL_SIZE_M)
        boundary, non_manifold = count_boundary_edges(root)
        watertight = is_watertight(root)
        log(f"After remesh — boundary: {boundary}, non-manifold: {non_manifold}, watertight: {watertight}")

    ensure_z_up(root)

    final_stats = mesh_stats(root)
    report = {
        "target_body_length_m": TARGET_BODY_LENGTH_M,
        "target_face_count": TARGET_FACE_COUNT,
        "watertight": watertight,
        "boundary_edges": boundary,
        "non_manifold_edges": non_manifold,
        "final": final_stats,
        "usd_output": str(USD_OUTPUT),
    }
    report_path().write_text(json.dumps(report, indent=2), encoding="utf-8")
    log(f"Report → {report_path()}")

    if EXPORT_USD:
        export_usd(root, USD_OUTPUT)

    # Save cleaned scene (export cleanup removes lights/cameras/default cube).
    cleanup_scene_for_export(root)
    prepared_blend = PROJECT_ROOT / "assets" / "usd" / "mouse_soft_prepared.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(prepared_blend))
    log(f"Saved prepared blend → {prepared_blend}")
    log("Phase 2 complete.")


if __name__ == "__main__":
    main()
