"""
push_test.py — kinematic rigid block pushes free deformable mouse.

Determines whether a *kinematic* rigid body (not an articulation) can transmit
contact force into a free GPU-FEM soft body.

Result interpretation
---------------------
  centroid X shift > 0.5 mm  OR  X-span change > 0.5 mm
    → FEM *does* respond to kinematic-rigid contact ✓
      Problem is specific to Franka articulation joints ↔ FEM.
      Fix: use PhysX Attachment API when fingers reach target.

  neither shift > 0.5 mm
    → FEM does *not* respond to any rigid contact (general issue).
      Must drive FEM nodes directly (kinematic-node teleport or attachment).

Run:
  $env:OMNI_KIT_ACCEPT_EULA = "YES"
  $py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"
  & $py scripts\push_test.py
"""

from __future__ import annotations
from pathlib import Path

PUSH_STEPS          = 240     # frames of pushing
SETTLE_STEPS        = 120     # gravity settle before push
LOG_INTERVAL        = 15      # frames between log lines
PUSH_VEL_M_PER_STEP = 0.0005  # 0.5 mm / frame → ~30 mm / s @ 60 Hz
BLOCK_GAP_M         = 0.030   # gap between block and mouse left edge at t=0

PROJECT_ROOT = Path(__file__).resolve().parent.parent
BAKED_USD    = PROJECT_ROOT / "assets" / "usd" / "mouse_soft_baked.usda"
MOUSE_USD    = PROJECT_ROOT / "assets" / "usd" / "mouse_soft.usd"

TABLE_SCALE  = (0.22, 0.18, 0.05)
TABLE_TOP_Z  = 0.180
TABLE_TRANS  = (0.52, 0.35, TABLE_TOP_Z - TABLE_SCALE[2] * 0.5)
MOUSE_HALF_H = 0.012567
MOUSE_TRANS  = (TABLE_TRANS[0], TABLE_TRANS[1], TABLE_TOP_Z + MOUSE_HALF_H)
MESH_PATH    = "/World/MouseAsset/Mouse/mesh_001"

YOUNG_MODULUS  = 1.0e4
POISSON_RATIO  = 0.48
DEFORM_FRIC    = 0.0          # zero friction so the mouse slides freely
DEFORM_DAMP    = 0.005
SIM_HEX_RES    = 8
CONTACT_OFFSET = 0.002


def _add_table(stage):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    p = "/World/PushTable"
    if stage.GetPrimAtPath(p).IsValid():
        return
    cube = UsdGeom.Cube.Define(stage, p)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_TRANS))
    xf.AddScaleOp().Set(Gf.Vec3d(*TABLE_SCALE))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    # Zero-friction material so mouse can slide freely on the table
    from pxr import Sdf, UsdShade
    mat_prim = stage.DefinePrim("/World/SlipperyMat", "Material")
    UsdPhysics.MaterialAPI.Apply(mat_prim)
    phys_mat = UsdPhysics.MaterialAPI(mat_prim)
    phys_mat.CreateStaticFrictionAttr(0.0)
    phys_mat.CreateDynamicFrictionAttr(0.0)
    phys_mat.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        UsdShade.Material(mat_prim),
        UsdShade.Tokens.weakerThanDescendants,
        "physics",
    )
    print("[push_test] table added (zero-friction material)", flush=True)


def _add_push_block(stage, mouse_xmin: float) -> str:
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    bw, bd, bh = 0.012, 0.050, 0.040  # width(X), depth(Y), height(Z)
    start_x = mouse_xmin - BLOCK_GAP_M - bw * 0.5
    path = "/World/PushBlock"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(start_x, MOUSE_TRANS[1], TABLE_TOP_Z + bh * 0.5))
    xf.AddScaleOp().Set(Gf.Vec3d(bw, bd, bh))
    rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    co = PhysxSchema.PhysxCollisionAPI(cube.GetPrim())
    co.CreateContactOffsetAttr(0.001)
    co.CreateRestOffsetAttr(0.0)
    print(f"[push_test] push block at start_x={start_x:.4f}  "
          f"(gap {BLOCK_GAP_M*1000:.0f} mm left of mouse)", flush=True)
    return path, start_x, bw, bh


def _move_block(stage, path: str, start_x: float, bw: float, bh: float, step: int):
    from pxr import Gf, UsdGeom
    new_x = start_x + step * PUSH_VEL_M_PER_STEP
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(new_x, MOUSE_TRANS[1], TABLE_TOP_Z + bh * 0.5))
    xf.AddScaleOp().Set(Gf.Vec3d(bw, 0.050, bh))


def _log_mouse(view, step: int, label: str):
    import numpy as np
    pos = view.get_simulation_mesh_nodal_positions()
    arr = pos.detach().cpu().numpy().reshape(-1, 3) if hasattr(pos, "detach") else \
          __import__("numpy").asarray(pos).reshape(-1, 3)
    cx, cy, cz = arr.mean(axis=0)
    xmin, xmax = float(arr[:, 0].min()), float(arr[:, 0].max())
    print(f"[push_test] {label} s{step:03d}: "
          f"centroid=({cx:.4f},{cy:.4f},{cz:.4f}) "
          f"X-span={(xmax-xmin)*1000:.1f}mm xmin={xmin:.4f} xmax={xmax:.4f}", flush=True)
    return cx, xmin, xmax


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "anti_aliasing": 0})

    import omni.usd
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.materials.deformable_material import DeformableMaterial
    from isaacsim.core.prims.impl.single_deformable_prim import SingleDeformablePrim
    from pxr import PhysxSchema

    # ── world + stage ─────────────────────────────────────────────────────────
    world = World(physics_dt=1/60, rendering_dt=1/60, stage_units_in_meters=1.0)
    world.get_physics_context().enable_gpu_dynamics(True)

    # Load baked (or original) mouse USD onto stage via reference
    stage = world.stage
    src_usd = str(BAKED_USD) if BAKED_USD.exists() else str(MOUSE_USD)
    from pxr import Sdf, UsdGeom
    mouse_ref_prim = stage.DefinePrim("/World/MouseAsset")
    mouse_ref_prim.GetReferences().AddReference(src_usd)
    print(f"[push_test] loaded mouse from {src_usd}", flush=True)

    _add_table(stage)

    # ── deformable body ───────────────────────────────────────────────────────
    mat = DeformableMaterial(
        prim_path="/World/MouseDeformMat",
        youngs_modulus=YOUNG_MODULUS,
        poissons_ratio=POISSON_RATIO,
        dynamic_friction=DEFORM_FRIC,
        elasticity_damping=DEFORM_DAMP,
    )
    deformable = SingleDeformablePrim(
        prim_path=MESH_PATH,
        name="mouse_deformable",
        deformable_material=mat,
        simulation_hexahedral_resolution=SIM_HEX_RES,
        self_collision=False,
        collision_simplification=False,
        vertex_velocity_damping=0.2,
        sleep_threshold=0.0,
        solver_position_iteration_count=16,
    )
    world.scene.add(deformable)

    # Contact offset on the mouse mesh
    prim = stage.GetPrimAtPath(MESH_PATH)
    if prim.IsValid():
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
        co = PhysxSchema.PhysxCollisionAPI(prim)
        co.CreateContactOffsetAttr(float(CONTACT_OFFSET))
        co.CreateRestOffsetAttr(0.0)

    world.reset()
    view = deformable._deformable_prim_view

    # ── gravity settle (no pin) ───────────────────────────────────────────────
    print(f"[push_test] gravity settle {SETTLE_STEPS} steps …", flush=True)
    for _ in range(SETTLE_STEPS):
        world.step(render=False)
        app.update()
    cx0, xmin0, xmax0 = _log_mouse(view, 0, "settled")
    span0 = xmax0 - xmin0

    # ── spawn kinematic push block ────────────────────────────────────────────
    block_path, block_start_x, bw, bh = _add_push_block(stage, xmin0)

    # Re-initialize physics to register the new rigid prim
    world.reset()
    for _ in range(30):
        world.step(render=False)
        app.update()
    _log_mouse(view, 0, "pre_push")

    # ── push loop ─────────────────────────────────────────────────────────────
    print(f"[push_test] pushing {PUSH_STEPS} steps "
          f"({PUSH_VEL_M_PER_STEP*1000:.1f} mm/step) …", flush=True)
    for step in range(1, PUSH_STEPS + 1):
        _move_block(stage, block_path, block_start_x, bw, bh, step)
        world.step(render=False)
        app.update()
        if step % LOG_INTERVAL == 0:
            _log_mouse(view, step, "push")

    # ── result ────────────────────────────────────────────────────────────────
    cx_f, xmin_f, xmax_f = _log_mouse(view, PUSH_STEPS, "final")
    span_f = xmax_f - xmin_f
    d_cx   = (cx_f - cx0) * 1000.0
    d_span = (span_f - span0) * 1000.0
    block_x_final = block_start_x + PUSH_STEPS * PUSH_VEL_M_PER_STEP
    print("\n[push_test] ═══ RESULT ═══", flush=True)
    print(f"  centroid X shift : {d_cx:+.3f} mm  (+→ pushed right)", flush=True)
    print(f"  X-span change    : {d_span:+.3f} mm  (−→ compressed)", flush=True)
    print(f"  block final X    : {block_x_final:.4f}  (mouse xmin was {xmin0:.4f})", flush=True)
    if abs(d_cx) > 0.5 or abs(d_span) > 0.5:
        print("  ✓ FEM responded — kinematic rigid CAN push FEM", flush=True)
        print("    → problem is Franka articulation-specific; fix via Attachment API", flush=True)
    else:
        print("  ✗ FEM did NOT respond — rigid contact generally cannot push FEM nodes", flush=True)
        print("    → need to drive FEM nodes directly (attachment or kinematic-node approach)", flush=True)
    print("[push_test] ═══════════════\n", flush=True)

    app.close()


if __name__ == "__main__":
    main()
