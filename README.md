# Isaac-mouse

Soft-body mouse grasp demo in **Isaac Sim 5.1** — PhysX FEM deformable + Factory Franka parallel gripper.

Repository: [github.com/Hahahehehehahahehehe/Isaac-mouse](https://github.com/Hahahehehehahahehehe/Isaac-mouse)

## What works today

- **Pinch**: gripper closes on a silicone-like FEM mouse with real contact-driven deformation (after kinematic-flag fix, Step 6).
- **Lift**: mouse rises with the gripper and passes grasp check (`mouse z` delta ≥ 45 mm).
- **Alignment**: `get_body_grip_center()` places the TCP over the torso at a chosen Y slice and centres X on that slice (handles banana-curved body geometry).
- **GUI gripper visuals**: white Franka hand/fingers follow physics (Step 8 — `disable_instanceable()` on load).

## Known limitations (2026-06-05)

| Issue | Status |
|-------|--------|
| **Mouse FEM visual jitter** during pinch | FEM sim is OK; mouse white render mesh may still jitter when nodes move fast (skinning artifact). Gripper visuals fixed in Step 8. |
| **Lift is not friction-based** | Parallel gripper cannot lift soft silicone by friction alone; lift uses **FEM nodal attachment** (`attach_mouse_to_hand` + per-frame `set_simulation_mesh_nodal_positions`). |
| **Commanded vs actual pinch opening** | Target may be 14 mm total (`GRIPPER_CLAMP_M=0.007`) but physics stalls higher (~20–24 mm) when contact + max finger force balance out. |
| **GUI vs physics (mouse only)** | Mouse render mesh can look like penetration during fast deformation; trust FEM logs / headless metrics for mouse physics. |

See `debug.md` (Steps 0–8) and `HANDOFF.md` for full history and tuning notes.

## Requirements

- NVIDIA Isaac Sim 5.1 (Python env, e.g. `.venv-isaacsim`)
- GPU with CUDA
- Mouse USD: `assets/usd/mouse_soft.usd` (+ baked pose `mouse_soft_baked.usda`, auto-generated on first run)

Large source meshes (`*.glb`, Blender `.blend`) are **not** in this repo. Regenerate via `plan.md` / Phase 2 scripts if needed.

## Run grasp demo

```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"

# GUI (hide debug overlays)
& $py scripts\grasp_demo.py --hide-collision-debug --hide-gripper-debug --no-export

# Headless full sequence (pinch → lift)
& $py scripts\grasp_demo.py --headless --no-export

# Pinch diagnosis only (no lift)
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export
```

## Tuning knobs (`scripts/grasp_demo.py` top)

Three main parameters for grasp pose and squeeze depth:

| Constant | Meaning |
|----------|---------|
| `GRASP_BODY_Y_FRAC` | Where along the body (head–tail, 0=tail … 1=head) to grasp. Current: **0.4** (abdomen/rib area). |
| `GRASP_BODY_X_OFFSET_M` | Fine X shift of grip target (m). Current: **−0.0024** (−2.4 mm). |
| `GRIPPER_CLAMP_M` | Per-finger close target (m); total opening = `× 2`. Current: **0.007** → **14 mm** total command. |

Related: `GRASP_BODY_Y_OFFSET_M`, `DEFAULT_FINGER_MAX_FORCE_N` (120 N), `DEFAULT_DEFORM_CONTACT_OFFSET_M` (2 mm), `DEFAULT_FINGER_CONTACT_OFFSET_M` (1 mm).

## Grasp sequence (60 Hz)

```
prepare_run → approach (hover) → descend (straddle) → pinch (close + squeeze + hold)
    → lift (FEM nodal attachment carry) → grasp confirmed if Δz ≥ 45 mm
```

**Important**: pinch deformation is physical (FEM contact). **Lift is kinematic carry**, not friction grasp.

## Project layout

| Path | Description |
|------|-------------|
| `scripts/grasp_demo.py` | Main Isaac Sim grasp sequence |
| `scripts/push_test.py` | Diagnostic: kinematic block push vs FEM |
| `scripts/phase2_*.py` | Mesh prep & USD export |
| `assets/usd/mouse_soft.usd` | Deformable mouse asset |
| `debug.md` | Debug log Steps 0–8 |
| `HANDOFF.md` | Session handoff & current status |
| `plan.md` | Pipeline roadmap |

## License

Research / lab use. Isaac Sim and Factory Franka assets subject to NVIDIA / vendor licenses.
