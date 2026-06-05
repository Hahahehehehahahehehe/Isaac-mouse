# Isaac-mouse

Soft-body mouse grasp demo in **Isaac Sim 5.1** — PhysX FEM deformable + Factory Franka parallel gripper.

Repository: [github.com/Hahahehehehahahehehe/Isaac-mouse](https://github.com/Hahahehehehahahehehe/Isaac-mouse)

## What works today

- **Pinch**: gripper closes on a silicone-like FEM mouse with real contact-driven deformation (Step 6 kinematic-flag fix).
- **Lift (default)**: friction-based carry — high mouse + finger friction, joint-space lift; GUI shows the mouse rising with the gripper (Step 9).
- **Lift (fallback)**: `--lift-mode attachment` — legacy FEM nodal kinematic carry (`attach_mouse_to_hand`).
- **Alignment**: `get_body_grip_center()` places the TCP over the torso at a chosen Y slice and centres X on that slice.
- **GUI gripper visuals**: white Franka hand/fingers follow physics through pinch (Step 8 — `disable_instanceable()`).

## Known limitations (2026-06-05)

| Issue | Status |
|-------|--------|
| **Mouse FEM visual jitter** during pinch | FEM sim is OK; render mesh may jitter (skinning artifact). Gripper visuals fixed in Step 8. |
| **Friction lift slip** | Mouse may slide out of the gripper after several seconds; centroid rise lags finger height when the head pitches down during lift. |
| **Commanded vs actual pinch opening** | Target 14 mm total (`GRIPPER_CLAMP_M=0.007`); physics often stalls ~20–24 mm. |
| **Grasp metric vs GUI** | `grasp confirmed` uses FEM **centroid** Δz (12 mm for friction, 45 mm for attachment) — lower than visual lift when the body tilts. |

See `debug.md` (Steps 0–9) and `HANDOFF.md` for full history and tuning notes.

## Requirements

- NVIDIA Isaac Sim 5.1 (Python env, e.g. `.venv-isaacsim`)
- GPU with CUDA
- Mouse USD: `assets/usd/mouse_soft.usd` (+ baked pose `mouse_soft_baked.usda`, auto-generated on first run)

Large source meshes (`*.glb`, Blender `.blend`) are **not** in this repo. Regenerate via `plan.md` / Phase 2 scripts if needed.

## Run grasp demo

```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"

# GUI — friction lift (default)
& $py scripts\grasp_demo.py --hide-collision-debug --hide-gripper-debug --no-export

# Legacy kinematic carry (Step 4c)
& $py scripts\grasp_demo.py --lift-mode attachment --hide-collision-debug --hide-gripper-debug --no-export

# Headless full sequence
& $py scripts\grasp_demo.py --headless --no-export

# Pinch diagnosis only (no lift)
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export
```

### CLI options (lift / friction)

| Flag | Default | Meaning |
|------|---------|---------|
| `--lift-mode friction` | ✓ | Pinch squeeze + friction; no FEM nodal attachment |
| `--lift-mode attachment` | | Legacy whole-mesh kinematic carry |
| `--mouse-friction` | 6.0 | Deformable mouse dynamic friction μ |
| `--gripper-friction` | 6.0 | Finger/hand collider static+dynamic friction μ |

## Tuning knobs (`scripts/grasp_demo.py` top)

**Grasp pose & squeeze**

| Constant | Meaning |
|----------|---------|
| `GRASP_BODY_Y_FRAC` | Where along the body (0=tail … 1=head). Current: **0.4**. |
| `GRASP_BODY_X_OFFSET_M` | Fine X shift (m). Current: **−0.0024**. |
| `GRIPPER_CLAMP_M` | Per-finger close target (m); total = ×2. Current: **0.007** → **14 mm** command. |

**Friction lift (Step 9)**

| Constant | Meaning |
|----------|---------|
| `DEFORMABLE_FRICTION` / `GRIPPER_FRICTION` | Mouse + finger μ. Current: **6.0** each. |
| `LIFT_DURATION_STEPS` | Lift length @ 60 Hz. Current: **360** (~6 s). Speed ∝ 1/steps. |
| `GRASP_LIFT_DELTA_FRICTION_M` | Centroid rise for `grasp confirmed`. Current: **12 mm**. |
| `DEFAULT_FINGER_MAX_FORCE_N` | Finger squeeze cap. Current: **180 N**. |

Related: `DEFAULT_DEFORM_CONTACT_OFFSET_M` (2 mm), `DEFAULT_FINGER_CONTACT_OFFSET_M` (1 mm).

**Note**: `MAX_GRASP_STEPS` auto-scales with `LIFT_DURATION_STEPS`. Do not set lift duration above ~455 while `MAX_GRASP_STEPS` is fixed at 900 (old behaviour) — use the current auto formula instead.

## Grasp sequence (60 Hz)

```
prepare_run → approach → descend → pinch (close 60f → squeeze 45f → hold 120f)
    → lift (friction: joint-space to ARM_LIFT_DEG + grip creep)
    → grasp confirmed if centroid Δz ≥ 12 mm (friction) or ≥ 45 mm (attachment)
```

**Pinch** is physical FEM contact. **Default lift** is friction, not nodal attachment. Pinch has a visible **second squeeze** after first contact (`GRIPPER_SQUEEZE_STEPS`, `force_grip=True`).

## Project layout

| Path | Description |
|------|-------------|
| `scripts/grasp_demo.py` | Main Isaac Sim grasp sequence |
| `scripts/push_test.py` | Diagnostic: kinematic block push vs FEM |
| `scripts/phase2_*.py` | Mesh prep & USD export |
| `assets/usd/mouse_soft.usd` | Deformable mouse asset |
| `debug.md` | Debug log Steps 0–9 |
| `HANDOFF.md` | Session handoff & current status |
| `plan.md` | Pipeline roadmap |

## License

Research / lab use. Isaac Sim and Factory Franka assets subject to NVIDIA / vendor licenses.
