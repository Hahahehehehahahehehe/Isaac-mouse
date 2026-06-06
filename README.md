# Isaac-mouse

Soft-body mouse grasp demo in **Isaac Sim 5.1** — PhysX FEM deformable + Factory Franka parallel gripper.

Repository: [github.com/Hahahehehehahahehehe/Isaac-mouse](https://github.com/Hahahehehehahahehehe/Isaac-mouse)

## What works today

- **Pinch**: gripper closes on a silicone-like FEM mouse with real contact-driven deformation (Step 6 kinematic-flag fix).
- **Lift**: three modes via `--lift-mode` (see below) — default is **friction**; **hybrid** adds partial kinematic constraint on the grip region.
- **Alignment**: `get_body_grip_center()` places the TCP over the torso at a chosen Y slice and centres X on that slice.
- **GUI gripper visuals**: white Franka hand/fingers follow physics through pinch (Step 8 — `disable_instanceable()`).
- **Demo cameras** (GUI): eye-in-hand wrist cam (both fingers in frame) + fixed side view as main viewport.

## Lift modes — pick one when you run the demo

**Pinch is the same for all modes** (physical FEM contact). **Lift differs** in how the mouse is carried after pinch hold.

| Mode | Flag | Mouse carry during lift | Best for |
|------|------|-------------------------|----------|
| **Friction** (default) | `--lift-mode friction` or omit flag | High μ only; Cartesian IK +12 cm; no FEM kinematic pins | Most physical; may slip or sag over time |
| **Hybrid** | `--lift-mode hybrid` | 30-frame settle: partial kinematic on **grip-region** nodes, then Cartesian +12 cm; tail/head stay soft FEM | Stable carry without rigid whole-body weld |
| **Attachment** (legacy) | `--lift-mode attachment` | Whole-mesh nodal kinematic carry (`attach_mouse_to_hand`) | Headless/debug; least physical but very stable |

### How to tell which mode is running (console log)

| Mode | You should see |
|------|----------------|
| **friction** | `lift via friction: Cartesian IK +120 mm` — **no** `grip attachment` |
| **hybrid** | `hybrid constraint settle (30 frames` → `grip attachment: …/… FEM nodes` → `hybrid constraint settle done` |
| **attachment** | `grip attachment: …` at lift start + whole mesh moves rigidly with hand |

If you expected hybrid but only see `lift via friction`, you forgot `--lift-mode hybrid` (default is still friction).

### Hybrid sequence (60 Hz)

```
pinch hold complete
  → hybrid_settle (30 frames: arm frozen, partial kinematic on grip-region nodes)
  → lift (Cartesian IK +12 cm vertical, constraint already active)
  → grasp confirmed if centroid Δz ≥ 12 cm
```

### Friction sequence

```
pinch hold complete → lift immediately (Cartesian IK +12 cm, friction only)
```

## Known limitations (2026-06-05)

| Issue | Status |
|-------|--------|
| **Mouse FEM visual jitter** during pinch | FEM sim is OK; render mesh may jitter (skinning artifact). Gripper visuals fixed in Step 8. |
| **Friction lift slip** | Mouse may slide out of the gripper after several seconds; use `--lift-mode hybrid` for grip-region constraint. |
| **Commanded vs actual pinch opening** | Target 14 mm total (`GRIPPER_CLAMP_M=0.007`); physics often stalls ~20–24 mm. |
| **Grasp metric vs GUI** | `grasp confirmed` when FEM **centroid** Δz ≥ **12 cm**; tail may still drag; hybrid pins torso only. |

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

# GUI (default: no debug overlays, side main view + floating wrist cam)
& $py scripts\grasp_demo.py --lift-mode hybrid --no-export

# GUI — lighter GPU (main view only, switch to wrist_cam in Camera menu)
& $py scripts\grasp_demo.py --lift-mode hybrid --no-extra-viewports --no-export

# Friction-only lift (default --lift-mode)
& $py scripts\grasp_demo.py --no-export

# Legacy whole-mesh kinematic carry
& $py scripts\grasp_demo.py --lift-mode attachment --no-export

# Headless full sequence
& $py scripts\grasp_demo.py --headless --lift-mode hybrid --no-export

# Pinch diagnosis only (no lift)
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export

# Re-enable orange/green physics debug overlays
& $py scripts\grasp_demo.py --show-collision-debug --show-gripper-debug --no-export
```

### CLI options (lift / friction)

| Flag | Default | Meaning |
|------|---------|---------|
| `--lift-mode friction` | ✓ | Friction + Cartesian lift; **no** FEM kinematic constraint |
| `--lift-mode hybrid` | | Partial kinematic on grip region after pinch, then Cartesian lift |
| `--lift-mode attachment` | | Legacy whole-mesh kinematic carry |
| `--mouse-friction` | 6.0 | Deformable mouse dynamic friction μ |
| `--gripper-friction` | 6.0 | Finger/hand collider static+dynamic friction μ |
| `--show-collision-debug` | | Orange FEM collision hull overlay |
| `--show-gripper-debug` | | Green gripper collider overlay |
| `--no-demo-cameras` | | Skip wrist + fixed viewports |
| `--no-extra-viewports` | | Main view only; no floating wrist window (lighter GPU) |

### Demo cameras (GUI)

On startup (unless `--no-demo-cameras`):

| Viewport | Camera prim | View |
|----------|-------------|------|
| **Main** | `/OmniverseKit_Persp` → `SideView` | From **+X**, looks **−X** (side-on arm + table) |
| **Floating: Grasp — Wrist** | `…/panda_hand/wrist_cam` | Eye-in-hand behind gripper; both fingers in lower frame |

Tune in `grasp_demo.py`: `DEMO_CAM_SIDE_DIST_M` / `DEMO_CAM_SIDE_HEIGHT_M`; wrist offset `WRIST_CAM_LOCAL_*` and `WRIST_CAM_ROT_DEG`.

If floating windows fail, use the viewport **Camera** menu to pick `wrist_cam` or `SideView`.

### GUI freeze / sudden exit (troubleshooting)

Terminal may show `carb.crashreporter-breakpad` / `gpu.foundation.plugin.dll` / `omni.kit.renderer.plugin.dll` — a **GPU renderer crash**, not a Python exception in `grasp_demo.py`. Common on laptop GPUs (e.g. RTX 4060 8 GB) when **main + wrist RTX viewport + FEM sim** overload VRAM.

**Try in order:**

1. **Stable GUI (recommended first)** — main view + camera prims, no extra windows:
   ```powershell
   & $py scripts\grasp_demo.py --no-extra-viewports --lift-mode hybrid --no-export
   ```
   Switch views from the viewport Camera dropdown.

2. **Minimal GPU** — single default viewport:
   ```powershell
   & $py scripts\grasp_demo.py --no-demo-cameras --no-export
   ```

3. **Close other GPU apps** (browsers, other Isaac instances, heavy IDE GPU use) before launching.

4. **Headless** for logic/regression without GUI:
   ```powershell
   & $py scripts\grasp_demo.py --headless --lift-mode hybrid --no-export
   ```

Crash logs: `%LOCALAPPDATA%\ov\pkg\isaac-sim-*\cache\Kit\logs\` or Omniverse crash reporter output in the terminal.

## Tuning knobs (`scripts/grasp_demo.py` top)

**Grasp pose & squeeze**

| Constant | Meaning |
|----------|---------|
| `GRASP_BODY_Y_FRAC` | Where along the body (0=tail … 1=head). Current: **0.4**. |
| `GRASP_BODY_X_OFFSET_M` | Fine X shift (m). Current: **−0.0024**. |
| `GRIPPER_CLAMP_M` | Per-finger close target (m); total = ×2. Current: **0.007** → **14 mm** command. |

**Lift & friction**

| Constant | Meaning |
|----------|---------|
| `DEFORMABLE_FRICTION` / `GRIPPER_FRICTION` | Mouse + finger μ. Current: **6.0** each. |
| `LIFT_DURATION_STEPS` | Lift length @ 60 Hz. Current: **360** (~6 s). |
| `GRASP_LIFT_DELTA_CONFIRM_M` | Centroid rise for `grasp confirmed`. Current: **12 cm** (all modes). |
| `HYBRID_CONSTRAINT_SETTLE_STEPS` | Hybrid only: frames with constraint on before Z lift. **30**. |
| `DEFAULT_FINGER_MAX_FORCE_N` | Finger squeeze cap. **180 N**. |

Related: `DEFAULT_DEFORM_CONTACT_OFFSET_M` (2 mm), `DEFAULT_FINGER_CONTACT_OFFSET_M` (1 mm).

## Grasp sequence overview (60 Hz)

```
prepare_run → approach → descend → pinch (close 60f → squeeze 45f → hold 120f)
    → [hybrid only: hybrid_settle 30f]
    → lift (Cartesian IK +12 cm)
    → grasp confirmed if centroid Δz ≥ 12 cm
```

**Pinch** is physical FEM contact for every mode. Pinch has a visible **second squeeze** after first contact (`GRIPPER_SQUEEZE_STEPS`, `force_grip=True`).

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
