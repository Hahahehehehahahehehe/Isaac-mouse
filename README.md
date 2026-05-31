# Isaac-mouse

Soft-body mouse grasp demo in **Isaac Sim 5.1** — PhysX FEM deformable + Factory Franka parallel gripper.

## Requirements

- NVIDIA Isaac Sim 5.1 (Python env, e.g. `.venv-isaacsim`)
- GPU with CUDA
- Mouse USD: `assets/usd/mouse_soft.usd` (+ baked pose `mouse_soft_baked.usda`)

Large source meshes (`*.glb`, Blender `.blend`) are **not** in this repo. Regenerate via `plan.md` / Phase 2 scripts if needed.

## Run grasp demo

```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
& "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe" scripts\grasp_demo.py --hide-collision-debug --hide-gripper-debug --no-export
```

Headless:

```powershell
& $py scripts\grasp_demo.py --headless --no-export
```

See `debug.md` for pinch/lift troubleshooting (contactOffset, collision filter, attachment lift).

## Project layout

| Path | Description |
|------|-------------|
| `scripts/grasp_demo.py` | Main Isaac Sim grasp sequence |
| `scripts/phase2_*.py` | Mesh prep & USD export |
| `assets/usd/mouse_soft.usd` | Deformable mouse asset |
| `debug.md` | Debug log & Step 0–4 results |
| `plan.md` | Pipeline roadmap |

## License

Research / lab use. Isaac Sim and Factory Franka assets subject to NVIDIA / vendor licenses.
