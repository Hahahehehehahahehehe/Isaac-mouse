# 会话交接说明 — Isaac-mouse 软体小鼠抓取 Demo

> 仓库：[Isaac-mouse](https://github.com/Hahahehehehahahehehe/Isaac-mouse)  
> 主脚本：`scripts/grasp_demo.py` · 排错日志：`debug.md`

---

## 1. 项目是什么

Isaac Sim **5.1** 里用 **PhysX FEM 软体小鼠** + **Factory Franka 平行夹爪** 做顶视 pinch → lift。  
环境：`D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe`，需 `$env:OMNI_KIT_ACCEPT_EULA = "YES"`。

```powershell
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"

# GUI（默认无 debug 叠加层）
& $py scripts\grasp_demo.py --lift-mode hybrid --no-export

# 摩擦 lift（默认 --lift-mode）
& $py scripts\grasp_demo.py --no-export

# 旧方案：FEM 节点刚性搬运
& $py scripts\grasp_demo.py --lift-mode attachment --no-export

& $py scripts\grasp_demo.py --headless --no-export
```

---

## 2. 已解决的问题（摘要）

| 问题 | 修复 |
|------|------|
| 小鼠悬浮在桌面上 | `DEFORMABLE_FLOOR_SINK_M` + `force_mouse_on_table` re-seat |
| 夹爪卡在 ~65 mm | `PhysxCollisionAPI` contactOffset：mouse 2 mm + finger 1 mm（Step 4b） |
| Lift 时小鼠不抬（旧） | FEM nodal attachment 刚性跟随（Step 4c，`--lift-mode attachment`） |
| Pinch 时 FEM 完全不形变 | kinematic flag 写反（`w=0`=pin, `w=1`=free）（Step 6） |
| 夹取点偏尾 / X 不对称 | `get_body_grip_center()` 躯干 Y 截面 X 对中（Step 7） |
| 夹爪视觉 mesh 接触后弹回（GUI） | `disable_instanceable()`（Step 8） |
| 默认改为摩擦 lift | 高摩擦 + 关节空间抬升 + 降低质心阈值（Step 9） |

---

## 3. 当前状态（2026-06-05）

### A. 摩擦 lift（默认，`--lift-mode friction`）

- **GUI**：可夹起小鼠并抬离桌面，达到预期；一段时间后可能从指间滑脱。
- **机制**：`DEFORMABLE_FRICTION` + `GRIPPER_FRICTION`（当前 μ=6.0）；lift 不用 `attach_mouse_to_hand`。
- **抬升**：`apply_cartesian_ik_step` 垂直抬升（`GRASP_LIFT_Z_OFFSET_M`，+12 cm）；`LIFT_DURATION_STEPS=360`（~6 s）。摩擦/hybrid 仍保留指间 creep。
- **判定**：FEM 质心 Δz ≥ **12 cm** → `grasp confirmed`（与 Cartesian lift 跨度一致）。
- **Pinch 二次发力**：60 帧闭合爬坡 → 45 帧 `squeeze`（`force_grip=True`）→ 120 帧 hold，GUI 可见接触后再次收紧。
- **Hybrid 约束时序**：pinch hold 结束 → **30 帧 `hybrid_settle`**（手臂冻结 + 局部 kinematic 约束）→ 再开始 Cartesian 抬升（避免先滑后拽）。

### B. 遗留 / 次要

| 项 | 说明 |
|----|------|
| 小鼠 FEM 视觉抖动 | pinch 时白色 render mesh 可能抖；以 FEM 日志为准 |
| 命令开度 vs 实际 | 命令 14 mm，实际常 ~20–24 mm |
| 滑脱 | 摩擦抓取持握时间有限，非 attachment 刚性搬运 |
| 对齐微调 | `GRASP_BODY_Y_FRAC` / `GRASP_BODY_X_OFFSET_M` |

### C. 旧方案（`--lift-mode attachment`）

整网 `set_simulation_mesh_nodal_positions` 刚性平移；质心阈值 **45 mm**；headless 稳定通过。

---

## 4. 三个主要调参旋钮（夹取位姿）

```python
GRASP_BODY_Y_FRAC       = 0.4
GRASP_BODY_X_OFFSET_M   = -0.0024
GRIPPER_CLAMP_M         = 0.007    # 命令总开口 14 mm
```

---

## 5. 当前关键常量（Step 9）

```python
# 摩擦 lift（默认）
DEFORMABLE_FRICTION          = 6.0
GRIPPER_FRICTION             = 6.0
DEFAULT_FINGER_MAX_FORCE_N   = 180.0
LIFT_DURATION_STEPS          = 360   # ~6 s；速度 ∝ 1/steps
GRASP_LIFT_DELTA_CONFIRM_M   = 0.12  # 12 cm 质心抬升 → confirmed（各 lift 模式统一）
DEFAULT_LIFT_MODE            = friction

# 通用
DEFAULT_DEFORM_CONTACT_OFFSET = 0.002
DEFAULT_FINGER_CONTACT_OFFSET = 0.001
YOUNG_MODULUS                 = 1e4
TABLE_PIN_ENABLED             = False
collision_filter 默认          = none
pinch_mode 默认               = position

# 步数预算（勿单独加大 LIFT 而忘记此项）
MAX_GRASP_STEPS = STEPS_DESCEND + _PINCH_MAX_STEPS + LIFT_DURATION_STEPS + 80
```

**注意**：旧版固定 `MAX_GRASP_STEPS=900` 时，`LIFT_DURATION_STEPS>~455` 会在抬升完成前退出循环（例如 720 步只完成 ~63%）。

---

## 6. 抓取流程

```
prepare_run → approach → descend → pinch (close → squeeze → hold)
    → lift (friction/hybrid: Cartesian IK + 指间 creep；attachment: Cartesian + 整网平移)
    → grasp confirmed（质心 Δz ≥ 12 cm）
```

---

## 7. 文件索引

| 文件 | 用途 |
|------|------|
| `scripts/grasp_demo.py` | 主 demo |
| `debug.md` | Step 0–9 实验记录 |
| `README.md` | 快速上手 |
| `plan.md` | 管线 roadmap |

---

## 8. 下一 session 建议方向

1. 延长摩擦持握时间（μ、法向力、lift 速度折中，减少头沉与滑脱）。
2. 小鼠 FEM 视觉抖动（降刚度 / 降形变速率）。
3. 混合策略：摩擦 lift 为主，滑脱检测后短时 attachment 保底（若需要）。
