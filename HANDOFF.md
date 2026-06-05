# 会话交接说明 — Isaac-mouse 软体小鼠抓取 Demo

> 仓库：[Isaac-mouse](https://github.com/Hahahehehehahahehehe/Isaac-mouse)  
> 主脚本：`scripts/grasp_demo.py` · 排错日志：`debug.md`

---

## 1. 项目是什么

Isaac Sim **5.1** 里用 **PhysX FEM 软体小鼠** + **Factory Franka 平行夹爪** 做顶视 pinch → lift。  
环境：`D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe`，需 `$env:OMNI_KIT_ACCEPT_EULA = "YES"`。

```powershell
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"

& $py scripts\grasp_demo.py --hide-collision-debug --hide-gripper-debug --no-export
& $py scripts\grasp_demo.py --headless --no-export
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export
```

---

## 2. 已解决的问题（摘要）

| 问题 | 修复 |
|------|------|
| 小鼠悬浮在桌面上 | `DEFORMABLE_FLOOR_SINK_M` + `force_mouse_on_table` re-seat |
| 夹爪卡在 ~65 mm | `PhysxCollisionAPI` contactOffset：mouse 2 mm + finger 1 mm（Step 4b） |
| Lift 时小鼠不抬 | FEM nodal attachment 刚性跟随 gripper（Step 4c） |
| Pinch 时 FEM 完全不形变 | kinematic flag 写反（`w=0`=pin, `w=1`=free）已修正（Step 6） |
| 夹取点偏尾 / X 不对称 | `get_body_grip_center()`：按 Y 截面宽度识别躯干，在夹取 Y 窄带内算 X 中心（Step 7） |
| 夹爪视觉 mesh 与绿色 collider 分离（GUI「崩飞」） | `disable_instanceable()`：加载 Franka 后取消 instanceable 原型（Step 8） |

---

## 3. 当前仍存在的问题（2026-06-05）

### A. 小鼠 FEM 视觉 mesh 快速形变（GUI，次要）

- **现象**：pinch 时小鼠白色 render mesh 顶点可能剧烈抖动或穿模感；**夹爪视觉已与 collider 同步**（Step 8 已修）。
- **性质**：FEM 仿真网格通常正常；小鼠 render 崩飞多为蒙皮 / Fabric 写回 artifact，不代表物理穿透。
- **未解决**：可降形变速率、调刚度，或接受 headless 数值验证。

### B. 抬起不靠摩擦力

- **现象**：平行夹爪无法单靠摩擦提起软硅胶小鼠。
- **现状**：lift 阶段用 `attach_mouse_to_hand()` 捕获 FEM 节点，`update_mouse_attachment()` 每帧 `set_simulation_mesh_nodal_positions` **刚性平移** → `grasp confirmed`。
- **含义**：这是 **kinematic carry**，不是物理摩擦抓取。

### C. 命令开度 vs 实际开度

- **命令**：`GRIPPER_CLAMP_M = 0.007` → 总开口目标 **14 mm**。
- **实际**：接触 + `DEFAULT_FINGER_MAX_FORCE_N`（120 N）平衡后常停在 **~20–24 mm**（如 `12.8 / 10.0 mm` 不对称读数）。
- **原因**：位置 PD 遇到软体接触后推不动；一侧先碰会加剧不对称。

### D. 对齐仍依赖手动微调

- 自动 `get_body_grip_center()` 在 headless 可对齐到 ~0.1 mm，但 GUI 下小鼠形变/滑动后视觉仍可能偏。
- 当前手动调优值见下节三个旋钮。

---

## 4. 三个主要调参旋钮（`grasp_demo.py` 顶部）

```python
GRASP_BODY_Y_FRAC       = 0.4      # 夹取沿身体前后：0=尾端，1=头端；0.4≈腹肋
GRASP_BODY_X_OFFSET_M   = -0.0024  # X 微调（m）；+ 向右，− 向左
GRIPPER_CLAMP_M         = 0.007    # 每指闭合目标；总开口 = ×2 = 14 mm（命令值）
```

| 旋钮 | 作用 |
|------|------|
| `GRASP_BODY_Y_FRAC` | 夹在哪一段（头–尾方向，排除细尾巴后的 body span 内插值） |
| `GRASP_BODY_X_OFFSET_M` | 在自动算的截面 X 中心上再左右平移 |
| `GRIPPER_CLAMP_M` | 夹多紧（命令目标）；实际开度可能更大 |

---

## 5. 当前关键常量

```python
GRIPPER_OPEN_M               = 0.04
DEFAULT_DEFORM_CONTACT_OFFSET  = 0.002   # mouse mesh
DEFAULT_FINGER_CONTACT_OFFSET = 0.001  # finger colliders
DEFAULT_FINGER_KP            = 800.0
DEFAULT_FINGER_MAX_FORCE_N   = 120.0
YOUNG_MODULUS                = 1.0e4
DEFORMABLE_FRICTION          = 0.2
TABLE_PIN_ENABLED            = False
GRASP_BODY_YBAND_M           = 0.012
collision_filter 默认         = none
pinch_mode 默认              = position
```

---

## 6. 抓取流程

```
prepare_run → approach → descend → pinch (FEM 接触形变)
    → lift (FEM nodal attachment，非摩擦)
    → grasp confirmed if Δz ≥ 0.045 m
```

---

## 7. 文件索引

| 文件 | 用途 |
|------|------|
| `scripts/grasp_demo.py` | 主 demo |
| `scripts/push_test.py` | kinematic 推块 vs FEM 诊断 |
| `debug.md` | Step 0–8 实验记录 |
| `README.md` | 快速上手 + 已知限制 |
| `plan.md` | 管线 roadmap |

---

## 8. 下一 session 建议方向

1. 若小鼠 FEM 视觉仍抖动：降力 / 降刚度 / 分阶段闭合。
2. 评估是否要在 pinch 阶段也做节点驱动形变（仅当接触形变视觉仍不够）。
3. 若要做「真摩擦抓取」，需换机制（更大摩擦、包裹式夹爪、或 attachment 仅作保底）。
