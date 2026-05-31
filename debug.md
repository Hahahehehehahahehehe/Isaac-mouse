# Grasp Demo Debug Log

Isaac Sim 5.1 soft-body mouse + Factory Franka top-down pinch.  
Script: `scripts/grasp_demo.py`

---

## 问题

Pinch 目标 24 mm 总开口；`filter=none` 时卡在 ~64.8 mm；`filter=mouse/all` 能收到 ~25 mm 但穿模。

---

## 已证明 vs 仍待证

| 陈述 | 状态 | 证据 |
|------|------|------|
| Orange hull **几何**顶死夹爪（hull 宽 43 mm 墙） | **已排除** | 卡住时 opening−hull_X = **21.6 mm**；截图绿指与橙 hull 有缝 |
| **Palm ↔ finger** 自碰撞是主 blocker | **已排除** | `hand<->finger \|force\| = 0 N`；`filter=hand` 仍 64.8 mm |
| 关掉 **gripper↔mouse FilteredPairs** 是收到位的**必要条件** | **已证明** | A/B：`none/hand`→64.8 mm；`mouse/all`→25.5 mm |
| 卡住是因为 **orange hull 已经几何接触** | **未证明** | 与 21.6 mm 余量矛盾 |
| 卡住是因为 **GPU rigid↔deformable 耦合/预接触/求解器约束** | **Step4b 已证实主因** | 未设 `physxCollision:contactOffset`（读回 unset）→ 64.8 mm；显式设 2+1 mm → **30 mm** |
| 绿色 debug collider = GPU rigid collider | **未证明** | finger collider AABB gap 与 RigidContactView 不一致，gap 指标暂不可信 |

**Filter 关的是 PhysX 子树间整类交互，不是「把已发生的接触变软」。**

---

## 排错路线图（按顺序执行）

### Step 0 — Filter A/B（已完成）

```powershell
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export
```

| filter | pinch 总开口 | 结论 |
|--------|-------------|------|
| `none` | **64.8 mm** | 基线 |
| `hand` | **64.8 mm**（历史） | palm 非主因 |
| `mouse` | **25.5 mm**（历史） | gripper↔mouse 通道必要 |
| `all` | **25.5 mm**（历史） | 同 mouse |

`none` 诊断摘录：
```
geometric clearance = 21.6 mm
hand<->finger force = 0.00 N
```

- [x] Step 0 完成

---

### Step 1 — 刚性盒对照（deformable 特有？）

**假设**：若 stall 仅来自 deformable 耦合，换成同尺寸 **kinematic rigid box** 后，`filter=none` 应能收到 24 mm（或卡在 ~43 mm 几何接触，而非 65 mm）。

```powershell
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --mouse-mode rigid-box --no-export
```

| 模式 | pinch 总开口 | 解读 |
|------|-------------|------|
| deformable + none | **64.8 mm** | 基线；hull 宽 43.3 mm，余量 21.5 mm |
| rigid-box + none | **43.5 mm** | 卡在盒宽 ~43 mm；`clearance=0.2 mm`；**非 65 mm** |

**Step1 结论**：stall 是 **deformable 特有**。同尺寸 kinematic box 在 `filter=none` 下能收到几何接触（43.5 mm），说明 65 mm 不是夹爪自碰撞或 collider 几何上限。

rigid-box 诊断摘录：
```
total opening 43.5 mm (target 24.0)
geometric clearance (opening - box_X) = 0.2 mm
finger<->target left_finger: |force|=11.53 N
finger<->target right_finger: |force|=11.11 N
hand<->finger force = 0.00 N
```

- [x] Step 1 运行
- [x] Step 1 结果写入本表

---

### Step 2 — 真实 collider 间距（非 green debug）

诊断块输出 `finger<->target gap X`：finger **CollisionAPI** 子 prim 的 world AABB vs 目标 AABB。

| 模式 | opening | hull/box 宽 | clearance | min gap X | 解读 |
|------|---------|------------|-----------|-----------|------|
| deformable + none | 64.8 mm | 43.3 mm | **21.5 mm** | -205 mm* | 有几何余量仍卡住 → 非 hull 硬顶 |
| rigid-box + none | 43.5 mm | 43.3 mm | **0.2 mm** | -410 mm* | 几何接触确认 |

\* finger collider AABB 沿整指延伸，gap 数值与 RigidContactView 不一致；**以 clearance + contact force 为准**。

- [x] deformable 模式记录
- [x] rigid-box 模式记录

---

### Step 3 — RigidContactView finger ↔ 目标（仅 rigid-box 可靠）

| 模式 | filter | opening | finger↔target \|force\| | 解读 |
|------|--------|---------|---------------------------|------|
| rigid-box | none | 43.5 mm | L **11.5 N** / R **11.1 N** | 经典 contact force，几何接触 |
| deformable | none | 64.8 mm | _无 probe_ | GPU 不支持 deformable filter |

- [x] rigid-box + none：pinch 时 force≈11 N，gap 不可信但 clearance≈0
- [x] deformable：无 RigidContactView；21.5 mm 余量 + 无 filter 仍卡 65 mm → **非经典 contact，是 deformable 通道约束**

---

### Step 4 — 解决方案（机制清楚后再做）

**机制摘要**：deformable 在 gripper↔mouse 未 filter 时，GPU 求解器在 ~65 mm 处施加非几何约束；filter 关掉该通道后可收到位但穿模。

#### 4a 力控 / 顺应 pinch（已实现，2026-05-29 测试）

新增 `--pinch-mode {position,force,compliant}`，配合 `--collision-filter none`：

```powershell
# 纯 effort 关闭（每指 20 N，stall 检测后 hold → lift）
& $py scripts\grasp_demo.py --headless --pinch-mode force --collision-filter none --no-export

# 低刚度 position creep（kp=80, maxForce=30 N）
& $py scripts\grasp_demo.py --headless --pinch-mode compliant --collision-filter none --diagnose-pinch --no-export
```

| pinch-mode | filter | 总开口 | clearance | 结论 |
|------------|--------|--------|-----------|------|
| position | none | **64.8 mm** | 21.5 mm | Step0 基线 |
| **force** | none | **64.9 mm** | 21.6 mm | effort 20 N/finger，**未突破 65 mm** |
| **compliant** | none | **69.3 mm** | 26.0 mm | 更差、更不对称 |

**Step4a 结论**：换 effort 或低刚度 position **不能**绕过 stall（在 offset 未修时）。

#### 4b contactOffset 修复（2026-05-29，**突破 65 mm**）

**根因**：旧代码把 offset 写在 `PhysxDeformableBodyAPI`（无效）；deformable 实际应设 **`PhysxCollisionAPI`** 于 mesh。未显式 authoring 时 PhysX 用内部默认，诊断读回 **unset**，pinch 卡在 **64.8 mm / 21.5 mm 余量**。

**修复**：默认 mouse **2 mm** + finger collider **1 mm**（`physxCollision:contactOffset/restOffset`）。

```powershell
#  tuned（默认，filter=none 可收到 ~30 mm）
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export

#  复现旧 65 mm stall
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-contact-offset-tuning --no-export

#  自定义 offset A/B
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none `
  --deform-contact-offset-mm 2 --finger-contact-offset-mm 1 --no-export
```

| 配置 | mouse/finger offset | filter | 总开口 | clearance | 结论 |
|------|---------------------|--------|--------|-----------|------|
| PhysX 默认（unset） | 未设 | none | **64.8 mm** | **21.5 mm** | 复现 Step0 stall |
| **tuned 2 + 1 mm** | 显式 | none | **30.0 mm** | **-12.2 mm** | **突破 65 mm**，几何+压缩接触 |
| 显式 20 + 10 mm | 显式 | none | **30.0 mm** | -12.2 mm | 显式 authoring 即可，大小不敏感 |

**Step4b 结论**：65 mm 主因是 **contactOffset 未正确设置**，不是 filter 唯一解。`filter=none` + tuned offset 可收到 **~30 mm**（仍略高于 24 mm 目标，因 soft body 压缩）。

- [x] 修 `PhysxCollisionAPI` + CLI `--deform-contact-offset-mm` 等
- [x] A/B：unset→64.8 mm；tuned→30 mm

#### 4c attachment lift（2026-05-30，**grasp confirmed**）

**问题**：Step4b 后 pinch 能收到 ~30 mm，但 lift 时 mouse z 不变（`delta=0`）。根因是 kinematic target carry 未抬升 FEM 节点。

**修复**：`update_mouse_attachment()` 改为每帧在 `world.step()` **之后**用 `set_simulation_mesh_nodal_positions` 刚性平移已捕获节点；attachment 推迟到 lift 开始时 capture（168/168 sim 节点）。

```powershell
# 默认完整 grasp（tuned offset + filter=none）
& $py scripts\grasp_demo.py --headless --no-export
```

| 阶段 | 总开口 | 备注 |
|------|--------|------|
| pinch_cmd | **29.9 mm** | clearance **-12.3 mm**（压缩接触） |
| squeeze_done / lift_start | **28.9 mm** | 仍略高于 24 mm 目标 |
| grasp_ok | **26.3 mm** | lift 过程中继续收拢 |
| lift | mouse z **+0.046 m** | step 524 **grasp confirmed** |

摘录：
```
grip attachment: 168/168 FEM nodes (r<=55 mm or body box), z-span=27.5 mm
grasp confirmed at step 524 (mouse lifted): mouse z=0.234 (+0.046 m)
```

**Step4c 结论**：`filter=none` + tuned contactOffset + post-step nodal carry **可完成 pinch + lift**，无需 filter 穿模方案。

- [x] post-step `set_simulation_mesh_nodal_positions` carry
- [x] lift 时 mouse z delta **+46 mm** ≥ 45 mm 阈值
- [ ] 进一步收到 24 mm（当前 26–29 mm，可延长 squeeze）
- [ ] 分阶段 filter 方案（**可能不再需要**）

---

## 诊断命令速查

```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
$py = "D:\Labworks\Project_Issac\.venv-isaacsim\Scripts\python.exe"

# Step 0 baseline（需 --no-contact-offset-tuning 复现 65 mm）
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-contact-offset-tuning --no-export

# Step 4b tuned offset（默认 2+1 mm）
& $py scripts\grasp_demo.py --headless --diagnose-pinch --collision-filter none --no-export
```

---

## 已排除错因（汇总）

Hull 几何、palm 自碰撞、桌面摩擦、IK 抢约束、joint 损坏、GPU memory 警告。

---

## 变更记录

| 日期 | 内容 |
|------|------|
| 2026-05-29 | Step 0：filter A/B + 21.6 mm clearance + hand force 0 |
| 2026-05-29 | Step 1：rigid-box+none→43.5 mm / 11 N contact；deformable 仍 64.8 mm |
| 2026-05-29 | Step 4b：contactOffset 修复；unset→64.8 mm，tuned 2+1 mm→**30 mm** |
| 2026-05-30 | Step 4c：post-step nodal carry；**grasp confirmed** mouse z +46 mm，`filter=none` |

---

## 常量

```
GRIPPER_CLAMP_M = 0.012  → 24 mm target
Stuck (offset unset)     ≈ 64.8 mm
Tuned 2+1 mm + none      ≈ 26–30 mm (pinch→lift)
Closed (mouse filter)    ≈ 25.5 mm (穿模)
Lift delta (4c)          ≥ +0.045 m → grasp confirmed
DEFAULT_DEFORM_CONTACT   = 2 mm (physxCollision on mesh)
DEFAULT_FINGER_CONTACT   = 1 mm (finger/hand colliders)
```
