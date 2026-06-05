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
| 2026-06-04 | Step 5：系统性穷举；当时误判为「刚体无法横向推动 FEM」（见下，已被 Step 6 推翻） |
| 2026-06-05 | Step 6：kinematic flag 写反（w=0=pin,w=1=free），修复后 FEM 正常形变 |
| 2026-06-05 | Step 7：`get_body_grip_center()` 躯干 Y 截面 X 对中；手动调 Y_FRAC/X_OFFSET；见下 |

---

## Step 5 — 刚体↔FEM 接触机制穷举（2026-06-04）

### 背景

Step 4c 后 pinch 正常、lift 正常，但 GUI/Headless 均无法看到软体在 pinch 时形变或被横向推动。
小鼠 FEM 节点在 pinch 期间 centroid/X-span 全程冻结到机器精度（4 位小数不变），如：

```
mouse dyn [pinch s225~s435]: centroid=(0.5253,0.3526,0.1876) X-span=43.0mm
```

排查过以下假设（全部被数据排除）：

| 假设 | 否定证据 |
|------|---------|
| 小鼠被 kinematic pin 钉死 → w=0 释放失败 | 手指物理坐标已插入小鼠 hull 内（Lfinger X=0.535，hull xmin=0.5056），若钉死则手指被挡住，不会穿入 |
| sleep_threshold 过高 → FEM 睡眠冻结 | 改 `sleep_threshold=0` + 每帧 `wake_deformable()` 回写速度缓冲，结果零变化 |
| kinematic pin 残留（w=0 API 不可靠） | `TABLE_PIN_ENABLED=False` 全局禁用所有 pin 调用；deformable 改由接触支撑在桌面——仍零位移 |
| collision_simplification 影响接触耦合 | 改 `True`（93 节点碰撞网格），结果零变化 |
| GPU CUDA 问题（无 N 卡） | 日志确认 RTX 4060 Laptop（device 0, Active, CUDA 12.8/sm_89）正常工作 |

### 决定性控制实验（6 组）

| # | 配置 | 手指最终开度 | FEM 仿真网格 |
|---|------|------------|------------|
| 1 | 软体 + 2 mm offset + pin 开 | 穿到 27.9 mm | **冻结** |
| 2 | 软体 + 无 offset（PhysX 默认）+ pin 开 | **卡在 64 mm**（被挡住） | **冻结** |
| 3 | **刚性盒** + 2 mm offset | 停在 43.5 mm（几何接触） | 接触力 L 3.26 N / R 3.20 N ✓ |
| 4 | 软体 + 2 mm offset + **pin 全关** | 穿到 27.9 mm | **冻结** |
| 5 | 软体 + 无 offset + **pin 全关** | 卡在 64 mm | **冻结** |
| 6 | **kinematic 刚体块**横向推 300 步（总推进 150 mm）| 块深入小鼠身体 | **冻结（零位移）** |

实验 #5 是决定性的：pin 已全关（小鼠自由）、桌面无摩擦、默认大接触距离可以把手指**挡在 64 mm** 处（接触约束进入了关节求解器），但自由的 FEM 节点四位小数全程不变。

实验 #6 用 `--push-test` 模式独立验证：一个 **kinematic 刚体块**以 0.5 mm/帧推了 300 帧（150 mm），最终深入小鼠身体内，FEM 节点 centroid X shift = **0.000 mm**。

### ⚠️ Step 5 结论已被推翻（2026-06-05）

> ~~**在 Isaac Sim 5.1 + PhysX GPU FEM 当前配置下，任何刚性物体（关节驱动或 kinematic）都无法通过接触力横向推动 GPU FEM 软体仿真网格。**~~

**Step 5 的结论是错误的。** 真正原因是 `set_mouse_kinematic_pin` 里 kinematic flag 的含义写反了。

PhysX / Isaac Lab 约定（`write_nodal_kinematic_target_to_sim` 官方文档）：
- `w = 0.0` → kinematic（钉死）
- `w = 1.0` → free（自由）

旧代码写的是 `w = 1.0 if enabled else 0.0`，且 `TABLE_PIN_ENABLED=False` 让所有调用走 `enabled=False` 分支，全程写 `w=0`，等于把所有 FEM 节点钉成 kinematic。手指物理位置"插入" hull 内只是接触约束进了关节求解器让手指停住，FEM 节点本身被冻结、不受力，所以 centroid/X-span 机器精度不变——**不是 PhysX 的物理限制，是代码 bug**。

`release_mouse_table_pin → set_mouse_kinematic_pin(False) → w=0` 也没有真正释放节点（w=0 本身就是 pin 值），与社区 [#5079](https://github.com/isaac-sim/IsaacLab/issues/5079)「节点释放后仍冻结」是同类问题。

修复（2026-06-05）：改为 `w = 0.0 if enabled else 1.0`。修复后 headless 验证：

```
mouse dyn [pinch s225~s270]: X-span 43.0→54.0→55.2 mm（逐帧增大，FEM 真实形变）
grasp confirmed at step 528 (mouse lifted): mouse z=0.233 (+0.045 m)
```

### 注意：桌面支撑仍有效

`TABLE_PIN_ENABLED=False` 后，重力沉降 90 步确认小鼠靠接触稳停桌面（`collision contact=0.180, table=0.180`），无需 kinematic pin。pin 机制保持禁用，小鼠由真实接触支撑。

---

## Step 6 — kinematic flag 修复与新现象（2026-06-05）

### 根因

`set_simulation_mesh_kinematic_targets` 第 4 分量约定：**`w=0` = kinematic，`w=1` = free**（PhysX / Isaac Lab 官方）。旧代码把含义写反，导致整个 approach / descend / pinch 期间所有 FEM 节点全程被 `w=0` 钉死，接触力无法进入 FEM 求解器，与「接触力单向」假说的表现完全一致，但原因截然不同。

**修复**（`scripts/grasp_demo.py`）：

```python
# 修复前（错误）
targets[..., 3] = 1.0 if enabled else 0.0
# 修复后（正确）
targets[..., 3] = 0.0 if enabled else 1.0  # w=0=kinematic, w=1=free
```

同时修复 `update_mouse_attachment`（lift carry）：由 `w=1.0` 改为 `w=0.0` 才是真正的 kinematic hold。

### 修复后 headless 结果（filter=none，2026-06-05）

```
mouse dyn [pinch s225]: X-span=54.0mm  ← 修复前 43.0mm（冻结）
mouse dyn [pinch s270]: X-span=55.2mm  ← FEM 被挤压向两侧鼓出
mouse dyn [pinch s285]: X-span=59.7mm  ← 接触冲击，之后回弹
squeeze_done:           X-span=52mm    ← 稳定压缩态
grasp confirmed at step 528: mouse z = +0.045 m ✓
```

FEM 节点在 pinch 期间真实形变，接触力正常传入 FEM 求解器。

### 修复后 GUI 现象（部分仍待解决）

| 现象 | 分析 |
|------|------|
| **视觉 mesh 崩飞** | FEM 正常；render mesh 蒙皮在快速形变时失效（Fabric 显示问题） |
| **绿色 collider 视觉穿模** | render mesh ≠ collision mesh；`filter=none` 下物理碰撞仍开启 |
| **命令 14 mm，实际 ~23 mm** | 位置 PD + 接触力上限；软体反推后关节达不到命令目标 |

---

## Step 7 — 躯干截面对中 + 手动调参（2026-06-05）

### 背景

小鼠身体在 X 方向呈香蕉形弯曲（头部/臀部 X 中心不同）。用全体节点或整体躯干 bbox 算 X 中心会把夹取点拉偏，导致 GUI 中左指先碰。

### 算法 `get_body_grip_center()`

1. 按 Y 切片，保留 X 宽度 ≥ 45% 最大宽度的切片（排除尾巴）。
2. `grasp_y = body_lo + GRASP_BODY_Y_FRAC × (body_hi - body_lo)`。
3. `grip_x` = 在 `grasp_y ± 12 mm` 窄带内节点 X bbox 中点 + `GRASP_BODY_X_OFFSET_M`。

HOVER 验证（无接触）：`finger_mid` vs `body_X@graspY_mid` 误差 **≤ 0.1 mm**。

### 当前手动调优值

```
GRASP_BODY_Y_FRAC     = 0.4      # 腹肋区（0=尾，1=头）
GRASP_BODY_X_OFFSET_M = -0.0024  # −2.4 mm
GRIPPER_CLAMP_M       = 0.007    # 命令总开口 14 mm
DEFAULT_FINGER_MAX_FORCE_N = 120
```

### 仍未解决

- GUI 视觉 mesh 崩飞
- Lift 为 FEM nodal attachment，**非摩擦抓取**
- 实际 pinch 开度常高于命令目标（接触平衡）

---

## 已知限制（当前版本）

1. **Pinch**：FEM 接触驱动形变 ✓（headless 可验证 X-span 变化）。
2. **Lift**：`attach_mouse_to_hand` 节点刚性跟随 ✗ 非摩擦力。
3. **显示**：白色 render mesh 可能在 GUI 崩飞；以 FEM 日志为准。
4. **开度**：`GRIPPER_CLAMP_M` 为命令值，实际 `f1+f2` 可能更大。

---

## 常量（2026-06-05）

```
GRIPPER_CLAMP_M          = 0.007  → 14 mm 总开口命令
GRIPPER_OPEN_M           = 0.04   → 80 mm 全开
GRASP_BODY_Y_FRAC        = 0.4
GRASP_BODY_X_OFFSET_M    = -0.0024
DEFAULT_DEFORM_CONTACT   = 2 mm
DEFAULT_FINGER_CONTACT   = 1 mm
DEFAULT_FINGER_KP        = 800
DEFAULT_FINGER_MAX_FORCE = 120 N
YOUNG_MODULUS            = 1e4 Pa
DEFORMABLE_FRICTION      = 0.2
TABLE_PIN_ENABLED        = False
Tuned 2+1 mm + none      ≈ 22–25 mm 实际 pinch（命令 14 mm）
Lift delta (4c)          ≥ +0.045 m → grasp confirmed（attachment）
```
