# Project: Mouse Grasping Simulation Modeling Workflow / 小鼠抓取仿真建模工作流

**Objective / 目标:** Create a physically accurate 3D mouse model from a single image for robotic grasping simulation in NVIDIA Isaac Sim. / 基于单张图像创建小鼠 3D 模型，用于在 NVIDIA Isaac Sim 中进行机器人抓取仿真。

**Status / 状态:** MVP Planning — Soft-Body Grasp Demo First / MVP 规划中 — 优先实现软体抓取 Demo

**Target Output (MVP) / 目标输出 (MVP):** A watertight `.usd` soft-body mouse asset with silicone-like elasticity, ready for a simple robotic grasp demonstration. / 一个水密、具备硅胶般弹性的软体小鼠 `.usd` 资产，可用于简单的机器人抓取演示。

**Target Output (Full) / 目标输出 (完整版):** A fully rigged `.usd` asset with articulated joints and accurate PBR collision meshes ready for robotic manipulation. / 一个完全绑定、具备铰接关节和精确 PBR 碰撞网格的 `.usd` 资产，可直接用于机器人操控。

---

## Roadmap Overview / 路线图概览

| Track / 路线 | Scope / 范围 | Status / 状态 |
| :----------- | :----------- | :------------ |
| **MVP — Soft Body** | Seed3D → GLB → USD → Isaac Sim 软体 + 抓取 Demo | **Current focus / 当前重点** |
| **Full — Articulated** | Blender 骨骼绑定 → 铰接体仿真 | Deferred / 后续阶段 |

**Why MVP first? / 为何先做 MVP？**

Skipping Blender rigging and treating the mouse as a single volume deformable body is **feasible and recommended** as a first milestone. Isaac Sim (PhysX FEM) supports soft-body simulation on imported meshes; a silicone-like material can approximate a small laboratory mouse well enough for contact-force and deformation testing during grasping. / 跳过 Blender 绑骨、将小鼠建模为单一软体在技术和工程上**完全可行**，且适合作为第一个里程碑。Isaac Sim 的 PhysX FEM 软体系统可直接对外部导入网格做体积变形仿真；通过调节杨氏模量、泊松比和阻尼，可以近似硅胶/软组织的小鼠，足以验证抓取时的接触力与形变。

**Known MVP limitations / MVP 已知局限:**

- No internal skeleton articulation (spine, tail, limbs move as one continuous soft mass). / 无内部骨骼铰接（脊椎、尾巴、四肢作为连续软块整体变形）。
- Deformable simulation requires **GPU** and a **watertight manifold mesh**; high-poly Seed3D output must be decimated. / 软体仿真依赖 **GPU**，且要求网格**水密、流形**；Seed3D 高面数模型需减面。
- PhysX generates **tetrahedral** simulation/collision meshes internally; triangle mesh from GLB is the input, not the sim mesh. / PhysX 会在内部自动生成**四面体**仿真/碰撞网格；GLB 三角网格仅为输入。
- Material is uniform (no differentiated fur/skin/muscle); tuning is empirical. / 材质为均匀软体（无法区分皮毛/皮肤/肌肉），参数需实验标定。

---

## MVP Track: Soft-Body Grasp Demo / MVP 路线：软体抓取 Demo

### Environment & Asset Prerequisites / 环境与资产准备

| Category / 类别 | Item / 项目 | Description / Requirement / 描述与要求 |
| :-------------- | :---------- | :--------------------------------------- |
| **Input Assets / 输入资产** | Reference Image / 参考图像 | 1× high-resolution, neutral-pose image of a laboratory mouse. / 1 张高分辨率、中性姿态、光照均匀的实验小鼠照片。 |
| **Model / 生成模型** | Seed3D 1.0 | Volcano Engine platform access for single-image-to-3D generation. / 火山引擎平台访问权限，用于单图生成 3D 资产。 |
| **Converter / 格式转换** | Blender (≥ 3.6) **or** Omniverse Asset Converter | GLB → USD conversion only (no rigging required). / 仅用于 GLB → USD 格式转换（无需绑骨）。 |
| **Simulation / 仿真平台** | NVIDIA Isaac Sim (≥ 4.x) | GPU-enabled; PhysX deformable body (FEM) support. / 需启用 GPU；支持 PhysX 软体 (FEM)。 |
| **Optional / 可选** | Isaac Lab | Python scripting for scene setup, robot control, and demo automation. / 用于场景搭建、机械臂控制和 Demo 自动化脚本。 |

---

### Phase 1: Static Geometry & Texture Generation (Seed3D) / 第一阶段：静态几何与纹理生成

**Goal / 目标:** Generate a watertight shell mesh and PBR textures from a single reference image. / 从单张参考图生成水密外壳网格与 PBR 纹理。

**Input / 输入:** `assets/input/mouse_reference.png`

**Output / 输出:** `assets/seed3d/mouse_static.glb` (+ texture maps)

#### Steps / 步骤

1. Upload / process the reference image through Seed3D 1.0. / 将参考图像输入 Seed3D 1.0 进行处理。
2. Generate the 3D asset and verify **watertight, manifold** geometry (no holes, no non-manifold edges). / 生成 3D 资产并验证**水密、流形**几何（无破洞、无非流形边）。
3. Extract PBR maps: albedo, metallic, roughness (and normal if available). / 提取 PBR 贴图：反照率、金属度、粗糙度（及法线贴图，如有）。
4. Export as `.glb` (preferred) or `.obj` + textures. / 导出为 `.glb`（推荐）或 `.obj` + 贴图。
5. **Quality gate / 质量门禁:** Open in Blender or MeshLab; confirm single connected mesh, reasonable topology, and scale roughly consistent with a real mouse (~7–10 cm body length). / 在 Blender 或 MeshLab 中打开；确认单一连通网格、拓扑合理、尺度与真实小鼠（体长约 7–10 cm）大致一致。

---

### Phase 2: Mesh Prep & GLB → USD Conversion / 第二阶段：网格预处理与 GLB → USD 转换

**Status / 状态:** Complete (Blender 5.1.2 headless) / 已完成（Blender 5.1.2 无头运行）

**Goal / 目标:** Produce a simulation-ready, decimated `.usd` asset with PBR materials preserved. / 产出可用于仿真的减面 `.usd` 资产，并保留 PBR 材质。

**Input / 输入:** `assets/seed3d/mouse_static.glb` (from `mesh_textured_pbr.glb`) / 源自 `mesh_textured_pbr.glb`

**Output / 输出:** `assets/usd/mouse_soft.usd`

**Current mesh stats (source GLB) / 当前源网格状态:**

| Metric / 指标 | Value / 值 |
| :------------ | :--------- |
| Vertices / 顶点 | 531,657 |
| Faces / 面 | 999,924 |
| Watertight / 水密 | No (~59k boundary edges) / 否 |
| UV / 纹理坐标 | Yes / 有 |
| Normalized max extent / 归一化最长轴 | ~2.0 units |

**Blender export result (`scripts/phase2_blender_prepare_mouse.py`) / Blender 导出结果:**

| Metric / 指标 | Value / 值 |
| :------------ | :--------- |
| Faces / 面 | 10,000 |
| Vertices / 顶点 | 5,002 |
| Watertight / 水密 | Yes / 是 |
| Real-world max extent / 真实最长轴 | 0.080 m (~8.0 cm) |
| Up axis / 向上轴 | Z (Isaac Sim ready) |
| Textures / 纹理 | `assets/usd/textures/` (PBR exported from Blender) |
| Prepared blend / 预处理工程 | `assets/usd/mouse_soft_prepared.blend` |

> **Note:** Prefer the Blender export over the headless fallback for simulation + rendering. / 仿真与渲染请优先使用 Blender 导出的 USD。

#### Steps / 步骤

1. **Import GLB into Blender.** / 将 GLB 导入 Blender。
2. **Inspect & clean mesh:** / **检查并清理网格：**
   - Merge duplicate vertices, remove loose geometry. / 合并重复顶点，删除游离几何体。
   - Confirm watertight / manifold (use Blender 3D Print Toolbox or MeshLab). / 确认水密/流形。
   - Apply uniform scale; set real-world dimensions (body length ≈ 0.07–0.10 m). / 应用统一缩放，设为真实尺寸（体长约 0.07–0.10 m）。
3. **Decimate for soft-body sim:** Target **5k–20k faces** (start conservative; increase only if GPU allows stable real-time sim). / **为软体仿真减面：** 目标 **5k–20k 面**（先从保守值开始，GPU 稳定后再酌情提高）。
   - Use `Decimate` modifier (Collapse); preserve UVs and PBR material slots. / 使用 `Decimate`（Collapse 模式）；保留 UV 与 PBR 材质槽。
4. **Optional:** Apply a slight `Smooth` or remesh only if decimation introduces visible artifacts. / **可选：** 仅在减面产生明显瑕疵时做轻微平滑或 Remesh。
5. **Export to USD:** / **导出 USD：**
   - **Option A (recommended):** Omniverse Blender Connector → export as `.usd` with materials. / **方案 A（推荐）：** Omniverse Blender 插件 → 导出带材质的 `.usd`。
   - **Option B:** Export `.glb` / `.fbx` and convert via Omniverse Asset Converter or Isaac Sim import pipeline. / **方案 B：** 导出 `.glb` / `.fbx`，经 Omniverse Asset Converter 或 Isaac Sim 导入管线转换。
6. **Verify USD:** Open in Isaac Sim or USD View; confirm single `Mesh` prim, materials bound, correct scale and orientation (Z-up vs Y-up — align with Isaac Sim convention). / **验证 USD：** 在 Isaac Sim 或 USD View 中打开；确认单一 `Mesh` prim、材质绑定正确、尺度与朝向符合 Isaac Sim 约定（Z-up / Y-up 对齐）。

#### Scripts / 脚本

| Script / 脚本 | Purpose / 用途 |
| :------------ | :------------- |
| `scripts/inspect_mesh.py` | Report vertex/face count, watertight status, bounds. / 输出网格统计与水密性报告。 |
| `scripts/phase2_export_usd.py` | Headless prep + USD export (no Blender). / 无 Blender 的减面/水密化/USD 导出。 |
| `scripts/phase2_blender_prepare_mouse.py` | **Recommended** — run inside Blender on `mouse.blend`; preserves PBR + UV via Decimate. / **推荐** — 在 Blender 中打开 `mouse.blend` 后运行；Decimate 保留 PBR 与 UV。 |

**Run Blender script / 运行 Blender 脚本:**

1. Open `mouse.blend` in Blender (≥ 3.6).
2. Scripting → Open `scripts/phase2_blender_prepare_mouse.py` → Run Script.
3. Check `assets/usd/phase2_mesh_report.json` and `assets/usd/mouse_soft.usd`.

---

### Phase 3: Soft-Body Physics Setup (Isaac Sim) / 第三阶段：软体物理配置 (Isaac Sim)

**Goal / 目标:** Configure the imported mesh as a volume deformable body with silicone-like material properties. / 将导入网格配置为体积软体，并赋予硅胶类材质参数。

**Input / 输入:** `assets/usd/mouse_soft.usd`

**Output / 输出:** `assets/usd/mouse_soft_deformable.usd` (or scene file `scenes/grasp_demo.usd`)

#### Steps / 步骤

1. **Create Isaac Sim project scene.** / **创建 Isaac Sim 项目场景。**
   - Enable **GPU dynamics** (Edit → Project Settings → Physics → GPU). / 启用 **GPU 动力学**。
   - Add ground plane, dome/HDRI lighting. / 添加地面、穹顶/HDRI 光照。
2. **Import mouse USD** at `/World/Mouse`. / 在 `/World/Mouse` 导入小鼠 USD。
3. **Apply Deformable Body schema** to the mesh prim (not the parent Xform): / **对 Mesh prim 应用 Deformable Body schema**（非父级 Xform）：
   - GUI: Select mesh → Add → Physics → Deformable Body. / GUI：选中 mesh → Add → Physics → Deformable Body。
   - Script (Isaac Lab): use `DeformableBodyPropertiesCfg` + `DeformableBodyMaterialCfg` on `UsdFileCfg`. / 脚本（Isaac Lab）：在 `UsdFileCfg` 上配置 `DeformableBodyPropertiesCfg` + `DeformableBodyMaterialCfg`。
4. **Tetrahedral mesh generation:** Let PhysX auto-tetrahedralize from the triangle mesh. / **四面体网格生成：** 由 PhysX 从三角网格自动四面体化。
   - Set `simulationHexahedralResolution` (or equivalent) — start with medium resolution; lower if sim is unstable or slow. / 设置 `simulationHexahedralResolution`（或等效参数）— 从中等分辨率起步；不稳定或过慢则降低。
   - PhysX maintains separate **simulation tet mesh** and **collision tet mesh** internally. / PhysX 内部分别维护**仿真四面体网格**与**碰撞四面体网格**。
5. **Silicone-like material tuning (starting points):** / **硅胶类材质参数（起始参考值）：**

   | Parameter / 参数 | Starting value / 起始值 | Notes / 说明 |
   | :--------------- | :---------------------- | :----------- |
   | Young's Modulus / 杨氏模量 | `1e4` – `5e5` Pa | Lower = softer; mouse tissue / silicone proxy. / 越低越软；近似小鼠软组织/硅胶。 |
   | Poisson's Ratio / 泊松比 | `0.45` – `0.49` | Near-incompressible soft solid. / 近似不可压缩软固体。 |
   | Density / 密度 | `950` – `1100` kg/m³ | Approximate water-like small animal mass. / 近似小型动物/含水组织密度。 |
   | Damping / 阻尼 | Tune until oscillation decays naturally | Avoid jitter or collapse. / 调参至振荡自然衰减，避免抖动或塌陷。 |

6. **Collision & contact:** Ensure deformable collision mesh interacts with rigid gripper fingers; set friction (`static` ≈ `0.6`–`1.0`, `dynamic` slightly lower) on both mouse and gripper. / **碰撞与接触：** 确保软体碰撞网格与刚性夹爪指面交互；为小鼠与夹爪设置摩擦系数（静摩擦 ≈ `0.6`–`1.0`，动摩擦略低）。
7. **Stability checks:** Press Play; mouse should rest on ground without exploding, inverting, or excessive sag. Reduce resolution or increase damping if unstable. / **稳定性检查：** 点击 Play；小鼠应稳定落于地面，无爆炸、翻面或过度塌陷。不稳定则降低分辨率或增大阻尼。

---

### Phase 4: Grasp Demo / 第四阶段：抓取 Demo

**Goal / 目标:** Demonstrate a robotic gripper approaching, closing on, lifting, and releasing the soft mouse with visible deformation and stable physics. / 演示机械臂夹爪接近、闭合、提起并释放软体小鼠，形变可见且物理稳定。

**Input / 输入:** `assets/usd/mouse_soft_deformable.usd`

**Output / 输出:** `scripts/grasp_demo.py` (or Isaac Sim Action Graph / standalone scene)

#### Scene setup / 场景搭建

1. **Robot:** Import a standard manipulator (e.g., Franka Panda + Robotiq 2F-85 or Isaac Sim built-in parallel gripper) from Isaac Sim / Isaac Lab assets. / 导入标准机械臂（如 Franka Panda + Robotiq 2F-85 或 Isaac Sim 内置平行夹爪）。
2. **Placement:** Mouse on table/ground; gripper starts above mouse centroid, approach axis roughly vertical or slightly angled. / 小鼠置于桌面/地面；夹爪初始位置在小鼠质心上方，接近方向大致垂直或微倾。
3. **Physics layers:** Gripper fingers = rigid bodies with collision; mouse = deformable body; table = static collider. / 物理分层：夹爪指 = 刚体碰撞；小鼠 = 软体；桌面 = 静态碰撞体。

#### Demo sequence / Demo 流程

| Step / 步骤 | Action / 动作 | Success indicator / 成功指标 |
| :---------- | :------------ | :--------------------------- |
| 1 | Move gripper to pre-grasp pose above mouse. / 夹爪移至小鼠上方预抓取位姿。 | No collision penetration at rest. / 静止无穿透。 |
| 2 | Lower gripper to grasp height. / 下降至抓取高度。 | Fingers close around mouse body. / 指面环绕小鼠躯干。 |
| 3 | Close gripper to target width (or force threshold). / 闭合夹爪至目标开度（或力阈值）。 | Visible soft deformation at contact; no sim blow-up. / 接触处可见软体形变；仿真不发散。 |
| 4 | Lift gripper ~5–10 cm. / 提起夹爪约 5–10 cm。 | Mouse lifts with gripper; moderate elastic sag. / 小鼠随夹爪抬起；有适度弹性下垂。 |
| 5 | Hold ~2 s, then open gripper. / 保持约 2 s 后张开夹爪。 | Mouse drops or rests stably; mesh recovers partially. / 小鼠落下或稳定放置；网格部分回弹。 |

#### Implementation options / 实现方式

- **Isaac Lab Python script (recommended):** `InteractiveScene` + robot joint-position / gripper control loop. / **Isaac Lab Python 脚本（推荐）：** `InteractiveScene` + 关节位置/夹爪控制循环。
- **Isaac Sim GUI + Action Graph:** Keyframed gripper motion for quick visual validation. / **Isaac Sim GUI + Action Graph：** 关键帧驱动夹爪，用于快速视觉验证。
- **ROS 2 bridge (optional, later):** If integrating with external motion planning. / **ROS 2 桥接（可选，后续）：** 与外部运动规划集成时使用。

---

### MVP Acceptance Criteria / MVP 验收标准

- [ ] **Geometry / 几何:** Seed3D mesh is watertight; PBR textures render correctly in Isaac Sim. / Seed3D 网格水密；PBR 材质在 Isaac Sim 中正确渲染。
- [ ] **Conversion / 转换:** GLB → USD pipeline produces a single-mesh asset at real-world scale (~7–10 cm body length). / GLB → USD 流程产出单一网格资产，尺度为真实尺寸（体长约 7–10 cm）。
- [ ] **Soft-body / 软体:** Mouse deforms under gripper contact; no catastrophic clipping, inversion, or simulation explosion. / 小鼠在夹爪接触下发生形变；无灾难性穿透、翻面或仿真爆炸。
- [ ] **Grasp demo / 抓取 Demo:** Gripper can grasp, lift, and release the mouse in a repeatable scripted sequence. / 夹爪可在脚本化流程中 repeatable 地完成抓取、提起与释放。
- [ ] **Performance / 性能:** Simulation runs at usable frame rate on target GPU (≥ 30 FPS target for demo). / 在目标 GPU 上以可用帧率运行（Demo 目标 ≥ 30 FPS）。

---

## Full Track (Deferred): Articulated Rigging Pipeline / 完整路线（后续）：铰接体绑定流水线

> The following phases are **deferred** until the MVP soft-body grasp demo is validated. / 以下阶段在 MVP 软体抓取 Demo 验证通过后再推进。

### Phase F1: Articulation & Rigging (Blender) / 骨骼绑定 (Blender)

**Input / 输入:** `mouse_static.glb`

**Output / 输出:** `mouse_rigged.usd`

1. Import & clean mesh (decimate if needed). / 导入并清理网格（必要时减面）。
2. Build armature: Spine_1–3, Tail_1–4, limbs, head. / 搭建骨骼：Spine_1–3、Tail_1–4、四肢、头部。
3. Weight painting for natural joint deformation. / 权重绘制以实现自然关节形变。
4. Export rigged model via Omniverse Exporter. / 通过 Omniverse 导出绑定模型。

### Phase F2: Articulated Physics (Isaac Sim) / 铰接体物理 (Isaac Sim)

**Input / 输入:** `mouse_rigged.usd`

1. Import & scale to real-world dimensions. / 导入并缩放至真实尺寸。
2. Configure as `Articulation Root`; set joint limits, stiffness, damping. / 配置为 `Articulation Root`；设置关节限位、刚度、阻尼。
3. Generate collision meshes; apply surface friction materials. / 生成碰撞网格；设置表面摩擦材质。
4. Advanced grasp demo with articulated body response. / 带铰接体响应的高级抓取 Demo。

### Full Track Acceptance Criteria / 完整版验收标准

- [ ] **Kinematic / 运动学:** Root bone moves entire mesh; individual joint rotation causes localized smooth deformation. / 根骨骼带动全身；单关节旋转引起局部平滑形变。
- [ ] **Physics / 物理:** Robotic end-effector applies force without unnatural collapse or clipping. / 机械臂末端施加力时无反常塌陷或穿透。

---

## Project Structure (Suggested) / 建议项目结构

```
Project_mouse/
├── plan.md
├── assets/
│   ├── input/
│   │   └── mouse_reference.png
│   ├── seed3d/
│   │   └── mouse_static.glb
│   └── usd/
│       ├── mouse_soft.usd
│       └── mouse_soft_deformable.usd
├── scenes/
│   └── grasp_demo.usd
└── scripts/
    └── grasp_demo.py
```

---

## Risk Register & Mitigations / 风险与应对

| Risk / 风险 | Impact / 影响 | Mitigation / 应对 |
| :---------- | :------------ | :---------------- |
| Seed3D mesh not watertight / Seed3D 网格不水密 | Soft-body tet generation fails / 四面体生成失败 | Remesh in Blender (voxel remesh / manual fix); re-run Seed3D with different settings. / Blender 中 Remesh 或手修；调整 Seed3D 参数重生成。 |
| High poly count / 面数过高 | GPU OOM, low FPS, unstable FEM / GPU 显存溢出、低帧率、FEM 不稳定 | Aggressive decimation to 5k–20k faces before USD export. / 导出 USD 前减面至 5k–20k。 |
| Soft body too stiff or too floppy / 软体过硬或过软 | Unrealistic grasp feel / 抓取手感不真实 | Sweep Young's modulus (`1e3`–`1e6`); validate visually against reference video. / 扫描杨氏模量范围；对照参考视频目视验证。 |
| Gripper penetration / 夹爪穿透 | Demo failure / Demo 失败 | Reduce approach speed; increase deformable contact offset; tune collision tet resolution. / 降低接近速度；增大软体接触偏移；调节碰撞四面体分辨率。 |
| GLB → USD material loss / 材质丢失 | Wrong appearance / 外观错误 | Prefer Omniverse Blender Connector; manually re-bind textures in USD if needed. / 优先用 Omniverse 插件；必要时在 USD 中手动重绑贴图。 |

---

## Next Actions / 下一步行动

1. [x] Seed3D → `mesh_textured_pbr.glb` / `assets/seed3d/mouse_static.glb`. / 已完成。
2. [x] Blender export → `assets/usd/mouse_soft.usd` (10k faces, watertight, Z-up, PBR textures). / Blender 导出已完成。
3. [ ] Verify `mouse_soft.usd` in Isaac Sim (scale, orientation, texture). / 在 Isaac Sim 中验证 USD。
4. [x] Phase 3+4 script — `scripts/grasp_demo.py` (soft body + Factory Franka grasp sequence). / 脚本已就绪。
5. [ ] Tune deformable material / arm poses after visual review. / 目视检查后调参。
