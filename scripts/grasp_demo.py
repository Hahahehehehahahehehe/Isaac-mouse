"""
Mouse soft-body grasp demo — Isaac Sim 5.1 (PhysX deformable + Factory Franka).

Loads ``assets/usd/mouse_soft.usd``, configures volume deformable (silicone-like),
spawns Factory Franka, places the mouse on the table, then drives the arm with
regression-tested joint targets (Project_Issac soft-block side pinch) → close → lift.

GUI: after a grasp cycle completes, press **R** to replay. Close the window to exit.

Run with Project_Issac venv (Isaac Sim already installed there):

  $env:OMNI_KIT_ACCEPT_EULA = "YES"
  & "D:\\Labworks\\Project_Issac\\.venv-isaacsim\\Scripts\\python.exe" scripts\\grasp_demo.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MOUSE_USD = PROJECT_ROOT / "assets" / "usd" / "mouse_soft.usd"
BAKED_MOUSE_USD = PROJECT_ROOT / "assets" / "usd" / "mouse_soft_baked.usda"
SCENE_OUT = PROJECT_ROOT / "scenes" / "grasp_demo.usd"
MOUSE_SETTLE_STEPS = 60
MOUSE_DEBUG_ROOT = "/World/DebugMouse"
RIGID_MOUSE_PROBE = "/World/RigidMouseProbe"
# Measured live collision hull extent (m) — used for rigid-box control probe.
MOUSE_PROBE_EXT_XYZ = (0.0433, 0.0795, 0.0252)
# Render mesh (white PBR) = /World/MouseAsset/.../mesh_001 — PhysX skins it from the FEM *sim* mesh.
COLLISION_DEBUG_COLOR = (1.0, 0.45, 0.0)   # orange — PhysX collision tet hull
SIM_DEBUG_COLOR = (0.1, 0.75, 1.0)         # cyan — FEM simulation nodes (optional)
COLLISION_TET_OPACITY = 0.50               # match green gripper collider style (render purpose)
COLLISION_WIRE_WIDTH = 0.008               # orange wireframe edges — visible in RTX viewport
COLLISION_BBOX_WIRE_WIDTH = 0.012          # bright AABB box — impossible to miss
RENDER_MESH_DEBUG_OPACITY = 0.12           # dim PBR skin via DisplayOpacity (not material override)
DEBUG_POINT_WIDTH_COLL = 0.003   # keep small; dense node cloud is misleading
DEBUG_POINT_WIDTH_SIM = 0.002
SHOW_DEBUG_NODE_CLOUDS = False   # hide SimNodes/CollisionNodes — use CollisionTetMesh instead
SHOW_COLLISION_WIRE = True       # always draw live collision-tet edges during simulation

FRANKA_USD_REL = "/Isaac/Robots/FrankaRobotics/FactoryFranka/factory_franka.usd"
FRANKA_ROOT = "/World/FactoryFranka"
HAND_LINK = f"{FRANKA_ROOT}/panda_hand"
LEFT_FINGER_LINK = f"{FRANKA_ROOT}/panda_leftfinger"
RIGHT_FINGER_LINK = f"{FRANKA_ROOT}/panda_rightfinger"
GRIPPER_DEBUG_ROOT = "/World/DebugGripper"
GRIPPER_COLLIDER_COLOR = (0.15, 0.95, 0.25)   # green — PhysX rigid colliders on the gripper
CONTACT_POINT_COLOR = (1.0, 0.05, 0.05)       # red — live PhysX contact points
CONTACT_POINT_WIDTH = 0.008

# Joint targets (degrees) — side-pinch grasp: straddle mouse width (X-axis) then lift.
ARM_HOME_DEG = (0.0, -30.0, 0.0, -100.0, 0.0, 55.0, 45.0)
# Pre-grasp hover: hand above mouse (~12 cm), gripper wide open, ready to descend.
ARM_PRE_GRASP_DEG = (25.0, -25.0, -15.0, -110.0, 10.0, 60.0, 30.0)
# Straddle pose: hand lowered to mouse body level, open fingers flanking the mouse sides.
ARM_OVER_MOUSE_DEG = (25.0, -40.0, -15.0, -95.0, 10.0, 60.0, 30.0)
ARM_LIFT_DEG = (22.0, -48.0, -12.0, -102.0, 10.0, 78.0, 28.0)
# Standard Franka finger convention: 0.0 = fully closed, 0.04 = fully open (40 mm each side).
GRIPPER_OPEN_M = 0.04    # fingers wide open (80 mm total) — used during approach & straddle
# Fixed clamp spacing used for the entire pinch+lift (per finger). 12 mm × 2 = 24 mm total —
# well below the ~43 mm FEM collision hull so the soft mouse compresses between the pads.
GRIPPER_CLAMP_M = 0.012

# Table top ~18 cm — matches Factory Franka reach for the soft-block grasp pose.
TABLE_SCALE = (0.22, 0.18, 0.05)
TABLE_TOP_Z = 0.18
TABLE_TRANSLATE = (0.52, 0.35, TABLE_TOP_Z - TABLE_SCALE[2] * 0.5)

# Mouse mesh bounds (see assets/usd/phase2_mesh_report.json): head-tail ~Y (0.08 m), width ~X (0.042 m).
MOUSE_MESH_HALF_HEIGHT_M = 0.012567
MOUSE_MESH_BODY_LENGTH_M = 0.080008
MOUSE_MESH_BODY_WIDTH_M = 0.042416

# Keep head–tail along +Y on the table (natural mesh orientation).
MOUSE_ROTATE_Z_DEG = 0.0
MOUSE_BOTTOM_CONTACT_TRI_PCT = 50.0
MOUSE_BOTTOM_CONTACT_NODAL_PCT = 10.0

MOUSE_TRANSLATE = (TABLE_TRANSLATE[0], TABLE_TRANSLATE[1], TABLE_TOP_Z + MOUSE_MESH_HALF_HEIGHT_M)

GRASP_HAND_CLOSE_DIST_M = 0.10
GRASP_HAND_HEIGHT_TOL_M = 0.08
ARM_DRIVE_SETTLE_STEPS = 45

# Grasp verification / replay timing (@ 60 Hz).
GRASP_LIFT_DELTA_M = 0.045
REPLAY_KEY = "R"
MOUSE_ROOT = "/World/MouseAsset"

# Silicone-like deformable material (Pa).
YOUNG_MODULUS = 5.0e4
POISSON_RATIO = 0.48
DEFORMABLE_FRICTION = 0.8
DEFORMABLE_DAMPING = 0.005
SIM_HEX_RESOLUTION = 8
MOUSE_GRAVITY_SETTLE_STEPS = 90

# Pre-sink visual mesh below table surface to compensate for PhysX FEM collision hull expansion.
# PhysX tetrahedralises the mesh with a slight outward offset; pre-sinking by this amount
# makes the collision hull bottom land flush on TABLE_TOP_Z after cooking.
DEFORMABLE_FLOOR_SINK_M = 0.003  # 3 mm — tune if mouse still floats after rebake

# PhysX collision offsets (physxCollision:* on mesh/collider prims).
# PhysX default contactOffset is often ~20 mm — can trigger rigid↔deformable coupling
# before the orange hull geometrically touches (see debug.md Step 4b).
DEFAULT_DEFORM_CONTACT_OFFSET_M = 0.002   # 2 mm
DEFAULT_DEFORM_REST_OFFSET_M = 0.0
DEFAULT_FINGER_CONTACT_OFFSET_M = 0.001  # 1 mm
DEFAULT_FINGER_REST_OFFSET_M = 0.0
DEFAULT_FINGER_KP = 2000.0   # match USD linear drive — higher values eject gripper at deformable contact
DEFAULT_FINGER_KD = 100.0
DEFAULT_FINGER_MAX_FORCE_N = 8000.0
GRIPPER_PINCH_CLOSE_CREEP_M = 0.0002  # max per-finger close step per frame when blocked (0.2 mm)
PINCH_AUTO_MOUSE_FILTER = True  # filter=none: disable gripper↔mouse GPU contact during pinch (prevents ejection)

# Physics steps between key poses (@ 60 Hz).
STEPS_APPROACH = 120   # home → hover above mouse (gripper open)
STEPS_DESCEND  = 220   # hover → straddle (descend to grasp level, gripper still open)
LIFT_DURATION_STEPS = 180
MAX_GRASP_STEPS = 900
GRIPPER_CLOSE_STEPS = 60          # ramp command open → GRIPPER_CLAMP_M
GRIPPER_SQUEEZE_STEPS = 45        # keep forcing target after ramp (soft body compression)
GRIPPER_CLAMP_HOLD_STEPS = 120    # hold forced clamp at straddle height before lift (~2 s)
GRASP_CHECK_START = STEPS_DESCEND + GRIPPER_CLOSE_STEPS + 20

# Step4 force/compliant pinch — keep gripper↔mouse contact (filter=none).
GRIPPER_PINCH_EFFORT_N = 20.0           # closing actuation force per finger (N)
GRIPPER_PINCH_EFFORT_MAX_N = 35.0       # PhysX max force cap per finger during effort pinch
GRIPPER_FORCE_PINCH_MAX_STEPS = 240     # max frames to close before giving up (~4 s)
GRIPPER_COMPLIANT_KP = 80.0
GRIPPER_COMPLIANT_KD = 15.0
GRIPPER_COMPLIANT_MAX_FORCE = 30.0
GRIPPER_COMPLIANT_CREEP_M = 0.00025     # per-frame position target step toward closed (per finger)
PINCH_STALL_VEL_MM_S = 0.08             # finger joint speed below this => blocked
PINCH_STALL_FRAMES = 20                 # consecutive stalled frames before hold

# Grip attachment: once clamped, FEM nodes near the fingers are kinematically carried with the
# hand during the lift. A parallel gripper cannot lift a soft body by friction alone (the sibling
# Project_Issac demo uses the same attachment trick), so this makes the mouse rise with the gripper.
GRASP_ATTACH_RADIUS_M = 0.055  # capture most of the mouse body (was 30 mm → too few nodes)
GRASP_ATTACH_BOX_SCALE = 0.55    # fraction of MOUSE_PROBE_EXT_XYZ half-extents for box capture
_grasp_attach_indices = None
_grasp_attach_offsets = None
_grasp_attach_rest_pos = None
_grasp_attach_grip_center: tuple[float, float, float] | None = None
_pinch_mouse_filter_added = False
# Release kinematic pin BEFORE gripper closes so the mouse can physically respond to the fingers.
MOUSE_PIN_UNTIL_STEP = STEPS_DESCEND
MOUSE_PIN_SETTLE_STEPS = 24
_mouse_pin_targets = None

# Top-down (vertical) grasp: gripper approach axis points straight down (world -Z).
# This quaternion (x, y, z, w) rotates panda_hand so its local +Z (approach/finger direction)
# maps to world -Z, and the finger-separation axis (local Y) maps to world +X (mouse width ~42 mm).
# Derived: 180° rotation about the (1,1,0)/√2 axis. If fingers separate along the mouse LENGTH
# instead (wrong axis), switch to (1.0, 0.0, 0.0, 0.0) for a 180° flip about X.
GRASP_DOWN_QUAT_XYZW = (0.7071068, 0.7071068, 0.0, 0.0)

# IK drives the panda_hand link, whose fingertips (TCP) sit ~0.10 m further along the approach axis.
# For a vertical descent we must hold panda_hand this far ABOVE the grasp point so the fingertips
# reach the mouse instead of driving the wrist into the table.
GRASP_TCP_OFFSET_M = 0.104

# Cartesian IK target heights for panda_hand, relative to the mouse center:
#   straddle = fingertips bracket the mouse body (TCP at mouse center)
#   hover    = clearance above straddle before descending
#   lift     = raise after the pinch
GRASP_HOVER_Z_OFFSET_M = GRASP_TCP_OFFSET_M + 0.12
GRASP_STRADDLE_Z_OFFSET_M = GRASP_TCP_OFFSET_M
GRASP_LIFT_Z_OFFSET_M = GRASP_TCP_OFFSET_M + 0.12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mouse soft-body grasp demo in Isaac Sim.")
    parser.add_argument("--headless", action="store_true", help="Run without GUI.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Extra idle seconds after the scripted sequence (0 = GUI runs until closed).",
    )
    parser.add_argument(
        "--mouse-usd",
        type=Path,
        default=MOUSE_USD,
        help="Path to mouse USD asset.",
    )
    parser.add_argument(
        "--export-scene",
        type=Path,
        default=SCENE_OUT,
        help="Export composed stage to this USD path.",
    )
    parser.add_argument("--no-export", action="store_true", help="Skip USD export.")
    parser.add_argument(
        "--show-fem-nodes",
        action="store_true",
        help="Also show cyan SimNodes + orange CollisionNodes point clouds (hidden by default).",
    )
    parser.add_argument(
        "--hide-collision-debug",
        action="store_true",
        help="Hide orange collision / cyan FEM debug overlays (visible by default).",
    )
    parser.add_argument(
        "--show-gripper-contacts",
        action="store_true",
        help="Enable PhysX RigidContactView (red contact points; can be noisy on GPU).",
    )
    parser.add_argument(
        "--hide-gripper-debug",
        action="store_true",
        help="Hide green gripper colliders + red PhysX contact points (shown by default).",
    )
    parser.add_argument(
        "--rebake",
        action="store_true",
        help="Delete mouse_soft_baked.usda and re-bake mesh placement from mouse_soft.usd.",
    )
    parser.add_argument(
        "--collision-filter",
        choices=("none", "hand", "mouse", "all"),
        default="none",
        help="Which collision pairs to disable via FilteredPairsAPI (default: none — tuned offsets). "
        "Use 'all' to reproduce filter workaround; 'none' for full contact.",
    )
    parser.add_argument(
        "--diagnose-pinch",
        action="store_true",
        help="At pinch end: print detailed blocker report (geometry, efforts, hand contact) "
        "then exit without lift. Run with each --collision-filter mode to isolate the blocker.",
    )
    parser.add_argument(
        "--mouse-mode",
        choices=("deformable", "rigid-box"),
        default="deformable",
        help="deformable=soft FEM mouse (default); rigid-box=Step1 kinematic box control (see debug.md).",
    )
    parser.add_argument(
        "--pinch-mode",
        choices=("position", "force", "compliant"),
        default="position",
        help="Pinch control: position=stiff position ramp (default); "
        "force=effort-mode closing force (Step4, use with --collision-filter none); "
        "compliant=low-gain position creep.",
    )
    parser.add_argument(
        "--no-contact-offset-tuning",
        action="store_true",
        help="Leave PhysX default contact/rest offsets (baseline A/B vs tuned offsets).",
    )
    parser.add_argument(
        "--deform-contact-offset-mm",
        type=float,
        default=DEFAULT_DEFORM_CONTACT_OFFSET_M * 1000.0,
        help=f"Mouse FEM mesh physxCollision:contactOffset in mm (default {DEFAULT_DEFORM_CONTACT_OFFSET_M*1000:.0f}).",
    )
    parser.add_argument(
        "--deform-rest-offset-mm",
        type=float,
        default=DEFAULT_DEFORM_REST_OFFSET_M * 1000.0,
        help="Mouse FEM mesh physxCollision:restOffset in mm (default 0).",
    )
    parser.add_argument(
        "--finger-contact-offset-mm",
        type=float,
        default=DEFAULT_FINGER_CONTACT_OFFSET_M * 1000.0,
        help=f"Gripper finger/hand collider contactOffset in mm (default {DEFAULT_FINGER_CONTACT_OFFSET_M*1000:.0f}).",
    )
    parser.add_argument(
        "--finger-rest-offset-mm",
        type=float,
        default=DEFAULT_FINGER_REST_OFFSET_M * 1000.0,
        help="Gripper collider restOffset in mm (default 0).",
    )
    return parser.parse_args()


def pose_to_targets(robot, arm_deg: tuple[float, ...], gripper_m: float):
    import numpy as np

    targets = np.zeros(robot.num_dof, dtype=np.float32)
    name_map = {f"panda_joint{i}": math.radians(arm_deg[i - 1]) for i in range(1, 8)}
    name_map["panda_finger_joint1"] = gripper_m
    name_map["panda_finger_joint2"] = gripper_m

    for idx, name in enumerate(robot.dof_names):
        for key, value in name_map.items():
            if key in name:
                targets[idx] = value
                break
    return targets


def configure_franka_drives(stage, arm_deg: tuple[float, ...], gripper_m: float) -> None:
    """Configure position drives on Factory Franka (must run before simulation play)."""
    from pxr import UsdPhysics

    revolute = {"stiffness": 4000.0, "damping": 400.0, "maxForce": 4000.0}
    linear = {"stiffness": 2000.0, "damping": 200.0, "maxForce": DEFAULT_FINGER_MAX_FORCE_N}

    for i in range(1, 8):
        joint_path = f"{FRANKA_ROOT}/panda_link{i - 1}/panda_joint{i}"
        _set_position_drive(stage, joint_path, arm_deg[i - 1], revolute, angular=True)

    for finger in ("panda_finger_joint1", "panda_finger_joint2"):
        _set_position_drive(stage, f"{HAND_LINK}/{finger}", gripper_m, linear, angular=False)


def update_franka_drives(stage, arm_deg: tuple[float, ...], gripper_m: float) -> None:
    """Keep USD drive targets in sync with articulation tensor commands."""
    configure_franka_drives(stage, arm_deg, gripper_m)


def _set_position_drive(stage, joint_path: str, target: float, params: dict, *, angular: bool) -> None:
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise RuntimeError(f"Robot joint prim not found: {joint_path}")

    drive_type = "angular" if angular else "linear"
    drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
    drive.CreateTargetPositionAttr(math.radians(target) if angular else target)
    drive.CreateStiffnessAttr(params["stiffness"])
    drive.CreateDampingAttr(params["damping"])
    drive.CreateMaxForceAttr(params["maxForce"])
    drive.CreateTypeAttr().Set("force")


def force_gripper_spacing(robot, finger_m: float) -> None:
    """Drive both finger joints toward ``finger_m`` via GPU tensor targets only."""
    import torch

    if not robot.handles_initialized or not robot._articulation_view.is_physics_handle_valid():
        return
    view = robot._articulation_view
    device = view._device
    pos = robot.get_joint_positions()
    if pos is None:
        return
    if torch.is_tensor(pos):
        pos = pos.detach().clone()
    else:
        pos = torch.as_tensor(pos, dtype=torch.float32, device=device).clone()
    if pos.dim() == 1:
        pos = pos.unsqueeze(0)
    pos[..., 7] = finger_m
    pos[..., 8] = finger_m
    view.set_joint_position_targets(pos)


def apply_frozen_arm_gripper(robot, arm_q, gripper_m: float, *, soft_close: bool = False) -> None:
    """Hold the 7 arm joints fixed; drive only the parallel gripper (pinch phase)."""
    import torch

    if not robot.handles_initialized or not robot._articulation_view.is_physics_handle_valid():
        return
    device = robot._articulation_view._device
    if torch.is_tensor(arm_q):
        arm = arm_q.detach().clone().reshape(-1)[:7]
    else:
        arm = torch.as_tensor(arm_q, dtype=torch.float32, device=device).reshape(-1)[:7]
    targets = torch.zeros(9, dtype=torch.float32, device=device)
    targets[:7] = arm
    if soft_close:
        pos = robot.get_joint_positions()
        if pos is not None:
            if torch.is_tensor(pos):
                cur = pos.detach().clone().reshape(-1)
            else:
                cur = torch.as_tensor(pos, dtype=torch.float32, device=device).reshape(-1)
            creep = GRIPPER_PINCH_CLOSE_CREEP_M
            for idx in (7, 8):
                c = float(cur[idx])
                targets[idx] = max(gripper_m, c - creep) if c > gripper_m + 1e-6 else gripper_m
        else:
            targets[7] = gripper_m
            targets[8] = gripper_m
    else:
        targets[7] = gripper_m
        targets[8] = gripper_m
    robot._articulation_view.set_joint_position_targets(targets.unsqueeze(0))


def apply_frozen_arm_only(robot, arm_q) -> None:
    """Hold arm joints fixed without overwriting finger actuation (effort pinch)."""
    import torch

    if not robot.handles_initialized or not robot._articulation_view.is_physics_handle_valid():
        return
    view = robot._articulation_view
    device = view._device
    pos = robot.get_joint_positions()
    if pos is None:
        return
    if torch.is_tensor(pos):
        targets = pos.detach().clone()
    else:
        targets = torch.as_tensor(pos, dtype=torch.float32, device=device).clone()
    if targets.dim() == 1:
        targets = targets.unsqueeze(0)
    if torch.is_tensor(arm_q):
        arm = arm_q.detach().clone().reshape(-1)[:7]
    else:
        arm = torch.as_tensor(arm_q, dtype=torch.float32, device=device).reshape(-1)[:7]
    targets[..., :7] = arm
    view.set_joint_position_targets(targets)


def get_finger_joint_state(robot) -> tuple[float, float, float, float] | None:
    """Return (f1_m, f2_m, v1_m_s, v2_m_s) or None."""
    import torch

    if not robot.handles_initialized or not robot._articulation_view.is_physics_handle_valid():
        return None
    pos = robot.get_joint_positions()
    if pos is None:
        return None
    if torch.is_tensor(pos):
        pos = pos.detach().cpu().numpy()
    pos = pos.reshape(-1)
    if pos.shape[0] < 9:
        return None
    v1 = v2 = 0.0
    try:
        vel = robot.get_joint_velocities()
        if vel is not None:
            if torch.is_tensor(vel):
                vel = vel.detach().cpu().numpy()
            vel = vel.reshape(-1)
            if vel.shape[0] >= 9:
                v1, v2 = float(vel[7]), float(vel[8])
    except Exception:
        pass
    return float(pos[7]), float(pos[8]), v1, v2


def enable_gripper_force_pinch(robot) -> None:
    """Switch finger joints to effort control for compliant closing against deformable contact."""
    import torch

    view = robot._articulation_view
    finger_idx = torch.tensor([7, 8], dtype=torch.int32, device=view._device)
    view.switch_control_mode("effort", joint_indices=finger_idx)
    max_eff = torch.tensor(
        [[4000.0] * 7 + [GRIPPER_PINCH_EFFORT_MAX_N, GRIPPER_PINCH_EFFORT_MAX_N]],
        dtype=torch.float32,
        device=view._device,
    )
    view.set_max_efforts(max_eff)
    print(
        f"[grasp_demo] force pinch: effort mode, {GRIPPER_PINCH_EFFORT_N:.0f} N/finger "
        f"(max {GRIPPER_PINCH_EFFORT_MAX_N:.0f} N)",
        flush=True,
    )


def apply_gripper_closing_effort(robot, effort_n: float = GRIPPER_PINCH_EFFORT_N, *, sign: float = -1.0) -> None:
    """Apply inward closing effort on both finger joints."""
    import torch

    if not robot.handles_initialized or not robot._articulation_view.is_physics_handle_valid():
        return
    view = robot._articulation_view
    device = view._device
    signed = sign * abs(effort_n)
    efforts = torch.zeros(9, dtype=torch.float32, device=device)
    efforts[7] = signed
    efforts[8] = signed
    view.set_joint_efforts(efforts.unsqueeze(0))


def enable_gripper_compliant_pinch(robot, stage=None) -> None:
    """Low-gain position drives so fingers creep closed instead of fighting the solver."""
    import torch

    view = robot._articulation_view
    finger_idx = torch.tensor([7, 8], dtype=torch.int32, device=view._device)
    view.switch_control_mode("position", joint_indices=finger_idx)
    kps = torch.tensor([[GRIPPER_COMPLIANT_KP, GRIPPER_COMPLIANT_KP]], dtype=torch.float32, device=view._device)
    kds = torch.tensor([[GRIPPER_COMPLIANT_KD, GRIPPER_COMPLIANT_KD]], dtype=torch.float32, device=view._device)
    view.set_gains(kps=kps, kds=kds, joint_indices=finger_idx)
    if stage is not None:
        compliant = {
            "stiffness": GRIPPER_COMPLIANT_KP,
            "damping": GRIPPER_COMPLIANT_KD,
            "maxForce": GRIPPER_COMPLIANT_MAX_FORCE,
        }
        for finger in ("panda_finger_joint1", "panda_finger_joint2"):
            _set_position_drive(stage, f"{HAND_LINK}/{finger}", GRIPPER_OPEN_M, compliant, angular=False)
    print(
        f"[grasp_demo] compliant pinch: kp={GRIPPER_COMPLIANT_KP:.0f}, "
        f"maxForce={GRIPPER_COMPLIANT_MAX_FORCE:.0f} N, creep={GRIPPER_COMPLIANT_CREEP_M*1000:.2f} mm/frame",
        flush=True,
    )


def restore_gripper_position_hold(robot, finger_m: float) -> None:
    """Switch fingers back to stiff position control for lift (after effort/compliant pinch)."""
    import torch

    view = robot._articulation_view
    finger_idx = torch.tensor([7, 8], dtype=torch.int32, device=view._device)
    view.switch_control_mode("position", joint_indices=finger_idx)
    kps = torch.tensor([[DEFAULT_FINGER_KP, DEFAULT_FINGER_KP]], dtype=torch.float32, device=view._device)
    kds = torch.tensor([[DEFAULT_FINGER_KD, DEFAULT_FINGER_KD]], dtype=torch.float32, device=view._device)
    view.set_gains(kps=kps, kds=kds, joint_indices=finger_idx)


def pinch_stalled(
    opening_history: list[float],
    v1_m_s: float,
    v2_m_s: float,
    *,
    min_frames: int = PINCH_STALL_FRAMES,
) -> bool:
    """True when opening stopped shrinking and finger joints are nearly static."""
    if len(opening_history) < min_frames:
        return False
    recent = opening_history[-min_frames:]
    if max(recent) - min(recent) > 0.0005:  # 0.5 mm total spread
        return False
    return abs(v1_m_s) * 1000.0 < PINCH_STALL_VEL_MM_S and abs(v2_m_s) * 1000.0 < PINCH_STALL_VEL_MM_S


def add_collision_filter(stage, path_a: str, path_b: str) -> bool:
    """Disable PhysX contacts between two prim subtrees (FilteredPairsAPI)."""
    from pxr import Sdf, UsdPhysics

    pa = stage.GetPrimAtPath(path_a)
    pb = stage.GetPrimAtPath(path_b)
    if not pa.IsValid() or not pb.IsValid():
        return False
    api = UsdPhysics.FilteredPairsAPI.Apply(pa)
    rel = api.GetFilteredPairsRel()
    if not rel:
        rel = api.CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(path_b))
    return True


def add_collision_filter_pair(stage, path_a: str, path_b: str) -> bool:
    """Apply filter in both directions (GPU articulations sometimes need reciprocal entries)."""
    ok_fwd = add_collision_filter(stage, path_a, path_b)
    ok_rev = add_collision_filter(stage, path_b, path_a)
    return ok_fwd and ok_rev


def apply_physx_collision_offsets(
    stage,
    prim_path: str,
    contact_m: float,
    rest_m: float,
    *,
    label: str = "",
) -> bool:
    """Set physxCollision:contactOffset / restOffset on a prim (deformable mesh or rigid collider)."""
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return False
    if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api = PhysxSchema.PhysxCollisionAPI(prim)
    api.CreateContactOffsetAttr(float(contact_m))
    api.CreateRestOffsetAttr(float(rest_m))
    tag = f"{label} " if label else ""
    print(
        f"[grasp_demo] {tag}physxCollision @ {prim_path}: "
        f"contactOffset={contact_m * 1000:.2f} mm, restOffset={rest_m * 1000:.2f} mm",
        flush=True,
    )
    return True


def read_physx_collision_offsets(stage, prim_path: str) -> tuple[float, float] | None:
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        return None
    api = PhysxSchema.PhysxCollisionAPI(prim)
    co = api.GetContactOffsetAttr().Get()
    ro = api.GetRestOffsetAttr().Get()
    if co is None and ro is None:
        return None
    co_f = float(co) if co is not None else float("nan")
    ro_f = float(ro) if ro is not None else float("nan")
    if not math.isfinite(co_f):
        co_f = float("nan")
    if not math.isfinite(ro_f):
        ro_f = float("nan")
    return co_f, ro_f


def set_gripper_collider_offsets(stage, contact_m: float, rest_m: float) -> int:
    """Apply contact/rest offsets to all finger/hand CollisionAPI prims under Franka."""
    from pxr import Usd, UsdPhysics

    count = 0
    root = stage.GetPrimAtPath(FRANKA_ROOT)
    if not root.IsValid():
        return 0
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath()).lower()
        if not any(k in path for k in ("finger", "hand", "gripper")):
            continue
        if apply_physx_collision_offsets(
            stage, str(prim.GetPath()), contact_m, rest_m, label="gripper"
        ):
            count += 1
    print(f"[grasp_demo] gripper collider offset pass: {count} prim(s)", flush=True)
    return count


def collision_filter_pairs(mode: str, collision_path: str) -> tuple[tuple[str, str], ...]:
    """Return collision pair paths to filter for the given isolation mode."""
    hand = (
        (HAND_LINK, LEFT_FINGER_LINK),
        (HAND_LINK, RIGHT_FINGER_LINK),
    )
    mouse = (
        (LEFT_FINGER_LINK, collision_path),
        (RIGHT_FINGER_LINK, collision_path),
        (HAND_LINK, collision_path),
    )
    if mode == "none":
        return ()
    if mode == "hand":
        return hand
    if mode == "mouse":
        return mouse
    return hand + mouse


def mouse_collision_path(mesh_path: str, mouse_mode: str) -> str:
    return RIGID_MOUSE_PROBE if mouse_mode == "rigid-box" else mesh_path


def setup_gripper_collision_filters(stage, collision_path: str, mode: str = "all") -> None:
    """Optionally disable contact between gripper subtrees and the mouse collision target."""
    pairs = collision_filter_pairs(mode, collision_path)
    ok = 0
    for a, b in pairs:
        if add_collision_filter_pair(stage, a, b):
            ok += 1
    print(
        f"[grasp_demo] collision filters ({mode}): {ok}/{len(pairs)} pair(s) disabled — see debug.md",
        flush=True,
    )


def apply_robot_pose(
    robot,
    arm_deg: tuple[float, ...],
    gripper_m: float,
    *,
    stage=None,
) -> None:
    """Drive the arm via GPU-compatible PD position targets."""
    import torch

    targets = pose_to_targets(robot, arm_deg, gripper_m)
    device = robot._articulation_view._device
    targets_t = torch.as_tensor(targets, dtype=torch.float32, device=device).unsqueeze(0)
    robot._articulation_view.set_joint_position_targets(targets_t)


def setup_robot_gains(robot) -> None:
    """High-gain PD for scripted grasp (torch backend)."""
    import torch

    device = robot._articulation_view._device
    # Finger gains match USD drives — high kp (12k) ejects finger links on deformable pinch in GUI.
    kps = torch.tensor(
        [4000.0] * 7 + [DEFAULT_FINGER_KP, DEFAULT_FINGER_KP], dtype=torch.float32, device=device
    ).unsqueeze(0)
    kds = torch.tensor(
        [400.0] * 7 + [DEFAULT_FINGER_KD, DEFAULT_FINGER_KD], dtype=torch.float32, device=device
    ).unsqueeze(0)
    robot._articulation_view.set_gains(kps=kps, kds=kds)


def create_hand_view():
    from isaacsim.core.simulation_manager import SimulationManager

    sim = SimulationManager.get_physics_sim_view()
    if sim is None:
        import omni.physics.tensors as physics_tensors

        sim = physics_tensors.create_simulation_view("torch")
    return sim, sim.create_rigid_body_view(HAND_LINK)


def get_hand_world_pose(hand_view):
    import numpy as np

    transform = hand_view.get_transforms()[0].detach().cpu().numpy()
    return transform[:3], transform[3:7].astype(np.float32)


def get_hand_world_pos(hand_view):
    return get_hand_world_pose(hand_view)[0]


def quat_conjugate_xyzw(quat):
    out = quat.copy()
    out[:3] *= -1.0
    return out


def quat_mul_xyzw(a, b):
    import numpy as np

    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float32,
    )


def orientation_error_xyzw(goal_xyzw, current_xyzw):
    q_r = quat_mul_xyzw(goal_xyzw, quat_conjugate_xyzw(current_xyzw))
    sign = 1.0 if q_r[3] >= 0.0 else -1.0
    return q_r[:3] * sign


def apply_cartesian_ik_step(
    robot,
    hand_view,
    target_xyz: tuple[float, float, float],
    gripper_m: float,
    *,
    target_quat_xyzw: tuple[float, float, float, float] = GRASP_DOWN_QUAT_XYZW,
) -> None:
    """One damped-least-squares IK step toward a world-space hand target pose."""
    import numpy as np
    import torch

    hand_pos, hand_quat = get_hand_world_pose(hand_view)
    dof_pos = robot.get_joint_positions().detach().cpu().numpy().flatten()
    j_eef = robot._articulation_view.get_jacobians()[0, 7, :, :7].detach().cpu().numpy()

    pos_err = np.array(target_xyz, dtype=np.float32) - hand_pos
    err_mag = float(np.linalg.norm(pos_err)) + 1e-6
    max_err = 0.05
    if err_mag > max_err:
        pos_err *= max_err / err_mag

    goal_quat = np.array(target_quat_xyzw, dtype=np.float32)
    orn_err = orientation_error_xyzw(goal_quat, hand_quat) * 1.5

    dpose = np.concatenate([pos_err, orn_err])
    damping = 0.05
    lmbda = np.eye(6, dtype=np.float32) * (damping**2)
    u = j_eef.T @ np.linalg.inv(j_eef @ j_eef.T + lmbda) @ dpose

    new_targets = dof_pos.copy()
    new_targets[:7] += u
    new_targets[7] = gripper_m
    new_targets[8] = gripper_m

    device = robot._articulation_view._device
    targets_t = torch.as_tensor(new_targets, dtype=torch.float32, device=device).unsqueeze(0)
    robot._articulation_view.set_joint_position_targets(targets_t)


def interpolate_xyz(step: int, schedule: list) -> tuple[tuple[float, float, float] | None, float]:
    """Interpolate Cartesian targets; None means joint-space home."""
    if step <= schedule[0][0]:
        return schedule[0][1], schedule[0][2]

    for i in range(len(schedule) - 1):
        s0, xyz0, g0, _ = schedule[i]
        s1, xyz1, g1, _ = schedule[i + 1]
        if step <= s1:
            t = 0.0 if s1 == s0 else (step - s0) / (s1 - s0)
            if xyz0 is None:
                return xyz1, g0 + (g1 - g0) * t
            if xyz1 is None:
                return xyz0, g0 + (g1 - g0) * t
            xyz = tuple(a0 + (a1 - a0) * t for a0, a1 in zip(xyz0, xyz1))
            return xyz, g0 + (g1 - g0) * t

    last_xyz, last_g, _ = schedule[-1][1:]
    return last_xyz, last_g


def interpolate_joints(
    step: int, schedule: list[tuple[int, tuple[float, ...], float, str]]
) -> tuple[tuple[float, ...], float]:
    """Linear interpolation between joint-space milestones (degrees + gripper meters)."""
    if step <= schedule[0][0]:
        return schedule[0][1], schedule[0][2]

    for i in range(len(schedule) - 1):
        s0, arm0, g0, _ = schedule[i]
        s1, arm1, g1, _ = schedule[i + 1]
        if step <= s1:
            t = 0.0 if s1 == s0 else (step - s0) / (s1 - s0)
            arm = tuple(a0 + (a1 - a0) * t for a0, a1 in zip(arm0, arm1))
            return arm, g0 + (g1 - g0) * t

    last_arm, last_g, _ = schedule[-1][1:]
    return last_arm, last_g


def get_mouse_world_pos(deformable, stage=None, *, mouse_mode: str = "deformable") -> tuple[float, float, float]:
    import numpy as np

    if mouse_mode == "rigid-box" and stage is not None:
        from pxr import Usd, UsdGeom

        prim = stage.GetPrimAtPath(RIGID_MOUSE_PROBE)
        if prim.IsValid():
            t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            return float(t[0]), float(t[1]), float(t[2])

    if deformable is None:
        raise RuntimeError("get_mouse_world_pos: deformable is None and mouse_mode != rigid-box")

    view = deformable._deformable_prim_view
    try:
        if view.is_physics_handle_valid():
            nodes = view.get_simulation_mesh_nodal_positions()
            if hasattr(nodes, "detach"):
                nodes = nodes.detach().cpu().numpy()
            nodes = np.asarray(nodes).reshape(-1, 3)
            if _grasp_attach_indices is not None and len(_grasp_attach_indices) > 0:
                attached = nodes[_grasp_attach_indices]
                center = attached.mean(axis=0)
            else:
                center = nodes.mean(axis=0)
            return float(center[0]), float(center[1]), float(center[2])
    except Exception:
        pass

    pos, _ori = deformable.get_world_pose()
    if hasattr(pos, "detach"):
        pos = pos.detach().cpu().numpy()
    pos = np.asarray(pos).reshape(-1)
    return float(pos[0]), float(pos[1]), float(pos[2])


def log_mouse_placement(deformable, label: str) -> None:
    import numpy as np

    pos = get_mouse_world_pos(deformable)
    view = deformable._deformable_prim_view
    try:
        if view.is_physics_handle_valid():
            nodes = view.get_simulation_mesh_nodal_positions()
            if hasattr(nodes, "detach"):
                nodes = nodes.detach().cpu().numpy()
            nodes = np.asarray(nodes).reshape(-1, 3)
            ext = nodes.max(axis=0) - nodes.min(axis=0)
            print(
                f"[grasp_demo] {label}: center=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), "
                f"nodal ext xyz=({ext[0]:.3f}, {ext[1]:.3f}, {ext[2]:.3f}) m, "
                f"above_table={pos[2] - TABLE_TOP_Z:.3f} m",
                flush=True,
            )
            return
    except Exception:
        pass
    print(f"[grasp_demo] {label}: center=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})", flush=True)


def check_grasp_success(
    initial_mouse_z: float,
    mouse_pos: tuple[float, float, float],
    hand_pos,
    hand_quat,
    gripper_m: float,
) -> bool:
    import numpy as np

    lifted = mouse_pos[2] > initial_mouse_z + GRASP_LIFT_DELTA_M
    # Franka convention: 0 = closed, 0.04 = open. Pinching means gripper at clamp target.
    pinching = gripper_m <= GRIPPER_CLAMP_M * 1.2
    if lifted and pinching:
        return True

    hand_pos = np.asarray(hand_pos)
    mouse_xyz = np.array(MOUSE_TRANSLATE, dtype=np.float32)
    dist = float(np.linalg.norm(hand_pos - mouse_xyz))
    height_ok = abs(float(hand_pos[2]) - float(mouse_xyz[2])) < GRASP_HAND_HEIGHT_TOL_M
    return pinching and dist < GRASP_HAND_CLOSE_DIST_M and height_ok


class ReplayController:
    """Listen for R key to restart the scripted grasp."""

    def __init__(self) -> None:
        self.requested = False
        self._subscription = None

    def start(self) -> None:
        import carb.input
        import omni.appwindow

        app_window = omni.appwindow.get_default_app_window()
        keyboard = app_window.get_keyboard()
        input_iface = carb.input.acquire_input_interface()
        self.requested = False
        self._subscription = input_iface.subscribe_to_keyboard_events(keyboard, self._on_keyboard)

    def stop(self) -> None:
        if self._subscription is not None:
            self._subscription = None
        self.requested = False

    def _on_keyboard(self, event, *_args):
        import carb.input

        if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name == REPLAY_KEY:
            self.requested = True
        return True

    def wait(self, world, app, deformable=None, *, headless: bool) -> bool:
        """Block until replay is requested. Returns False if the app exits."""
        self.requested = False
        print(f"[grasp_demo] grasp cycle complete — press {REPLAY_KEY} to replay, close window to exit", flush=True)
        while app.is_running():
            world.step(render=not headless)
            app.update()
            if self.requested:
                return True
        return False


def find_first_mesh_path(stage, root_path: str) -> str:
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f"Root prim not found: {root_path}")
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            return str(prim.GetPath())
    raise RuntimeError(f"No mesh under {root_path}")


def wait_stage_loaded(usd_context, app, max_updates: int = 600) -> None:
    for _ in range(max_updates):
        if usd_context.get_stage_loading_status()[2] <= 0:
            return
        app.update()
    raise TimeoutError("Timed out waiting for USD references to load.")


def set_translate(stage, prim_path: str, xyz: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))


def mesh_triangle_indices(mesh) -> "np.ndarray":
    """Triangulate USD mesh faces to (F, 3) vertex indices."""
    import numpy as np

    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    tris: list[list[int]] = []
    cursor = 0
    for count in counts:
        face = indices[cursor : cursor + count]
        cursor += count
        if count == 3:
            tris.append(list(face))
        elif count == 4:
            tris.append([face[0], face[1], face[2]])
            tris.append([face[0], face[2], face[3]])
        elif count > 4:
            for i in range(1, count - 1):
                tris.append([face[0], face[i], face[i + 1]])
    return np.asarray(tris, dtype=np.int64)


def mesh_bottom_contact_z(
    pts: "np.ndarray",
    tri_indices: "np.ndarray | None" = None,
    *,
    tri_percentile: float = MOUSE_BOTTOM_CONTACT_TRI_PCT,
    nodal_percentile: float = MOUSE_BOTTOM_CONTACT_NODAL_PCT,
) -> tuple[float, float]:
    """Return (contact_z, min_z). Belly plane is usually several mm above global min_z."""
    import numpy as np

    z = np.asarray(pts[:, 2], dtype=np.float64)
    min_z = float(z.min())
    if tri_indices is not None and len(tri_indices) > 0:
        v0 = pts[tri_indices[:, 0]]
        v1 = pts[tri_indices[:, 1]]
        v2 = pts[tri_indices[:, 2]]
        normals = np.cross(v1 - v0, v2 - v0)
        lens = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(lens, 1e-12)
        downward = normals[:, 2] < -0.5
        if np.any(downward):
            tri_min_z = np.min(
                np.stack([v0[downward, 2], v1[downward, 2], v2[downward, 2]], axis=1),
                axis=1,
            )
            contact_z = float(np.percentile(tri_min_z, tri_percentile))
            return max(contact_z, min_z), min_z
    contact_z = float(np.percentile(z, nodal_percentile))
    return max(contact_z, min_z), min_z


def shift_points_to_table(
    pts: "np.ndarray",
    tri_indices: "np.ndarray | None" = None,
    *,
    table_z: float = TABLE_TOP_Z,
    pre_sink_m: float = 0.0,
) -> tuple[float, float, float]:
    """Shift pts in-place so the lowest vertex sits on ``table_z - pre_sink_m``.

    ``pre_sink_m > 0`` embeds the visual mesh slightly below the table surface
    to compensate for the PhysX FEM collision hull expansion (which inflates the
    mesh outward, causing the visible mouse to appear to float above the table).
    """
    min_z = float(pts[:, 2].min())
    pts[:, 2] += (table_z - pre_sink_m) - min_z
    contact_z, _ = mesh_bottom_contact_z(pts, tri_indices)
    return contact_z, min_z, float(pts[:, 2].min())


def read_mouse_mesh_center(stage, mesh_path: str) -> tuple[float, float, float]:
    from pxr import UsdGeom
    import numpy as np

    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        return MOUSE_TRANSLATE
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    center = pts.mean(axis=0)
    return float(center[0]), float(center[1]), float(center[2])


def mouse_mesh_needs_rotation(pts: "np.ndarray") -> bool:
    """True when head-tail is still on Y (source USD) rather than X (grasp layout)."""
    import numpy as np

    ext = np.ptp(pts, axis=0)
    return float(ext[1]) > float(ext[0]) + 0.01


def write_baked_mouse_usd(source_usd: Path, baked_usd: Path, stage, mesh_path: str) -> Path:
    """Persist baked vertex positions so world.reset() reloads the grasp pose from disk."""
    from pxr import Gf, Usd, UsdGeom

    src_mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not src_mesh:
        raise RuntimeError(f"Cannot export baked mouse: {mesh_path}")

    baked_points = src_mesh.GetPointsAttr().Get()
    out_stage = Usd.Stage.Open(str(source_usd.resolve()))
    out_mesh = None
    for prim in out_stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            out_mesh = UsdGeom.Mesh(prim)
            break
    if not out_mesh:
        raise RuntimeError(f"No mesh in source USD: {source_usd}")

    out_mesh.GetPointsAttr().Set(baked_points)
    for path in (str(out_mesh.GetPath()), str(out_mesh.GetPath().GetParentPath()), "/Mouse"):
        prim = out_stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Xformable(prim).ClearXformOpOrder()

    baked_usd.parent.mkdir(parents=True, exist_ok=True)
    out_stage.GetRootLayer().Export(str(baked_usd.resolve()))
    print(f"[grasp_demo] wrote baked mouse USD → {baked_usd}", flush=True)
    return baked_usd


def resolve_mouse_asset(mouse_usd: Path, *, prefer_baked: bool) -> Path:
    """Use baked USD on reset; always cook deformable from freshly baked source on scene build."""
    if prefer_baked and BAKED_MOUSE_USD.exists():
        return BAKED_MOUSE_USD
    return mouse_usd


def zero_deformable_velocities(deformable) -> None:
    import numpy as np
    import torch

    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid():
        return

    nodes = view.get_simulation_mesh_nodal_positions()
    if hasattr(nodes, "detach"):
        count = nodes.detach().reshape(-1, 3).shape[0]
    else:
        count = np.asarray(nodes).reshape(-1, 3).shape[0]
    zeros = torch.zeros((1, count, 3), dtype=torch.float32, device=view._device)
    view.set_simulation_mesh_nodal_velocities(zeros)


def get_display_mesh_contact_z(stage, mesh_path: str) -> tuple[float, float]:
    """Contact/min Z of the mesh Hydra draws (updated each step by PhysX skinning)."""
    from pxr import UsdGeom
    import numpy as np

    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        return float("nan"), float("nan")

    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    tris = mesh_triangle_indices(mesh)
    contact_z, min_z = mesh_bottom_contact_z(pts, tris)
    return contact_z, min_z


def log_mesh_layer_gap(stage, mesh_path: str, deformable, label: str) -> None:
    """Compare GUI render mesh vs FEM sim nodes vs collision nodes."""
    import numpy as np

    display_contact, display_min = get_display_mesh_contact_z(stage, mesh_path)
    view = deformable._deformable_prim_view
    sim_contact = sim_min = float("nan")
    coll_contact = coll_min = float("nan")

    if view.is_physics_handle_valid():
        nodes = view.get_simulation_mesh_nodal_positions()
        if hasattr(nodes, "detach"):
            arr = nodes.detach().cpu().numpy().reshape(-1, 3)
        else:
            arr = np.asarray(nodes).reshape(-1, 3)
        sim_contact, sim_min = mesh_bottom_contact_z(arr, nodal_percentile=MOUSE_BOTTOM_CONTACT_NODAL_PCT)

        try:
            coll = view.get_collision_mesh_nodal_positions()
            if hasattr(coll, "detach"):
                coll = coll.detach().cpu().numpy().reshape(-1, 3)
            else:
                coll = np.asarray(coll).reshape(-1, 3)
            if coll.size > 0:
                coll_contact, coll_min = mesh_bottom_contact_z(coll, nodal_percentile=MOUSE_BOTTOM_CONTACT_NODAL_PCT)
        except Exception:
            pass

    print(
        f"[grasp_demo] {label}: display contact={display_contact:.3f} min={display_min:.3f} | "
        f"sim contact={sim_contact:.3f} min={sim_min:.3f} | "
        f"collision contact={coll_contact:.3f} min={coll_min:.3f} | "
        f"table={TABLE_TOP_Z:.3f}",
        flush=True,
    )


class MousePhysicsDebug:
    """Visualize the PhysX collision tet hull as an orange surface mesh.

    Layer guide (viewport):
      - White/yellow mouse = render/visual mesh (PBR), skinned from FEM *sim* mesh.
      - Orange shell     = PhysX *collision* tet hull (/World/DebugMouse/CollisionTetMesh).
      - Orange wireframe = hull surface edges (/World/DebugMouse/CollisionWire).
      - Orange AABB box  = hull bounding box (/World/DebugMouse/CollisionHullBBox).
      - Cyan points      = optional SimNodes (FEM sim nodes, off by default).
      - Orange points    = optional CollisionNodes (off by default — 5000+ pts obscure the shell).
    """

    def __init__(
        self,
        stage,
        mesh_path: str = "",
        *,
        enabled: bool = True,
        show_node_clouds: bool = SHOW_DEBUG_NODE_CLOUDS,
        dim_render_mesh: bool = True,
    ) -> None:
        self.stage = stage
        self.mesh_path = mesh_path
        self.enabled = enabled
        self.show_node_clouds = show_node_clouds
        self.dim_render_mesh = dim_render_mesh
        self._collision_indices_cache: "np.ndarray | None" = None
        self._logged_collision_stats = False
        self._render_dimmed = False

    @staticmethod
    def _collision_tet_boundary_triangles(tets: "np.ndarray") -> "np.ndarray":
        """Outer surface triangles of the collision tet mesh (interior faces removed)."""
        import numpy as np
        from collections import Counter

        face_counts: Counter[tuple[int, int, int]] = Counter()
        face_oriented: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        for tet in tets:
            nodes = [int(v) for v in tet if int(v) >= 0]
            if len(nodes) < 4:
                continue
            a, b, c, d = nodes[:4]
            for face in ((a, b, c), (a, b, d), (a, c, d), (b, c, d)):
                key = tuple(sorted(face))
                face_counts[key] += 1
                face_oriented[key] = face
        boundary = [face_oriented[k] for k, n in face_counts.items() if n == 1]
        if not boundary:
            return np.zeros((0, 3), dtype=np.int32)
        return np.asarray(boundary, dtype=np.int32)

    @staticmethod
    def _set_prim_visible(stage, path: str, visible: bool) -> None:
        from pxr import UsdGeom

        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return
        img = UsdGeom.Imageable(prim)
        img.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)

    def _bind_orange_material(self, mesh_path: str) -> None:
        """Same pattern as green gripper debug — purpose=render + PreviewSurface opacity."""
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

        prim = self.stage.GetPrimAtPath(mesh_path)
        if not prim.IsValid():
            return
        mesh = UsdGeom.Mesh(prim)
        mesh.CreatePurposeAttr(UsdGeom.Tokens.render)
        mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*COLLISION_DEBUG_COLOR)]))
        mat_path = f"{MOUSE_DEBUG_ROOT}/CollisionTetMaterial"
        if not self.stage.GetPrimAtPath(mat_path).IsValid():
            mat = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*COLLISION_DEBUG_COLOR))
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(COLLISION_TET_OPACITY)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(prim).Bind(UsdShade.Material.Get(self.stage, mat_path))

    def _dim_render_mesh(self) -> None:
        """Fade render skin via DisplayOpacity so orange hull reads through in RTX."""
        if self._render_dimmed or not self.dim_render_mesh or not self.mesh_path:
            return
        try:
            from pxr import UsdGeom, Vt

            prim = self.stage.GetPrimAtPath(self.mesh_path)
            if not prim.IsValid():
                return
            gprim = UsdGeom.Gprim(prim)
            attr = gprim.GetDisplayOpacityAttr()
            if not attr:
                gprim.CreateDisplayOpacityAttr(Vt.FloatArray([RENDER_MESH_DEBUG_OPACITY]))
            else:
                attr.Set(Vt.FloatArray([RENDER_MESH_DEBUG_OPACITY]))
        except Exception as exc:
            print(f"[grasp_demo] could not dim render mesh for debug: {exc}", flush=True)
        self._render_dimmed = True

    @staticmethod
    def _aabb_wire_segments(coll: "np.ndarray") -> tuple[list, list[int]]:
        import numpy as np

        mn = coll.min(axis=0)
        mx = coll.max(axis=0)
        c = [
            (mn[0], mn[1], mn[2]),
            (mx[0], mn[1], mn[2]),
            (mx[0], mx[1], mn[2]),
            (mn[0], mx[1], mn[2]),
            (mn[0], mn[1], mx[2]),
            (mx[0], mn[1], mx[2]),
            (mx[0], mx[1], mx[2]),
            (mn[0], mx[1], mx[2]),
        ]
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        from pxr import Gf

        seg_points: list[Gf.Vec3f] = []
        counts: list[int] = []
        for i, j in edges:
            a, b = c[i], c[j]
            seg_points.append(Gf.Vec3f(float(a[0]), float(a[1]), float(a[2])))
            seg_points.append(Gf.Vec3f(float(b[0]), float(b[1]), float(b[2])))
            counts.append(2)
        return seg_points, counts

    def ensure_prims(self) -> None:
        if not self.enabled:
            return
        from pxr import Gf, UsdGeom, Vt

        for name, color, width in (
            ("CollisionNodes", COLLISION_DEBUG_COLOR, DEBUG_POINT_WIDTH_COLL),
            ("SimNodes", SIM_DEBUG_COLOR, DEBUG_POINT_WIDTH_SIM),
        ):
            path = f"{MOUSE_DEBUG_ROOT}/{name}"
            if not self.stage.GetPrimAtPath(path).IsValid():
                pts = UsdGeom.Points.Define(self.stage, path)
                pts.CreatePointsAttr().Set(Vt.Vec3fArray())
                pts.CreateWidthsAttr().Set(Vt.FloatArray([width]))
                pts.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            self._set_prim_visible(self.stage, path, self.show_node_clouds)

        wire_path = f"{MOUSE_DEBUG_ROOT}/CollisionWire"
        if not self.stage.GetPrimAtPath(wire_path).IsValid():
            curves = UsdGeom.BasisCurves.Define(self.stage, wire_path)
            curves.CreateTypeAttr("linear")
            curves.CreateBasisAttr("bezier")
            curves.CreatePointsAttr().Set(Vt.Vec3fArray())
            curves.CreateCurveVertexCountsAttr().Set(Vt.IntArray())
            curves.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*COLLISION_DEBUG_COLOR)]))
            curves.CreateWidthsAttr().Set(Vt.FloatArray([COLLISION_WIRE_WIDTH]))
        wire = UsdGeom.BasisCurves.Get(self.stage, wire_path)
        wire.CreatePurposeAttr(UsdGeom.Tokens.render)
        self._set_prim_visible(self.stage, wire_path, SHOW_COLLISION_WIRE)

        bbox_path = f"{MOUSE_DEBUG_ROOT}/CollisionHullBBox"
        if not self.stage.GetPrimAtPath(bbox_path).IsValid():
            bbox = UsdGeom.BasisCurves.Define(self.stage, bbox_path)
            bbox.CreateTypeAttr("linear")
            bbox.CreateBasisAttr("bezier")
            bbox.CreatePointsAttr().Set(Vt.Vec3fArray())
            bbox.CreateCurveVertexCountsAttr().Set(Vt.IntArray())
            bbox.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.55, 0.0)]))
            bbox.CreateWidthsAttr().Set(Vt.FloatArray([COLLISION_BBOX_WIRE_WIDTH]))
        bbox = UsdGeom.BasisCurves.Get(self.stage, bbox_path)
        bbox.CreatePurposeAttr(UsdGeom.Tokens.render)
        self._set_prim_visible(self.stage, bbox_path, True)

        tet_path = f"{MOUSE_DEBUG_ROOT}/CollisionTetMesh"
        if not self.stage.GetPrimAtPath(tet_path).IsValid():
            mesh = UsdGeom.Mesh.Define(self.stage, tet_path)
            mesh.CreatePointsAttr().Set(Vt.Vec3fArray())
            mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray())
            mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray())
            mesh.CreateDoubleSidedAttr(True)
            self._bind_orange_material(tet_path)
            self._set_prim_visible(self.stage, tet_path, True)
        else:
            self._bind_orange_material(tet_path)

        self._dim_render_mesh()

    def _read_collision_indices(self, deformable, mesh_path: str) -> "np.ndarray | None":
        import numpy as np

        if self._collision_indices_cache is not None:
            return self._collision_indices_cache

        view = deformable._deformable_prim_view
        if view.is_physics_handle_valid():
            try:
                idx = view.get_collision_mesh_indices()
                if hasattr(idx, "detach"):
                    idx = idx.detach().cpu().numpy()
                idx = np.asarray(idx).reshape(-1, 4)
                valid = idx[(idx >= 0).all(axis=1)]
                if valid.size > 0:
                    self._collision_indices_cache = valid
                    return valid
            except Exception:
                pass

        from pxr import PhysxSchema

        prim = self.stage.GetPrimAtPath(mesh_path)
        body = PhysxSchema.PhysxDeformableBodyAPI(prim) if prim.IsValid() else None
        if not body:
            return None
        attr = body.GetCollisionIndicesAttr()
        if not attr or not attr.IsAuthored():
            return None
        flat = np.asarray(attr.Get(), dtype=np.int64)
        if flat.size < 4:
            return None
        valid = flat.reshape(-1, 4)
        valid = valid[(valid >= 0).all(axis=1)]
        self._collision_indices_cache = valid
        return valid

    def update(self, deformable, mesh_path: str) -> None:
        if not self.enabled:
            return
        import numpy as np
        from pxr import Gf, UsdGeom, Vt

        self.ensure_prims()
        view = deformable._deformable_prim_view
        if not view.is_physics_handle_valid():
            return

        def _to_np(tensor) -> np.ndarray:
            if hasattr(tensor, "detach"):
                return tensor.detach().cpu().numpy()
            return np.asarray(tensor)

        coll = _to_np(view.get_collision_mesh_nodal_positions()).reshape(-1, 3)
        n_coll = _collision_rest_node_count(self.stage, mesh_path)
        if n_coll > 0:
            coll = coll[:n_coll]
        coll = coll[np.isfinite(coll).all(axis=1)]
        if coll.size > 0:
            coll_pts = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in coll]
            prim = UsdGeom.Points.Get(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionNodes")
            prim.GetPointsAttr().Set(coll_pts)
            prim.GetWidthsAttr().Set(Vt.FloatArray([DEBUG_POINT_WIDTH_COLL] * len(coll_pts)))

            tets = self._read_collision_indices(deformable, mesh_path)
            if tets is not None and coll.size > 0:
                tris = self._collision_tet_boundary_triangles(tets)
                max_idx = coll.shape[0]
                valid_tris = tris[(tris >= 0).all(axis=1) & (tris < max_idx).all(axis=1)]
                if valid_tris.size > 0:
                    tet_mesh = UsdGeom.Mesh.Get(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionTetMesh")
                    tet_mesh.GetPointsAttr().Set(coll_pts)
                    tet_mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(valid_tris)))
                    tet_mesh.GetFaceVertexIndicesAttr().Set(
                        Vt.IntArray(valid_tris.reshape(-1).tolist())
                    )
                    tet_mesh.CreatePurposeAttr(UsdGeom.Tokens.render)
                    if not self._logged_collision_stats:
                        ext = coll.max(axis=0) - coll.min(axis=0)
                        print(
                            f"[grasp_demo] collision tet hull: {coll.shape[0]} nodes, "
                            f"{len(valid_tris)} tri faces, ext xyz="
                            f"({ext[0]*1000:.1f}, {ext[1]*1000:.1f}, {ext[2]*1000:.1f}) mm "
                            f"→ orange mesh at {MOUSE_DEBUG_ROOT}/CollisionTetMesh",
                            flush=True,
                        )
                        self._logged_collision_stats = True

                edges: set[tuple[int, int]] = set()
                for tet in tets:
                    nodes = [int(v) for v in tet if 0 <= int(v) < max_idx]
                    if len(nodes) < 4:
                        continue
                    for a in range(4):
                        for b in range(a + 1, 4):
                            i, j = nodes[a], nodes[b]
                            edges.add((min(i, j), max(i, j)))
                if edges:
                    seg_points: list[Gf.Vec3f] = []
                    counts: list[int] = []
                    for i, j in edges:
                        a, b = coll[i], coll[j]
                        seg_points.append(Gf.Vec3f(float(a[0]), float(a[1]), float(a[2])))
                        seg_points.append(Gf.Vec3f(float(b[0]), float(b[1]), float(b[2])))
                        counts.append(2)
                    wire = UsdGeom.BasisCurves.Get(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionWire")
                    wire.GetPointsAttr().Set(seg_points)
                    wire.GetCurveVertexCountsAttr().Set(Vt.IntArray(counts))
                    wire.GetWidthsAttr().Set(Vt.FloatArray([COLLISION_WIRE_WIDTH] * len(counts)))
                    wire.CreatePurposeAttr(UsdGeom.Tokens.render)
                    self._set_prim_visible(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionWire", SHOW_COLLISION_WIRE)

                bbox_pts, bbox_counts = self._aabb_wire_segments(coll)
                if bbox_pts:
                    bbox = UsdGeom.BasisCurves.Get(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionHullBBox")
                    bbox.GetPointsAttr().Set(bbox_pts)
                    bbox.GetCurveVertexCountsAttr().Set(Vt.IntArray(bbox_counts))
                    bbox.GetWidthsAttr().Set(Vt.FloatArray([COLLISION_BBOX_WIRE_WIDTH] * len(bbox_counts)))
                    bbox.CreatePurposeAttr(UsdGeom.Tokens.render)
                    self._set_prim_visible(self.stage, f"{MOUSE_DEBUG_ROOT}/CollisionHullBBox", True)

        sim = _to_np(view.get_simulation_mesh_nodal_positions()).reshape(-1, 3)
        sim = sim[np.isfinite(sim).all(axis=1)]
        sim = sim[(np.abs(sim) < 10.0).all(axis=1)]
        if sim.size > 0:
            sim_pts = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in sim]
            prim = UsdGeom.Points.Get(self.stage, f"{MOUSE_DEBUG_ROOT}/SimNodes")
            prim.GetPointsAttr().Set(sim_pts)
            prim.GetWidthsAttr().Set(Vt.FloatArray([DEBUG_POINT_WIDTH_SIM] * len(sim_pts)))

    def log_extent(self, deformable, label: str) -> None:
        """Log live collision-hull width (X) vs gripper opening for pinch diagnosis."""
        import numpy as np

        view = deformable._deformable_prim_view
        if not view.is_physics_handle_valid():
            return
        coll = view.get_collision_mesh_nodal_positions()
        if hasattr(coll, "detach"):
            coll = coll.detach().cpu().numpy()
        coll = np.asarray(coll).reshape(-1, 3)
        coll = coll[np.isfinite(coll).all(axis=1)]
        if coll.size == 0:
            return
        ext = coll.max(axis=0) - coll.min(axis=0)
        print(
            f"[grasp_demo] collision hull ({label}): "
            f"ext xyz=({ext[0]*1000:.1f}, {ext[1]*1000:.1f}, {ext[2]*1000:.1f}) mm "
            f"(width X={ext[0]*1000:.1f} mm — compare to green finger inner gap)",
            flush=True,
        )


class GripperBlockerDebug:
    """Visualize gripper rigid colliders as green meshes parented under each finger link.

    Meshes use *link-local* coordinates so they follow the articulation. Do not bake
    world-space copies under /World/DebugGripper — USD rest transforms won't track GPU sim.
    """

    _SENSOR_PATHS = (LEFT_FINGER_LINK, RIGHT_FINGER_LINK)
    _SENSOR_LABELS = ("left_finger", "right_finger")
    _DEBUG_SUFFIX = "PhysXColliderDebug"

    def __init__(self, stage, mesh_path: str, *, enabled: bool = True, enable_contacts: bool = False) -> None:
        self.stage = stage
        self.mesh_path = mesh_path
        self.enabled = enabled
        self.enable_contacts = enable_contacts
        self._contact_view = None
        self._filter_labels: list[str] = []
        self._logged_blockers = False
        self._built = False

    def _filter_paths(self) -> list[str]:
        # GPU RigidContactView: rigid colliders only — no deformable mesh, no globs, no ground plane.
        return [HAND_LINK]

    def _filter_labels_for(self, n: int) -> list[str]:
        return ["panda_hand"][:n]

    @staticmethod
    def _sanitize_name(path: str) -> str:
        return path.strip("/").replace("/", "_")

    def _bind_material(self, mesh_path: str, color: tuple[float, float, float], opacity: float) -> None:
        from pxr import Gf, Sdf, UsdShade

        mat_path = f"{GRIPPER_DEBUG_ROOT}/Materials/{self._sanitize_name(mesh_path)}"
        if not self.stage.GetPrimAtPath(mat_path).IsValid():
            mat = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(self.stage.GetPrimAtPath(mesh_path)).Bind(
            UsdShade.Material.Get(self.stage, mat_path)
        )

    @staticmethod
    def _link_for_collider(stage, collider_path: str) -> str:
        from pxr import Usd, UsdPhysics

        prim = stage.GetPrimAtPath(collider_path)
        while prim and prim.IsValid():
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) or path.endswith(("leftfinger", "rightfinger", "panda_hand")):
                return path
            prim = prim.GetParent()
        return collider_path

    @staticmethod
    def _local_mesh_triangles(stage, collider_path: str, link_path: str):
        from pxr import Gf, Usd, UsdGeom

        link_xf = UsdGeom.Xformable(stage.GetPrimAtPath(link_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        link_inv = link_xf.GetInverse()

        def _to_link_local(world_pt: Gf.Vec3d) -> Gf.Vec3f:
            local = link_inv.Transform(world_pt)
            return Gf.Vec3f(float(local[0]), float(local[1]), float(local[2]))

        root = stage.GetPrimAtPath(collider_path)
        if not root.IsValid():
            return None, None, None

        stack = [str(root.GetPath())]
        while stack:
            path = stack.pop()
            prim = stage.GetPrimAtPath(path)
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                pts_local = mesh.GetPointsAttr().Get()
                counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
                indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
                if pts_local and counts and indices:
                    mesh_xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    pts = [_to_link_local(mesh_xf.Transform(Gf.Vec3d(p))) for p in pts_local]
                    return pts, counts, indices
            for child in prim.GetChildren():
                stack.append(str(child.GetPath()))

        img = UsdGeom.Imageable(root)
        bound = img.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        box = bound.GetRange()
        if box.IsEmpty():
            return None, None, None
        mn, mx = box.GetMin(), box.GetMax()
        corners = [
            _to_link_local(Gf.Vec3d(mn[0], mn[1], mn[2])),
            _to_link_local(Gf.Vec3d(mx[0], mn[1], mn[2])),
            _to_link_local(Gf.Vec3d(mx[0], mx[1], mn[2])),
            _to_link_local(Gf.Vec3d(mn[0], mx[1], mn[2])),
            _to_link_local(Gf.Vec3d(mn[0], mn[1], mx[2])),
            _to_link_local(Gf.Vec3d(mx[0], mn[1], mx[2])),
            _to_link_local(Gf.Vec3d(mx[0], mx[1], mx[2])),
            _to_link_local(Gf.Vec3d(mn[0], mx[1], mx[2])),
        ]
        tris = [
            (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
            (0, 4, 5), (0, 5, 1), (2, 6, 7), (2, 7, 3),
            (0, 3, 7), (0, 7, 4), (1, 5, 6), (1, 6, 2),
        ]
        return corners, [3] * len(tris), [idx for tri in tris for idx in tri]

    def _discover_gripper_colliders(self) -> list[str]:
        from pxr import Usd, UsdPhysics

        keywords = ("finger", "hand", "gripper")
        found: list[str] = []
        root = self.stage.GetPrimAtPath(FRANKA_ROOT)
        if not root.IsValid():
            return found
        for prim in Usd.PrimRange(root):
            path = str(prim.GetPath())
            if self._DEBUG_SUFFIX in path:
                continue
            if not any(k in path.lower() for k in keywords):
                continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                found.append(path)
        return found

    def ensure_prims(self) -> None:
        if not self.enabled or self._built:
            return
        from pxr import Gf, Sdf, UsdGeom, Vt

        # Remove legacy world-baked collider copies from earlier versions.
        stale = f"{GRIPPER_DEBUG_ROOT}/Colliders"
        if self.stage.GetPrimAtPath(stale).IsValid():
            self.stage.RemovePrim(Sdf.Path(stale))

        if not self.stage.GetPrimAtPath(GRIPPER_DEBUG_ROOT).IsValid():
            UsdGeom.Xform.Define(self.stage, GRIPPER_DEBUG_ROOT)

        built = 0
        for src in self._discover_gripper_colliders():
            link = self._link_for_collider(self.stage, src)
            dbg = f"{link}/{self._DEBUG_SUFFIX}_{self._sanitize_name(src)}"
            if self.stage.GetPrimAtPath(dbg).IsValid():
                built += 1
                continue
            pts, counts, indices = self._local_mesh_triangles(self.stage, src, link)
            if pts is None:
                continue
            mesh = UsdGeom.Mesh.Define(self.stage, dbg)
            mesh.CreatePointsAttr().Set(Vt.Vec3fArray(pts))
            mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
            mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreatePurposeAttr(UsdGeom.Tokens.render)
            self._bind_material(dbg, GRIPPER_COLLIDER_COLOR, 0.35)
            built += 1

        if self.enable_contacts:
            pts_path = f"{GRIPPER_DEBUG_ROOT}/ContactPoints"
            if not self.stage.GetPrimAtPath(pts_path).IsValid():
                pts = UsdGeom.Points.Define(self.stage, pts_path)
                pts.CreatePointsAttr().Set(Vt.Vec3fArray())
                pts.CreateWidthsAttr().Set(Vt.FloatArray([CONTACT_POINT_WIDTH]))
                pts.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*CONTACT_POINT_COLOR)]))

        self._built = True
        if built:
            print(
                f"[grasp_demo] gripper collider debug: {built} green mesh(es) parented under finger/hand links",
                flush=True,
            )

    def initialize_contacts(self) -> None:
        if not self.enabled or not self.enable_contacts or self._contact_view is not None:
            return
        self.ensure_prims()
        from isaacsim.core.api.sensors import RigidContactView

        filters = self._filter_paths()
        self._filter_labels = self._filter_labels_for(len(filters))
        # One filter list per sensor path — required by PhysX tensors API.
        per_sensor_filters = [filters for _ in self._SENSOR_PATHS]
        try:
            self._contact_view = RigidContactView(
                prim_paths_expr=list(self._SENSOR_PATHS),
                filter_paths_expr=per_sensor_filters,
                name="gripper_blocker_contacts",
                max_contact_count=256,
            )
            self._contact_view.initialize()
            if not self._contact_view.is_physics_handle_valid():
                raise RuntimeError("rigid contact view handle invalid after initialize")
            print(
                f"[grasp_demo] gripper contact debug: sensors={list(self._SENSOR_PATHS)}, "
                f"filters={filters}",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[grasp_demo] gripper contact debug disabled ({exc}) — "
                "green collider meshes still update",
                flush=True,
            )
            self._contact_view = None

    def update(self, robot, *, label: str = "", log_on_pinch: bool = False) -> None:
        if not self.enabled:
            return
        if not self._built:
            self.ensure_prims()
        if self.enable_contacts:
            hits = self._update_contact_points()
            if log_on_pinch and hits:
                self.log_blockers(robot, label, force=True)

    def _decode_contacts(self) -> list[tuple[str, str, tuple[float, float, float], float]]:
        import numpy as np

        if self._contact_view is None or not self._contact_view.is_physics_handle_valid():
            return []
        data = self._contact_view.get_contact_force_data(dt=1.0 / 60.0)
        if data is None:
            return []
        _forces, points, _normals, distances, starts, counts = data
        if hasattr(points, "detach"):
            points = points.detach().cpu().numpy()
            distances = distances.detach().cpu().numpy()
            starts = starts.detach().cpu().numpy()
            counts = counts.detach().cpu().numpy()
        else:
            points = np.asarray(points)
            distances = np.asarray(distances)
            starts = np.asarray(starts)
            counts = np.asarray(counts)

        hits: list[tuple[str, str, tuple[float, float, float], float]] = []
        for si, sensor in enumerate(self._SENSOR_LABELS):
            for fi, filt in enumerate(self._filter_labels[: starts.shape[1]]):
                n = int(counts[si, fi])
                if n <= 0:
                    continue
                start = int(starts[si, fi])
                for k in range(n):
                    idx = start + k
                    if idx >= len(points):
                        break
                    p = points[idx]
                    d = float(distances[idx, 0]) if distances.ndim > 1 else float(distances[idx])
                    hits.append((sensor, filt, (float(p[0]), float(p[1]), float(p[2])), d))
        return hits

    def _update_contact_points(self) -> list[tuple[str, str, tuple[float, float, float], float]]:
        from pxr import Gf, UsdGeom, Vt

        hits = self._decode_contacts()
        pts_path = f"{GRIPPER_DEBUG_ROOT}/ContactPoints"
        prim = self.stage.GetPrimAtPath(pts_path)
        if not prim.IsValid():
            return hits
        pts_prim = UsdGeom.Points(prim)
        if hits:
            pts = [Gf.Vec3f(*h[2]) for h in hits]
            pts_prim.GetPointsAttr().Set(pts)
            pts_prim.GetWidthsAttr().Set(Vt.FloatArray([CONTACT_POINT_WIDTH] * len(pts)))
        else:
            pts_prim.GetPointsAttr().Set(Vt.Vec3fArray())
        return hits

    def log_blockers(self, robot, label: str, *, force: bool = False) -> None:
        import numpy as np

        if not self.enabled:
            return
        if self._logged_blockers and not force:
            return

        lines = [f"[grasp_demo] blocker report ({label}):"]
        reported = False

        if self._contact_view is not None and self._contact_view.is_physics_handle_valid():
            try:
                matrix = self._contact_view.get_contact_force_matrix(dt=1.0 / 60.0)
                hits = self._update_contact_points()
                if matrix is not None:
                    arr = matrix.detach().cpu().numpy() if hasattr(matrix, "detach") else np.asarray(matrix)
                    for si, sensor in enumerate(self._SENSOR_LABELS):
                        for fi, filt in enumerate(self._filter_labels[: arr.shape[1]]):
                            fvec = arr[si, fi]
                            mag = float(np.linalg.norm(fvec))
                            if mag > 0.05:
                                reported = True
                                lines.append(
                                    f"  {sensor} <-> {filt}: |force|={mag:.1f} N "
                                    f"(fx={fvec[0]:.1f}, fy={fvec[1]:.1f}, fz={fvec[2]:.1f})"
                                )
                if hits:
                    reported = True
                    lines.append(
                        f"  {len(hits)} contact point(s) at {GRIPPER_DEBUG_ROOT}/ContactPoints"
                    )
                    for sensor, filt, pos, dist in hits[:12]:
                        lines.append(
                            f"    {sensor} <-> {filt} @ ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm "
                            f"separation={dist*1000:.2f} mm"
                        )
            except Exception as exc:
                lines.append(f"  (contact sensor unavailable: {exc})")
                reported = True

        # Fallback: net contact on each finger (works even when deformable filters fail).
        if robot.handles_initialized and robot._articulation_view.is_physics_handle_valid():
            import torch

            positions = robot.get_joint_positions()
            if positions is None:
                pass
            else:
                if torch.is_tensor(positions):
                    positions = positions.detach().cpu().numpy()
                positions = positions.reshape(-1)
                if positions.shape[0] >= 9:
                    f1, f2 = float(positions[7]), float(positions[8])
                    lines.append(
                        f"  finger joints: {f1*1000:.1f} / {f2*1000:.1f} mm "
                        f"(total {(f1+f2)*1000:.1f} mm, target {GRIPPER_CLAMP_M*2000:.1f} mm)"
                    )
                    reported = True

        if reported and len(lines) > 1:
            print("\n".join(lines), flush=True)
            self._logged_blockers = True


def _collision_rest_node_count(stage, mesh_path: str) -> int:
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(mesh_path)
    if not prim.IsValid():
        return 0
    body = PhysxSchema.PhysxDeformableBodyAPI(prim)
    if not body:
        return 0
    attr = body.GetCollisionRestPointsAttr()
    if not attr or not attr.IsAuthored():
        return 0
    pts = attr.Get()
    return len(pts) if pts else 0


def _physics_bottom_z(deformable) -> float:
    """Lowest Z among FEM + collision nodes — use this to touch the table."""
    _sim_c, sim_m, _coll_c, coll_m = _physics_contact_z(deformable)
    bottom = sim_m
    if not math.isnan(coll_m):
        bottom = min(bottom, coll_m)
    return bottom


def set_mouse_kinematic_pin(deformable, enabled: bool, *, positions=None) -> None:
    """Pin FEM nodes (w=1 in 4th component). Use cached ``positions`` to avoid re-teleport jitter."""
    import torch

    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid():
        return

    if positions is None:
        pos = view.get_simulation_mesh_nodal_positions(clone=True)
    else:
        pos = positions
    if not torch.is_tensor(pos):
        pos = torch.as_tensor(pos, dtype=torch.float32, device=view._device)
    if pos.dim() == 2:
        pos = pos.unsqueeze(0)
    targets = torch.zeros((pos.shape[0], pos.shape[1], 4), dtype=torch.float32, device=view._device)
    targets[..., :3] = pos[..., :3]
    targets[..., 3] = 1.0 if enabled else 0.0
    view.set_simulation_mesh_kinematic_targets(targets)


def attach_mouse_to_hand(deformable, grip_center: tuple[float, float, float]) -> int:
    """Capture FEM nodes near grip center (sphere + body box) for lift carry."""
    global _grasp_attach_indices, _grasp_attach_offsets, _grasp_attach_rest_pos, _grasp_attach_grip_center
    import numpy as np

    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid():
        return 0

    nodes = view.get_simulation_mesh_nodal_positions()
    if hasattr(nodes, "detach"):
        arr = nodes.detach().cpu().numpy().reshape(-1, 3)
    else:
        arr = np.asarray(nodes).reshape(-1, 3)

    center = np.asarray(grip_center, dtype=np.float64)
    dist = np.linalg.norm(arr - center, axis=1)
    mask = dist <= GRASP_ATTACH_RADIUS_M
    half = np.array(MOUSE_PROBE_EXT_XYZ, dtype=np.float64) * GRASP_ATTACH_BOX_SCALE
    in_box = np.all(np.abs(arr - center) <= half[None, :], axis=1)
    mask = mask | in_box
    if not np.any(mask):
        mask = np.zeros(arr.shape[0], dtype=bool)
        mask[np.argsort(dist)[: max(32, arr.shape[0] // 8)]] = True

    _grasp_attach_indices = np.where(mask)[0]
    _grasp_attach_offsets = arr[_grasp_attach_indices] - center
    _grasp_attach_rest_pos = arr.copy()
    _grasp_attach_grip_center = (float(center[0]), float(center[1]), float(center[2]))
    z_span = float(arr[_grasp_attach_indices, 2].max() - arr[_grasp_attach_indices, 2].min()) * 1000.0
    print(
        f"[grasp_demo] grip attachment: {len(_grasp_attach_indices)}/{arr.shape[0]} FEM nodes "
        f"(r<={GRASP_ATTACH_RADIUS_M*1000:.0f} mm or body box), z-span={z_span:.1f} mm",
        flush=True,
    )
    return len(_grasp_attach_indices)


def update_mouse_attachment(deformable, grip_center: tuple[float, float, float]) -> None:
    """Translate the FEM mesh with the grip center during lift (kinematic pin, GUI-visible)."""
    import numpy as np
    import torch

    if _grasp_attach_rest_pos is None or _grasp_attach_grip_center is None:
        return
    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid():
        return

    delta = np.asarray(grip_center, dtype=np.float64) - np.asarray(_grasp_attach_grip_center, dtype=np.float64)
    new_pos = _grasp_attach_rest_pos + delta
    pos_t = torch.as_tensor(new_pos, dtype=torch.float32, device=view._device)
    if pos_t.dim() == 2:
        pos_t = pos_t.unsqueeze(0)
    targets = torch.zeros((pos_t.shape[0], pos_t.shape[1], 4), dtype=torch.float32, device=view._device)
    targets[..., :3] = pos_t
    targets[..., 3] = 1.0
    view.set_simulation_mesh_kinematic_targets(targets)
    zero_deformable_velocities(deformable)


def enable_pinch_mouse_filter(stage, collision_path: str, filter_mode: str) -> None:
    """During pinch with filter=none, disable gripper↔mouse GPU contact to avoid ejection."""
    global _pinch_mouse_filter_added
    if not PINCH_AUTO_MOUSE_FILTER or filter_mode != "none" or _pinch_mouse_filter_added:
        return
    ok = 0
    for a, b in collision_filter_pairs("mouse", collision_path):
        if add_collision_filter_pair(stage, a, b):
            ok += 1
    if ok:
        _pinch_mouse_filter_added = True
        print(
            f"[grasp_demo] pinch: auto mouse collision filter ({ok} pair(s)) — "
            "prevents gripper ejection on deformable contact",
            flush=True,
        )


def clear_pinch_mouse_filter_state() -> None:
    global _pinch_mouse_filter_added
    _pinch_mouse_filter_added = False


def clear_mouse_attachment(deformable=None) -> None:
    global _grasp_attach_indices, _grasp_attach_offsets, _grasp_attach_rest_pos, _grasp_attach_grip_center
    if deformable is not None:
        set_mouse_kinematic_pin(deformable, False)
    _grasp_attach_indices = None
    _grasp_attach_offsets = None
    _grasp_attach_rest_pos = None
    _grasp_attach_grip_center = None


def lower_mesh_z(stage, mesh_path: str, dz: float, *, label: str) -> float:
    """Shift render mesh points by dz (negative = down). Used before deformable cook."""
    from pxr import Gf, UsdGeom
    import numpy as np

    if abs(dz) < 1e-9:
        return 0.0
    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        return 0.0
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    pts[:, 2] += dz
    mesh.GetPointsAttr().Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])
    print(f"[grasp_demo] {label}: shifted mesh z by {dz * 1000:.1f} mm", flush=True)
    return dz


def log_physics_scene(world) -> None:
    ctx = world.get_physics_context()
    direction, magnitude = ctx.get_gravity()
    print(
        f"[grasp_demo] physics: gravity dir={direction} magnitude={magnitude:.3f} m/s^2, "
        f"table_top_z={TABLE_TOP_Z:.3f}",
        flush=True,
    )


def lower_render_mesh_to_table(stage, mesh_path: str) -> float:
    """Drop USD render vertices so the visible shell sits on the table."""
    from pxr import Gf, UsdGeom
    import numpy as np

    contact, _ = get_display_mesh_contact_z(stage, mesh_path)
    dz = contact - TABLE_TOP_Z
    if dz <= 0.0005:
        return 0.0

    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        return 0.0
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    pts[:, 2] -= dz
    mesh.GetPointsAttr().Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])
    print(f"[grasp_demo] lowered render mesh by {dz * 1000:.1f} mm to table", flush=True)
    return dz


def establish_mouse_table_pose(deformable, stage, mesh_path: str) -> None:
    """Align FEM + render mesh to the table, then kinematic-pin."""
    global _mouse_pin_targets
    import torch

    lower_render_mesh_to_table(stage, mesh_path)
    align_deformable_mouse_on_table(deformable, quiet=True)
    _sim_c, sim_m, _coll_c, coll_m = _physics_contact_z(deformable)
    bottoms = [v for v in (sim_m, coll_m) if not math.isnan(v)]
    if bottoms:
        bottom = min(bottoms)
        if bottom > TABLE_TOP_Z + 0.001:
            nudge_sim_mesh_by_dz(deformable, TABLE_TOP_Z - bottom)
            align_deformable_mouse_on_table(deformable, quiet=True)
    lower_render_mesh_to_table(stage, mesh_path)

    view = deformable._deformable_prim_view
    _mouse_pin_targets = view.get_simulation_mesh_nodal_positions(clone=True)
    if not torch.is_tensor(_mouse_pin_targets):
        _mouse_pin_targets = torch.as_tensor(
            _mouse_pin_targets, dtype=torch.float32, device=view._device
        )
    if _mouse_pin_targets.dim() == 2:
        _mouse_pin_targets = _mouse_pin_targets.unsqueeze(0)
    set_mouse_kinematic_pin(deformable, True, positions=_mouse_pin_targets)
    zero_deformable_velocities(deformable)
    log_mesh_layer_gap(stage, mesh_path, deformable, "mouse pinned on table")


def refresh_mouse_table_pin(deformable, stage=None, mesh_path: str | None = None) -> None:
    """Re-apply cached pin without shifting nodes (prevents sim/render fight)."""
    global _mouse_pin_targets
    if _mouse_pin_targets is None:
        if stage is not None and mesh_path is not None:
            establish_mouse_table_pose(deformable, stage, mesh_path)
        return
    set_mouse_kinematic_pin(deformable, True, positions=_mouse_pin_targets)
    zero_deformable_velocities(deformable)


def release_mouse_table_pin(deformable) -> None:
    global _mouse_pin_targets
    set_mouse_kinematic_pin(deformable, False)
    _mouse_pin_targets = None
    print("[grasp_demo] released FEM kinematic pin", flush=True)


def shift_prim_rest_meshes_to_table(stage, mesh_path: str, *, dz: float | None = None) -> None:
    """Lower authored FEM/collision rest points so the cooked hull can touch the table."""
    from pxr import Gf
    import numpy as np

    prim = stage.GetPrimAtPath(mesh_path)
    if not prim.IsValid():
        return

    shifted = 0
    for attr_name in ("physxDeformable:collisionRestPoints", "physxDeformable:simulationRestPoints"):
        attr = prim.GetAttribute(attr_name)
        if not attr or not attr.IsValid():
            continue
        raw = attr.Get()
        if not raw:
            continue
        pts = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float64)
        min_z = float(pts[:, 2].min())
        if dz is None:
            pts[:, 2] += TABLE_TOP_Z - min_z
        else:
            pts[:, 2] += dz
        attr.Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])
        print(
            f"[grasp_demo] shifted {attr_name.split(':')[-1]} min_z {min_z:.3f} "
            f"by {(TABLE_TOP_Z - min_z if dz is None else dz) * 1000:.1f} mm",
            flush=True,
        )
        shifted += 1
    if shifted == 0:
        print(f"[grasp_demo] no deformable rest points on {mesh_path} (cook may still offset collision)", flush=True)


def nudge_sim_mesh_by_dz(deformable, dz: float) -> None:
    import numpy as np
    import torch

    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid() or abs(dz) < 1e-6:
        return

    nodes = view.get_simulation_mesh_nodal_positions()
    if hasattr(nodes, "detach"):
        arr = nodes.detach().cpu().numpy().reshape(-1, 3).copy()
    else:
        arr = np.asarray(nodes).reshape(-1, 3).copy()
    arr[:, 2] += dz
    tensor = torch.as_tensor(arr.reshape(1, -1, 3), dtype=torch.float32, device=view._device)
    view.set_simulation_mesh_nodal_positions(tensor)


def force_mouse_on_table(
    world,
    deformable,
    app,
    mesh_path: str,
    debug: MousePhysicsDebug | None = None,
    robot=None,
) -> tuple[float, float, float]:
    """Put render + FEM on the table (only shift down when floating above it)."""
    stage = world.stage
    release_mouse_table_pin(deformable)
    log_mesh_layer_gap(stage, mesh_path, deformable, "before force on table")

    lower_render_mesh_to_table(stage, mesh_path)
    view = deformable._deformable_prim_view
    if view.is_physics_handle_valid():
        _sim_c, sim_m, _coll_c, coll_m = _physics_contact_z(deformable)
        for label, bottom in (("collision", coll_m), ("sim", sim_m)):
            if math.isnan(bottom) or bottom <= TABLE_TOP_Z + 0.001:
                continue
            dz = TABLE_TOP_Z - bottom
            print(
                f"[grasp_demo] lowering {label} hull by {-dz * 1000:.1f} mm (was {bottom:.3f})",
                flush=True,
            )
            nudge_sim_mesh_by_dz(deformable, dz)
            shift_prim_rest_meshes_to_table(stage, mesh_path, dz=dz)
        align_deformable_mouse_on_table(deformable, quiet=True)
        lower_render_mesh_to_table(stage, mesh_path)

    zero_deformable_velocities(deformable)
    log_mesh_layer_gap(stage, mesh_path, deformable, "after force on table")

    for step_i in range(MOUSE_GRAVITY_SETTLE_STEPS):
        if robot is not None:
            apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=stage)
        world.step(render=False)
        app.update()
        if debug is not None and (step_i + 1) % 30 == 0:
            debug.update(deformable, mesh_path)

    log_mesh_layer_gap(stage, mesh_path, deformable, "after gravity settle")
    return read_mouse_mesh_center(stage, mesh_path)


def bake_mouse_mesh_for_grasp(stage, mesh_path: str) -> tuple[float, float, float]:
    """Bake rotate+translate into mesh points; belly contact plane on table."""
    from pxr import Gf, UsdGeom
    import numpy as np

    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        raise RuntimeError(f"Mesh not found: {mesh_path}")

    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    tris = mesh_triangle_indices(mesh)

    if mouse_mesh_needs_rotation(pts):
        angle = math.radians(MOUSE_ROTATE_Z_DEG)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        rot = np.array([[cos_a, -sin_a, 0.0], [sin_a, cos_a, 0.0], [0.0, 0.0, 1.0]])
        pts = pts @ rot.T
        pts[:, 0] += TABLE_TRANSLATE[0] - float(pts[:, 0].mean())
        pts[:, 1] += TABLE_TRANSLATE[1] - float(pts[:, 1].mean())

    contact_z, min_z, placed_min = shift_points_to_table(pts, tris, pre_sink_m=DEFORMABLE_FLOOR_SINK_M)
    print(
        f"[grasp_demo] bake contact: min_z={min_z:.4f}, contact_z={contact_z:.4f}, "
        f"belly_above_min={(contact_z - min_z) * 1000:.2f} mm, "
        f"placed_min_z={placed_min:.3f} (pre-sunk {DEFORMABLE_FLOOR_SINK_M * 1000:.0f} mm below table "
        f"to compensate for FEM hull expansion)",
        flush=True,
    )

    mesh.GetPointsAttr().Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])

    for path in (mesh_path, f"{MOUSE_ROOT}/Mouse", MOUSE_ROOT):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Xformable(prim).ClearXformOpOrder()

    center = pts.mean(axis=0)
    return float(center[0]), float(center[1]), float(center[2])


def log_visual_mesh_placement(stage, mesh_path: str, label: str) -> None:
    from pxr import UsdGeom
    import numpy as np

    mesh = UsdGeom.Mesh.Get(stage, mesh_path)
    if not mesh:
        print(f"[grasp_demo] {label}: visual mesh not found", flush=True)
        return

    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    tris = mesh_triangle_indices(mesh)
    contact_z, min_z = mesh_bottom_contact_z(pts, tris)
    center = pts.mean(axis=0)
    print(
        f"[grasp_demo] {label}: visual min_z={min_z:.3f}, contact_z={contact_z:.3f} "
        f"(belly {(contact_z - min_z) * 1000:.1f} mm above min), center z={center[2]:.3f}, "
        f"contact_above_table={contact_z - TABLE_TOP_Z:.3f} m",
        flush=True,
    )


def _physics_contact_z(deformable) -> tuple[float, float, float, float]:
    """Return (sim_contact, sim_min, coll_contact, coll_min) for the FEM body."""
    import numpy as np

    view = deformable._deformable_prim_view
    nodes = view.get_simulation_mesh_nodal_positions()
    if hasattr(nodes, "detach"):
        sim = nodes.detach().cpu().numpy().reshape(-1, 3)
    else:
        sim = np.asarray(nodes).reshape(-1, 3)
    sim_contact, sim_min = mesh_bottom_contact_z(sim, nodal_percentile=MOUSE_BOTTOM_CONTACT_NODAL_PCT)

    coll_contact, coll_min = float("nan"), float("nan")
    try:
        coll = view.get_collision_mesh_nodal_positions()
        if hasattr(coll, "detach"):
            coll = coll.detach().cpu().numpy().reshape(-1, 3)
        else:
            coll = np.asarray(coll).reshape(-1, 3)
        if coll.size > 0:
            coll_contact, coll_min = mesh_bottom_contact_z(coll, nodal_percentile=MOUSE_BOTTOM_CONTACT_NODAL_PCT)
    except Exception:
        pass
    return sim_contact, sim_min, coll_contact, coll_min


def snap_collision_hull_to_table(deformable) -> float:
    """Shift FEM nodes so the collision hull (table contact) reaches TABLE_TOP_Z."""
    _sim_c, _sim_m, _coll_c, coll_m = _physics_contact_z(deformable)
    if math.isnan(coll_m):
        return 0.0
    dz = TABLE_TOP_Z - coll_m
    if abs(dz) < 1e-5:
        return 0.0
    nudge_sim_mesh_by_dz(deformable, dz)
    print(f"[grasp_demo] snap collision hull: dz={dz * 1000:.1f} mm (coll min -> {TABLE_TOP_Z:.3f})", flush=True)
    return dz


def align_deformable_mouse_on_table(deformable, *, quiet: bool = False) -> tuple[float, float, float] | None:
    """Shift FEM nodes so the lowest sim/collision vertex sits on the table."""
    import numpy as np
    import torch

    view = deformable._deformable_prim_view
    if not view.is_physics_handle_valid():
        print("[grasp_demo] align: physics handle not ready", flush=True)
        return None

    nodes = view.get_simulation_mesh_nodal_positions()
    if hasattr(nodes, "detach"):
        arr = nodes.detach().cpu().numpy().reshape(-1, 3).copy()
    else:
        arr = np.asarray(nodes).reshape(-1, 3).copy()

    sim_c0, sim_m0, coll_c0, coll_m0 = _physics_contact_z(deformable)
    bottom_before = _physics_bottom_z(deformable)
    if bottom_before <= TABLE_TOP_Z + 0.001:
        center = arr.mean(axis=0)
        if not quiet:
            print(
                f"[grasp_demo] align: skip (bottom {bottom_before:.3f} already on/below table {TABLE_TOP_Z:.3f})",
                flush=True,
            )
        return float(center[0]), float(center[1]), float(center[2])
    arr[:, 2] += TABLE_TOP_Z - bottom_before
    device = view._device
    tensor = torch.as_tensor(arr.reshape(1, -1, 3), dtype=torch.float32, device=device)
    view.set_simulation_mesh_nodal_positions(tensor)
    bottom_after = _physics_bottom_z(deformable)
    center = arr.mean(axis=0)
    if not quiet:
        print(
            f"[grasp_demo] align: bottom min_z {bottom_before:.3f} -> {bottom_after:.3f} (table {TABLE_TOP_Z:.3f}); "
            f"sim min {sim_m0:.3f}->{float(arr[:, 2].min()):.3f}, collision min {coll_m0:.3f}, center z={center[2]:.3f}",
            flush=True,
        )
    return float(center[0]), float(center[1]), float(center[2])


def add_kinematic_table(stage) -> None:
    """Static table slab — matches soft-block grasp workcell height."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    root_path = Sdf.Path("/World/DemoTable")
    UsdGeom.Xform.Define(stage, root_path)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(root_path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*TABLE_TRANSLATE))
    xf.AddScaleOp().Set(Gf.Vec3f(*TABLE_SCALE))

    cube_path = root_path.AppendChild("CollisionCube")
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(1.0)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    rb = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(root_path))
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(True)


def get_gripper_total_opening_m(robot) -> float | None:
    """Return the sum of both finger joint openings in metres (None if unavailable)."""
    import torch

    if not (robot.handles_initialized and robot._articulation_view.is_physics_handle_valid()):
        return None
    positions = robot.get_joint_positions()
    if torch.is_tensor(positions):
        positions = positions.detach().cpu().numpy()
    positions = positions.reshape(-1)
    if positions.shape[0] < 9:
        return None
    return float(positions[7] + positions[8])


def log_gripper_state(robot, label: str) -> None:
    """Print the actual finger joint openings (mm) so we can confirm the gripper truly closes."""
    import torch

    if not (robot.handles_initialized and robot._articulation_view.is_physics_handle_valid()):
        return
    positions = robot.get_joint_positions()
    if torch.is_tensor(positions):
        positions = positions.detach().cpu().numpy()
    positions = positions.reshape(-1)
    if positions.shape[0] >= 9:
        f1, f2 = float(positions[7]), float(positions[8])
        print(
            f"[grasp_demo] {label}: fingers = {f1 * 1000:.1f} / {f2 * 1000:.1f} mm "
            f"(total opening {(f1 + f2) * 1000:.1f} mm, target {GRIPPER_CLAMP_M * 2000:.1f} mm)",
            flush=True,
        )


def add_rigid_mouse_probe(stage, center: tuple[float, float, float]) -> None:
    """Step1 control: kinematic box matching measured collision hull extent."""
    from pxr import Gf, UsdGeom, UsdPhysics

    ext = MOUSE_PROBE_EXT_XYZ
    path = RIGID_MOUSE_PROBE
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2]))
    xf.AddScaleOp().Set(Gf.Vec3d(ext[0], ext[1], ext[2]))

    cube = UsdGeom.Cube.Define(stage, f"{path}/Box")
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.2, 0.9)])

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    rb = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(path))
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(True)

    print(
        f"[grasp_demo] rigid-box probe at ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}), "
        f"ext xyz=({ext[0]*1000:.1f}, {ext[1]*1000:.1f}, {ext[2]*1000:.1f}) mm",
        flush=True,
    )


def hide_mesh_for_probe(stage, mesh_path: str) -> None:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(mesh_path)
    if prim.IsValid():
        UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)


def _link_world_aabb(stage, link_path: str):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(link_path)
    if not prim.IsValid():
        return None
    box = UsdGeom.Imageable(prim).ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_).GetRange()
    if box.IsEmpty():
        return None
    mn, mx = box.GetMin(), box.GetMax()
    return (float(mn[0]), float(mn[1]), float(mn[2])), (float(mx[0]), float(mx[1]), float(mx[2]))



def _finger_collider_aabbs(stage):
    """World AABB union of CollisionAPI prims under each finger link (not whole link)."""
    from pxr import Usd, UsdPhysics

    def _merge(link_path: str):
        root = stage.GetPrimAtPath(link_path)
        if not root.IsValid():
            return None
        mns: list[tuple[float, float, float]] = []
        mxs: list[tuple[float, float, float]] = []
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            aabb = _link_world_aabb(stage, str(prim.GetPath()))
            if aabb is not None:
                mns.append(aabb[0])
                mxs.append(aabb[1])
        if not mns:
            return _link_world_aabb(stage, link_path)
        mn = (min(p[0] for p in mns), min(p[1] for p in mns), min(p[2] for p in mns))
        mx = (max(p[0] for p in mxs), max(p[1] for p in mxs), max(p[2] for p in mxs))
        return mn, mx

    return _merge(LEFT_FINGER_LINK), _merge(RIGHT_FINGER_LINK)


def _measure_pinch_axis_gaps(stage, target_path: str) -> list[str]:
    """Min gap along world X between finger collider AABBs and target AABB (pinch axis)."""
    lines: list[str] = []
    target = _link_world_aabb(stage, target_path)
    left, right = _finger_collider_aabbs(stage)
    if target is None:
        return ["  finger<->target gap: target AABB unavailable"]
    tmn, tmx = target
    if left is not None:
        gap_l = (tmn[0] - left[1][0]) * 1000.0
        lines.append(f"  finger<->target gap X (left collider vs target -X face) = {gap_l:.1f} mm")
    if right is not None:
        gap_r = (right[0][0] - tmx[0]) * 1000.0
        lines.append(f"  finger<->target gap X (right collider vs target +X face) = {gap_r:.1f} mm")
    if left is not None and right is not None:
        min_gap = min(tmn[0] - left[1][0], right[0][0] - tmx[0])
        lines.append(f"  finger<->target min gap X = {min_gap*1000:.1f} mm (>0 => no collider overlap on X)")
    return lines


def _probe_finger_target_contacts(target_path: str) -> list[str]:
    """GPU RigidContactView: fingers vs rigid target (Step3 — works for rigid-box only)."""
    import numpy as np

    lines: list[str] = []
    try:
        from isaacsim.core.api.sensors import RigidContactView

        view = RigidContactView(
            prim_paths_expr=[LEFT_FINGER_LINK, RIGHT_FINGER_LINK],
            filter_paths_expr=[[target_path], [target_path]],
            name="pinch_diagnosis_finger_target",
            max_contact_count=64,
        )
        view.initialize()
        if not view.is_physics_handle_valid():
            return ["  finger<->target contact: probe handle invalid"]
        matrix = view.get_contact_force_matrix(dt=1.0 / 60.0)
        if matrix is None:
            return ["  finger<->target contact: no data"]
        arr = matrix.detach().cpu().numpy() if hasattr(matrix, "detach") else np.asarray(matrix)
        for si, sensor in enumerate(("left_finger", "right_finger")):
            fvec = arr[si, 0]
            mag = float(np.linalg.norm(fvec))
            lines.append(
                f"  finger<->target {sensor}: |force|={mag:.2f} N "
                f"(fx={fvec[0]:.1f}, fy={fvec[1]:.1f}, fz={fvec[2]:.1f})"
            )
    except Exception as exc:
        lines.append(f"  finger<->target contact probe failed: {exc}")
    return lines


def _probe_hand_finger_contacts() -> list[str]:
    """One-shot rigid contact probe: each finger vs panda_hand (GPU-safe filter)."""
    import numpy as np

    lines: list[str] = []
    try:
        from isaacsim.core.api.sensors import RigidContactView

        view = RigidContactView(
            prim_paths_expr=[LEFT_FINGER_LINK, RIGHT_FINGER_LINK],
            filter_paths_expr=[[HAND_LINK], [HAND_LINK]],
            name="pinch_diagnosis_hand_contact",
            max_contact_count=64,
        )
        view.initialize()
        if not view.is_physics_handle_valid():
            return ["  hand<->finger contact: probe handle invalid"]
        matrix = view.get_contact_force_matrix(dt=1.0 / 60.0)
        if matrix is None:
            return ["  hand<->finger contact: no data"]
        arr = matrix.detach().cpu().numpy() if hasattr(matrix, "detach") else np.asarray(matrix)
        for si, sensor in enumerate(("left_finger", "right_finger")):
            fvec = arr[si, 0]
            mag = float(np.linalg.norm(fvec))
            lines.append(
                f"  hand<->finger {sensor}: |force|={mag:.2f} N "
                f"(fx={fvec[0]:.1f}, fy={fvec[1]:.1f}, fz={fvec[2]:.1f})"
            )
    except Exception as exc:
        lines.append(f"  hand<->finger contact probe failed: {exc}")
    return lines


def log_pinch_diagnosis(
    robot,
    deformable,
    mesh_path: str,
    stage,
    *,
    filter_mode: str,
    gripper_cmd_m: float,
    mouse_mode: str = "deformable",
    collision_path: str | None = None,
) -> None:
    """Structured snapshot when pinch stops short — use with each --collision-filter mode."""
    import numpy as np
    import torch

    if collision_path is None:
        collision_path = mouse_collision_path(mesh_path, mouse_mode)

    lines = [f"[grasp_demo] === pinch diagnosis (filter={filter_mode}, mouse={mouse_mode}) ==="]
    offs = read_physx_collision_offsets(stage, collision_path)
    if offs is not None:
        co_s = f"{offs[0]*1000:.2f} mm" if math.isfinite(offs[0]) else "unset (PhysX default)"
        ro_s = f"{offs[1]*1000:.2f} mm" if math.isfinite(offs[1]) else "unset"
        lines.append(
            f"  target physxCollision: contactOffset={co_s} restOffset={ro_s} @ {collision_path}"
        )
    target_mm = GRIPPER_CLAMP_M * 2000.0
    f1 = f2 = total = float("nan")
    v1 = v2 = e1 = e2 = float("nan")

    if robot.handles_initialized and robot._articulation_view.is_physics_handle_valid():
        pos = robot.get_joint_positions()
        if torch.is_tensor(pos):
            pos = pos.detach().cpu().numpy()
        pos = pos.reshape(-1)
        if pos.shape[0] >= 9:
            f1, f2 = float(pos[7]), float(pos[8])
            total = f1 + f2
        try:
            vel = robot.get_joint_velocities()
            if vel is not None:
                if torch.is_tensor(vel):
                    vel = vel.detach().cpu().numpy()
                vel = vel.reshape(-1)
                if vel.shape[0] >= 9:
                    v1, v2 = float(vel[7]), float(vel[8])
        except Exception:
            pass
        try:
            eff = robot.get_joint_efforts()
            if eff is None:
                eff = robot._articulation_view.get_measured_joint_efforts()
            if eff is not None:
                if torch.is_tensor(eff):
                    eff = eff.detach().cpu().numpy()
                eff = eff.reshape(-1)
                if eff.shape[0] >= 9:
                    e1, e2 = float(eff[7]), float(eff[8])
        except Exception:
            pass

    lines.append(
        f"  joints: pos {f1*1000:.1f}/{f2*1000:.1f} mm "
        f"(total {total*1000:.1f}, target {target_mm:.1f}, cmd {gripper_cmd_m*1000:.1f}/finger)"
    )
    if not math.isnan(v1):
        lines.append(f"  joint vel: {v1*1000:.2f}/{v2*1000:.2f} mm/s (~0 => blocked in place)")
    if not math.isnan(e1):
        lines.append(f"  joint effort: {e1:.1f}/{e2:.1f} N (high + vel~0 => pushing against constraint)")

    view = None if deformable is None else deformable._deformable_prim_view
    if view is not None and view.is_physics_handle_valid():
        coll = view.get_collision_mesh_nodal_positions()
        if hasattr(coll, "detach"):
            coll = coll.detach().cpu().numpy()
        coll = np.asarray(coll).reshape(-1, 3)
        coll = coll[np.isfinite(coll).all(axis=1)]
        if coll.size > 0:
            ext = coll.max(axis=0) - coll.min(axis=0)
            hull_w = float(ext[0])
            clearance_mm = (total - hull_w) * 1000.0
            lines.append(
                f"  hull ext xyz=({ext[0]*1000:.1f}, {ext[1]*1000:.1f}, {ext[2]*1000:.1f}) mm"
            )
            lines.append(f"  geometric clearance (opening - hull_X) = {clearance_mm:.1f} mm")
            if clearance_mm > 5.0 and total * 1000.0 > target_mm + 5.0:
                lines.append(
                    "  => hull geometry is NOT the limiter (pads still outside hull by several mm)"
                )
    elif mouse_mode == "rigid-box":
        target = _link_world_aabb(stage, collision_path)
        if target is not None:
            tmn, tmx = target
            ext = (tmx[0] - tmn[0], tmx[1] - tmn[1], tmx[2] - tmn[2])
            hull_w = ext[0]
            clearance_mm = (total - hull_w) * 1000.0
            lines.append(
                f"  rigid box ext xyz=({ext[0]*1000:.1f}, {ext[1]*1000:.1f}, {ext[2]*1000:.1f}) mm"
            )
            lines.append(f"  geometric clearance (opening - box_X) = {clearance_mm:.1f} mm")

    lines.extend(_measure_pinch_axis_gaps(stage, collision_path))
    lines.extend(_probe_hand_finger_contacts())
    if mouse_mode == "rigid-box":
        lines.extend(_probe_finger_target_contacts(collision_path))
    lines.append("[grasp_demo] === end pinch diagnosis ===")
    print("\n".join(lines), flush=True)


def print_hand_pose(stage, robot, hand_view, label: str) -> None:
    import numpy as np
    import torch

    if robot.handles_initialized and robot._articulation_view.is_physics_handle_valid():
        positions = robot.get_joint_positions()
        if torch.is_tensor(positions):
            positions = positions.detach().cpu().numpy()
        j1 = math.degrees(float(positions[0]))
        j2 = math.degrees(float(positions[1]))
        print(f"[grasp_demo] {label}: joint1={j1:.1f} deg joint2={j2:.1f} deg", flush=True)

    if hand_view is not None:
        pos, quat = get_hand_world_pose(hand_view)
        mouse_dist = math.dist(pos.tolist(), MOUSE_TRANSLATE)
        print(
            f"[grasp_demo] {label}: panda_hand world pos = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), "
            f"dist_to_mouse={mouse_dist:.3f} m",
            flush=True,
        )
        return

    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(HAND_LINK)
    if not prim.IsValid():
        print(f"[grasp_demo] {label}: hand link not found", flush=True)
        return
    pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    print(f"[grasp_demo] {label}: panda_hand world pos = ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) [usd]", flush=True)


def build_scene(
    app,
    mouse_usd: Path,
    *,
    show_fem_nodes: bool = False,
    show_gripper_contacts: bool = False,
    collision_filter_mode: str = "all",
    mouse_mode: str = "deformable",
    tune_contact_offsets: bool = True,
    deform_contact_offset_m: float = DEFAULT_DEFORM_CONTACT_OFFSET_M,
    deform_rest_offset_m: float = DEFAULT_DEFORM_REST_OFFSET_M,
    finger_contact_offset_m: float = DEFAULT_FINGER_CONTACT_OFFSET_M,
    finger_rest_offset_m: float = DEFAULT_FINGER_REST_OFFSET_M,
):
    import isaacsim.core.utils.stage as stage_utils
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.materials.deformable_material import DeformableMaterial
    from isaacsim.core.prims import SingleArticulation, SingleDeformablePrim
    from isaacsim.storage.native import get_assets_root_path
    from pxr import UsdLux

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        backend="torch",
        device="cuda",
    )
    world.get_physics_context().enable_gpu_dynamics(True)
    log_physics_scene(world)

    world.scene.add_default_ground_plane(
        z_position=0.0,
        name="ground",
        prim_path="/World/GroundPlane",
        static_friction=0.8,
        dynamic_friction=0.7,
        restitution=0.0,
    )

    add_kinematic_table(world.stage)

    mouse_root = "/World/MouseAsset"
    source_mouse_usd = mouse_usd.resolve()
    mouse_asset = BAKED_MOUSE_USD if BAKED_MOUSE_USD.exists() else source_mouse_usd
    stage_utils.add_reference_to_stage(str(mouse_asset), mouse_root)

    franka_usd = get_assets_root_path() + FRANKA_USD_REL
    stage_utils.add_reference_to_stage(franka_usd, FRANKA_ROOT)

    usd_context = omni.usd.get_context()
    wait_stage_loaded(usd_context, app)

    mesh_path = find_first_mesh_path(world.stage, mouse_root)
    if mouse_asset == source_mouse_usd:
        mouse_center = bake_mouse_mesh_for_grasp(world.stage, mesh_path)
        update_mouse_grasp_targets(mouse_center)
        write_baked_mouse_usd(Path(source_mouse_usd), BAKED_MOUSE_USD, world.stage, mesh_path)
    else:
        mouse_center = read_mouse_mesh_center(world.stage, mesh_path)
        update_mouse_grasp_targets(mouse_center)
        print(f"[grasp_demo] loaded baked mesh pose from {BAKED_MOUSE_USD.name}", flush=True)

    print(f"[grasp_demo] mouse asset: {source_mouse_usd}", flush=True)
    print(f"[grasp_demo] mouse mesh: {mesh_path}", flush=True)
    print(f"[grasp_demo] mouse at {mouse_center}, rotateZ={MOUSE_ROTATE_Z_DEG} deg", flush=True)
    print(
        f"[grasp_demo] table top z={TABLE_TOP_Z:.3f}, mouse body L×W="
        f"{MOUSE_MESH_BODY_LENGTH_M * 1000:.0f}×{MOUSE_MESH_BODY_WIDTH_M * 1000:.0f} mm (joint side-pinch)",
        flush=True,
    )

    # Place robot base directly behind the mouse (same Y) so the arm can arch up and descend
    # straight down onto it. 0.45 m back keeps the forearm clear of the ~18 cm table edge,
    # leaving a clean vertical column above the mouse for a top-down pinch.
    franka_base_xyz = (mouse_center[0] - 0.45, mouse_center[1], 0.0)
    set_translate(world.stage, FRANKA_ROOT, franka_base_xyz)
    print(
        f"[grasp_demo] franka base placed behind mouse for top-down grasp: "
        f"({franka_base_xyz[0]:.3f}, {franka_base_xyz[1]:.3f}, {franka_base_xyz[2]:.3f})",
        flush=True,
    )

    configure_franka_drives(world.stage, ARM_HOME_DEG, GRIPPER_OPEN_M)
    collision_path = mouse_collision_path(mesh_path, mouse_mode)
    setup_gripper_collision_filters(world.stage, collision_path, collision_filter_mode)

    deformable = None
    if mouse_mode == "rigid-box":
        hide_mesh_for_probe(world.stage, mesh_path)
        add_rigid_mouse_probe(world.stage, mouse_center)
        print("[grasp_demo] mouse-mode=rigid-box: FEM disabled, using kinematic probe for Step1", flush=True)
        if tune_contact_offsets:
            apply_physx_collision_offsets(
                world.stage,
                f"{RIGID_MOUSE_PROBE}/Box",
                deform_contact_offset_m,
                deform_rest_offset_m,
                label="probe box",
            )
    else:
        mat_path = f"{mouse_root}/DeformableMaterial"
        deformable_mat = DeformableMaterial(
            prim_path=mat_path,
            youngs_modulus=YOUNG_MODULUS,
            poissons_ratio=POISSON_RATIO,
            dynamic_friction=DEFORMABLE_FRICTION,
            elasticity_damping=DEFORMABLE_DAMPING,
        )

        deformable = SingleDeformablePrim(
            prim_path=mesh_path,
            name="mouse_deformable",
            deformable_material=deformable_mat,
            simulation_hexahedral_resolution=SIM_HEX_RESOLUTION,
            self_collision=False,
            collision_simplification=False,
            vertex_velocity_damping=0.2,
            sleep_threshold=0.005,
            solver_position_iteration_count=16,
        )
        world.scene.add(deformable)
        shift_prim_rest_meshes_to_table(world.stage, mesh_path)

        if tune_contact_offsets:
            apply_physx_collision_offsets(
                world.stage,
                mesh_path,
                deform_contact_offset_m,
                deform_rest_offset_m,
                label="mouse",
            )
        else:
            print("[grasp_demo] contact offsets: mouse unchanged (PhysX defaults)", flush=True)

    robot = SingleArticulation(prim_path=FRANKA_ROOT, name="factory_franka")
    world.scene.add(robot)

    if tune_contact_offsets:
        set_gripper_collider_offsets(
            world.stage, finger_contact_offset_m, finger_rest_offset_m
        )
    elif mouse_mode == "deformable":
        print("[grasp_demo] contact offsets: gripper unchanged (PhysX defaults)", flush=True)

    light_path = "/World/DomeLight"
    if not world.stage.GetPrimAtPath(light_path).IsValid():
        dome = UsdLux.DomeLight.Define(world.stage, light_path)
        dome.CreateIntensityAttr(800.0)

    debug = MousePhysicsDebug(
        world.stage, mesh_path, enabled=True, show_node_clouds=show_fem_nodes
    )
    gripper_debug = GripperBlockerDebug(
        world.stage, collision_path, enabled=True, enable_contacts=show_gripper_contacts
    )
    debug.ensure_prims()
    # gripper collider debug builds after sim play (see prepare_run)
    print(
        "[grasp_demo] physics debug ON:\n"
        "  white mesh  = render/visual (dimmed to ~15% opacity for debug)\n"
        "  orange mesh = live PhysX collision tet hull (/World/DebugMouse/CollisionTetMesh)\n"
        "  orange wire = hull surface edges (/World/DebugMouse/CollisionWire)\n"
        "  orange box  = hull AABB (/World/DebugMouse/CollisionHullBBox) — always visible in RTX\n"
        "  green mesh  = gripper rigid colliders (under finger/hand links as PhysXColliderDebug_*)\n"
        + (
            "  red points  = live PhysX contacts (/World/DebugGripper/ContactPoints)\n"
            if show_gripper_contacts
            else "  (contact points off — pass --show-gripper-contacts to enable)\n"
        )
        + (
            "  cyan points = FEM sim nodes (/World/DebugMouse/SimNodes)\n"
            "  orange pts  = collision nodes (/World/DebugMouse/CollisionNodes)"
            if show_fem_nodes
            else "  (SimNodes/CollisionNodes hidden — pass --show-fem-nodes to show)"
        ),
        flush=True,
    )

    return world, robot, deformable, mesh_path, debug, gripper_debug, mouse_mode, collision_path


def update_mouse_grasp_targets(mouse_center: tuple[float, float, float]) -> None:
    globals()["MOUSE_TRANSLATE"] = mouse_center


def prepare_run(
    world,
    robot,
    deformable,
    app,
    mesh_path: str,
    debug: MousePhysicsDebug | None = None,
    gripper_debug: GripperBlockerDebug | None = None,
    *,
    headless: bool = False,
    mouse_mode: str = "deformable",
) -> tuple:
    global _mouse_pin_targets
    _mouse_pin_targets = None
    print(f"[grasp_demo] reset + pin mouse on table (mouse-mode={mouse_mode})", flush=True)
    world.reset()

    mouse_center = read_mouse_mesh_center(world.stage, mesh_path)
    update_mouse_grasp_targets(mouse_center)
    log_visual_mesh_placement(world.stage, mesh_path, "after reset")

    world.play()
    for _ in range(2):
        world.step(render=False)
        app.update()

    if not robot.handles_initialized:
        robot.initialize()
    setup_robot_gains(robot)
    apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=world.stage)
    if deformable is not None:
        try:
            deformable.initialize()
        except Exception:
            pass
    if gripper_debug is not None:
        gripper_debug._logged_blockers = False
        gripper_debug._contact_view = None
        gripper_debug._built = False

    if mouse_mode == "rigid-box":
        for _ in range(MOUSE_PIN_SETTLE_STEPS):
            apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=world.stage)
            world.step(render=not headless)
            app.update()
    else:
        establish_mouse_table_pose(deformable, world.stage, mesh_path)
        force_mouse_on_table(world, deformable, app, mesh_path, debug, robot=robot)
        establish_mouse_table_pose(deformable, world.stage, mesh_path)

        for attempt in range(5):
            _sim_c, _sim_m, _coll_c, coll_m = _physics_contact_z(deformable)
            if math.isnan(coll_m) or coll_m <= TABLE_TOP_Z + 0.003:
                break
            print(
                f"[grasp_demo] re-seat attempt {attempt + 1}: collision min {coll_m:.3f} "
                f"> table {TABLE_TOP_Z:.3f}, snapping down",
                flush=True,
            )
            snap_collision_hull_to_table(deformable)
            align_deformable_mouse_on_table(deformable, quiet=True)
            lower_render_mesh_to_table(world.stage, mesh_path)
            for _ in range(20):
                apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=world.stage)
                world.step(render=not headless)
                app.update()
        establish_mouse_table_pose(deformable, world.stage, mesh_path)

        settled = read_mouse_mesh_center(world.stage, mesh_path)
        update_mouse_grasp_targets(settled)
        for _ in range(MOUSE_PIN_SETTLE_STEPS):
            apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=world.stage)
            refresh_mouse_table_pin(deformable)
            world.step(render=not headless)
            app.update()
            if debug is not None:
                debug.update(deformable, mesh_path)
            if gripper_debug is not None and not headless:
                gripper_debug.update(robot)

    for _ in range(ARM_DRIVE_SETTLE_STEPS):
        apply_robot_pose(robot, ARM_HOME_DEG, GRIPPER_OPEN_M, stage=world.stage)
        world.step(render=not headless)
        app.update()

    if gripper_debug is not None:
        gripper_debug.ensure_prims()
        gripper_debug.initialize_contacts()

    _, hand_view = create_hand_view()

    log_visual_mesh_placement(world.stage, mesh_path, "ready for grasp")
    if debug is not None and deformable is not None:
        debug.update(deformable, mesh_path)
    if gripper_debug is not None:
        gripper_debug.update(robot)
    return hand_view


def run_demo_sequence(
    world,
    robot,
    deformable,
    app,
    mesh_path: str,
    debug: MousePhysicsDebug | None = None,
    gripper_debug: GripperBlockerDebug | None = None,
    *,
    headless: bool = False,
    diagnose_pinch: bool = False,
    collision_filter_mode: str = "all",
    mouse_mode: str = "deformable",
    collision_path: str | None = None,
    pinch_mode: str = "position",
) -> bool:
    if collision_path is None:
        collision_path = mouse_collision_path(mesh_path, mouse_mode)

    milestone_steps = {0, STEPS_APPROACH, STEPS_DESCEND}
    milestone_names = {
        0: "home",
        STEPS_APPROACH: "pre_grasp",
        STEPS_DESCEND: "straddle",
    }

    hand_view = prepare_run(
        world,
        robot,
        deformable,
        app,
        mesh_path,
        debug,
        gripper_debug,
        headless=headless,
        mouse_mode=mouse_mode,
    )
    initial_mouse_z = get_mouse_world_pos(deformable, world.stage, mouse_mode=mouse_mode)[2]
    print(f"[grasp_demo] robot dofs={robot.num_dof} names={robot.dof_names}", flush=True)
    print(
        f"[grasp_demo] mouse-mode={mouse_mode}, collision-path={collision_path}, "
        f"filter={collision_filter_mode}, pinch-mode={pinch_mode}, table_top_z={TABLE_TOP_Z:.3f}, "
        f"pinch target {GRIPPER_CLAMP_M * 2000:.0f} mm",
        flush=True,
    )
    if pinch_mode in ("force", "compliant") and collision_filter_mode != "none":
        print(
            "[grasp_demo] warning: force/compliant pinch works best with --collision-filter none "
            "(full gripper↔mouse contact)",
            flush=True,
        )
    if deformable is not None:
        log_mouse_placement(deformable, "mouse placement")
    print_hand_pose(world.stage, robot, hand_view, "after init")

    pin_released = False
    clear_mouse_attachment(deformable)
    clear_pinch_mouse_filter_state()
    step = 0
    grasp_success = False
    phase = "approach"  # approach → descend → pinch → lift → done
    pinch_anchor: tuple[float, float, float] | None = None
    grasp_anchor: tuple[float, float, float] | None = None
    pinch_steps = 0
    lift_steps = 0
    attachment_done = False
    pinch_arm_q = None
    pinch_hold_start: int | None = None
    pinch_hold_gripper_m = GRIPPER_CLAMP_M
    pinch_diag_logged = False
    opening_history: list[float] = []
    compliant_cmd_m = GRIPPER_OPEN_M
    close_effort_sign = -1.0

    while phase != "done" and step < MAX_GRASP_STEPS:
        mouse_now = get_mouse_world_pos(deformable, world.stage, mouse_mode=mouse_mode)
        target_xy = (mouse_now[0], mouse_now[1])
        hover_z = mouse_now[2] + GRASP_HOVER_Z_OFFSET_M
        straddle_z = mouse_now[2] + GRASP_STRADDLE_Z_OFFSET_M
        force_grip = False

        if phase == "approach":
            target_xyz = (target_xy[0], target_xy[1], hover_z)
            gripper_m = GRIPPER_OPEN_M
            if step >= STEPS_APPROACH:
                phase = "descend"
        elif phase == "descend":
            t = (step - STEPS_APPROACH) / max(1, STEPS_DESCEND - STEPS_APPROACH)
            target_xyz = (target_xy[0], target_xy[1], hover_z + (straddle_z - hover_z) * t)
            gripper_m = GRIPPER_OPEN_M
            if step >= STEPS_DESCEND:
                phase = "pinch"
                pinch_anchor = (target_xy[0], target_xy[1], straddle_z)
                pinch_steps = 0
                pinch_hold_start = None
                opening_history.clear()
                compliant_cmd_m = GRIPPER_OPEN_M
                if pinch_mode == "position":
                    msg = (
                        f"force clamp to {GRIPPER_CLAMP_M * 2000:.0f} mm total, "
                        f"hold {GRIPPER_CLAMP_HOLD_STEPS} frames, then lift"
                    )
                elif pinch_mode == "force":
                    msg = (
                        f"effort close {GRIPPER_PINCH_EFFORT_N:.0f} N/finger, "
                        f"stall hold {GRIPPER_CLAMP_HOLD_STEPS} frames, then lift"
                    )
                else:
                    msg = (
                        f"compliant creep toward {GRIPPER_CLAMP_M * 2000:.0f} mm total, "
                        f"stall hold {GRIPPER_CLAMP_HOLD_STEPS} frames, then lift"
                    )
                print(f"[grasp_demo] phase pinch: {msg}", flush=True)
        elif phase == "pinch":
            assert pinch_anchor is not None
            pinch_steps += 1
            if pinch_steps == 1:
                import torch

                pos = robot.get_joint_positions()
                if pos is not None:
                    if torch.is_tensor(pos):
                        pinch_arm_q = pos.detach().clone().reshape(-1)[:7]
                    else:
                        pinch_arm_q = torch.as_tensor(pos, dtype=torch.float32).reshape(-1)[:7]
                if pinch_mode == "force":
                    enable_gripper_force_pinch(robot)
                elif pinch_mode == "compliant":
                    enable_gripper_compliant_pinch(robot, world.stage)
                enable_pinch_mouse_filter(world.stage, collision_path, collision_filter_mode)

            target_xyz = pinch_anchor
            finger_state = get_finger_joint_state(robot)
            if finger_state is not None:
                f1, f2, v1, v2 = finger_state
                opening_history.append(f1 + f2)
            else:
                f1 = f2 = v1 = v2 = 0.0

            if pinch_mode == "position":
                if pinch_steps <= GRIPPER_CLOSE_STEPS:
                    close_t = pinch_steps / max(1, GRIPPER_CLOSE_STEPS)
                    gripper_m = GRIPPER_OPEN_M + (GRIPPER_CLAMP_M - GRIPPER_OPEN_M) * close_t
                    force_grip = False
                elif pinch_steps <= GRIPPER_CLOSE_STEPS + GRIPPER_SQUEEZE_STEPS:
                    gripper_m = GRIPPER_CLAMP_M
                    force_grip = True
                else:
                    gripper_m = GRIPPER_CLAMP_M
                    force_grip = True
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        log_gripper_state(robot, "squeeze_done")
            elif pinch_mode == "force":
                gripper_m = (f1 + f2) / 2.0 if finger_state else GRIPPER_OPEN_M
                force_grip = False
                if pinch_steps == 12 and len(opening_history) >= 2:
                    if opening_history[-1] >= opening_history[0] - 0.0005:
                        close_effort_sign = 1.0
                        print(
                            "[grasp_demo] force pinch: flipped closing effort sign (+)",
                            flush=True,
                        )
                target_total = GRIPPER_CLAMP_M * 2.0
                if finger_state and (f1 + f2) <= target_total * 1.05:
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        print(
                            f"[grasp_demo] force pinch: target opening reached "
                            f"({(f1+f2)*1000:.1f} mm)",
                            flush=True,
                        )
                elif pinch_stalled(opening_history, v1, v2):
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        print(
                            f"[grasp_demo] force pinch: stalled at "
                            f"{opening_history[-1]*1000:.1f} mm total opening",
                            flush=True,
                        )
                elif pinch_steps >= GRIPPER_FORCE_PINCH_MAX_STEPS:
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        last_mm = opening_history[-1] * 1000.0 if opening_history else float("nan")
                        print(
                            f"[grasp_demo] force pinch: max steps at {last_mm:.1f} mm total opening",
                            flush=True,
                        )
            else:  # compliant
                if finger_state:
                    cur = min(f1, f2)
                    compliant_cmd_m = max(GRIPPER_CLAMP_M, cur - GRIPPER_COMPLIANT_CREEP_M)
                gripper_m = compliant_cmd_m
                force_grip = False
                target_total = GRIPPER_CLAMP_M * 2.0
                if finger_state and (f1 + f2) <= target_total * 1.05:
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        print(
                            f"[grasp_demo] compliant pinch: target opening reached "
                            f"({(f1+f2)*1000:.1f} mm)",
                            flush=True,
                        )
                elif pinch_stalled(opening_history, v1, v2):
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        print(
                            f"[grasp_demo] compliant pinch: stalled at "
                            f"{opening_history[-1]*1000:.1f} mm total opening",
                            flush=True,
                        )
                elif pinch_steps >= GRIPPER_FORCE_PINCH_MAX_STEPS:
                    if pinch_hold_start is None:
                        pinch_hold_start = pinch_steps
                        print("[grasp_demo] compliant pinch: max steps reached", flush=True)

            if pinch_hold_start is not None:
                if pinch_hold_start == pinch_steps:
                    pinch_hold_gripper_m = gripper_m
                    if pinch_mode in ("force", "compliant"):
                        restore_gripper_position_hold(robot, gripper_m)
                if pinch_steps >= pinch_hold_start + GRIPPER_CLAMP_HOLD_STEPS:
                    phase = "lift"
                    grasp_anchor = pinch_anchor
                    lift_steps = 0
                    log_gripper_state(robot, "lift_start")
                    if deformable is not None and mouse_mode == "deformable":
                        hand_pos0, _ = get_hand_world_pose(hand_view)
                        grip_center0 = (
                            float(hand_pos0[0]),
                            float(hand_pos0[1]),
                            float(hand_pos0[2]) - GRASP_TCP_OFFSET_M,
                        )
                        n = attach_mouse_to_hand(deformable, grip_center0)
                        attachment_done = n > 0
                        if attachment_done:
                            update_mouse_attachment(deformable, grip_center0)
                    print(
                        f"[grasp_demo] pinch hold complete at step {step} — start vertical lift from "
                        f"({grasp_anchor[0]:.3f}, {grasp_anchor[1]:.3f}, {grasp_anchor[2]:.3f})",
                        flush=True,
                    )
        else:  # lift
            assert grasp_anchor is not None
            lift_steps += 1
            t = min(1.0, lift_steps / max(1, LIFT_DURATION_STEPS))
            lift_delta = GRASP_LIFT_Z_OFFSET_M - GRASP_STRADDLE_Z_OFFSET_M
            target_xyz = (grasp_anchor[0], grasp_anchor[1], grasp_anchor[2] + lift_delta * t)
            gripper_m = GRIPPER_CLAMP_M if pinch_mode == "position" else pinch_hold_gripper_m
            force_grip = False  # hold opening during lift — re-squeeze blows finger links off
            if lift_steps >= LIFT_DURATION_STEPS:
                phase = "done"

        if phase == "pinch" and pinch_arm_q is not None:
            if pinch_mode == "force":
                apply_frozen_arm_only(robot, pinch_arm_q)
                apply_gripper_closing_effort(robot, sign=close_effort_sign)
            elif pinch_mode == "compliant":
                apply_frozen_arm_gripper(robot, pinch_arm_q, gripper_m)
            else:
                apply_frozen_arm_gripper(
                    robot, pinch_arm_q, gripper_m, soft_close=force_grip
                )
        else:
            apply_cartesian_ik_step(robot, hand_view, target_xyz, gripper_m)

        if deformable is not None:
            if phase in ("approach", "descend"):
                refresh_mouse_table_pin(deformable)
            elif not pin_released:
                release_mouse_table_pin(deformable)
                pin_released = True

        if phase == "lift" and attachment_done and deformable is not None:
            hand_pos_l, _ = get_hand_world_pose(hand_view)
            grip_center = (
                float(hand_pos_l[0]),
                float(hand_pos_l[1]),
                float(hand_pos_l[2]) - GRASP_TCP_OFFSET_M,
            )
            update_mouse_attachment(deformable, grip_center)

        if step in milestone_steps:
            label = milestone_names[step]
            print(f"[grasp_demo] pose step {step}: {label}", flush=True)
            print_hand_pose(world.stage, robot, hand_view, label)
            log_gripper_state(robot, label)
        elif phase == "pinch" and pinch_mode != "position" and pinch_hold_start is not None and not pinch_diag_logged:
            pinch_diag_logged = True
            label = f"pinch ({pinch_mode})"
            print(f"[grasp_demo] pose step {step}: {label} hold started", flush=True)
            log_gripper_state(robot, "pinch_hold")
            if debug is not None and deformable is not None:
                debug.log_extent(deformable, "pinch_hold")
            log_pinch_diagnosis(
                robot,
                deformable,
                mesh_path,
                world.stage,
                filter_mode=collision_filter_mode,
                gripper_cmd_m=gripper_m,
                mouse_mode=mouse_mode,
                collision_path=collision_path,
            )
            if gripper_debug is not None:
                gripper_debug.log_blockers(robot, "pinch_stuck", force=True)
            if diagnose_pinch:
                print(
                    "[grasp_demo] diagnose-pinch: stopping after pinch hold (no lift). "
                    "See debug.md Step4",
                    flush=True,
                )
                phase = "done"
                break
        elif phase == "pinch" and pinch_mode == "position" and pinch_steps == GRIPPER_CLOSE_STEPS:
            print(f"[grasp_demo] pose step {step}: pinch (clamp target reached)", flush=True)
            log_gripper_state(robot, "pinch_cmd")
            if debug is not None and deformable is not None:
                debug.log_extent(deformable, "pinch_cmd")
            log_pinch_diagnosis(
                robot,
                deformable,
                mesh_path,
                world.stage,
                filter_mode=collision_filter_mode,
                gripper_cmd_m=gripper_m,
                mouse_mode=mouse_mode,
                collision_path=collision_path,
            )
            if gripper_debug is not None:
                gripper_debug.log_blockers(robot, "pinch_stuck", force=True)
            if diagnose_pinch:
                print(
                    "[grasp_demo] diagnose-pinch: stopping after pinch (no lift). "
                    "Re-run with other --collision-filter modes — see debug.md",
                    flush=True,
                )
                phase = "done"
                break

        world.step(render=not headless)
        app.update()
        if debug is not None and deformable is not None:
            debug.update(deformable, mesh_path)
        if gripper_debug is not None and not headless:
            gripper_debug.update(robot)
        step += 1

        if phase == "lift" and step >= GRASP_CHECK_START:
            mouse_pos = get_mouse_world_pos(deformable, world.stage, mouse_mode=mouse_mode)
            hand_pos, hand_quat = get_hand_world_pose(hand_view)
            if check_grasp_success(initial_mouse_z, mouse_pos, hand_pos, hand_quat, gripper_m):
                grasp_success = True
                lifted = mouse_pos[2] > initial_mouse_z + GRASP_LIFT_DELTA_M
                mode = "mouse lifted" if lifted else "top-down pinch"
                print(
                    f"[grasp_demo] grasp confirmed at step {step} ({mode}): mouse z={mouse_pos[2]:.3f} "
                    f"(+{mouse_pos[2] - initial_mouse_z:.3f} m)",
                    flush=True,
                )
                print_hand_pose(world.stage, robot, hand_view, "grasp_ok")
                log_gripper_state(robot, "grasp_ok")
                break

    if not grasp_success:
        mouse_pos = get_mouse_world_pos(deformable, world.stage, mouse_mode=mouse_mode)
        print(
            f"[grasp_demo] grasp not confirmed (mouse z={mouse_pos[2]:.3f}, "
            f"delta={mouse_pos[2] - initial_mouse_z:.3f} m, phase={phase})",
            flush=True,
        )
        print_hand_pose(world.stage, robot, hand_view, "final")
        log_gripper_state(robot, "final")

    return grasp_success


def export_stage(export_path: Path) -> None:
    import omni.usd

    export_path.parent.mkdir(parents=True, exist_ok=True)
    omni.usd.get_context().save_as_stage(str(export_path))
    print(f"[grasp_demo] exported scene → {export_path}", flush=True)


def main() -> int:
    if not os.environ.get("OMNI_KIT_ACCEPT_EULA"):
        os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

    args = parse_args()
    if args.rebake and BAKED_MOUSE_USD.exists():
        BAKED_MOUSE_USD.unlink()
        print(f"[grasp_demo] deleted stale baked asset → {BAKED_MOUSE_USD}", flush=True)
    if not args.mouse_usd.exists():
        print(f"Missing mouse USD: {args.mouse_usd}", file=sys.stderr)
        return 1

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    exit_code = 0

    try:
        world, robot, deformable, mesh_path, debug, gripper_debug, mouse_mode, collision_path = build_scene(
            simulation_app,
            args.mouse_usd,
            show_fem_nodes=args.show_fem_nodes,
            show_gripper_contacts=args.show_gripper_contacts,
            collision_filter_mode=args.collision_filter,
            mouse_mode=args.mouse_mode,
            tune_contact_offsets=not args.no_contact_offset_tuning,
            deform_contact_offset_m=args.deform_contact_offset_mm / 1000.0,
            deform_rest_offset_m=args.deform_rest_offset_mm / 1000.0,
            finger_contact_offset_m=args.finger_contact_offset_mm / 1000.0,
            finger_rest_offset_m=args.finger_rest_offset_mm / 1000.0,
        )
        if args.hide_collision_debug:
            debug.enabled = False
        if args.hide_gripper_debug:
            gripper_debug.enabled = False

        if not args.headless:
            from isaacsim.core.utils.viewports import set_camera_view

            set_camera_view(
                eye=[0.85, -0.50, 0.55],
                target=[0.52, 0.35, 0.20],
                camera_prim_path="/OmniverseKit_Persp",
            )

        replay = ReplayController()
        if not args.headless:
            replay.start()

        try:
            while simulation_app.is_running():
                run_demo_sequence(
                    world,
                    robot,
                    deformable,
                    simulation_app,
                    mesh_path,
                    debug,
                    gripper_debug,
                    headless=args.headless,
                    diagnose_pinch=args.diagnose_pinch,
                    collision_filter_mode=args.collision_filter,
                    mouse_mode=mouse_mode,
                    collision_path=collision_path,
                    pinch_mode=args.pinch_mode,
                )

                if args.headless or args.diagnose_pinch:
                    extra_steps = max(0, int(args.duration * 60))
                    for _ in range(extra_steps):
                        if not simulation_app.is_running():
                            break
                        world.step(render=False)
                        simulation_app.update()
                    break

                if not replay.wait(world, simulation_app, deformable, headless=False):
                    break
        finally:
            replay.stop()

        if not args.no_export:
            export_stage(args.export_scene)

    except Exception:
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
