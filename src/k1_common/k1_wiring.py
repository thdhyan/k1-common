"""Canonical Booster K1 joint order / wiring spec (pure python, stdlib only).

Single source of truth for joint ORDER, names, limits, default poses, PD gains
and action scale.  Everything here is asserted against the real URDF / MJCF /
USD / SDK header by ``tests/test_k1_wiring.py``.  Narrative + evidence with
file:line is in ``docs/k1_joint_order_canonical.md``.

Four orders exist for the same 22 joints:

  SDK    : ``booster::robot::b1::JointIndexK1`` = motor_cmd[i] / motor_state_*[i]
           = real robot ``/joint_states`` order = URDF file order = MJCF
           joint/actuator order.  Call it the CANONICAL (a.k.a. "deploy") order.
  BFS    : Isaac Lab / PhysX articulation order (``robot.joint_names``), what a
           ``find_joints(names)`` call returns when ``preserve_order=False``.
  LEG    : the 12-dim leg slice used by policy code.  Two layouts:
           LEG_DEPLOY_ORDER = [L x6, R x6]   (locomotion_node, docs, run_sim)
           LEG_BFS_ORDER    = interleaved    (what every PhysX-trained leg
                                              policy actually saw, see doc)

Conventions: radians, rad/s, Nm.  Index maps are plain lists ``perm`` such that
``dst[i] == src[perm[i]]``; use :func:`reorder` / :func:`index_map`.
"""
from __future__ import annotations

import math

# --------------------------------------------------------------------------
# 1. Canonical (SDK == URDF == MJCF == real /joint_states) order, 22 DoF
# --------------------------------------------------------------------------
# Names are the URDF/MJCF spellings, including the odd "A" prefixes.
JOINTS = [
    "AAHead_yaw", "Head_pitch",                                                   # 0-1
    "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",      # 2-5
    "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",  # 6-9
    "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch",         # 10-13
    "Left_Ankle_Pitch", "Left_Ankle_Roll",                                        # 14-15
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",     # 16-19
    "Right_Ankle_Pitch", "Right_Ankle_Roll",                                      # 20-21
]
N_JOINTS = 22

# SDK enum names (b1_api_const.hpp JointIndexK1), same order as JOINTS.
SDK_ENUM_NAMES = [
    "kHeadYaw", "kHeadPitch",
    "kLeftShoulderPitch", "kLeftShoulderRoll", "kLeftElbowPitch", "kLeftElbowYaw",
    "kRightShoulderPitch", "kRightShoulderRoll", "kRightElbowPitch", "kRightElbowYaw",
    "kLeftHipPitch", "kLeftHipRoll", "kLeftHipYaw", "kLeftKneePitch",
    "kCrankUpLeft", "kCrankDownLeft",
    "kRightHipPitch", "kRightHipRoll", "kRightHipYaw", "kRightKneePitch",
    "kCrankUpRight", "kCrankDownRight",
]
SDK_INDEX = {n: i for i, n in enumerate(JOINTS)}

# The real robot's own /joint_states (recorded bag, 2026-09-29) spells two
# names with a capital: match case-insensitively when looking up by name.
REAL_JOINT_STATES_NAMES = ["AAHead_Yaw", "Head_Pitch"] + JOINTS[2:]

# SDK LowCmd.cmd_type values (booster_ros2_interface LowCmd.msg).
CMD_TYPE_PARALLEL = 0
CMD_TYPE_SERIAL = 1
# From the recorded bag: /joint_states == LowState.motor_state_SERIAL
# (ankle pitch/roll space).  motor_state_PARALLEL[14,15,20,21] are crank-space
# motor angles and must NOT be read as ankle angles.  All other 18 joints are
# identical in both arrays.
STATE_ARRAY_FOR_JOINT_SPACE = "motor_state_serial"
CRANK_INDICES = (14, 15, 20, 21)

HEAD = JOINTS[0:2]
LEFT_ARM = JOINTS[2:6]
RIGHT_ARM = JOINTS[6:10]
LEFT_LEG = JOINTS[10:16]
RIGHT_LEG = JOINTS[16:22]

# --------------------------------------------------------------------------
# 2. Other orders
# --------------------------------------------------------------------------
URDF_ORDER = list(JOINTS)          # K1_22dof.urdf, file order of revolute joints
MJCF_JOINT_ORDER = list(JOINTS)    # K1_22dof.xml <joint> order (after world_joint)
MJCF_ACTUATOR_ORDER = list(JOINTS)  # K1_22dof.xml <actuator><motor> order (torque motors)

# Isaac Lab / PhysX articulation order for K1_22dof.urdf imported by
# BOOSTER_K1_CFG: BFS over the link tree, Trunk children in URDF order
# (Head, L arm, R arm, L leg, R leg).  Confirmed by HANDOFF.md:644, where the
# home-env play log printed leg ids [3,4,8,9,...].
BFS_ORDER = [
    "AAHead_yaw", "ALeft_Shoulder_Pitch", "ARight_Shoulder_Pitch",
    "Left_Hip_Pitch", "Right_Hip_Pitch",                                  # depth 1: 0-4
    "Head_pitch", "Left_Shoulder_Roll", "Right_Shoulder_Roll",
    "Left_Hip_Roll", "Right_Hip_Roll",                                     # depth 2: 5-9
    "Left_Elbow_Pitch", "Right_Elbow_Pitch", "Left_Hip_Yaw", "Right_Hip_Yaw",   # 10-13
    "Left_Elbow_Yaw", "Right_Elbow_Yaw", "Left_Knee_Pitch", "Right_Knee_Pitch",  # 14-17
    "Left_Ankle_Pitch", "Right_Ankle_Pitch",                               # 18-19
    "Left_Ankle_Roll", "Right_Ankle_Roll",                                 # 20-21
]

# 12-dim leg layouts used by policy code.
LEG_DEPLOY_ORDER = LEFT_LEG + RIGHT_LEG  # [L x6, R x6]
LEG_BFS_ORDER = [j for j in BFS_ORDER if j in set(LEG_DEPLOY_ORDER)]
# == [L_HipP, R_HipP, L_HipR, R_HipR, L_HipY, R_HipY, L_Knee, R_Knee,
#     L_AnkP, R_AnkP, L_AnkR, R_AnkR]


def index_map(src: list[str], dst: list[str]) -> list[int]:
    """perm with ``dst[i] == src[perm[i]]`` (so ``reorder(v, src, dst)`` works)."""
    pos = {n: i for i, n in enumerate(src)}
    return [pos[n] for n in dst]


def reorder(values, src: list[str], dst: list[str]) -> list:
    """Re-order ``values`` (laid out per ``src``) into ``dst`` layout."""
    return [values[i] for i in index_map(src, dst)]


# Ready-made maps.  "A_TO_B[i]" is the index in A-layout of B's i-th entry.
SDK_TO_BFS = index_map(JOINTS, BFS_ORDER)          # bfs[i]  = sdk[SDK_TO_BFS[i]]
BFS_TO_SDK = index_map(BFS_ORDER, JOINTS)          # sdk[i]  = bfs[BFS_TO_SDK[i]]
SDK_TO_URDF = list(range(N_JOINTS))                # identity
SDK_TO_MJCF = list(range(N_JOINTS))                # identity (joints and actuators)
# 12-dim legs: deploy [L*6,R*6] <-> PhysX-trained interleaved layout.
LEG_DEPLOY_TO_BFS = index_map(LEG_DEPLOY_ORDER, LEG_BFS_ORDER)  # bfs12[i] = dep12[map[i]]
LEG_BFS_TO_DEPLOY = index_map(LEG_BFS_ORDER, LEG_DEPLOY_ORDER)  # dep12[i] = bfs12[map[i]]
# 12 leg joints -> motor_cmd[] index.
LEG_SDK_INDEX = [SDK_INDEX[n] for n in LEG_DEPLOY_ORDER]        # [10..21]

# --------------------------------------------------------------------------
# 3. Joint limits / axes (URDF == MJCF; asserted).  (lower, upper) rad.
# --------------------------------------------------------------------------
LIMITS = {
    "AAHead_yaw": (-1.0, 1.0), "Head_pitch": (-0.349, 0.855),
    "ALeft_Shoulder_Pitch": (-3.316, 1.22), "Left_Shoulder_Roll": (-1.74, 1.57),
    "Left_Elbow_Pitch": (-2.27, 2.27), "Left_Elbow_Yaw": (-2.44, 0.0),
    "ARight_Shoulder_Pitch": (-3.316, 1.22), "Right_Shoulder_Roll": (-1.57, 1.74),
    "Right_Elbow_Pitch": (-2.27, 2.27), "Right_Elbow_Yaw": (0.0, 2.44),
    "Left_Hip_Pitch": (-3.0, 2.21), "Left_Hip_Roll": (-0.4, 1.57),
    "Left_Hip_Yaw": (-1.0, 1.0), "Left_Knee_Pitch": (0.0, 2.23),
    "Left_Ankle_Pitch": (-0.87, 0.345), "Left_Ankle_Roll": (-0.345, 0.345),
    "Right_Hip_Pitch": (-3.0, 2.21), "Right_Hip_Roll": (-1.57, 0.4),
    "Right_Hip_Yaw": (-1.0, 1.0), "Right_Knee_Pitch": (0.0, 2.23),
    "Right_Ankle_Pitch": (-0.87, 0.345), "Right_Ankle_Roll": (-0.345, 0.345),
}
# Joint axis in the child frame (URDF == MJCF).  Positive rotation is
# right-hand about the axis; NOTHING is sign-flipped between left and right
# (only the roll/elbow-yaw LIMITS are mirrored).
AXES = {}
for _n in JOINTS:
    if _n in ("AAHead_yaw",) or _n.endswith("Elbow_Yaw") or _n.endswith("Hip_Yaw"):
        AXES[_n] = (0, 0, 1)
    elif _n.endswith("Roll"):
        AXES[_n] = (1, 0, 0)
    else:
        AXES[_n] = (0, 1, 0)

# URDF/MJCF effort limits (Nm).  NOTE the Isaac training actuator model uses
# bigger limits, see SIM_EFFORT_LIMIT.
URDF_EFFORT = {n: (6.0 if n in HEAD else 14.0 if n in LEFT_ARM + RIGHT_ARM else
                   30.0 if n.endswith("Hip_Pitch") else 35.0 if n.endswith("Hip_Roll") else
                   20.0 if n.endswith("Hip_Yaw") else 40.0 if n.endswith("Knee_Pitch") else 20.0)
               for n in JOINTS}
SIM_EFFORT_LIMIT = {n: (6.0 if n in HEAD else 14.0 if n in LEFT_ARM + RIGHT_ARM else
                        68.0 if n.endswith("Hip_Pitch") else 76.0 if n.endswith("Hip_Roll") else
                        38.3 if n.endswith("Hip_Yaw") else 112.0 if n.endswith("Knee_Pitch") else 38.3)
                    for n in JOINTS}

# --------------------------------------------------------------------------
# 4. Default / stand poses (rad), canonical order
# --------------------------------------------------------------------------
# (a) What every Isaac policy was trained around: BOOSTER_K1_CFG.init_state,
#     legs straight, only the shoulder rolls non-zero.  Used as the action
#     offset (use_default_offset=True) and in joint_pos_rel observations.
SIM_DEFAULT_POS = {n: 0.0 for n in JOINTS}
SIM_DEFAULT_POS["Left_Shoulder_Roll"] = -1.3
SIM_DEFAULT_POS["Right_Shoulder_Roll"] = 1.3
SIM_INIT_BASE_Z = 0.57

# (b) What the real robot reports when standing in Booster's prepare/walk
#     posture (first /joint_states sample of the recorded bag
#     real_robot/bags/k1_walk_20260929_023045/seg001, 3 d.p.).
HW_STAND_SNAPSHOT = {
    "AAHead_yaw": -0.001, "Head_pitch": 0.0,
    "ALeft_Shoulder_Pitch": 0.187, "Left_Shoulder_Roll": -1.451,
    "Left_Elbow_Pitch": -0.011, "Left_Elbow_Yaw": -0.496,
    "ARight_Shoulder_Pitch": 0.194, "Right_Shoulder_Roll": 1.452,
    "Right_Elbow_Pitch": -0.011, "Right_Elbow_Yaw": 0.485,
    "Left_Hip_Pitch": 0.010, "Left_Hip_Roll": -0.002, "Left_Hip_Yaw": -0.008,
    "Left_Knee_Pitch": 0.105, "Left_Ankle_Pitch": -0.103, "Left_Ankle_Roll": 0.006,
    "Right_Hip_Pitch": -0.011, "Right_Hip_Roll": -0.002, "Right_Hip_Yaw": -0.009,
    "Right_Knee_Pitch": 0.105, "Right_Ankle_Pitch": -0.088, "Right_Ankle_Roll": 0.016,
}
# (c) Booster's official booster_deploy K1 "prepare" leg offsets (secondhand,
#     docs/k1_ros_deployment_plan.md:238): knee +0.105, ankle pitch -0.10.
#     Agrees with (b).
OFFICIAL_PREPARE_LEG_OFFSETS = {"Left_Knee_Pitch": 0.105, "Right_Knee_Pitch": 0.105,
                                "Left_Ankle_Pitch": -0.10, "Right_Ankle_Pitch": -0.10}

# --------------------------------------------------------------------------
# 5. PD gains
# --------------------------------------------------------------------------
# NOTE: this repo has NEVER commanded a joint on hardware (/joint_ctrl has zero
# publishers, docs/k1_walk_data_plan.md:74).  There are therefore no
# "gains used on hardware" by us; below are (a) the gains the policies were
# trained with in sim (derived by booster_train from armature, f_n, zeta),
# (b) vendor numbers, (c) what our never-run C++ bridge would send.


def derive_pd(armature: float, natural_freq: float, damping_ratio: float) -> tuple[float, float]:
    """booster_train BoosterJointCfg: kp = I w^2, kd = 2 zeta I w, w = 2 pi f."""
    w = 2.0 * math.pi * natural_freq
    return armature * w * w, 2.0 * damping_ratio * armature * w


# (armature kg m^2, natural_freq Hz, damping_ratio) as in booster.py:94-165
# BOOSTER_K1_CFG (+ actuator.py:210-236, 421-470).  Ankle armature = ratio x E4310.
SIM_ACTUATOR_PARAMS = {
    "Hip_Pitch": (0.0478125, 4.0, 1.5), "Hip_Roll": (0.0339552, 4.0, 1.5),
    "Hip_Yaw": (0.0282528, 4.0, 1.5), "Knee_Pitch": (0.095625, 4.0, 1.0),
    "Ankle_Pitch": (1.4 * 0.0282528, 4.0, 1.5), "Ankle_Roll": (0.4 * 0.0282528, 4.0, 1.5),
    "arm_head": (0.001, 10.0, 2.0),
}


def _sim_key(name: str) -> str:
    for k in ("Hip_Pitch", "Hip_Roll", "Hip_Yaw", "Knee_Pitch", "Ankle_Pitch", "Ankle_Roll"):
        if name.endswith(k):
            return k
    return "arm_head"


# Gains of the PUBLISHED booster_train K1 (what HEAD of this repo trains with
# since 8e019b2/2026-10-02; the gaitfix-era student used hand-tuned 100/5).
SIM_TRAIN_GAINS = {n: derive_pd(*SIM_ACTUATOR_PARAMS[_sim_key(n)]) for n in JOINTS}
# Hand-tuned override used by checkpoints trained before 8e019b2
# (docs/sim2sim_mujoco_findings.md:22-25), legs only (kp, kd).
LEGACY_OVERRIDE_GAINS = {
    "Hip_Pitch": (100.0, 5.0), "Hip_Roll": (100.0, 5.0), "Hip_Yaw": (60.0, 3.0),
    "Knee_Pitch": (100.0, 5.0), "Ankle_Pitch": (100.0, 5.0), "Ankle_Roll": (35.69, 4.26),
}

# Vendor SDK example b1_low_sdk_example.cpp:82-95 (23-DoF JointIndex layout incl.
# waist).  Leg part only, per leg [HipP, HipR, HipY, Knee, AnkP, AnkR].
SDK_EXAMPLE_LEG_KP = (350.0, 350.0, 180.0, 350.0, 550.0, 550.0)
SDK_EXAMPLE_LEG_KD = (7.5, 7.5, 3.0, 5.5, 1.5, 1.5)
# Booster official booster_deploy K1 prepare state (secondhand, UNVERIFIED:
# docs/k1_ros_deployment_plan.md:236-238).  Leg part, per leg.
OFFICIAL_PREPARE_LEG_KP = (350.0, 350.0, 180.0, 350.0, 250.0, 250.0)
OFFICIAL_PREPARE_LEG_KD = (7.5, 7.5, 3.0, 5.5, 5.0, 5.0)
# Our never-run sdk_bridge_node.cpp:121-148 (kp; kd shown as hip=kp*0.12).
BRIDGE_LEG_KP = (30.2, 21.4, 17.8, 60.4, 35.7, 35.7)

# --------------------------------------------------------------------------
# 6. Action scale / timing
# --------------------------------------------------------------------------
# q_target = default + scale * action (use_default_offset=True), clip applies
# to the RAW action before scaling.
ACTION_SCALE_VELOCITY_CFG = 1.0     # velocity_env_cfg.py:304-309, clip +-1.0, since 5d5af78
ACTION_SCALE_LEGACY = 0.25          # locomotion_node/run_sim/docs defaults (stale)
ACTION_SCALE_OTHER = {"kick_legs": 0.25, "carry_arm": 0.25, "head": 0.5, "push_arms": 1.0}
CONTROL_HZ = 50.0                   # sim.dt 0.005 * decimation 4
PHYSICS_DT = 0.005
OBS_HISTORY = 10
# DelayedPDActuator delay: integer PHYSICS steps in [2, 8] -> 10..40 ms.
ACTUATOR_DELAY_PHYSICS_STEPS = (2, 8)
ACTUATOR_DELAY_SECONDS = (2 * PHYSICS_DT, 8 * PHYSICS_DT)


def leg_target(action_deploy_order, default=None, scale=ACTION_SCALE_VELOCITY_CFG, clip=1.0):
    """12-dim action in LEG_DEPLOY order -> q target (rad), canonical helper.

    NOTE: a policy trained on PhysX BFS layout emits its action in LEG_BFS_ORDER;
    convert with ``reorder(a, LEG_BFS_ORDER, LEG_DEPLOY_ORDER)`` first.
    """
    default = default if default is not None else [SIM_DEFAULT_POS[n] for n in LEG_DEPLOY_ORDER]
    return [d + scale * max(-clip, min(clip, a)) for d, a in zip(default, action_deploy_order)]
