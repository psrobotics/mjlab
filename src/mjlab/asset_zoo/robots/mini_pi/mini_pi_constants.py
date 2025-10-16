""" Motion 1 constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import (
  ElectricActuator,
  reflected_inertia,
)
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import ActuatorCfg, CollisionCfg

##
# MJCF and assets.
##

G1_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "motion_1" / "xmls" / "motion1.xml"
)
assert G1_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, G1_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(G1_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Motor specs (Estimation).
ROTOR_INERTIAS_ROBOSTRIDE02 = 0.5e-4
GEARS_ROBOSTRIDE02 = 7.75
ARMATURE_ROBOSTRIDE02 = reflected_inertia(ROTOR_INERTIAS_ROBOSTRIDE02, GEARS_ROBOSTRIDE02)

ROTOR_INERTIAS_ROBOSTRIDE03 = 1.0e-4
GEARS_ROBOSTRIDE03 = 9.0
ARMATURE_ROBOSTRIDE03 = reflected_inertia(ROTOR_INERTIAS_ROBOSTRIDE03, GEARS_ROBOSTRIDE03)

ROTOR_INERTIAS_ROBOSTRIDE04 = 1.0e-4
GEARS_ROBOSTRIDE04 = 9.0
ARMATURE_ROBOSTRIDE04 = reflected_inertia(ROTOR_INERTIAS_ROBOSTRIDE04, GEARS_ROBOSTRIDE04)


ACTUATOR_ROBOSTRIDE02 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBOSTRIDE02,
  velocity_limit=42.93,
  effort_limit=17.0,
)

ACTUATOR_ROBOSTRIDE03 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBOSTRIDE03,
  velocity_limit=18.84,
  effort_limit=60.0,
)

ACTUATOR_ROBOSTRIDE04 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBOSTRIDE04,
  velocity_limit=17.48,
  effort_limit=120.0,
)

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_02 = ARMATURE_ROBOSTRIDE02 * NATURAL_FREQ**2
STIFFNESS_03 = ARMATURE_ROBOSTRIDE03 * NATURAL_FREQ**2
STIFFNESS_04 = ARMATURE_ROBOSTRIDE04 * NATURAL_FREQ**2

DAMPING_02 = 2.0 * DAMPING_RATIO * ARMATURE_ROBOSTRIDE02 * NATURAL_FREQ
DAMPING_03 = 2.0 * DAMPING_RATIO * ARMATURE_ROBOSTRIDE03 * NATURAL_FREQ
DAMPING_04 = 2.0 * DAMPING_RATIO * ARMATURE_ROBOSTRIDE04 * NATURAL_FREQ

M1_ACTUATOR_02 = ActuatorCfg(
  joint_names_expr=[
    ".*_shoulder_pitch_joint",
    ".*_shoulder_roll_joint",
    ".*_shoulder_yaw_joint",
    ".*_elbow_joint",
    ".*_wrist_joint",
  ],
  effort_limit=ACTUATOR_ROBOSTRIDE02.effort_limit,
  armature=ACTUATOR_ROBOSTRIDE02.reflected_inertia,
  stiffness=STIFFNESS_02,
  damping=DAMPING_02,
)

M1_ACTUATOR_03 = ActuatorCfg(
  joint_names_expr=[
    "waist_joint",
    ".*_hip_pitch_joint",
    ".*_hip_roll_joint",
    ".*_hip_yaw_joint",
  ],
  effort_limit=ACTUATOR_ROBOSTRIDE03.effort_limit,
  armature=ACTUATOR_ROBOSTRIDE03.reflected_inertia,
  stiffness=STIFFNESS_03,
  damping=DAMPING_03,
)

M1_ACTUATOR_04 = ActuatorCfg(
  joint_names_expr=[
    ".*_knee_joint",
  ],
  effort_limit=ACTUATOR_ROBOSTRIDE04.effort_limit,
  armature=ACTUATOR_ROBOSTRIDE04.reflected_inertia,
  stiffness=STIFFNESS_04,
  damping=DAMPING_04,
)

# Waist pitch/roll and ankles are 4-bar linkages with 2 5020 actuators.
# Due to the parallel linkage, the effective armature at the ankle and waist joints
# is configuration dependent. Since the exact geometry of the linkage is unknown, we
# assume a nominal 1:1 gear ratio. Under this assumption, the joint armature in the
# nominal configuration is approximated as the sum of the 2 actuators' armatures.

M1_ACTUATOR_ANKLE_02 = ActuatorCfg(
  joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
  effort_limit=ACTUATOR_ROBOSTRIDE02.effort_limit * 2,
  armature=ACTUATOR_ROBOSTRIDE02.reflected_inertia * 2,
  stiffness=STIFFNESS_02 * 2,
  damping=DAMPING_02 * 2,
)

##
# Keyframe config.
##

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.805),
  joint_pos={
    "left_shoulder_pitch_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0,
    "waist_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "left_hip_pitch_joint": -0.5,
    "right_hip_pitch_joint": -0.5,
    "left_shoulder_yaw_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "left_elbow_joint": 0.0,
    "right_elbow_joint": 0.0,
    "left_hip_yaw_joint": -0.3,
    "right_hip_yaw_joint": 0.3,
    "left_wrist_joint": 0.0,
    "right_wrist_joint": 0.0,
    "left_knee_joint": 1.0,
    "right_knee_joint": 1.0,
    "left_ankle_pitch_joint": -0.49,
    "right_ankle_pitch_joint": -0.49,
    "left_ankle_roll_joint": 0.0,
    "right_ankle_roll_joint": 0.0,
  },
  joint_vel={".*": 0.0},
)


##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3 and custom friction and solimp.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=[".*_collision"],
  condim={r"^(left|right)_foot[1-4]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-4]_collision$": 1},
  friction={r"^(left|right)_foot[1-4]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=[".*_collision"],
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot[1-4]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-4]_collision$": 1},
  friction={r"^(left|right)_foot[1-4]_collision$": (0.6,)},
)

# This disables all collisions except the feet.
# Feet get condim=3, all other geoms are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=[r"^(left|right)_foot[1-4]_collision$"],
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

M1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    M1_ACTUATOR_02,
    M1_ACTUATOR_03,
    M1_ACTUATOR_04,
    M1_ACTUATOR_ANKLE_02,
  ),
  soft_joint_pos_limit_factor=0.9,
)

M1_ROBOT_CFG = EntityCfg(
  init_state=KNEES_BENT_KEYFRAME,
  collisions=(FULL_COLLISION,),
  spec_fn=get_spec,
  articulation=M1_ARTICULATION,
)

M1_ACTION_SCALE: dict[str, float] = {}
for a in M1_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.joint_names_expr
  if not isinstance(e, dict):
    e = {n: e for n in names}
  if not isinstance(s, dict):
    s = {n: s for n in names}
  for n in names:
    if n in e and n in s and s[n]:
      M1_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(M1_ROBOT_CFG)

  print(robot.actuator_names)
  print(robot.body_names)
  print(ARMATURE_ROBOSTRIDE02)
  print(ARMATURE_ROBOSTRIDE03)
  print(ARMATURE_ROBOSTRIDE04)
  print(STIFFNESS_02)
  print(STIFFNESS_03)
  print(STIFFNESS_04)
  print(DAMPING_02)
  print(DAMPING_03)
  print(DAMPING_04)

  print("Joint level constants\n")
  for a in M1_ARTICULATION.actuators:
    e = a.effort_limit
    s = a.stiffness
    d = a.damping
    names = a.joint_names_expr
    print("J_name - ", names, " kp - ", s, " kd - ", d, "\n")
  print(M1_ACTION_SCALE)

  viewer.launch(robot.spec.compile())
