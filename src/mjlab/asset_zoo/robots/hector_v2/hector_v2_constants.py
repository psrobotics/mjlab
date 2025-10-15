""" Hector_v2 constants."""

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

HECTOR_V2_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "hector_v2" / "xmls" / "hector_v2.xml"
)
assert HECTOR_V2_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, HECTOR_V2_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(HECTOR_V2_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Motor specs (Estimation).
ROTOR_INERTIAS_ROBOSTRIDE01 = 0.5e-4
GEARS_ROBOSTRIDE01 = 7.75
ARMATURE_ROBOSTRIDE01 = reflected_inertia(ROTOR_INERTIAS_ROBOSTRIDE01, GEARS_ROBOSTRIDE01)

ROTOR_INERTIAS_ROBOSTRIDE01_ELBOW = 0.5e-4
GEARS_ROBOSTRIDE01_ELBOW = 7.75#7.75*1.5
ARMATURE_ROBOSTRIDE01_ELBOW = reflected_inertia(ROTOR_INERTIAS_ROBOSTRIDE01_ELBOW, GEARS_ROBOSTRIDE01_ELBOW)

ROTOR_INERTIAS_UNITREE_A1 = 1.0e-4
GEARS_UNITREE_A1 = 9.0
ARMATURE_UNITREE_A1= reflected_inertia(ROTOR_INERTIAS_UNITREE_A1, GEARS_UNITREE_A1)

ROTOR_INERTIAS_UNITREE_A1_KNEE = 1.0e-4
GEARS_UNITREE_A1_KNEE = 9.0#9.0*2.0
ARMATURE_UNITREE_A1_KNEE= reflected_inertia(ROTOR_INERTIAS_UNITREE_A1_KNEE, GEARS_UNITREE_A1_KNEE)

ACTUATOR_ROBOSTRIDE01 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBOSTRIDE01,
  velocity_limit=28.79,
  effort_limit=17.0,
)

ACTUATOR_ROBOSTRIDE01_ELBOW = ElectricActuator(
  reflected_inertia=ARMATURE_ROBOSTRIDE01_ELBOW,
  velocity_limit=28.79/1.5,
  effort_limit=17.0*1.5,
)

ACTUATOR_UNITREE_A1 = ElectricActuator(
  reflected_inertia=ARMATURE_UNITREE_A1,
  velocity_limit=21.0,
  effort_limit=33.5,
)

ACTUATOR_UNITREE_A1_KNEE = ElectricActuator(
  reflected_inertia=ARMATURE_UNITREE_A1_KNEE,
  velocity_limit=21.0/2.0,
  effort_limit=33.5*2.0,
)

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_01 = ARMATURE_ROBOSTRIDE01 * NATURAL_FREQ**2
STIFFNESS_A1 = ARMATURE_UNITREE_A1 * NATURAL_FREQ**2
STIFFNESS_01_ELBOW = ARMATURE_ROBOSTRIDE01_ELBOW * NATURAL_FREQ**2
STIFFNESS_A1_KNEE = ARMATURE_UNITREE_A1_KNEE * NATURAL_FREQ**2

DAMPING_01 = 2.0 * DAMPING_RATIO * ARMATURE_ROBOSTRIDE01 * NATURAL_FREQ
DAMPING_A1 = 2.0 * DAMPING_RATIO * ARMATURE_UNITREE_A1 * NATURAL_FREQ
DAMPING_01_ELBOW = 2.0 * DAMPING_RATIO * ARMATURE_ROBOSTRIDE01_ELBOW * NATURAL_FREQ
DAMPING_A1_KNEE = 2.0 * DAMPING_RATIO * ARMATURE_UNITREE_A1_KNEE * NATURAL_FREQ

H2_ACTUATOR_01 = ActuatorCfg(
  joint_names_expr=[
    ".*_shoulder_yaw_joint",
    ".*_shoulder_pitch_joint",
    ".*_shoulder_roll_joint",
  ],
  effort_limit=ACTUATOR_ROBOSTRIDE01.effort_limit,
  armature=ACTUATOR_ROBOSTRIDE01.reflected_inertia,
  stiffness=STIFFNESS_01,
  damping=DAMPING_01,
)

H2_ACTUATOR_01_ELBOW = ActuatorCfg(
  joint_names_expr=[
    ".*_elbow_joint",
  ],
  effort_limit=ACTUATOR_ROBOSTRIDE01_ELBOW.effort_limit,
  armature=ACTUATOR_ROBOSTRIDE01_ELBOW.reflected_inertia,
  stiffness=STIFFNESS_01_ELBOW,
  damping=DAMPING_01_ELBOW,
)

H2_ACTUATOR_A1 = ActuatorCfg(
  joint_names_expr=[
    ".*_hip_yaw_joint",
    ".*_hip_roll_joint",
    ".*_hip_pitch_joint",
    ".*_ankle_pitch_joint",
  ],
  effort_limit=ACTUATOR_UNITREE_A1.effort_limit,
  armature=ACTUATOR_UNITREE_A1.reflected_inertia,
  stiffness=STIFFNESS_A1,
  damping=DAMPING_A1,
)

H2_ACTUATOR_A1_KNEE = ActuatorCfg(
  joint_names_expr=[
    ".*_knee_joint",
  ],
  effort_limit=ACTUATOR_UNITREE_A1_KNEE.effort_limit,
  armature=ACTUATOR_UNITREE_A1_KNEE.reflected_inertia,
  stiffness=STIFFNESS_A1_KNEE,
  damping=DAMPING_A1_KNEE,
)

##
# Keyframe config.
##

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.55),
  joint_pos={
    "left_hip_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_pitch_joint": 0.785,
    "left_knee_joint": -1.57,
    "left_ankle_joint": 0.785,
    "right_hip_yaw_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_pitch_joint": 0.785,
    "right_knee_joint": -1.57,
    "right_ankle_joint": 0.785,
    ".*_shoulder_yaw_joint": 0.00,
    ".*_shoulder_pitch_joint": 0.785,
    ".*_shoulder_roll_joint": 0.00,
     ".*_elbow_joint": -1.57,
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
  condim={r"^(left|right)_foot[1]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1]_collision$": 1},
  friction={r"^(left|right)_foot[1]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=[".*_collision"],
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot[1]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1]_collision$": 1},
  friction={r"^(left|right)_foot[1]_collision$": (0.6,)},
)

# This disables all collisions except the feet.
# Feet get condim=3, all other geoms are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=[r"^(left|right)_foot[1]_collision$"],
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

H2_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    H2_ACTUATOR_01,
    H2_ACTUATOR_01_ELBOW,
    H2_ACTUATOR_A1,
    H2_ACTUATOR_A1_KNEE,
  ),
  soft_joint_pos_limit_factor=0.9,
)

H2_ROBOT_CFG = EntityCfg(
  init_state=KNEES_BENT_KEYFRAME,
  collisions=(FULL_COLLISION,),
  spec_fn=get_spec,
  articulation=H2_ARTICULATION,
)

H2_ACTION_SCALE: dict[str, float] = {}
for a in H2_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.joint_names_expr
  if not isinstance(e, dict):
    e = {n: e for n in names}
  if not isinstance(s, dict):
    s = {n: s for n in names}
  for n in names:
    if n in e and n in s and s[n]:
      H2_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(H2_ROBOT_CFG)

  print(robot.actuator_names)
  print(robot.body_names)
  print(STIFFNESS_01)
  print(STIFFNESS_01_ELBOW)
  print(STIFFNESS_A1)
  print(STIFFNESS_A1_KNEE)
  print(DAMPING_01)
  print(DAMPING_01_ELBOW)
  print(DAMPING_A1)
  print(DAMPING_A1_KNEE)

  print("Joint level constants\n")
  for a in H2_ARTICULATION.actuators:
    e = a.effort_limit
    s = a.stiffness
    d = a.damping
    names = a.joint_names_expr
    print("J_name - ", names, " kp - ", s, " kd - ", d, "\n")
  print(H2_ACTION_SCALE)

  viewer.launch(robot.spec.compile())
