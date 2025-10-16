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

PI_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "mini_pi" / "xmls" / "mini_pi.xml"
)
assert PI_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, PI_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(PI_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Motor specs (Estimation).
ROTOR_INERTIAS_504736NE = 0.3e-5
GEARS_504736NE = 36
ARMATURE_504736NE = reflected_inertia(ROTOR_INERTIAS_504736NE, GEARS_504736NE)

ACTUATOR_504736NE = ElectricActuator(
  reflected_inertia=ARMATURE_504736NE,
  velocity_limit=4.188,
  effort_limit=16.0,
)

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_504736NE = ARMATURE_504736NE * NATURAL_FREQ**2

DAMPING_504736NE = 2.0 * DAMPING_RATIO * ARMATURE_504736NE * NATURAL_FREQ

PI_ACTUATOR = ActuatorCfg(
  joint_names_expr=[
    ".*",
  ],
  effort_limit=ACTUATOR_504736NE.effort_limit,
  armature=ACTUATOR_504736NE.reflected_inertia,
  stiffness=STIFFNESS_504736NE,
  damping=DAMPING_504736NE,
)


##
# Keyframe config.
##

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.347),
  joint_pos={".*": 0.0},
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

PI_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    PI_ACTUATOR,
  ),
  soft_joint_pos_limit_factor=0.9,
)

PI_ROBOT_CFG = EntityCfg(
  init_state=KNEES_BENT_KEYFRAME,
  collisions=(FULL_COLLISION,),
  spec_fn=get_spec,
  articulation=PI_ARTICULATION,
)

PI_ACTION_SCALE: dict[str, float] = {}
for a in PI_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.joint_names_expr
  if not isinstance(e, dict):
    e = {n: e for n in names}
  if not isinstance(s, dict):
    s = {n: s for n in names}
  for n in names:
    if n in e and n in s and s[n]:
      PI_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(PI_ROBOT_CFG)

  print(robot.actuator_names)
  print(robot.body_names)
  print(ARMATURE_504736NE)
  print(STIFFNESS_504736NE)
  print(DAMPING_504736NE)

  print("Joint level constants\n")
  for a in PI_ARTICULATION.actuators:
    e = a.effort_limit
    s = a.stiffness
    d = a.damping
    names = a.joint_names_expr
    print("J_name - ", names, " kp - ", s, " kd - ", d, "\n")
  print(PI_ACTION_SCALE)

  viewer.launch(robot.spec.compile())
