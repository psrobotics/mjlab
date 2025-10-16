from dataclasses import dataclass, replace

from mjlab.asset_zoo.robots.mini_pi.mini_pi_constants import (
  PI_ACTION_SCALE,
  PI_ROBOT_CFG,
)
from mjlab.tasks.velocity.velocity_env_cfg import (
  LocomotionVelocityEnvCfg,
)
from mjlab.utils.spec_config import ContactSensorCfg


@dataclass
class MinipiRoughEnvCfg(LocomotionVelocityEnvCfg):
  def __post_init__(self):
    super().__post_init__()

    foot_contact_sensors = [
      ContactSensorCfg(
        name=f"{side}_foot_ground_contact",
        body1=f"{side}_ankle_roll_link",
        body2="terrain",
        num=1,
        data=("found",),
        reduce="netforce",
      )
      for side in ["left", "right"]
    ]
    pi_cfg = replace(PI_ROBOT_CFG, sensors=tuple(foot_contact_sensors))
    self.scene.entities = {"robot": pi_cfg}

    sensor_names = ["left_foot_ground_contact", "right_foot_ground_contact"]
    geom_names = []
    geom_names.append(f"left_foot{1}_collision")
    geom_names.append(f"right_foot{1}_collision")

    self.events.foot_friction.params["asset_cfg"].geom_names = geom_names

    self.actions.joint_pos.scale = PI_ACTION_SCALE

    self.rewards.air_time.params["sensor_names"] = sensor_names
    # self.rewards.pose.params["std"] = {
    #   r"^(left|right)_knee_joint$": 0.6,
    #   r"^(left|right)_hip_pitch_joint$": 0.6,
    #   r"^(left|right)_elbow_joint$": 0.6,
    #   r"^(left|right)_shoulder_pitch_joint$": 0.6,
    #   r"^(?!.*(knee_joint|hip_pitch|elbow_joint|shoulder_pitch)).*$": 0.3,
    # }
    self.rewards.pose.params["std"] = {
      # Lower body.
      r".*hip_pitch.*": 0.3,
      r".*hip_roll.*": 0.15,
      r".*hip_yaw.*": 0.15,
      r".*knee.*": 0.35,
      r".*ankle_pitch.*": 0.01,
      r".*ankle_roll.*": 0.01,
    }

    self.viewer.body_name = "torso_link"
    self.commands.twist.viz.z_offset = 0.45

    self.curriculum.command_vel = None


@dataclass
class MinipiRoughEnvCfg_PLAY(MinipiRoughEnvCfg):
  def __post_init__(self):
    super().__post_init__()

    # Effectively infinite episode length.
    self.episode_length_s = int(1e9)

    if self.scene.terrain is not None:
      if self.scene.terrain.terrain_generator is not None:
        self.scene.terrain.terrain_generator.curriculum = False
        self.scene.terrain.terrain_generator.num_cols = 5
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.border_width = 10.0
