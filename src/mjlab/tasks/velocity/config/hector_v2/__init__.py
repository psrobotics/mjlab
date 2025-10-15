import gymnasium as gym

gym.register(
  id="Mjlab-Velocity-Rough-Hectorv2",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:Hectorv2RoughEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:Hectorv2PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Rough-Hectorv2-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:Hectorv2RoughEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:Hectorv2PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Hectorv2",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:Hectorv2FlatEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:Hectorv2PPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Hectorv2-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:Hectorv2FlatEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:Hectorv2PPORunnerCfg",
  },
)
