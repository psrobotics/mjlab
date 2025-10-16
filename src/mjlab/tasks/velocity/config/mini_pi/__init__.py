import gymnasium as gym

gym.register(
  id="Mjlab-Velocity-Rough-Minipi",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:MinipiRoughEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:MinipiPPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Rough-Minipi-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.rough_env_cfg:MinipiRoughEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:MinipiPPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Minipi",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:MinipiFlatEnvCfg",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:MinipiPPORunnerCfg",
  },
)

gym.register(
  id="Mjlab-Velocity-Flat-Minipi-Play",
  entry_point="mjlab.envs:ManagerBasedRlEnv",
  disable_env_checker=True,
  kwargs={
    "env_cfg_entry_point": f"{__name__}.flat_env_cfg:MinipiFlatEnvCfg_PLAY",
    "rl_cfg_entry_point": f"{__name__}.rl_cfg:MinipiPPORunnerCfg",
  },
)
