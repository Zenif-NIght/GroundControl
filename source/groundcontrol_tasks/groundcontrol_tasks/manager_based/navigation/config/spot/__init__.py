# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents, navigation_env_cfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Navigation-Flat-Spot-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.NavigationEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:NavigationEnvPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Navigation-Flat-Spot-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.NavigationEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:NavigationEnvPPORunnerCfg",
    },
)
