# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
import gymnasium as gym

##
# Register Gym environments.
##

gym.register(
    id=os.path.basename(os.path.dirname(__file__)),
    entry_point=f"{__name__}.env:Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:EnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
    },
)

gym.register(
    id=os.path.basename(os.path.dirname(__file__)) + "_play",
    entry_point=f"{__name__}.env:Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:EnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
    },
)
