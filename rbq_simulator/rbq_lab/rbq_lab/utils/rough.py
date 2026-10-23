# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for custom terrains."""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg


ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=25.0,
    num_rows=10,
    num_cols=30,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "hf_pyramid_slope_inv": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, 
            slope_range=(0.1, 0.6), 
            platform_width=1.7, 
            border_width=0.0, 
            inverted=True,
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, 
            slope_range=(0.1, 0.6),
            platform_width=1.7, 
            border_width=0.0
        ),
        "hf_pyramid_stairs_inv": terrain_gen.HfPyramidStairsTerrainCfg(
            proportion=0.2, 
            step_height_range=(0.05, 0.25), 
            step_width=0.31, 
            platform_width=3.0, 
            inverted=True
        ),
        "hf_pyramid_stairs": terrain_gen.HfPyramidStairsTerrainCfg(
            proportion=0.2, 
            step_height_range=(0.05, 0.25), 
            step_width=0.31, 
            platform_width=3.0,
        ),
        "hf_discrete_obstacles": terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.2, 
            obstacle_height_mode="choice", 
            obstacle_width_range=(1.0, 2.0), 
            obstacle_height_range=(0.05, 0.2),
            num_obstacles=20,
            border_width=0.1,
            platform_width=3.0
        ),
        "hf_random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2,
            noise_range=(-0.05, 0.05), 
            noise_step=0.005, 
            downsampled_scale=0.2,
            border_width=0.1,
        ),
    },
)