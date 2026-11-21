import os

from rbq_gym import GYM_ROOT_DIR, GYM_ENVS_DIR, GYM_ASSET_DIR
from rbq_gym.utils.task_registry import task_registry
from .base.rbquad_env import RBQuadEnv

# - RBQ10
from .rbq10.rbq10_env import RBQ10Env
from .rbq10.rbq10_config import RBQ10Cfg, RBQ10CfgPPO
task_registry.register( "rbq10", RBQ10Env, RBQ10Cfg(), RBQ10CfgPPO() )
