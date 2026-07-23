import os

GYM_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
GYM_ENVS_DIR = os.path.join(GYM_ROOT_DIR, 'envs')
GYM_ASSET_DIR = os.path.join(GYM_ROOT_DIR, '..', 'resources')
