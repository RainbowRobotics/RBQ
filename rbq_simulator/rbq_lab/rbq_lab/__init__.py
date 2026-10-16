import os

LAB_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
print("LAB_ROOT_DIR : ", LAB_ROOT_DIR)
LAB_ENVS_DIR = os.path.join(LAB_ROOT_DIR, 'envs')
LAB_ASSET_DIR = os.path.join(LAB_ROOT_DIR, '..', 'resources')