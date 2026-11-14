from setuptools import find_packages
from distutils.core import setup

setup(
    name='rbq_gym',
    version='1.0.0',
    packages=find_packages(),
    install_requires=['isaacgym',
                      'matplotlib',
                      'torch>=1.4.0',
                      'torchvision>=0.5.0',
                      'numpy<1.24',
                      'setuptools==59.5.0',
                      'tensorboard',
                      'pynput',
                      'protobuf==4.25',
                      'onnx',
                      'rsl-rl==1.0.2']
)
