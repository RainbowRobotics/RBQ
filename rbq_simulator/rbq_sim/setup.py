from setuptools import find_packages, setup

setup(
    name="RainbowSim",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "psutil",
        "pynput",
    ],
)
