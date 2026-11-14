from setuptools import setup, find_packages

setup(
    name="RainbowLab",
    version="0.1",    
    packages = find_packages(),
    install_requires=[
    "psutil",
    "pynput",
    ],
)
