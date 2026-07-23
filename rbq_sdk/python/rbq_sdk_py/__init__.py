from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("rbq_sdk")
except PackageNotFoundError:  # running from a source checkout without an install
    __version__ = "0.0.0"

from . import idl, utils, core, rpc, sport, video, robot_state

__all__ = [
    "idl",
    "utils",
    "core",
    "rpc",
    "sport",
    "video",
    "robot_state",
]
