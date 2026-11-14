from isaaclab.sensors import RayCasterCameraCfg
from isaaclab.sensors.ray_caster.patterns import PinholeCameraPatternCfg

from .math import *

def OAK_D_pattern_cfg(width: int=640, height: int | None=None, enforce_aspect_ratio: bool = True):
    resolution = 360/640
    if height is None:
        height = int(width*resolution)
    else:
        ratio = height / width
        if enforce_aspect_ratio and not abs(ratio - resolution) < 1e-6:
            raise ValueError(
                f"OAK-D aspect ratio mismatch: "
                f"expected {resolution:.6f} (≈360:640=9:16), "
                f"got {ratio:.6f} (height/width). "
                f"width={width}, height={height}"
            )
        
    return PinholeCameraPatternCfg(
        focal_length=0.169,                      # [cm]
        width=width,                             # [pixel]
        height=height,                           # [pixel]
        horizontal_aperture=0.37360967184801386, # [cm]
        vertical_aperture=0.2101554404145078,    # [cm]
    )

def OAK_D_cfg_from_intrinsic_matrix_cfg(width: int=640, height: int | None=None, enforce_aspect_ratio: bool = True):
    resolution = 360/640
    if height is None:
        height = int(width*resolution)
    else:
        ratio = height / width
        if enforce_aspect_ratio and not abs(ratio - resolution) < 1e-6:
            raise ValueError(
                f"OAK-D aspect ratio mismatch: "
                f"expected {resolution:.6f} (≈360:640=9:16), "
                f"got {ratio:.6f} (height/width). "
                f"width={width}, height={height}"
            )

    default_width = 640
    scale = width / default_width

    f_x = 289.5 * scale
    f_y = 289.5 * scale
    c_x = 320 * scale
    c_y = 180 * scale

    intrinsic_matrix = [f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1]

    return PinholeCameraPatternCfg.from_intrinsic_matrix(
            intrinsic_matrix=intrinsic_matrix,
            width=width,
            height=height,
            focal_length=0.169,
        )

def D430_pattern_cfg(width: int=640, height: int | None=None, enforce_aspect_ratio: bool = True):
    resolution = 360/640
    if height is None:
        height = int(width*resolution)
    else:
        ratio = height / width
        if enforce_aspect_ratio and not abs(ratio - resolution) < 1e-6:
            raise ValueError(
                f"OAK-D aspect ratio mismatch: "
                f"expected {resolution:.6f} (≈360:640=9:16), "
                f"got {ratio:.6f} (height/width). "
                f"width={width}, height={height}"
            )
        
    return PinholeCameraPatternCfg(
        focal_length=0.193,                      # [cm]
        width=width,                             # [pixel]
        height=height,                           # [pixel]
        horizontal_aperture=0.3804443239813597, # [cm]
        vertical_aperture=0.21399993223951483,    # [cm]
    )

def D430_cfg_from_intrinsic_matrix_cfg(width: int=640, height: int | None=None, enforce_aspect_ratio: bool = True):
    resolution = 360/640
    if height is None:
        height = int(width*resolution)
    else:
        ratio = height / width
        if enforce_aspect_ratio and not abs(ratio - resolution) < 1e-6:
            raise ValueError(
                f"OAK-D aspect ratio mismatch: "
                f"expected {resolution:.6f} (≈360:640=9:16), "
                f"got {ratio:.6f} (height/width). "
                f"width={width}, height={height}"
            )

    default_width = 640
    scale = width / default_width

    f_x = 324.673 * scale
    f_y = 324.673 * scale
    c_x = 320 * scale
    c_y = 180 * scale

    intrinsic_matrix = [f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1]

    return PinholeCameraPatternCfg.from_intrinsic_matrix(
            intrinsic_matrix=intrinsic_matrix,
            width=width,
            height=height,
            focal_length=0.193,
        )