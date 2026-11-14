import torch
import numpy as np

def torch_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(*shape, device=device) + lower

def euler_from_quat_zyx(quat: torch.Tensor, wrap_to_pi: bool = True) -> torch.Tensor:
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    yaw = torch.atan2(
        2 * (w * z + x * y),
        1 - 2 * (y * y + z * z),
    )
    pitch = torch.asin(torch.clamp(
        2 * (w * y - z * x),
        -1.0, 1.0,
    ))
    roll = torch.atan2(
        2 * (w * x + y * z),
        1 - 2 * (x * x + y * y),
    )

    if wrap_to_pi:
        yaw   = (yaw + torch.pi) % (2 * torch.pi) - torch.pi
        pitch = (pitch + torch.pi) % (2 * torch.pi) - torch.pi
        roll  = (roll + torch.pi) % (2 * torch.pi) - torch.pi

    return torch.stack([roll, pitch, yaw], dim=-1)

def quat_from_euler_xyz_tuple(roll: torch.Tensor, pitch: torch.Tensor, yaw: torch.Tensor) -> tuple:
    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    cr = torch.cos(roll * 0.5)
    sr = torch.sin(roll * 0.5)
    cp = torch.cos(pitch * 0.5)
    sp = torch.sin(pitch * 0.5)
    # compute quaternion
    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp
    convert = torch.stack([qw, qx, qy, qz], dim=-1) * torch.tensor([1.,1.,1.,-1])
    return tuple(convert.numpy().tolist())

def quat_apply(a, b):
    shape = b.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 3)
    xyz = a[:, 1:]
    t = xyz.cross(b, dim=-1) * 2
    return (b + a[:, 0:1] * t + xyz.cross(t, dim=-1)).view(shape)

def quat_to_pitch_rad(quat):
    """
    Extract pitch (in degrees) from quaternion [w, x, y, z] tensor (N, 4)
    """
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    sinp = 2 * (w * y - z * x)
    sinp = torch.clamp(sinp, -1.0, 1.0)
    pitch_rad = torch.asin(sinp)
    return pitch_rad