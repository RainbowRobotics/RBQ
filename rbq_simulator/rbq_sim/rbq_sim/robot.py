"""RBQ10 articulation config + joint-order mapping for the integration sim.

Single source of truth = rbq_lab's RBQ10_URDF_CFG (same URDF, actuators, init pose), so the
Isaac model stays consistent with the RL trainer.
"""

import copy

import torch

from rbq_lab.envs.rbq10.rbq10 import RBQ10_LEG_JOINT, RBQ10_URDF_CFG

# Sitting spawn/hold pose — the SAME pose every reference sim spawns and pre-Motion holds
# (rbq_mujoco, rbq_gazebo). Keeping this identical matters: Motion's
# go_to_motion_ready uses is_grounded = all(pitch > 60deg) to pick the sit->stand sequence, so
# the standing-ready RL init pose (46.5deg) would take a different handover branch.
SIT_SPAWN_Z = 0.30  # empty-world spawn height (same as rbq_gazebo)
SIT_JOINT_POS = {
    "joint0_HRR": -0.59341195, "joint1_HRP": 1.13446401, "joint2_HRK": -2.79252680,
    "joint3_HLR": 0.59341195, "joint4_HLP": 1.13446401, "joint5_HLK": -2.79252680,
    "joint6_FRR": -0.59341195, "joint7_FRP": 1.13446401, "joint8_FRK": -2.79252680,
    "joint9_FLR": 0.59341195, "joint10_FLP": 1.13446401, "joint11_FLK": -2.79252680,
}


def make_robot_cfg(prim_path: str = "/World/Robot", effort_mode: bool = True):
    """Articulation cfg for rbq_sim, derived from rbq_lab's single-source RBQ10_URDF_CFG.

    Two rbq_sim-specific overrides vs rbq_lab's training cfg:
      1. init_state -> the sitting spawn/hold pose (matches MuJoCo/Gazebo; see SIT_JOINT_POS).
      2. effort_mode (default): actuator stiffness/damping -> 0 so the only PD is the MotionRef_
         kp/kd applied per-step via set_joint_effort_target. rbq_lab's stiffness (123/127) is for
         RL position-target training and must NOT drive the integration sim (double PD).
    """
    cfg = copy.deepcopy(RBQ10_URDF_CFG)
    cfg.prim_path = prim_path
    cfg.init_state.pos = (0.0, 0.0, SIT_SPAWN_Z)
    cfg.init_state.joint_pos = dict(SIT_JOINT_POS)
    if effort_mode:
        for actuator in cfg.actuators.values():
            actuator.stiffness = 0.0
            actuator.damping = 0.0
    return cfg


def compute_joint_order(robot):
    """Map Isaac articulation order (sim) <-> canonical wire order (model = RBQ10_LEG_JOINT).

    Isaac does not guarantee URDF declaration order, so never assume identity. Mirrors
    rbq_lab env.set_joint_order(): index sim tensors with sim2model to get model/wire order,
    and target tensors with model2sim to apply back to the articulation.
    """
    sim2model = torch.as_tensor(
        robot.find_joints(RBQ10_LEG_JOINT, preserve_order=True)[0],
        device=robot.device,
        dtype=torch.long,
    )
    model2sim = torch.empty_like(sim2model)
    model2sim[sim2model] = torch.arange(len(sim2model), device=robot.device, dtype=torch.long)
    return sim2model, model2sim
