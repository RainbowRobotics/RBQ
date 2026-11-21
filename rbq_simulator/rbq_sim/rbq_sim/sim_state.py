"""Build SimInfo_ from the Isaac scene (robot + IMU + contact sensors).

Field/frame conventions, matching rbq_lab's ArticulationData usage:
  - imu.orientation        : IMU quat (world), Isaac order (w, x, y, z) == wire order
  - imu.angular_velocity   : body-frame gyro from the Imu sensor (imu.data.ang_vel_b)
  - imu.linear_acceleration: body-frame specific force from the Imu sensor (imu.data.lin_acc_b);
                             ImuCfg default gravity_bias makes this ~+9.81 z at rest, as the
                             Motion estimator expects (do NOT differentiate pose)
  - leg_joint[12]          : pos/vel/torque in model (wire) order
  - body_task              : ground-truth root pos/vel/quat/omega (world) — GUI telemetry
  - leg_contact[4]         : foot contact force (world) in order [RR, RL, FR, FL] = [HR, HL, FR, FL]
"""

import torch

from rbq_sim.idl._sim_idl import SimInfo_

CONTACT_FORCE_THRESHOLD = 1.0  # N, for the boolean `active` flag


def build_siminfo(info: SimInfo_, robot, imu, contact, sim2model, foot_ids, tau_model) -> SimInfo_:
    """Fill `info` in place for env 0 (reused every tick — building the nested dataclass is not free).

    sim2model maps wire(model) joint index -> articulation(sim) index;
    foot_ids are the contact-sensor body indices for [HR, HL, FR, FL] feet;
    tau_model is the bridge's last commanded PD torque (model order) — published as the joint
    torque to match MuJoCo/Gazebo (the clamped commanded torque, not post-actuator applied).
    """
    rd = robot.data
    # One device->host copy for everything (each .tolist() on its own is a separate sync).
    v = torch.cat((
        imu.data.quat_w[0],                         # 0:4   (w, x, y, z)
        imu.data.ang_vel_b[0],                      # 4:7   body-frame gyro
        imu.data.lin_acc_b[0],                      # 7:10  body-frame specific force (incl. gravity bias)
        rd.joint_pos[0][sim2model],                 # 10:22 model (wire) order
        rd.joint_vel[0][sim2model],                 # 22:34
        tau_model,                                  # 34:46 bridge's commanded clamped torque
        rd.root_link_pos_w[0],                      # 46:49 ground-truth body, world
        rd.root_link_lin_vel_w[0],                  # 49:52
        rd.root_link_ang_vel_w[0],                  # 52:55
        rd.root_link_quat_w[0],                     # 55:59
        contact.data.net_forces_w[0][foot_ids].reshape(-1),  # 59:71 feet [HR, HL, FR, FL], world
    )).tolist()

    # --- IMU (from the Imu sensor) ---
    o, g, a = info.imu.orientation, info.imu.angular_velocity, info.imu.linear_acceleration
    o.w, o.x, o.y, o.z = v[0:4]
    g.x, g.y, g.z = v[4:7]
    a.x, a.y, a.z = v[7:10]

    # --- Leg joints (model/wire order) ---
    for k, j in enumerate(info.leg_joint):
        j.pos, j.vel, j.torque = v[10 + k], v[22 + k], v[34 + k]

    # --- Body task (ground-truth, world) ---
    info.body_task.pos = v[46:49]
    info.body_task.vel = v[49:52]
    info.body_task.omega = v[52:55]
    info.body_task.quat = v[55:59]

    # --- Foot contacts (world), order [RR, RL, FR, FL] = [HR, HL, FR, FL] ---
    for k, c in enumerate(info.leg_contact):
        f = v[59 + 3 * k:62 + 3 * k]
        c.force = f
        c.active = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]) > (CONTACT_FORCE_THRESHOLD ** 2)

    return info
