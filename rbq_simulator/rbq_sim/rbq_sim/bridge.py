"""DDS bridge, the Python counterpart of rbq_gazebo/src/Bridge.cpp, lock-stepped to Motion's RT tick.

Per control tick (CONTROL_DT = Motion RT_MS = 2 ms, PHYSICS_SUBSTEPS physics steps):
  apply_command()  - every physics step: PD-in-loop torque from the latest MotionRef_
                     (rt/rbq/ref/motion/_0), set as the articulation effort target.
  publish_state()  - SimInfo_ (rt/rbq/_sim) stamped with the SIM time. Motion --sim runs one RT tick
                     per new stamp, so the stamp is the tick trigger.
  sync()           - lockstep: block until the MotionRef_ whose `time` echoes that stamp arrives.
                     Network --sim publishes exactly one MotionRef_ per Motion RT tick (after the tick
                     distributed its references), so every 2 ms of sim time gets exactly one
                     Motion/QuadWalk tick regardless of how slowly physics/rendering/vision run.
                     Without Motion (not started / killed) it falls back to real-time pacing.

Until the first MotionRef_ with a meaningful kp or feed-forward torque arrives, the bridge holds the
spawn pose itself (kp=200, kd=5), then hands control to Motion for good. The articulation actuators run
with stiffness=0 / damping=0 (set in robot.make_robot_cfg) so MotionRef_ kp/kd are the only PD (matches
MuJoCo/Gazebo).

Torque clamp is the motor T-N envelope (same as rbq_mujoco, rbq_gazebo, rbq_gym): while motoring
(tau*dq >= 0) the limit is Tmax*GR up to wbase, then linear to 0 at wmax (hip <=83.2 / knee <=112 Nm).
Regen is not clipped.
"""

import threading
import time

import torch

from rbq_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from rbq_sim.idl._sim_idl import MotionRef_, SimInfo_
from rbq_sim.scene import FOOT_LINKS
from rbq_sim.sim_state import build_siminfo

REF_TOPIC = "rt/rbq/ref/motion/_0"  # MotionRef_  — sim subscribes (final joint commands from Motion)
SIM_TOPIC = "rt/rbq/_sim"           # SimInfo_    — sim publishes (state feedback to Motion)

HOLD_KP = 200.0
HOLD_KD = 5.0
# Motor T-N envelope per joint type: (GR, Tmax [Nm], wbase, wmax [rad/s at motor])
TN_RP = (13.0, 6.4, 120.0, 200.0)   # roll/pitch
TN_K = (14.0, 8.0, 100.0, 160.0)    # knee
FF_TORQUE_EPS = 0.5     # Nm, Motion handover detection (rbq_gazebo kFFtEps)

ACK_TIMEOUT_S = 0.25    # one Motion tick may stall this long (process spawn, log flush) before we move on
ACK_MISS_LIMIT = 8      # consecutive misses => Motion is gone, fall back to real-time pacing
FREE_RUN_RESYNC_S = 0.05  # real-time pacing: if this far behind, drop the backlog instead of catching up


class MotionBridge:
    def __init__(self, robot, imu, contact, sim2model, control_dt: float, interface: str = "lo"):
        self.robot = robot
        self.imu = imu
        self.contact = contact
        self.sim2model = sim2model        # wire(model) index -> articulation(sim) index
        self.device = robot.device
        self.control_dt = control_dt

        # Contact-sensor body indices for the feet, in SimInfo_ order [HR, HL, FR, FL].
        self.foot_ids = contact.find_bodies(FOOT_LINKS, preserve_order=True)[0]

        # Pose to hold (model order) before Motion takes over = the spawn (sitting) pose.
        self.hold_pose = robot.data.default_joint_pos[0][sim2model].clone()
        # Last commanded PD torque (model order) — published as SimInfo_ joint torque, matching
        # what MuJoCo/Gazebo report (the clamped commanded torque, not post-actuator applied).
        self._last_tau = torch.zeros(12, device=self.device)
        self._info = SimInfo_()
        self._latest_ref = None
        self._ref_gains_src = None        # MotionRef_ sample the cached command below was built from
        self._ref_active = False
        self._ref_gains = None            # (kp, kd, pos, vel, ff) rows, model order
        self._warned = False
        self._steps = 0

        # Lockstep state. _wait_ns is written by the sim thread, read by the DDS listener thread.
        self._ack_event = threading.Event()
        self._wait_ns = -1
        self._locked = False
        self._misses = 0
        self._next_wall = None
        self.stats = {"ticks": 0, "acked": 0, "timeouts": 0, "wait_s": 0.0, "wait_max_s": 0.0}

        # Per-joint T-N envelope params (model order): knee is every 3rd joint. Rows: GR, Tmax*GR, wbase, wmax.
        tn = [TN_K if (i % 3 == 2) else TN_RP for i in range(12)]
        self._tn_gr, self._tn_tau_max, self._tn_wbase, self._tn_wmax = torch.tensor(
            [[gr, tmax * gr, wb, wm] for gr, tmax, wb, wm in tn], device=self.device
        ).T.contiguous()

        ChannelFactoryInitialize(0, interface)
        self._pub = ChannelPublisher(SIM_TOPIC, SimInfo_)
        self._pub.Init()
        self._sub = ChannelSubscriber(REF_TOPIC, MotionRef_)
        self._sub.Init(handler=self._on_ref)
        print(f"[bridge] sub {REF_TOPIC} (MotionRef_), pub {SIM_TOPIC} (SimInfo_) on iface '{interface}'")

    # --- DDS listener thread ---
    def _on_ref(self, sample):
        self._latest_ref = sample
        if round(sample.time * 1e9) == self._wait_ns:
            self._ack_event.set()

    def _command(self, ref):
        """(active, gains) for a MotionRef_, rebuilt once per new sample — one host->device copy, not per physics step."""
        if ref is not self._ref_gains_src:
            self._ref_gains_src = ref
            # Motion takes over on its first real PD gain OR feed-forward torque, and keeps control from then
            # on: stand-up and 4-leg stance are pure torque (kp = 0 on every joint), so re-checking kp per
            # sample would drop those ticks back to the spawn-pose hold (same rule as rbq_gazebo).
            if not self._ref_active and ref is not None and any(
                j.kp > 1.0 or abs(j.torque) > FF_TORQUE_EPS for j in ref.leg_joint
            ):
                self._ref_active = True
                print("[bridge] Motion ACTIVE: applying MotionRef_ (spawn-pose hold released)")
            if self._ref_active:
                rows = [[j.kp for j in ref.leg_joint], [j.kd for j in ref.leg_joint],
                        [j.pos for j in ref.leg_joint], [j.vel for j in ref.leg_joint],
                        [j.torque for j in ref.leg_joint]]
                self._ref_gains = torch.tensor(rows, device=self.device)
        return self._ref_active, self._ref_gains

    def apply_command(self):
        d = self.robot.data
        qm = d.joint_pos[0][self.sim2model]
        dqm = d.joint_vel[0][self.sim2model]
        active, gains = self._command(self._latest_ref)

        if active:
            kp, kd, pos_ref, vel_ref, ff = gains
            tau_m = kp * (pos_ref - qm) + kd * (vel_ref - dqm) + ff
        else:
            if self._steps > 3000 and not self._warned:
                self._warned = True
                print(f"[bridge] WARNING: no MotionRef on {REF_TOPIC} after ~3s — holding spawn pose.")
            tau_m = HOLD_KP * (self.hold_pose - qm) + HOLD_KD * (-dqm)

        # Motor T-N envelope: clamp motoring only (regen not clipped).
        wm = dqm.abs() * self._tn_gr
        tau_lim = torch.minimum(
            (self._tn_tau_max * (self._tn_wmax - wm) / (self._tn_wmax - self._tn_wbase)).clamp_min(0.0),
            self._tn_tau_max,
        )
        tau_m = torch.where(tau_m * dqm >= 0, torch.clamp(tau_m, -tau_lim, tau_lim), tau_m)
        self._last_tau = tau_m

        # Map model-order torques back to articulation (sim) order and apply. The scatter
        # `tau_sim[sim2model] = tau_m` is the exact inverse of a model2sim gather, so no separate
        # model2sim index is needed here.
        tau_sim = torch.zeros_like(d.joint_pos[0])
        tau_sim[self.sim2model] = tau_m
        self.robot.set_joint_effort_target(tau_sim.unsqueeze(0))
        self._steps += 1

    def publish_state(self, stamp_ns: int):
        info = build_siminfo(self._info, self.robot, self.imu, self.contact, self.sim2model, self.foot_ids,
                             self._last_tau)
        info.imu.header.stamp.sec = stamp_ns // 1_000_000_000
        info.imu.header.stamp.nanosec = stamp_ns % 1_000_000_000
        # Arm the ack before the write so an immediate echo cannot slip past the event.
        self._wait_ns = stamp_ns
        self._ack_event.clear()
        self._pub.Write(info)

    def sync(self):
        """Lockstep on Motion's echo of the last published stamp; real-time pacing while Motion is absent."""
        st = self.stats
        st["ticks"] += 1
        t0 = time.perf_counter()
        if self._locked:
            acked = self._ack_event.wait(ACK_TIMEOUT_S)
            if acked:
                self._misses = 0
            else:
                st["timeouts"] += 1
                self._misses += 1
                if self._misses >= ACK_MISS_LIMIT:
                    self._locked = False
                    self._next_wall = None
                    print(f"[bridge] lockstep OFF: no Motion tick for {ACK_MISS_LIMIT} periods — real-time pacing")
        else:
            now = time.perf_counter()
            if self._next_wall is None or now - self._next_wall > FREE_RUN_RESYNC_S:
                self._next_wall = now
            self._next_wall += self.control_dt
            acked = self._ack_event.wait(max(0.0, self._next_wall - now))
            if acked:
                self._locked = True
                self._misses = 0
                print("[bridge] lockstep ON: stepping one control period per Motion RT tick")
        waited = time.perf_counter() - t0
        if acked:
            st["acked"] += 1
            st["wait_s"] += waited
            st["wait_max_s"] = max(st["wait_max_s"], waited)
        return acked

    @property
    def locked(self) -> bool:
        return self._locked
