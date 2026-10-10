import { getHalf, getHalves } from './half';

export const SIZEOF = {
  REQUEST: 120,
  ROBOT_STATE: 592,
  DEVICE_STATES: 236,
  PDU_STATE: 200,
} as const;

export const PROGRAM = { Motion: 0, Network: 1, WalkReady: 2, QuadWalk: 3, ManiControl: 4 } as const;
export const WALKREADY_CMD = {
  GO_RECOVERY_READY: 106,
  FALL_RECOVERY_MOTION: 107,
  JOINT_LOCK_UNLOCK: 109,
  JOINT_SPACE_JOG: 110,
} as const;

export const QUADWALK_CMD = {
  CRUISE_VEL_SET: 120,
  OBS_AVOID: 700,
  JOINT_COMM_CHECK: 127,
  GAIT_CHECK_TROT: 128,
} as const;
export const CRUISE = { START: 0, INCREASE: 1, DECREASE: 2 } as const;

export function buildRequest(requestID: number, requestNumber = 1, floats: number[] = []): Uint8Array {
  const buf = new ArrayBuffer(SIZEOF.REQUEST);
  const dv = new DataView(buf);
  dv.setUint8(0, 255);
  dv.setUint8(1, 254);
  dv.setUint8(2, 2);
  dv.setInt32(4, requestID, true);
  dv.setUint32(8, requestNumber >>> 0, true);
  floats.forEach((v, i) => dv.setFloat32(76 + i * 4, v, true));
  dv.setUint8(116, 128);
  dv.setUint8(117, 127);
  return new Uint8Array(buf);
}

export function isRequestEcho(buf: ArrayBufferLike, byteOffset = 0): boolean {
  if ((buf.byteLength - byteOffset) < SIZEOF.REQUEST) return false;
  const dv = new DataView(buf, byteOffset, SIZEOF.REQUEST);
  return dv.getUint8(0) === 255 && dv.getUint8(1) === 254 && dv.getUint8(2) === 2
    && dv.getUint8(116) === 128 && dv.getUint8(117) === 127;
}

export function parseRequestEcho(buf: ArrayBufferLike, byteOffset = 0) {
  const dv = new DataView(buf, byteOffset, SIZEOF.REQUEST);
  const floats: number[] = [];
  for (let i = 0; i < 6; i++) floats.push(dv.getFloat32(76 + i * 4, true));
  return { requestID: dv.getInt32(4, true), ok: dv.getUint8(14) !== 0, floats };
}


export type JointState = {
  connected: boolean;
  temperature: number;
  locked: boolean;
  position: number;
  torque: number;
  current: number;
  run: boolean;
  calib: boolean;
  errors: string[];
  statorTemp: number;
};

const JOINT_ERR_NAMES = ['JAM', 'CUR', 'BIG', 'INP', 'FLT', 'TMP', 'PS1', 'PS2'];
const NO_JOINT_ERRS: string[] = [];

export type ExtDevState = { connected: boolean; motorOn: boolean; moving: boolean; load: number; status: number };

export type RobotState = {
  time: number;
  gaitId: number;
  isFall: boolean;
  imuSuccess: boolean;
  extJoy: boolean;
  isStanding: boolean;
  obsAvoidEnabled: boolean;
  attached: {
    arm: boolean;
    ext1: boolean;
    ext2: boolean;
    cctv: boolean;
    thermal: boolean;
    ptz: boolean;
  };
  battery: {
    percentage: number;
    voltage: number;
    current: number;
  };
  imu: {
    quaternion: [number, number, number, number];
    rpy: [number, number, number];
    gyro: [number, number, number];
    acc: [number, number, number];
  };
  worldPos: [number, number, number];
  worldRpy: [number, number, number];
  groundPos: [number, number, number];
  groundRpy: [number, number, number];
  jointCount: number;
  joints: JointState[];
  tripTotals: { distMm: number; stepCnt: number; timeS: number };
  extDev: {
    j12?: { connected: boolean };
    j13?: ExtDevState;
    j14?: ExtDevState;
  };
  armStat: {
    canCheck: boolean; brakeRelease: boolean; conStart: boolean;
    isPacking: boolean; isReady: boolean; isHome: boolean; isStraight: boolean;
    motionCmd: number;
    missionType: number;
    manualControl: boolean; lockPosition: boolean;
  };
};

const JOINT_BASE = 228;
const JOINT_SIZE = 16;

export function parseRobotState(buf: ArrayBuffer, byteOffset = 0): RobotState {
  const dv = new DataView(buf, byteOffset, SIZEOF.ROBOT_STATE);
  const MAX_JOINTS = Math.floor((SIZEOF.ROBOT_STATE - JOINT_BASE) / JOINT_SIZE);
  const jointCount = Math.min(dv.getUint8(224), MAX_JOINTS);
  const joints: JointState[] = [];
  for (let i = 0; i < jointCount; i++) {
    const o = JOINT_BASE + i * JOINT_SIZE;
    const st = dv.getUint8(o + 4);
    const errBits = dv.getUint8(o + 5);
    joints.push({
      connected: dv.getUint8(o) !== 0,
      temperature: dv.getInt8(o + 1),
      locked: dv.getUint16(o + 8, true) !== 0,
      position: getHalf(dv, o + 12),
      torque: getHalf(dv, o + 14),
      current: getHalf(dv, o + 2),
      run: !!(st & 0b10),
      calib: !!(st & 0b1000000),
      errors: errBits === 0 ? NO_JOINT_ERRS : JOINT_ERR_NAMES.filter((_, b) => errBits & (1 << b)),
      statorTemp: dv.getUint8(o + 6),
    });
  }
  return {
    time: dv.getFloat64(8, true),
    gaitId: dv.getInt8(32),
    isFall: dv.getUint8(33) !== 0,
    imuSuccess: dv.getUint8(35) !== 0,
    extJoy: dv.getUint8(28) !== 0,
    isStanding: dv.getUint8(29) !== 0,
    obsAvoidEnabled: dv.getUint8(36) !== 0,
    attached: {
      arm: dv.getUint8(40) !== 0,
      ext1: dv.getUint8(41) !== 0,
      ext2: dv.getUint8(42) !== 0,
      cctv: dv.getUint8(43) !== 0,
      thermal: dv.getUint8(44) !== 0,
      ptz: dv.getUint8(45) !== 0,
    },
    battery: {
      percentage: dv.getUint8(60),
      voltage: dv.getUint8(62),
      current: getHalf(dv, 66),
    },
    imu: {
      quaternion: getHalves(dv, 88, 4) as [number, number, number, number],
      rpy: getHalves(dv, 96, 3) as [number, number, number],
      gyro: getHalves(dv, 102, 3) as [number, number, number],
      acc: getHalves(dv, 108, 3) as [number, number, number],
    },
    worldPos: getHalves(dv, 128, 3) as [number, number, number],
    worldRpy: getHalves(dv, 134, 3) as [number, number, number],
    groundPos: getHalves(dv, 192, 3) as [number, number, number],
    groundRpy: getHalves(dv, 198, 3) as [number, number, number],
    tripTotals: {
      distMm: Number(dv.getBigUint64(548, true)),
      stepCnt: Number(dv.getBigUint64(556, true)),
      timeS: Number(dv.getBigUint64(564, true)),
    },

    jointCount,
    joints,
    extDev: (() => {
      const rc = (idx: number): ExtDevState | undefined => {
        if (idx >= jointCount) return undefined;
        const o = JOINT_BASE + idx * JOINT_SIZE;
        return {
          connected: dv.getUint8(o) !== 0,
          motorOn: dv.getUint8(o + 4) !== 0,
          moving: dv.getUint8(o + 5) !== 0,
          load: dv.getUint8(o + 6),
          status: dv.getUint8(o + 7),
        };
      };
      const j12 = 12 < jointCount ? { connected: dv.getUint8(JOINT_BASE + 12 * JOINT_SIZE) !== 0 } : undefined;
      return { j12, j13: rc(13), j14: rc(14) };
    })(),
    armStat: (() => {
      const b0 = dv.getUint8(48);
      const b3 = dv.getUint8(51);
      return {
        canCheck: !!(b0 & 1), brakeRelease: !!(b0 & 2), conStart: !!(b0 & 4),
        isPacking: !!(b0 & 8), isReady: !!(b0 & 16), isHome: !!(b0 & 32), isStraight: !!(b0 & 64),
        motionCmd: dv.getUint8(49),
        missionType: dv.getUint8(50),
        manualControl: !!(b3 & 2), lockPosition: !!(b3 & 4),
      };
    })(),
  };
}


export type DeviceStatus = {
  attached: boolean;
  powered: boolean;
  connected: boolean;
  normal: boolean;
  running: boolean;
};

const DEVICE_BASE = 8;
const DEVICE_SIZE = 5;
const DEVICE_COUNT = 45;

export function parseDeviceStates(buf: ArrayBuffer, byteOffset = 0): DeviceStatus[] {
  const dv = new DataView(buf, byteOffset, SIZEOF.DEVICE_STATES);
  const out: DeviceStatus[] = new Array(DEVICE_COUNT);
  for (let i = 0; i < DEVICE_COUNT; i++) {
    const o = DEVICE_BASE + i * DEVICE_SIZE;
    out[i] = {
      attached: dv.getUint8(o) !== 0,
      powered: dv.getUint8(o + 1) !== 0,
      connected: dv.getUint8(o + 2) !== 0,
      normal: dv.getUint8(o + 3) !== 0,
      running: dv.getUint8(o + 4) !== 0,
    };
  }
  return out;
}


export type PduState = {
  cam5v: boolean; audio5v: boolean;
  visionPc: boolean; comm: boolean; lidar: boolean;
  cctv: boolean; thermal: boolean; irled: boolean;
  amp: boolean;
  fetLeg: boolean; fetAdd: boolean; fetExt: boolean;
  batSwL: boolean; batSwR: boolean; chgE: boolean; chgS: boolean;
  tempPdu: number; tempPs: number; batPct: number;
  rails: { total: Rail; batL: Rail; batR: Rail; leg: Rail; add: Rail; ext: Rail; chg: Rail };
};
export type Rail = { v: number; a: number };

const PDU_BITS = 40;

export function parsePduState(buf: ArrayBuffer, byteOffset = 0): PduState {
  const dv = new DataView(buf, byteOffset, SIZEOF.PDU_STATE);
  const b0 = dv.getUint8(PDU_BITS);
  const b1 = dv.getUint8(PDU_BITS + 1);
  const b2 = dv.getUint8(PDU_BITS + 2);
  const rail = (o: number): Rail => ({ v: getHalf(dv, o), a: getHalf(dv, o + 2) });
  return {
    cam5v: !!(b0 & 1), audio5v: !!(b0 & 2),
    visionPc: !!(b0 & 4), comm: !!(b0 & 8), lidar: !!(b0 & 16),
    cctv: !!(b0 & 32), thermal: !!(b0 & 64), irled: !!(b0 & 128),
    amp: !!(b1 & 1),
    fetLeg: !!(b2 & 16), fetAdd: !!(b2 & 32), fetExt: !!(b2 & 64),
    batSwR: !!(b1 & 2), batSwL: !!(b1 & 4), chgE: !!(b1 & 8), chgS: !!(b1 & 16),
    tempPdu: dv.getUint8(PDU_BITS + 3), tempPs: dv.getUint8(PDU_BITS + 4), batPct: dv.getUint8(PDU_BITS + 5),
    rails: { total: rail(52), batL: rail(56), batR: rail(60), leg: rail(64), add: rail(68), ext: rail(72), chg: rail(76) },
  };
}

export const PDU_PORT = {
  LEG_48V: 0x00, ADD_48V: 0x01, EXT_48V: 0x02,
  VISION_PC: 0x10, COMM: 0x11, LIDAR: 0x12, CCTV: 0x13, THERMAL: 0x14, IRLED: 0x15, SPEAKER: 0x16,
  CAMERAS_5V: 0x20, AUDIO_USBHUB_5V: 0x21,
} as const;

const CHARGING_MIN_A = 1;
const CHARGE_TAU_MS = 3000;

export type ChargeAvg = { a: number; t: number };

export function nextChargeAvg(prev: ChargeAvg | null, a: number, now: number): ChargeAvg {
  if (!prev) return { a, t: now };
  const k = 1 - Math.exp(-Math.max(0, now - prev.t) / CHARGE_TAU_MS);
  return { a: prev.a + (a - prev.a) * k, t: now };
}

export function isRobotCharging(avg: ChargeAvg | null | undefined): boolean {
  return !!avg && avg.a > CHARGING_MIN_A;
}
