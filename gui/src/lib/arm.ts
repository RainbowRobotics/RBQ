import { sendUserCommand } from './userCommand';
import { PROGRAM } from './robotState';

export const MC = {
  CONTROL_START: 103,
  GO_MOTION: 102,
  MOVE_DELTA_XYZ: 107,
  MOVE_DELTA_RPY: 108,
  MANUAL_CONTROL_MODE: 120,
  CONTROL_LOCK_JOINT: 121,
  CONTROL_UNLOCK_JOINT: 122,
  MANUAL_CONTROL_BTN_MODE: 130,
  MANUAL_CONTROL_BUTTON: 131,
  GRP_OPEN: 501,
  GRP_CLOSE: 502,
  DO_REPEAT_MOTION: 300,
  MOVE_CIRCLE: 109,
  MOVE_INFINITY: 110,
  MOVE_HEART: 111,
  MOVE_RB_LOGO: 115,
  DO_GREETING_MOTION: 360,
  DEMO_DO_GREETING: 1000,
  DEMO_ALL_JOINT_MOVE: 1001,
  DEMO_DO_HANDSHAKE: 1002,
  DEMO_CHICKEN_HEAD_MODE: 1010,
  DEMO_ARM_CONTROL_MODE: 1011,
  DEMO_ROBOT_CONTROL_MODE: 1012,
  DEMO_LOCK_JOINTS_ON: 1013,
  DEMO_LOCK_JOINTS_OFF: 1014,
  DEMO_DRAWING_READY: 1020,
  DEMO_DRAWING_START: 1021,
  DEMO_GRASPING_READY: 1030,
  DEMO_PAYLOAD_ON_BODY: 1031,
  DEMO_PAYLOAD_ON_FLOOR: 1032,
  DEMO_CHAMCHAMCHAM_INIT: 1040,
  DEMO_CHAMCHAMCHAM_LEFT: 1041,
  DEMO_CHAMCHAMCHAM_RIGHT: 1042,
  DEMO_AUTO_RECOVERY: 1050,
  DEMO_DAMPING_MODE: 1051,
  JOINT_LOCK_UNLOCK: 400,
  JOINT_SPACE_JOG: 401,
  DOOR_DETECT_HANDLE: 200,
  DOOR_APPROACH_HANDLE: 201,
  DOOR_CATCH_UNLOCK_HANDLE: 202,
  DOOR_OPEN_DOOR: 203,
  DOOR_GO_FINISH_POS: 204,
  DOOR_HANDLE_RELEASE: 205,
  DOOR_AUTO_DOCKING: 211,
} as const;

const DAEMON = {
  MANIPULATOR_INIT: 800,
  MANIPULATOR_REF_ON: 801,
  MANI_AUTO_READY: 802,
  LAN2CAN_GENERAL_MSG: 600,
} as const;

export const DEMO = {
  DO_GREETING: 1000, ALL_JOINT_MOVE: 1001, DO_HANDSHAKE: 1002,
  CHICKEN_HEAD_MODE: 1010, ARM_CONTROL_MODE: 1011, ROBOT_CONTROL_MODE: 1012,
  LOCK_JOINTS_ON: 1013, LOCK_JOINTS_OFF: 1014,
  DRAWING_READY: 1020, DRAWING_START: 1021,
  GRASPING_READY: 1030, PAYLOAD_ON_BODY: 1031, PAYLOAD_ON_FLOOR: 1032,
  CHAMCHAMCHAM_INIT: 1040, CHAMCHAMCHAM_LEFT: 1041, CHAMCHAMCHAM_RIGHT: 1042,
  AUTO_RECOVERY: 1050, DAMPING_MODE: 1051,
} as const;
export const DEMO_ARG = { Circle: 10, Heart: 11, Star: 12, Vertical: 10, Horizontal: 11, Forward: 10, Downward: 11 } as const;

export const MANI_MOTION = {
  Straight: 100, Folding: 101, Ready: 102, Lift: 103, Packing: 104, DoorReady: 105,
  DoorFinish: 106, ShakeHand: 107, PayloadBody: 108, PickUpReady: 109, TaskMotionReady: 110,
} as const;
export type ManiMotion = keyof typeof MANI_MOTION;

export const DRAWING_SHAPE = { Circle: 10, Heart: 11, Star: 12, Infinity: 13 } as const;
export type DrawingShape = keyof typeof DRAWING_SHAPE;
export const GRASP_ROLL = { Vertical: 10, Horizontal: 11, None: -1 } as const;
export const GRASP_PITCH = { Forward: 10, Downward: 11, None: -1 } as const;
export type GraspRoll = keyof typeof GRASP_ROLL;
export type GraspPitch = keyof typeof GRASP_PITCH;

export const ARM_MISSION: Record<number, string> = {
  0: 'DOCKING_TRY', 1: 'DOCKING_FIN', 2: 'GET_HANDLE_INFO', 3: 'APPROACH_HANDLE',
  4: 'UNLOCK_HANDLE', 5: 'OPEN_TRY', 6: 'OPEN_FIN', 7: 'HANDLE_FRONT_FAR',
  8: 'HANDLE_FRONT_CLOSE', 9: 'HANDLE_SIDE_FAR', 10: 'HANDLE_POS_ERROR',
  11: 'DOCKING_INFO_ERROR', 12: 'DOCKING_INFO_RETRY', 13: 'DOCKING_INFO_GOOD', 20: 'GRASP_TODO',
};

export type DoorParams = { handleType: number; hingeSide: number; openType: number };
export type Pose6 = [number, number, number, number, number, number];

const mani = (cmd: number, paraInt: number[] = [], paraChar: number[] = [], paraFloat: number[] = []) =>
  sendUserCommand(PROGRAM.ManiControl, cmd, paraInt, paraChar, paraFloat, {replayOnReconnect:false});
const motion = (cmd: number, paraInt: number[] = [], paraChar: number[] = []) =>
  sendUserCommand(PROGRAM.Motion, cmd, paraInt, paraChar, [], {replayOnReconnect:false});

export const arm = {
  canCheck: () => motion(DAEMON.MANIPULATOR_INIT),
  brakeRelease: () => motion(DAEMON.MANIPULATOR_REF_ON),
  autoReady: () => motion(DAEMON.MANI_AUTO_READY),
  controlStart: () => mani(MC.CONTROL_START, [1]),

  goMotion: (m: ManiMotion, mode = 0) => mani(MC.GO_MOTION, [MANI_MOTION[m], mode]),

  moveCircle: () => mani(MC.MOVE_CIRCLE),
  moveInfinity: () => mani(MC.MOVE_INFINITY),
  moveHeart: () => mani(MC.MOVE_HEART),
  moveRbLogo: () => mani(MC.MOVE_RB_LOGO),
  greeting: () => mani(MC.DO_GREETING_MOTION),

  demoGreeting: () => mani(MC.DEMO_DO_GREETING),
  demoAllJointMove: () => mani(MC.DEMO_ALL_JOINT_MOVE),
  demoHandshake: () => mani(MC.DEMO_DO_HANDSHAKE),
  demoChickenHead: () => mani(MC.DEMO_CHICKEN_HEAD_MODE),
  demoArmControlMode: () => mani(MC.DEMO_ARM_CONTROL_MODE),
  demoRobotControlMode: () => mani(MC.DEMO_ROBOT_CONTROL_MODE),
  demoLockJoints: (on: boolean) => mani(on ? MC.DEMO_LOCK_JOINTS_ON : MC.DEMO_LOCK_JOINTS_OFF),
  demoDrawingReady: () => mani(MC.DEMO_DRAWING_READY),
  demoDrawingStart: (shape: DrawingShape) => mani(MC.DEMO_DRAWING_START, [DRAWING_SHAPE[shape]]),
  demoGraspingReady: (roll: GraspRoll, pitch: GraspPitch) =>
    mani(MC.DEMO_GRASPING_READY, [GRASP_ROLL[roll], GRASP_PITCH[pitch]]),
  demoPayload: (onBody: boolean) => mani(onBody ? MC.DEMO_PAYLOAD_ON_BODY : MC.DEMO_PAYLOAD_ON_FLOOR),
  demoChamChamInit: () => mani(MC.DEMO_CHAMCHAMCHAM_INIT),
  demoChamChamLeft: () => mani(MC.DEMO_CHAMCHAMCHAM_LEFT),
  demoChamChamRight: () => mani(MC.DEMO_CHAMCHAMCHAM_RIGHT),
  demoAutoRecovery: () => mani(MC.DEMO_AUTO_RECOVERY),
  demoDampingMode: () => mani(MC.DEMO_DAMPING_MODE),

  aimingMode: () => sendUserCommand(PROGRAM.QuadWalk, 114, [], [], [], {replayOnReconnect:false}),

  deltaXyz: (axis: 1 | 2 | 3, m: number) => mani(MC.MOVE_DELTA_XYZ, [axis, 1], [], [m]),
  deltaRpy: (axis: 1 | 2 | 3, deg: number) => mani(MC.MOVE_DELTA_RPY, [axis, 1], [], [deg]),

  jogXyz: (axis: 1 | 2 | 3, sign: -1 | 0 | 1) => mani(MC.MANUAL_CONTROL_BUTTON, [1, axis, sign]),
  jogRpy: (axis: 1 | 2 | 3, sign: -1 | 0 | 1) => mani(MC.MANUAL_CONTROL_BUTTON, [2, axis, sign]),

  manualReset: () => mani(MC.MANUAL_CONTROL_BTN_MODE),
  fix: () => mani(MC.CONTROL_LOCK_JOINT),
  control: () => mani(MC.CONTROL_UNLOCK_JOINT),

  jointLock: (id: number, lock: boolean) => mani(MC.JOINT_LOCK_UNLOCK, [id - 12, lock ? 1 : 0]),
  jointJogStart: (id: number, positive: boolean) => mani(MC.JOINT_SPACE_JOG, [id - 12, positive ? 1 : -1, 1]),
  jointJogStop: (id: number) => mani(MC.JOINT_SPACE_JOG, [id - 12, 0, 0]),

  doorDetect: () => mani(MC.DOOR_DETECT_HANDLE, [1]),
  doorAutoDocking: (p: DoorParams, doorPose: Pose6) =>
    mani(MC.DOOR_AUTO_DOCKING, [p.handleType, p.hingeSide, p.openType], [1], doorPose),
  doorApproach: (p: DoorParams, handlePose: Pose6) =>
    mani(MC.DOOR_APPROACH_HANDLE, [p.handleType, p.hingeSide, p.openType], [1], handlePose),
  doorCatch: (p: DoorParams, handlePose: Pose6) =>
    mani(MC.DOOR_CATCH_UNLOCK_HANDLE, [p.handleType, p.hingeSide, p.openType], [1], handlePose),
  doorOpen: (p: DoorParams, handlePose: Pose6) => {
    mani(MC.DOOR_OPEN_DOOR, [p.handleType, p.hingeSide, p.openType], [1], handlePose);
    sendUserCommand(PROGRAM.QuadWalk, 119, [], [], [], {replayOnReconnect:false});
  },
  doorFinish: () => mani(MC.DOOR_GO_FINISH_POS),
  doorRelease: () => mani(MC.DOOR_HANDLE_RELEASE),

  intRequest: (req: number, i0 = 0, i1 = 0, i2 = 0) => mani(req, [i0, i1, i2]),

  gripper: (dir: 'open' | 'close' | 'stop') => {
    if (dir === 'stop') {
      mani(MC.GRP_OPEN, [], [0]);
    } else {
      mani(dir === 'open' ? MC.GRP_OPEN : MC.GRP_CLOSE, [], [1]);
    }
  },
};
