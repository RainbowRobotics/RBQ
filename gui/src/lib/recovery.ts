import { sendUserCommand } from './userCommand';
import { PROGRAM, WALKREADY_CMD } from './robotState';

export const recovery = {
  clearErrors: () =>
    sendUserCommand(PROGRAM.WalkReady, WALKREADY_CMD.GO_RECOVERY_READY),
  autoRecovery: () =>
    sendUserCommand(PROGRAM.WalkReady, WALKREADY_CMD.FALL_RECOVERY_MOTION),
  lockJoint: (jointId: number, lock: boolean) =>
    sendUserCommand(PROGRAM.WalkReady, WALKREADY_CMD.JOINT_LOCK_UNLOCK, [jointId, lock ? 1 : 0]),
  startJog: (jointId: number, positive: boolean) =>
    sendUserCommand(PROGRAM.WalkReady, WALKREADY_CMD.JOINT_SPACE_JOG, [jointId, positive ? 1 : -1, 1]),
  stopJog: (jointId: number) =>
    sendUserCommand(PROGRAM.WalkReady, WALKREADY_CMD.JOINT_SPACE_JOG, [jointId, 0, 0]),
};
