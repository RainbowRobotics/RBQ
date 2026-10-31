import type { RobotState, PduState } from './robotState';

export function armAvailable(robot: RobotState | undefined, pdu: PduState | undefined, simulation = false) {
  const joints = robot?.joints.slice(12, 18) ?? [];
  const powered = simulation || pdu?.fetAdd === true;
  return !!robot?.attached.arm && powered && joints.length === 6 && joints.every(j => j.connected);
}
