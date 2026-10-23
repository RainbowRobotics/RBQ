import { sendUserCommand } from './userCommand';
import { PROGRAM, QUADWALK_CMD, CRUISE } from './robotState';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import type { ButtonRole } from '@/lib/gamepad/profiles';

const CRUISE_GAIT_IDS = new Set([3, 6, 30, 42]);

export const cruiseRun = (mode: number) =>
  sendUserCommand(PROGRAM.QuadWalk, QUADWALK_CMD.CRUISE_VEL_SET, [], [mode]);

export function maybeCruise(role: ButtonRole | undefined): boolean {
  if (!role || !useSettings.getState().gpCruise) return false;
  const gid = useRobot.getState().robot?.gait_id;
  if (gid == null || !CRUISE_GAIT_IDS.has(gid)) return false;
  if (role === 'DPAD_U') { cruiseRun(CRUISE.INCREASE); return true; }
  if (role === 'DPAD_D') { cruiseRun(CRUISE.DECREASE); return true; }
  if (role === 'R3') { cruiseRun(CRUISE.START); return true; }
  return false;
}
