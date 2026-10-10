import { rest } from '@/lib/rest';
import { PROGRAM } from '@/lib/robotState';
import { simEngine } from '@/lib/simEngine';

export const RL_GAIT = { RL_TROT_RUN: 45, RL_WALK_VISION: 48, RL_WALK: 49 } as const;
const QUADWALK_POLICY_CHANGE = 125;
const QUADWALK_SYS_ID = 115;

export const gait = {
  aiWalk(ip: string, id: number) {
    if (id < 30 || id >= 80) return Promise.resolve(undefined);
    if (simEngine.active && simEngine.aiWalk(id)) return Promise.resolve(undefined);
    return rest.commandStruct(ip, PROGRAM.QuadWalk, QUADWALK_POLICY_CHANGE, { char: [id] });
  },
  zmpCalibrate: (ip: string) => rest.commandStruct(ip, PROGRAM.QuadWalk, QUADWALK_SYS_ID, { int: [10] }),
};
