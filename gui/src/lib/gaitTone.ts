export type GaitTone = 'fault' | 'idle' | 'active' | 'unknown';

function toneFromName(name?: string | null): GaitTone {
  if (!name) return 'unknown';
  if (/^(FALL_MODE|FALL_RECOVERY|CONTROL_OFF)$/.test(name)) return 'fault';
  if (/^(SITTING|STANDING)$/.test(name)) return 'idle';
  if (/^(TROTTING|WALKING|RUNNING|STAIR|DOCKING|AIMING|RL_)/.test(name)) return 'active';
  return 'unknown';
}

export function gaitTone(gaitId?: number | null, name?: string | null): GaitTone {
  if (gaitId == null || !Number.isFinite(gaitId)) return toneFromName(name);
  if (gaitId < 0) return 'fault';
  if (gaitId <= 1) return 'idle';
  if (gaitId <= 10) return 'active';
  if (gaitId >= 30 && gaitId <= 49) return 'active';
  return 'unknown';
}
