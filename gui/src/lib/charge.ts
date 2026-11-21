import { isRobotCharging, type ChargeAvg, type PduState } from '@/lib/robotState';

export const PACK_AH = 9;
export const TAIL_MAX_MIN = 100;
const PACK_CHARGING_A = 0.5;
const NO_CURRENT_GRACE_MS = 20_000;
const NO_CURRENT_BELOW_PCT = 95;
const NO_VALUE_A = 300;

export function packInAmps(pdu: PduState): number {
  let a = 0;
  if (pdu.batSwL && Math.abs(pdu.rails.batL.a) < NO_VALUE_A) a -= pdu.rails.batL.a;
  if (pdu.batSwR && Math.abs(pdu.rails.batR.a) < NO_VALUE_A) a -= pdu.rails.batR.a;
  return a;
}

export function chargeMinutes(fromPct: number, toPct: number, packs: number, amps: number): number {
  return Math.max(1, Math.round(((toPct - fromPct) / 100) * PACK_AH * packs / amps * 60));
}

export function robotCharging(fd: boolean, packAvg: ChargeAvg | null | undefined, railAvg: ChargeAvg | null | undefined): boolean {
  return fd ? !!packAvg && packAvg.a > PACK_CHARGING_A : isRobotCharging(railAvg);
}

export type ChargeIssue = 'no-current' | 'one-pack' | 'no-pack';
export type ChargeView = {
  charging: boolean;
  amps: number | null;
  to80Min: number | null;
  tail: boolean;
  issues: ChargeIssue[];
};

export function chargeView(i: {
  fd: boolean; pdu?: PduState; pct: number;
  packAvg?: ChargeAvg | null; railAvg?: ChargeAvg | null; plugSince?: number | null; now: number;
}): ChargeView {
  const charging = robotCharging(i.fd, i.packAvg, i.railAvg);
  if (!i.fd || !i.pdu) return { charging, amps: null, to80Min: null, tail: false, issues: [] };
  const packs = (i.pdu.batSwL ? 1 : 0) + (i.pdu.batSwR ? 1 : 0);
  const issues: ChargeIssue[] = [];
  if (packs === 0) issues.push('no-pack');
  else if (packs === 1) issues.push('one-pack');
  if (!charging && i.pct < NO_CURRENT_BELOW_PCT && i.plugSince != null && i.now - i.plugSince >= NO_CURRENT_GRACE_MS) issues.push('no-current');
  const amps = charging && i.packAvg ? i.packAvg.a : null;
  const to80Min = amps && packs > 0 && i.pct < 80 ? chargeMinutes(i.pct, 80, packs, amps) : null;
  return { charging, amps, to80Min, tail: charging && i.pct < 100, issues };
}
