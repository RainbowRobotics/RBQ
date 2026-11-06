import { useEffect, useRef, useState } from 'react';
import { rest, actions, type LegHomeSetStatus, type LegHomeSetLeg, type LegHomeGroup, type LegHomeRef } from './rest';
import { t } from './i18n';

export type { LegHomeSetStatus, LegHomeSetLeg, LegHomeGroup, LegHomeRef };

export const LEG_COUNT = 4;

export const LEG_INFO: { name: 'RR' | 'RL' | 'FR' | 'FL'; label: string; side: 'right' | 'left' }[] = [
  { name: 'RR', label: '후방 우측', side: 'right' },
  { name: 'RL', label: '후방 좌측', side: 'left' },
  { name: 'FR', label: '전방 우측', side: 'right' },
  { name: 'FL', label: '전방 좌측', side: 'left' },
];

export function doneLabel(entry: LegHomeSetLeg | undefined): string | null {
  const done = entry?.joint_done;
  if (!done) return null;
  if (done[0] && done[1] && done[2]) return t('roll/pitch/knee 완료');
  if (done[0]) return t('roll 완료');
  if (done[1] && done[2]) return t('pitch/knee 완료');
  return null;
}

export function groupDone(entry: LegHomeSetLeg | undefined, group: LegHomeGroup): boolean {
  const done = entry?.joint_done;
  if (!done) return false;
  return group === 'roll' ? done[0] : done[1] && done[2];
}

export const HOME_SET_ROLL_DEG = -42;
export const HOME_SET_PITCH_DEG = 180;
export const HOME_SET_KNEE_DEG = -158.2;


export function homeTargetDeg(leg: number): [number, number, number] {
  return [leg % 2 === 0 ? HOME_SET_ROLL_DEG : -HOME_SET_ROLL_DEG, HOME_SET_PITCH_DEG, HOME_SET_KNEE_DEG];
}

export function rollTargetDeg(ref: LegHomeRef, limitRollDeg: number): number {
  return ref === 'level' ? 0 : limitRollDeg;
}

const NEUTRAL_DEG = 0.05;

export function isRollNeutral(expectedRollDeg: number): boolean {
  return Math.abs(expectedRollDeg) < NEUTRAL_DEG;
}

export function jointGuide(leg: number, joint: 0 | 1 | 2, expectedDeg: number): string {
  const right = LEG_INFO[leg]?.side === 'right';
  const near0 = Math.abs(expectedDeg) < NEUTRAL_DEG;
  if (joint === 0) {
    if (near0) return t('몸통과 롤 조인트의 상대각도를 0도로 유지하세요');
    return t('발을 몸통 안쪽(로봇 {dir}쪽)으로 끝까지 당겨 붙입니다.')
      .replace('{dir}', right ? t('왼') : t('오른'));
  }
  if (joint === 1) {
    return t('허벅지를 위로(뒤로) 접어 몸통 옆면에 붙입니다 — 수직 아래가 0°, 목표는 완전히 접힌 자세입니다.');
  }
  return t('캘리브레이션 치구에 막대를 먼저 넣은 뒤 끝까지 당깁니다.');
}

export function legTitle(leg: number): string {
  const info = LEG_INFO[leg];
  return info ? `Leg${leg} · ${info.name} (${t(info.label)})` : `Leg${leg}`;
}

export const JOINT_LABELS = ['Roll', 'Pitch', 'Knee'];

export function humanError(e: unknown): string {
  const msg = String((e as { message?: string })?.message ?? e);
  if (msg.includes('unknown command route') || msg.includes('404') || msg.includes('group must be roll or pitch_knee')) {
    return t('이 로봇의 소프트웨어에는 이 기능이 없습니다 — 로봇을 업데이트하세요.');
  }
  if (msg.includes('503') || msg.includes('motor boards not responding') || msg.includes('can_check_failed')) {
    return t('모터 보드와 통신이 없습니다 — 48V 구동(LEGS) 전원을 켠 뒤 다시 실행하세요.');
  }
  if (msg.includes('robot_not_stopped')) {
    return t('앉히거나 제어를 끈 뒤 실행하세요 — 실행하면 제어가 끊겨 서 있으면 주저앉습니다');
  }
  if (msg.includes('home_set_unconfigured')) {
    return t('이 로봇에는 치구 홈각이 설정돼 있지 않습니다(J_HOME_SET_R=0) — 설정 파일에서 먼저 지정하세요.');
  }
  if (msg.includes('409')) return t('이미 다른 다리를 캘리브레이션 중입니다.');
  if (msg.includes('403')) return t('제어권이 없습니다 — 상단바에서 제어권을 가져오세요.');
  return msg;
}

export async function startLeg(ip: string, leg: number, group: LegHomeGroup, ref: LegHomeRef = 'limit'): Promise<string | null> {
  try {
    await actions.legHomeSetStart(ip, leg, group, ref);
    return null;
  } catch (e: any) {
    return humanError(e);
  }
}

export function useLegHomeSet(ip: string, enabled: boolean) {
  const [status, setStatus] = useState<LegHomeSetStatus | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const kick = useRef<() => void>(() => {});
  useEffect(() => {
    if (!ip || !enabled) return;
    let alive = true;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const tick = async () => {
      try {
        const next = await rest.legHomeSet(ip);
        if (!alive) return;
        setStatus(next);
        setErr(null);
        timer = setTimeout(tick, next.running ? 400 : 2000);
      } catch (e: unknown) {
        if (!alive) return;
        setErr(humanError(e));
        timer = setTimeout(tick, 3000);
      }
    };
    kick.current = () => { if (timer) clearTimeout(timer); tick(); };
    tick();
    return () => { alive = false; kick.current = () => {}; if (timer) clearTimeout(timer); };
  }, [ip, enabled]);
  return { status, err, refresh: () => kick.current() };
}
