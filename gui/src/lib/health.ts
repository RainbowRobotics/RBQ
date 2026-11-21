import type { Href } from 'expo-router';
import type { HealthFinding } from '@/types/robot';
import { t } from '@/lib/i18n';

const TEXT: Record<string, string> = {
  'robot.fallen': '비상정지 상태 (넘어짐·E-STOP)',
  'boot.imu': 'IMU 준비 안 됨',
  'boot.can_bus': 'CAN 점검 안 됨',
  'boot.find_pose': '자세 찾기 안 됨',
  'boot.control': '제어 시작 안 됨',
  'imu.disconnected': 'IMU 연결 끊김',
  'motor.disconnected': '{c} 모터 연결 끊김',
  'motor.error': '{c} 모터 에러 ({d})',
  'motor.over_temp': '{c} 모터 온도 {v}{u} (기준 {th}{u})',
  'board.over_temp': '{c} 보드 온도 {v}{u} (기준 {th}{u})',
  'battery.low': '배터리 부족 {v}{u} (기준 {th}{u})',
  'battery.protection': '배터리 {c} 보호 동작 ({d})',
  'pdu.over_temp': '전원부 {c} 온도 {v}{u} (기준 {th}{u})',
  'can.channel': 'CAN {c} 통신 오류',
  'vision.sensor_failed': '카메라 {c} 오류 (코드 {code})',
  'vision.app_error': '비전 {c} 오류 (코드 {code})',
};

export const isBootFinding = (f: HealthFinding) => f.id.startsWith('boot.');

const num = (v?: number) => (v === undefined ? '' : String(Math.round(v * 10) / 10));

const ERR_WORDS: Record<string, string> = { CMD: '지령 끊김' };
const errDetail = (d: string) => d.split(',').map((n) => (ERR_WORDS[n] ? `${n} ${t(ERR_WORDS[n])}` : n)).join(',');

export function healthText(f: HealthFinding): string {
  const tpl = TEXT[f.id];
  if (!tpl) return f.component ? `${f.id} (${f.component})` : f.id;
  return t(tpl)
    .replace('{c}', f.component ?? '')
    .replace('{v}', num(f.value))
    .replace(/\{th\}/g, num(f.threshold))
    .replace(/\{u\}/g, f.unit === 'C' ? '°C' : (f.unit ?? ''))
    .replace('{code}', String(f.code ?? ''))
    .replace('{d}', f.id === 'motor.error' && f.detail ? errDetail(f.detail) : (f.detail ?? ''));
}

export function healthTarget(f: HealthFinding, devMode: boolean): Href | undefined {
  if (f.id.startsWith('battery.') || f.id.startsWith('pdu.')) return '/maintenance?sec=pwr';
  if (f.id.startsWith('motor.') || f.id.startsWith('board.') || f.id === 'can.channel') return '/maintenance?sec=chk';
  if (f.id.startsWith('vision.')) return devMode ? '/maintenance?sec=vision' : undefined;
  return undefined;
}

function healthGroup(id: string): string {
  if (id === 'robot.fallen') return t('비상정지 상태');
  if (id === 'battery.low') return t('배터리 부족');
  if (id.startsWith('battery.') || id.startsWith('pdu.')) return t('배터리 이상');
  if (id.startsWith('motor.') || id.startsWith('board.')) return t('다리 이상');
  if (id === 'imu.disconnected' || id === 'can.channel') return t('통신 이상');
  if (id.startsWith('vision.')) return t('카메라 이상');
  return t('로봇 이상');
}

export type HealthRow = { key: string; severity: HealthFinding['severity']; text: string; to?: Href };

export function healthRows(findings: HealthFinding[] | undefined, level: number): HealthRow[] {
  const items = (findings ?? []).filter((f) => !isBootFinding(f))
    .sort((a, b) => (a.severity === b.severity ? 0 : a.severity === 'fault' ? -1 : 1));
  const devMode = level >= 2;
  if (devMode) {
    return items.map((f) => ({ key: `${f.id}/${f.component ?? ''}`, severity: f.severity, text: healthText(f), to: healthTarget(f, devMode) }));
  }
  const rows: HealthRow[] = [];
  for (const f of items) {
    const g = healthGroup(f.id);
    if (!rows.some((r) => r.key === g)) rows.push({ key: g, severity: f.severity, text: g, to: healthTarget(f, devMode) });
  }
  return rows;
}
