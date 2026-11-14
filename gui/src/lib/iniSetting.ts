import { robotAuth } from './auth';
import { restBase, streamerBase } from './endpoints';
import { actions, robotFetch } from './rest';
import { isDemo } from './demoFlag';
import { resolveEndpoints } from './resolveEndpoints';
import { useRobot } from '@/store/robot';
import { t } from './i18n';

export type IniTarget = 'motion' | 'hal' | 'handler' | 'streamer';
export type IniKV = { key: string; value: string; source?: 'ini' | 'profile'; profile?: string };
export type IniSection = { name: string; keys: IniKV[] };
export type IniFile = {
  sections: IniSection[];
  path: string;
  timestamp: string;
};
export type IniApply = { label: string; run: (ip: string) => Promise<unknown>; service?: boolean };
export type IniSource = {
  key: string; label: string; apply: IniApply; liveSections?: string[]; cannotApply?: () => string | null;
  get: (ip: string) => Promise<IniFile>;
  put: (ip: string, sections: IniSection[]) => Promise<IniFile>;
};

export const SERVICE_RESTART: IniApply = { label: '로봇 서비스 재시작', run: (ip) => actions.restart(ip), service: true };
const visionRestart = (label: string, id: number): IniApply => ({ label, run: (ip) => actions.visionProgram(ip, id, false) });
const twoPc = () => { const { ip, visionIp } = useRobot.getState(); const e = resolveEndpoints(ip, visionIp); return e.vision !== e.motion; };

export async function iniFetch(url: string, init?: RequestInit): Promise<IniFile> {
  if (isDemo()) {
    return {
      sections: [{ name: 'DEMO', keys: [{ key: 'demo_mode', value: 'true' }, { key: 'note', value: 'canned — real INI untouched' }] }],
      path: '/demo/' + url.replace(/^.*\/api\//, ''), timestamp: new Date().toISOString(),
    };
  }
  const res = await robotFetch(url, { ...init, headers: { ...robotAuth(), ...(init?.headers ?? {}) } });
  if (!res.ok) throw new Error(`${url.replace(/^https?:\/\/[^/]+/, '')} → HTTP ${res.status}`);
  const o: any = await res.json();
  return {
    sections: o?.ini?.sections ?? [],
    path: o?.path ?? '',
    timestamp: o?.timestamp ?? '',
  };
}

const base = (ip: string, t: IniTarget) => (t === 'motion' ? restBase(ip) : streamerBase(ip));
const fixed = (key: IniTarget, label: string, apply: IniApply, more?: Pick<IniSource, 'liveSections' | 'cannotApply'>): IniSource => ({
  key, label, apply, ...more,
  get: (ip) => iniFetch(`${base(ip, key)}/api/${key}/setting`),
  put: (ip, sections) => iniFetch(`${base(ip, key)}/api/${key}/setting`, {
    method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ sections }),
  }),
});

export const INI_SOURCES: IniSource[] = [
  fixed('motion', 'Motion.ini', SERVICE_RESTART),
  fixed('hal', 'HAL · config.ini', visionRestart('HAL 재시작', 1)),
  fixed('handler', 'Handler · config.ini', SERVICE_RESTART, {
    cannotApply: () => (twoPc() ? t('비전 PC 가 따로 있는 로봇이라 앱에서 Vision 을 다시 시작할 수 없습니다 — 비전 PC 를 다시 시작하면 반영됩니다') : null),
  }),
  fixed('streamer', 'Streamer.ini', visionRestart('Streamer 재시작', 2), { liveSections: ['BlackBox'] }),
];

export const isBoolValue = (v: string) => /^(true|false)$/i.test(v);

export function toggleBoolValue(v: string): string {
  const next = /^t/i.test(v) ? 'false' : 'true';
  if (v === v.toUpperCase()) return next.toUpperCase();
  if (v[0] === v[0].toUpperCase()) return next[0].toUpperCase() + next.slice(1);
  return next;
}
