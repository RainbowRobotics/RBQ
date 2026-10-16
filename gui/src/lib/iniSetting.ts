import { robotAuth } from './auth';
import { restBase, streamerBase } from './endpoints';
import { isDemo } from './demoFlag';

export type IniTarget = 'motion' | 'hal' | 'handler' | 'streamer';
export type IniKV = { key: string; value: string };
export type IniSection = { name: string; keys: IniKV[] };
export type IniFile = {
  sections: IniSection[];
  path: string;
  timestamp: string;
};

export const INI_TARGETS: { key: IniTarget; label: string; daemon: string }[] = [
  { key: 'motion', label: 'Motion.ini', daemon: 'Motion' },
  { key: 'hal', label: 'HAL · config.ini', daemon: 'HAL' },
  { key: 'handler', label: 'Handler · config.ini', daemon: 'Handler' },
  { key: 'streamer', label: 'Streamer.ini', daemon: 'Streamer' },
];

const base = (ip: string, t: IniTarget) => (t === 'motion' ? restBase(ip) : streamerBase(ip));

async function req(ip: string, t: IniTarget, init?: RequestInit): Promise<IniFile> {
  if (isDemo()) {
    return {
      sections: [{ name: 'DEMO', keys: [{ key: 'demo_mode', value: 'true' }, { key: 'note', value: 'canned — real INI untouched' }] }],
      path: `/demo/${t}.ini`, timestamp: new Date().toISOString(),
    };
  }
  const res = await fetch(`${base(ip, t)}/api/${t}/setting`, {
    ...init,
    headers: { ...robotAuth(), ...(init?.headers ?? {}) },
  });
  if (!res.ok) throw new Error(`${t}/setting → HTTP ${res.status}`);
  const o: any = await res.json();
  return {
    sections: o?.ini?.sections ?? [],
    path: o?.path ?? '',
    timestamp: o?.timestamp ?? '',
  };
}

export const getIniSetting = (ip: string, t: IniTarget) => req(ip, t);

export const putIniSetting = (ip: string, t: IniTarget, sections: IniSection[]) =>
  req(ip, t, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sections }),
  });

export const isBoolValue = (v: string) => /^(true|false)$/i.test(v);

export function toggleBoolValue(v: string): string {
  const next = /^t/i.test(v) ? 'false' : 'true';
  if (v === v.toUpperCase()) return next.toUpperCase();
  if (v[0] === v[0].toUpperCase()) return next[0].toUpperCase() + next.slice(1);
  return next;
}
