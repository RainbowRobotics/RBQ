import { Platform } from 'react-native';
import { demoMode, type DemoDriver } from '@/modules/registry';

const KEY = 'rbq-demo';

export function isDemo(): boolean {
  if (!demoMode.driver || Platform.OS !== 'web' || typeof location === 'undefined') return false;
  try {
    const q = new URLSearchParams(location.search).get('demo');
    if (q === '1') sessionStorage.setItem(KEY, '1');
    else if (q === '0') sessionStorage.removeItem(KEY);
    return sessionStorage.getItem(KEY) === '1';
  } catch { return false; }
}

export const demoDriver = (): DemoDriver | undefined => (isDemo() ? demoMode.driver : undefined);

export function exitDemo() {
  try { sessionStorage.removeItem(KEY); } catch {}
  location.href = location.pathname + '?demo=0';
}

let demoBaseLevel: 1 | 2 | 3 | 4 | null = null;
export const setDemoBaseLevel = (v: 1 | 2 | 3 | 4) => { demoBaseLevel = v; };
export const getDemoBaseLevel = () => demoBaseLevel;

let demoBaseRobot: { ip: string; visionIp: string } | null = null;
export const setDemoBaseRobot = (v: { ip: string; visionIp: string }) => { demoBaseRobot = v; };
export const getDemoBaseRobot = () => demoBaseRobot;
