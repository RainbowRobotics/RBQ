
import { connectAll, disconnectAll as disconnectTransports } from './transports';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import { settingsFor, directTarget, type Target } from './connectTarget';
import { isDemo } from './demoFlag';
import { remote, remoteActive } from '@/modules/registry';
import { restoreAddr } from './restoreAddr';

let current: Target | null = null;
let dropped = false;
export const currentTarget = (): Target | null => current;

export function disconnectAll(): void {
  current = null;
  dropped = true;
  try { disconnectTransports(); } catch { }
}

export function connectNow(target: Target): void {
  disconnectAll();

  const s = useSettings.getState();
  const p = settingsFor(target);
  s.setConnProfile(p.connProfile);
  if (p.lanIp !== undefined) s.setLanIp(p.lanIp);
  if (p.wanIp !== undefined) s.setWanIp(p.wanIp);
  remote.robots?.apply(target);
  s.setWebrtcToken(p.webrtcToken);
  if (target.route === 'remote') useRobot.getState().setVisionIp('');

  current = target;
  dropped = false;
  useRobot.getState().setConnError(null);
  connectAll(target.addr, useRobot.getState().visionIp);
}

export function restoreLastTarget(): boolean {
  if (isDemo()) { const d = directTarget('demo'); if (d.ok) connectNow(d.target); return true; }
  const s = useSettings.getState();
  const addr = restoreAddr(s.connProfile, s.lanIp, s.wanIp, useRobot.getState().ip);
  if (!addr) return false;
  if (remoteActive()) return false;
  const r = directTarget(addr, { token: s.webrtcToken });
  if (!r.ok) return false;
  connectNow(r.target);
  return true;
}

export function reconnectCurrent(): boolean {
  if (current) { connectNow(current); return true; }
  if (dropped) return false;
  return restoreLastTarget();
}
