
import { connectAll, disconnectAll as disconnectTransports } from './transports';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import { settingsFor, directTarget, type Target } from './connectTarget';
import { isDemo } from './demoFlag';
import { restoreAddr } from './restoreAddr';

let current: Target | null = null;
export const currentTarget = (): Target | null => current;

export function disconnectAll(): void {
  current = null;
  try { disconnectTransports(); } catch { }
}

export function connectNow(target: Target): void {
  disconnectAll();

  const s = useSettings.getState();
  const p = settingsFor(target);
  s.setConnProfile(p.connProfile);
  if (p.lanIp !== undefined) s.setLanIp(p.lanIp);
  if (p.wanIp !== undefined) s.setWanIp(p.wanIp);
  s.setRendezvousUrl(p.rendezvousUrl);
  s.setRobotId(p.robotId);
  s.setWebrtcToken(p.webrtcToken);
  if (target.route === 'rendezvous') useRobot.getState().setVisionIp('');

  current = target;
  useRobot.getState().setConnError(null);
  connectAll(target.addr, useRobot.getState().visionIp);
}

export function restoreLastTarget(): boolean {
  if (isDemo()) { const d = directTarget('demo'); if (d.ok) connectNow(d.target); return true; }
  const s = useSettings.getState();
  const addr = restoreAddr(s.connProfile, s.lanIp, s.wanIp, useRobot.getState().ip);
  if (!addr) return false;
  if (s.connProfile === 'wan' && (s.rendezvousUrl ?? '').trim() && (s.robotId ?? '').trim()) return false;
  const r = directTarget(addr, { token: s.webrtcToken });
  if (!r.ok) return false;
  connectNow(r.target);
  return true;
}

export function reconnectCurrent(): boolean {
  if (current) { connectNow(current); return true; }
  return restoreLastTarget();
}
