import { useSettings } from '@/store/settings';

export const isRemoteProfile = () => useSettings.getState().connProfile === 'wan';

export const REMOTE_STUN = 'stun:stun.cloudflare.com:3478';

export const rendezvousUrl = (): string => useSettings.getState().rendezvousUrl?.trim() ?? '';

export const robotId = (): string => useSettings.getState().robotId?.trim() ?? '';

export const rendezvousEnabled = (): boolean =>
  isRemoteProfile() && rendezvousUrl() !== '' && robotId() !== '';

export const routeKey = (): string => (rendezvousEnabled() ? `rv:${robotId()}` : 'direct');

function iceUrlFromRendezvous(wsUrl: string): string {
  const http = wsUrl.replace(/^ws/, 'http');
  if (!/\/ws$/.test(wsUrl))
    console.warn(`[remoteIce] 랑데부 URL이 /ws로 끝나지 않아 /ice 경로를 유도하지 못함: ${wsUrl}`);
  return http.replace(/\/ws$/, '/ice');
}

export interface IceServer { urls: string; username?: string; credential?: string }

export async function resolveIceServers(): Promise<IceServer[]> {
  if (!isRemoteProfile()) return [];
  const rv = rendezvousUrl();
  if (rv === '') return [{ urls: REMOTE_STUN }];
  const ac = new AbortController();
  const timer = setTimeout(() => ac.abort(), 4000);
  try {
    const res = await fetch(`${iceUrlFromRendezvous(rv)}?id=${encodeURIComponent(robotId())}`, {
      cache: 'no-store', signal: ac.signal,
    });
    const j = await res.json();
    const servers = Array.isArray(j?.iceServers) ? (j.iceServers as IceServer[]) : [];
    return servers.length > 0 ? servers : [{ urls: REMOTE_STUN }];
  } catch {
    return [{ urls: REMOTE_STUN }];
  } finally {
    clearTimeout(timer);
  }
}

export const modeField = (): { mode?: 'remote' } => (isRemoteProfile() ? { mode: 'remote' } : {});
