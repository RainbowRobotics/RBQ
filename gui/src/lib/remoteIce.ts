import { useSettings } from '@/store/settings';
import { remote, remoteActive } from '@/modules/registry';

export const isRemoteProfile = () => useSettings.getState().connProfile === 'wan';

export const REMOTE_STUN = 'stun:stun.cloudflare.com:3478';

export const routeKey = (): string => (remoteActive() ? remote.net!.key() : 'direct');

export interface IceServer { urls: string; username?: string; credential?: string }

export async function resolveIceServers(): Promise<IceServer[]> {
  if (!isRemoteProfile()) return [];
  return remote.net ? remote.net.iceServers() : [{ urls: REMOTE_STUN }];
}

export const modeField = (): { mode?: 'remote' } => (isRemoteProfile() ? { mode: 'remote' } : {});
