import AsyncStorage from '@react-native-async-storage/async-storage';
import { useSettings } from '@/store/settings';
import { useAccount } from '@/store/account';
import { isExpired, type RemoteAccount, type RemoteLoginCache } from '@/lib/remoteLogin';
import { registerPush, unregisterPush } from '@/lib/push';
import { setTicketSession, clearTicketSession } from '@/lib/connectTicket';
import { disconnectAll, currentTarget } from '@/lib/connectNow';

export const ACCOUNT_CACHE_KEY = 'rbq.remoteLogin.cache';

function revokeAccountLevel() {
  const g = useSettings.getState();
  if (g.accountLevel === null) return;
  g.setAccountLevel(null);
  g.setAccessLevel(1);
}

export async function restoreAccountSession(now = Date.now()): Promise<void> {
  try {
    await restoreInner(now);
  } finally {
    useAccount.getState().markRestored();
  }
}

async function restoreInner(now: number): Promise<void> {
  let raw: string | null = null;
  try { raw = await AsyncStorage.getItem(ACCOUNT_CACHE_KEY); } catch { raw = null; }
  if (!raw) { revokeAccountLevel(); return; }
  try {
    const cached = JSON.parse(raw) as RemoteLoginCache;
    const a = cached.account;
    if (isExpired(a, now)) {
      AsyncStorage.removeItem(ACCOUNT_CACHE_KEY).catch(() => {});
      revokeAccountLevel();
      return;
    }
    const list = cached.accounts?.length ? cached.accounts : [a];
    const pin = cached.pin ?? '';
    useAccount.getState().setSession(pin, a, list);
    if (pin) setTicketSession(pin, list);
    const g = useSettings.getState();
    g.setAccountLevel(a.level); g.setAccessLevel(a.level);
  } catch {
    AsyncStorage.removeItem(ACCOUNT_CACHE_KEY).catch(() => {});
    revokeAccountLevel();
  }
}

export function applyLogin(pinV: string, account: RemoteAccount, list: RemoteAccount[], first: boolean): void {
  const st = useAccount.getState();
  if (first && st.account && !st.pin) { st.refresh(account, list); st.setPin(pinV); }
  else if (first) st.setSession(pinV, account, list);
  else st.refresh(account, list);
  setTicketSession(pinV, list);
  const g = useSettings.getState();
  g.setAccountLevel(account.level); g.setAccessLevel(account.level);
  registerPush(pinV, g.pushAlerts).catch(() => {});
}

export function logoutAccount(): void {
  unregisterPush().catch(() => {});
  useAccount.getState().clear();
  clearTicketSession();
  AsyncStorage.removeItem(ACCOUNT_CACHE_KEY).catch(() => {});
  const g = useSettings.getState(); g.setAccountLevel(null); g.setAccessLevel(1);
  if (currentTarget()?.route === 'rendezvous') disconnectAll();
  g.setRendezvousUrl(''); g.setRobotId('');
  if (g.connProfile === 'wan') g.setConnProfile('lan');
}
