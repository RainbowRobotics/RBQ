import { Platform } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import Constants from 'expo-constants';
import { SB_URL, SB_KEY } from '@/lib/logUploadCommon';

const TOKEN_KEY = 'push.token.v1';
const SECRET_KEY = 'push.secret.v1';
export const ANDROID_CHANNEL = 'rbq-alerts';

const isMobile = Platform.OS === 'android' || Platform.OS === 'ios';

let wantPush = false;

function notifications(): typeof import('expo-notifications') | null {
  if (!isMobile) return null;
  try { return require('expo-notifications'); } catch { return null; }
}

export async function initPush(): Promise<void> {
  const N = notifications();
  if (!N) return;
  N.setNotificationHandler({
    handleNotification: async () => ({
      shouldShowBanner: true, shouldShowList: true, shouldPlaySound: true, shouldSetBadge: false,
    }),
  });
  if (Platform.OS === 'android') {
    await N.setNotificationChannelAsync(ANDROID_CHANNEL, {
      name: '로봇 알림', importance: N.AndroidImportance.HIGH, sound: 'default',
    }).catch(() => {});
  }
  N.addPushTokenListener((t) => {
    if (!wantPush) return;
    void rotateIfChanged(typeof t.data === 'string' ? t.data : '');
  });
}

export async function syncPush(enabled: boolean, hasSession: boolean): Promise<void> {
  const N = notifications();
  if (!N) return;
  wantPush = enabled && hasSession;
  if (!wantPush) { await unregisterPush(); return; }
  try {
    const perm = await N.getPermissionsAsync();
    if (!perm.granted && perm.canAskAgain !== false) await N.requestPermissionsAsync();
    if ((await N.getPermissionsAsync()).granted) {
      const { data } = await N.getDevicePushTokenAsync();
      if (typeof data === 'string') await rotateIfChanged(data);
    }
  } catch (e) { console.warn('[push] sync', e); }
}

async function rotateIfChanged(next: string): Promise<void> {
  try {
    if (!next || next.length < 20) return;
    const [prev, secret] = await Promise.all([
      AsyncStorage.getItem(TOKEN_KEY), AsyncStorage.getItem(SECRET_KEY),
    ]);
    if (!prev || prev === next) return;
    const s = await rpc<string | null>('rotate_push_token', {
      p_old: prev, p_new: next, p_secret: secret, p_platform: Platform.OS, p_channel: channel(),
    });
    if (!s) { console.warn('[push] rotate 실패 — 다음 로그인에 재등록'); return; }
    await AsyncStorage.multiSet([[TOKEN_KEY, next], [SECRET_KEY, s]]);
    console.log(`[push] rotate ok …${next.slice(-8)}`);
  } catch (e) { console.warn('[push] rotate', e); }
}

async function rpc<T = unknown>(fn: string, body: Record<string, unknown>): Promise<T | undefined> {
  if (!SB_URL || !SB_KEY) return undefined;
  const res = await fetch(`${SB_URL}/rest/v1/rpc/${fn}`, {
    method: 'POST',
    headers: { apikey: SB_KEY, Authorization: `Bearer ${SB_KEY}`, 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) return undefined;
  try { return (await res.json()) as T; } catch { return undefined; }
}

function channel(): 'release' | 'nightly' {
  const pkg = Constants.expoConfig?.android?.package ?? '';
  const ch = (Constants.expoConfig?.extra as { channel?: string } | undefined)?.channel;
  return pkg.endsWith('.nightly') || ch === 'nightly' ? 'nightly' : 'release';
}

export async function registerPush(pin: string, enabled: boolean): Promise<boolean> {
  const N = notifications();
  if (!N || !pin || !enabled) return false;
  try {
    const perm = await N.getPermissionsAsync();
    const granted = perm.granted || (await N.requestPermissionsAsync()).granted;
    if (!granted) return false;
    const { data: token } = await N.getDevicePushTokenAsync();
    if (typeof token !== 'string' || token.length < 20) { console.warn('[push] token 없음', token); return false; }
    const secret = await rpc<string | null>('register_push_token', {
      p_pin: pin, p_token: token, p_platform: Platform.OS, p_channel: channel(),
    });
    console.log(`[push] register ${secret ? 'ok' : 'FAIL'} ${Platform.OS}/${channel()} …${token.slice(-8)}`);
    if (!secret) return false;
    await AsyncStorage.multiSet([[TOKEN_KEY, token], [SECRET_KEY, secret]]);
    wantPush = true;
    return true;
  } catch (e) {
    console.warn('[push] register', e);
    return false;
  }
}

export async function unregisterPush(): Promise<void> {
  if (!isMobile) return;
  try {
    const [token, secret] = await Promise.all([
      AsyncStorage.getItem(TOKEN_KEY), AsyncStorage.getItem(SECRET_KEY),
    ]);
    if (!token) return;
    const gone = await rpc<boolean | null>('unregister_push_token', { p_token: token, p_secret: secret });
    if (gone !== undefined && gone !== null) {
      wantPush = false;
      await AsyncStorage.multiRemove([TOKEN_KEY, SECRET_KEY]);
    }
  } catch { }
}
