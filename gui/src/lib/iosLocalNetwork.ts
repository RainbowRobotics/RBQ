import { Platform } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { t } from '@/lib/i18n';

const SEEN_KEY = 'rbq-ios-local-network-seen';
const RETRY_MS = 1_500;
const FIRST_USE_WAIT_MS = 20_000;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export async function probeThroughLocalNetworkPrompt<T>(probe: () => Promise<T | null>): Promise<T | null> {
  if (Platform.OS !== 'ios') return probe();
  let r = await probe();
  if (r === null && !(await seen())) {
    const until = Date.now() + FIRST_USE_WAIT_MS;
    while (r === null && Date.now() < until) {
      await sleep(RETRY_MS);
      r = await probe();
    }
  }
  if (r !== null) await AsyncStorage.setItem(SEEN_KEY, '1').catch(() => {});
  return r;
}

async function seen(): Promise<boolean> {
  try { return (await AsyncStorage.getItem(SEEN_KEY)) === '1'; } catch { return false; }
}

export const localNetworkHint = (): string => (Platform.OS === 'ios'
  ? ` ${t('iOS 는 설정 › 개인정보 보호 및 보안 › 로컬 네트워크 에서 RBQ 를 켜야 로봇에 닿습니다.')}`
  : '');
