import { useState } from 'react';
import { View, Text, Image, StyleSheet } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { SafetyCorner } from '@/components/hub/HubHeader';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { loginWithPin, type RemoteLoginCache } from '@/lib/remoteLogin';
import { SB_URL, SB_KEY } from '@/lib/logUploadCommon';
import { ACCOUNT_CACHE_KEY, applyLogin } from '@/lib/accountSession';
import { useAccount } from '@/store/account';
import { useCompactH } from '@/lib/layout';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

const KEYS = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '⌫', '0', ''] as const;

export default function Pin() {
  const { c, fonts, radius } = useTheme();
  const compactH = useCompactH();
  const [pin, setPin] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const size = compactH ? 56 : 68;
  const acct = useAccount((s) => s.account);

  const submit = async (code: string) => {
    setBusy(true); setErr(null);
    let cache: RemoteLoginCache | null = null;
    try { const raw = await AsyncStorage.getItem(ACCOUNT_CACHE_KEY); if (raw) cache = JSON.parse(raw); } catch { }
    const r = await loginWithPin(code, { url: SB_URL, anonKey: SB_KEY, cache: acct ? null : cache });
    if (!r.ok) {
      setBusy(false); setPin('');
      setErr(r.reason === 'bad_pin' ? t('코드가 맞지 않습니다')
        : r.reason === 'offline' ? t('서버에 닿지 않습니다 — 인터넷을 확인하세요')
        : r.reason === 'throttled' ? t('코드를 여러 번 틀려 잠시 막혔습니다 — 10분 뒤 다시 시도하세요')
        : t('서버 오류 — 잠시 후 다시 시도하세요'));
      return;
    }
    applyLogin(code, r.account, r.accounts, true);
    AsyncStorage.setItem(ACCOUNT_CACHE_KEY, JSON.stringify({ account: r.account, accounts: r.accounts, at: Date.now(), pin: code } satisfies RemoteLoginCache)).catch(() => {});
    goTop('/hub');
  };
  const press = (k: string) => {
    if (busy) return;
    if (k === '⌫') { setPin((p) => p.slice(0, -1)); return; }
    const next = pin + k;
    setPin(next);
    if (next.length === 4) submit(next);
  };
  const skip = () => goTop('/hub');

  return (
    <Screen>
      <View style={styles.root}>
        <View style={styles.left}>
          <Image source={require('@/assets/images/rb-logo.png')} style={{ width: 44, height: 44, borderRadius: 12 }} />
          <Text style={{ color: c.text, fontSize: 24, fontWeight: '800', marginTop: 14 }}>{t('접근 코드')}</Text>
          <Text style={{ color: c.muted, fontSize: 13, marginTop: 6 }}>{acct ? `${acct.accountName} · ${t('원격 연결을 위해 코드를 다시 입력하세요')}` : t('대시보드에서 받은 4자리를 입력하세요')}</Text>
          <View style={styles.dots}>
            {[0, 1, 2, 3].map((i) => (
              <View key={i} style={[styles.dot, { borderColor: c.line, backgroundColor: i < pin.length ? c.accent : 'transparent' }]} />
            ))}
          </View>
          <Text style={{ color: c.redTx, fontSize: 12, minHeight: 18, fontFamily: fonts.mono }}>{err ?? (busy ? t('확인 중…') : '')}</Text>
          <Tappable onPress={skip} disabled={busy} style={[styles.skip, { borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('건너뛰기')}</Text>
            <Icon name="next" size={16} color={c.muted} />
          </Tappable>
          <Text style={{ color: c.dim, fontSize: 11, marginTop: 6 }}>{t('코드 없이 로봇망 직결로 씁니다 · 코드는 나중에 계정에서')}</Text>
        </View>
        <View style={[styles.pad, { width: size * 3 + 24 }]}>
          {KEYS.map((k, i) => k === '' ? <View key={i} style={{ width: size, height: size }} /> : (
            <Tappable key={k} onPress={() => press(k)} disabled={busy} accessibilityLabel={k === '⌫' ? t('지우기') : k}
              style={[styles.key, { width: size, height: size, borderRadius: size / 2, backgroundColor: c.elev, borderColor: c.line, opacity: busy ? 0.5 : 1 }]}>
              <Text style={{ color: c.text, fontSize: k === '⌫' ? 20 : 26, fontWeight: '600' }}>{k}</Text>
            </Tappable>
          ))}
        </View>
      </View>
      <SafetyCorner />
    </Screen>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 48, paddingHorizontal: 24 },
  left: { maxWidth: 320, alignItems: 'flex-start' },
  dots: { flexDirection: 'row', gap: 14, marginTop: 22, marginBottom: 8 },
  dot: { width: 16, height: 16, borderRadius: 8, borderWidth: 2 },
  skip: { flexDirection: 'row', alignItems: 'center', gap: 6, borderWidth: 1, paddingHorizontal: 16, paddingVertical: 10, marginTop: 14 },
  pad: { flexDirection: 'row', flexWrap: 'wrap', gap: 12, justifyContent: 'center' },
  key: { alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
});
