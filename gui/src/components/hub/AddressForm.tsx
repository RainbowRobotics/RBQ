import { useState } from 'react';
import { ActivityIndicator } from 'react-native';
import { probeAddress, ROBOT_LAN_IP } from '@/lib/robotScan';
import { View, Text, TextInput, StyleSheet, Platform } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { directTarget } from '@/lib/connectTarget';
import { connectNow } from '@/lib/connectNow';
import { isDesktop } from '@/lib/desktopBridge';
import { syncProxyTarget } from '@/lib/proxyTarget';
import { useSteamOS } from '@/lib/platformInfo';
import { useSettings, useDevMode } from '@/store/settings';
import { useRobots } from '@/store/robots';
import { t } from '@/lib/i18n';

export function AddressForm({ onDone }: { onDone: () => void }) {
  const { c, fonts, radius } = useTheme();
  const steamOS = useSteamOS();
  const [addr, setAddr] = useState<string>(ROBOT_LAN_IP);
  const [mode, setMode] = useState<'lan' | 'local' | 'wan'>('lan');
  const wan = mode === 'wan';
  const [token, setToken] = useState(useSettings.getState().webrtcToken || '');
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const dev = useDevMode();
  const [visionOpen, setVisionOpen] = useState(false);
  const [vision, setVision] = useState('');
  const localOk = (Platform.OS === 'web' || isDesktop()) && !steamOS;
  const go = async () => {
    const r = directTarget(addr, { token: wan ? token : undefined });
    if (!r.ok) { setErr(t('접속 주소를 입력해 주세요')); return; }
    const g = useSettings.getState();
    if (!wan) {
      setErr(null); setBusy(true);
      const found = await probeAddress(addr.trim(), Platform.OS === 'web');
      setBusy(false);
      if (!found) { setErr(t('이 주소에서 로봇이 응답하지 않습니다 — IP 와 로봇 전원을 확인하세요')); return; }
      g.setLanIp(addr.trim()); g.setConnProfile('lan');
      await useRobots.getState().addAddress(addr.trim(), found.serial, dev ? vision : undefined);
      onDone();
      return;
    }
    g.setWanIp(addr.trim()); g.setWebrtcToken(token.trim()); g.setConnProfile('wan');
    const ip = addr.trim();
    const pend = { serial: `pending:${ip}`, name: t('내 로봇'), lastSeenAt: Date.now(), wan: { ip, token: token.trim() || undefined }, lastVia: 'wan' as const };
    useRobots.setState((s) => ({ local: [pend, ...s.local.filter((p) => p.serial !== pend.serial)], currentSerial: pend.serial }));
    connectNow(r.target);
    onDone();
  };
  const pureWeb = Platform.OS === 'web' && !isDesktop();
  const goWeb = () => {
    if (busy) return;
    if (wan || !pureWeb) { void go(); return; }
    if (!addr.trim()) { setErr(t('접속 주소를 입력해 주세요')); return; }
    void syncProxyTarget(addr.trim(), '').then((res) => {
      if (res === 'failed') { setErr(t('프록시가 대상을 바꾸지 못했습니다 — 이 PC 에서 localhost 로 열었는지 확인하세요')); return; }
      void go();
    });
  };
  const field = (label: string, v: string, set: (s: string) => void, secure = false, ph = secure ? '' : '192.168.0.10') => (
    <View style={{ gap: 6 }}>
      <Text style={{ color: c.muted, fontSize: 11 }}>{label}</Text>
      <TextInput value={v} onChangeText={set} onSubmitEditing={goWeb} autoCapitalize="none" autoCorrect={false} secureTextEntry={secure}
        keyboardType={secure ? 'default' : 'numbers-and-punctuation'} placeholder={ph} placeholderTextColor={c.dim}
        style={{ color: c.text, fontFamily: fonts.mono, fontSize: 14, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, paddingHorizontal: 10, paddingVertical: 8, backgroundColor: c.bg }} />
    </View>
  );
  return (
    <View style={{ gap: 12 }}>
      <View style={styles.seg}>
        {([
          ['lan', t('로봇망')],
          ...(localOk ? [['local', t('로컬')] as const] : []),
          ['wan', 'WAN'],
        ] as const).map(([k, label]) => {
          const on = mode === k;
          return (
            <Tappable key={k} onPress={() => {
              setMode(k); setErr(null);
              if (k === 'lan') setAddr(ROBOT_LAN_IP);
              else if (k === 'local') setAddr('127.0.0.1');
              else setAddr(useSettings.getState().wanIp || '');
            }}
              style={[styles.segBtn, { borderRadius: radius.sm, backgroundColor: on ? 'rgba(77,156,245,0.16)' : 'transparent', borderColor: on ? 'rgba(77,156,245,0.5)' : c.line }]}>
              <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 12.5, fontWeight: '700' }}>{label}</Text>
            </Tappable>
          );
        })}
      </View>
      {field(wan ? t('공인 IP 또는 도메인') : t('로봇 IP'), addr, setAddr)}
      {wan && field(t('WebRTC 토큰 (로봇에 설정한 경우만)'), token, setToken, true)}
      {!wan && dev && (
        <View style={{ gap: 8 }}>
          <Tappable onPress={() => setVisionOpen((v) => !v)} accessibilityLabel={t('Vision PC IP')} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
            <Icon name={visionOpen ? 'caret' : 'chevR'} size={13} color={c.muted} />
            <Text style={{ color: c.muted, fontSize: 12, fontWeight: '600' }}>{t('Vision PC IP (선택)')}</Text>
            {!visionOpen && vision.trim() ? <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono }}>{vision.trim()}</Text> : null}
          </Tappable>
          {visionOpen && field(t('비우면 로봇 IP 사용 — Vision PC 분리 구성 전용'), vision, setVision, false, '')}
        </View>
      )}
      <Text style={{ color: c.dim, fontSize: 11, lineHeight: 15 }}>
        {wan ? t('로봇의 공인 주소로 바로 붙습니다. 8080·8081 포트 개방이 필요합니다.')
          : mode === 'local' ? t('이 PC 에서 도는 시뮬레이터에 붙습니다(127.0.0.1). 시뮬을 먼저 띄워 두세요.')
          : t('로봇이 응답하면 시리얼로 목록에 저장하고 연결합니다. 같은 로봇은 한 줄로 합쳐집니다.')}
      </Text>
      {err && <Text style={{ color: c.redTx, fontSize: 12 }}>{err}</Text>}
      <Tappable onPress={goWeb} accessibilityLabel={t('연결')} style={[styles.go, { backgroundColor: c.accent, borderRadius: radius.md }]}>
        {busy ? <ActivityIndicator size="small" color={c.onAccent} /> : <Icon name="wifi" size={16} color={c.onAccent} />}
        <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '800' }}>{busy ? t('로봇 확인 중…') : t('연결')}</Text>
      </Tappable>
    </View>
  );
}
const styles = StyleSheet.create({
  seg: { flexDirection: 'row', gap: 8, alignItems: 'center' },
  segBtn: { paddingHorizontal: 12, paddingVertical: 6, borderWidth: 1 },
  go: { height: 44, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 8 },
});
