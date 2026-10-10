import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Platform, ActivityIndicator } from 'react-native';
import { RobotModel3D } from '@/components/RobotModel3D';
import { STANDING_RAD } from '@/lib/robotPose';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Logo } from '@/components/TopBar';
import { SafetyCorner } from '@/components/hub/HubHeader';
import { AddressForm } from '@/components/hub/AddressForm';
import { probeRobotLan, ROBOT_LAN_IP } from '@/lib/robotScan';
import { currentSsid } from '@/lib/currentSsid';
import { restBase } from '@/lib/endpoints';
import { isDemo } from '@/lib/demoFlag';
import { useMyNetwork, openWifiPanel } from '@/components/WifiStatus';
import { WifiPicker } from '@/components/WifiPicker';
import { isDesktop } from '@/lib/desktopBridge';
import { useDevMode } from '@/store/settings';
import { noSerialKey, useRobots, useProfiles } from '@/store/robots';
import { useAccount } from '@/store/account';
import { useRobot } from '@/store/robot';
import { useCompactH } from '@/lib/layout';
import { t } from '@/lib/i18n';
import { cloudConfigured } from '@/lib/logUploadCommon';
import { goTop } from '@/lib/nav';

const HERO_POSE = { joints: STANDING_RAD, rpy: [0, 0, 0] as [number, number, number] };

export default function AddRobot() {
  const { c, fonts, radius } = useTheme();
  const [heroReady, setHeroReady] = useState(false);
  useEffect(() => { const t = setTimeout(() => setHeroReady(true), 600); return () => clearTimeout(t); }, []);
  const router = useRouter();
  const compactH = useCompactH();
  const profiles = useProfiles();
  const authed = !!useAccount((s) => s.account);
  const dev = useDevMode();
  const net = useMyNetwork();
  const canProbe = !isDemo();
  const [probe, setProbe] = useState<null | 'busy' | 'none' | 'same'>(null);
  const [wifiPick, setWifiPick] = useState(false);
  const runProbe = async () => {
    setProbe('busy');
    const found = await probeRobotLan({ base: restBase(ROBOT_LAN_IP) });
    const rb = useRobots.getState();
    const key = found === null ? null : found || noSerialKey(ROBOT_LAN_IP, await currentSsid());
    if (key && useRobot.getState().conn === 'connected' && key === rb.currentSerial) { setProbe('same'); return; }
    setProbe(key ? null : 'none');
    if (found !== null) { await (found ? rb.addLan(found) : rb.addAddress(ROBOT_LAN_IP, '')); goTop('/hub'); }
  };
  useEffect(() => { if (canProbe && useRobot.getState().conn !== 'connected') runProbe(); }, []); // eslint-disable-line react-hooks/exhaustive-deps
  const done = () => goTop('/hub');
  const card = (icon: 'wifi' | 'plus' | 'tag', title: string, sub: string, body: React.ReactNode) => (
    <View key={title} style={[styles.card, { backgroundColor: c.glassHi, borderColor: c.glassLine, borderRadius: radius.lg }]}>
      <View style={styles.cardHead}>
        <View style={[styles.cardIcon, { backgroundColor: c.elev, borderRadius: radius.sm }]}><Icon name={icon} size={18} color={c.accent2} /></View>
        <View style={{ flex: 1 }}>
          <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{title}</Text>
          <Text style={{ color: c.muted, fontSize: 11.5 }}>{sub}</Text>
        </View>
      </View>
      {body}
    </View>
  );
  return (
    <Screen>
      {!compactH && (
        <View style={[StyleSheet.absoluteFill, { left: '-34%', top: '4%' }]} pointerEvents="none">
          {heroReady && <RobotModel3D pose={HERO_POSE} controls={false} listenPresets={false} showPresetRow={false} gridVisible={false}
            backgroundColor={c.bg} idleSpin groundShadow initialOrbit={{ yaw: 2.3, pitch: 0.25 }} initialDist={1.45} />}
        </View>
      )}
      <View style={[styles.top, compactH && { paddingTop: 8 }]}>
        <Logo />
        <Text style={{ color: c.text, fontSize: compactH ? 20 : 26, fontWeight: '800' }}>{profiles.length === 0 ? t('로봇을 추가하세요') : t('로봇 추가')}</Text>
        <Text style={{ color: c.muted, fontSize: 12.5, maxWidth: 360 }}>{profiles.length === 0 ? t('로봇에 붙는 방법을 고르세요.') : t('붙으면 시리얼로 저장됩니다. 같은 로봇은 한 줄로 합쳐집니다.')}</Text>
      </View>
      <SafetyCorner />
      <ScrollView style={compactH ? styles.colCompact : styles.col}
        contentContainerStyle={styles.cards} showsVerticalScrollIndicator={false}>
        {card('wifi', t('로봇 WiFi'), net.ssid ? `${t('현재')}: ${net.ssid}` : t('로봇의 WiFi(AP)에 붙으면 자동으로 찾습니다'),
          <View style={{ gap: 8 }}>
            {!canProbe ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('앱(APK·데스크탑)에서 자동으로 찾습니다.')}</Text> : (
              <>
                {probe === 'busy' && <View style={styles.busy}><ActivityIndicator size="small" color={c.accent2} /><Text style={{ color: c.muted, fontSize: 12 }}>{t('로봇을 확인하는 중…')}</Text></View>}
                {probe === 'same' && <Text style={{ color: c.muted, fontSize: 12 }}>{t('지금 연결된 로봇입니다 — 다른 로봇을 추가하려면 그 로봇의 WiFi 로 바꾼 뒤 다시 확인하세요.')}</Text>}
                {probe === 'none' && <Text style={{ color: c.amberTx, fontSize: 12 }}>{t('로봇이 응답하지 않습니다 — 로봇 전원과 이 기기의 WiFi 가 로봇 AP 인지 확인하세요.')}</Text>}
                <View style={{ flexDirection: 'row', gap: 8 }}>
                  <Tappable onPress={runProbe} disabled={probe === 'busy'} style={[styles.again, { borderColor: c.accent, borderRadius: radius.sm }]}>
                    <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{t('다시 확인')}</Text>
                  </Tappable>
                  {(Platform.OS === 'android' || isDesktop()) && (
                    <Tappable onPress={() => (isDesktop() ? setWifiPick(true) : openWifiPanel())} style={[styles.again, { borderColor: c.line, borderRadius: radius.sm }]}>
                      <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{isDesktop() ? t('WiFi 선택') : t('WiFi 설정')}</Text>
                    </Tappable>
                  )}
                </View>
              </>
            )}
          </View>)}
        {(dev || Platform.OS === 'web') && card('plus', t('주소로 연결'), t('공유기 밑 로봇 · 로컬 · 공인 IP'), <AddressForm onDone={done} />)}
        {cloudConfigured && card('tag', t('접근 코드'), authed ? t('로그인됨 — 배정된 로봇은 허브에 있습니다') : t('대시보드에서 배정받은 로봇(원격) · 처음 한 번만'),
          <View style={{ gap: 8 }}>
            <Text style={{ color: c.dim, fontSize: 11, lineHeight: 15 }}>{t('4자리 코드를 한 번 넣으면 계정에 배정된 로봇이 목록에 들어오고, 이후엔 원격(LTE)으로 바로 붙습니다.')}</Text>
            <Tappable onPress={() => router.push('/pin')} accessibilityLabel={t('접근 코드 입력')}
              style={[styles.go, { borderColor: c.accent, borderRadius: radius.md }]}>
              <Text style={{ color: c.accent2, fontSize: 14, fontWeight: '800' }}>{authed ? t('다른 코드로 로그인') : t('접근 코드 입력')}</Text>
            </Tappable>
          </View>)}
      </ScrollView>
      {wifiPick && <WifiPicker onClose={() => { setWifiPick(false); runProbe(); }} />}
    </Screen>
  );
}
const styles = StyleSheet.create({
  top: { paddingHorizontal: 20, paddingTop: 14, gap: 4 },
  col: { position: 'absolute', right: 16, top: 64, bottom: 16, width: 400 },
  colCompact: { flex: 1, width: '100%' },
  cards: { gap: 12, paddingHorizontal: 12, paddingBottom: 12 },
  card: { borderWidth: 1, padding: 14, gap: 12 },
  cardHead: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  cardIcon: { width: 34, height: 34, alignItems: 'center', justifyContent: 'center' },
  foundRow: { flexDirection: 'row', alignItems: 'center', gap: 10, borderWidth: 1, paddingHorizontal: 10, paddingVertical: 8 },
  dot: { width: 8, height: 8, borderRadius: 4 },
  busy: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 4 },
  again: { alignSelf: 'flex-start', borderWidth: 1, paddingHorizontal: 10, paddingVertical: 6 },
  go: { height: 44, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
});
