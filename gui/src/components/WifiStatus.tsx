import { useEffect, useState, useCallback } from 'react';
import { View, Text, StyleSheet, Platform, PermissionsAndroid } from 'react-native';
import NetInfo, { type NetInfoState } from '@react-native-community/netinfo';
import * as IntentLauncher from 'expo-intent-launcher';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { isDesktop } from '@/lib/desktopBridge';
import { useWifi } from '@/store/wifi';
import { WifiPicker } from '@/components/WifiPicker';
import { t } from '@/lib/i18n';

export const sameSubnet = (a?: string | null, b?: string | null) =>
  !!a && !!b && a.split('.').slice(0, 3).join('.') === b.split('.').slice(0, 3).join('.');

export async function openWifiPanel() {
  await IntentLauncher.startActivityAsync(IntentLauncher.ActivityAction.WIFI_SETTINGS).catch(() => {});
}

function Bars({ strength, color }: { strength: number | null; color: string }) {
  const { c } = useTheme();
  const lit = strength == null ? 0 : Math.max(1, Math.ceil((strength / 100) * 4));
  return (
    <View style={{ flexDirection: 'row', alignItems: 'flex-end', gap: 2, height: 14 }}>
      {[5, 8, 11, 14].map((h, i) => (
        <View key={h} style={{ width: 3.5, height: h, borderRadius: 1.5, backgroundColor: i < lit ? color : c.elev2 }} />
      ))}
    </View>
  );
}

export function WifiStatus({ embedded, onGoConn }: BlockProps = {}) {
  if (isDesktop()) return <DesktopWifiBlock embedded={embedded} onGoConn={onGoConn} />;
  if (Platform.OS === 'android') return <AndroidWifiBlock embedded={embedded} onGoConn={onGoConn} />;
  return null;
}

export function useMyNetwork(): { ssid: string | null; ip: string | null; supported: boolean } {
  const desktop = isDesktop();
  const cur = useWifi((s) => s.current);
  const refreshCurrent = useWifi((s) => s.refreshCurrent);
  const [droid, setDroid] = useState<{ ssid: string | null; ip: string | null } | null>(null);
  useEffect(() => { if (desktop) refreshCurrent(); }, [desktop, refreshCurrent]);
  useEffect(() => {
    if (Platform.OS !== 'android') return;
    return NetInfo.addEventListener((st) => {
      const d: any = st.type === 'wifi' ? st.details : null;
      setDroid(d ? { ssid: d.ssid && d.ssid !== '<unknown ssid>' ? d.ssid : null, ip: d.ipAddress ?? null } : null);
    });
  }, []);
  if (desktop) return { ssid: cur?.ssid ?? null, ip: cur?.ip ?? null, supported: true };
  if (Platform.OS === 'android') return { ssid: droid?.ssid ?? null, ip: droid?.ip ?? null, supported: true };
  return { ssid: null, ip: null, supported: false };
}

export function WifiActions() {
  const { c, radius } = useTheme();
  const [picker, setPicker] = useState(false);
  const refreshCurrent = useWifi((s) => s.refreshCurrent);
  if (Platform.OS === 'android') {
    return (
      <Tappable onPress={openWifiPanel} style={[styles.actBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <Icon name="wifi" size={12} color={c.text} />
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('WiFi 설정')}</Text>
      </Tappable>
    );
  }
  if (!isDesktop()) return null;
  return (
    <>
      <Tappable onPress={() => { try { fetch('/wifi-settings'); } catch { } }}
        style={[styles.actBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <Icon name="gear" size={12} color={c.muted} />
        <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>{t('시스템 설정')}</Text>
      </Tappable>
      <Tappable onPress={() => setPicker(true)}
        style={[styles.actBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <Icon name="wifi" size={12} color={c.text} />
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('WiFi 선택')}</Text>
      </Tappable>
      {picker && <WifiPicker onClose={() => { setPicker(false); refreshCurrent(); }} />}
    </>
  );
}

type BlockProps = {
  embedded?: boolean;
  onGoConn?: () => void;
};

function DesktopWifiBlock({ embedded, onGoConn }: BlockProps) {
  const { c, radius, fonts } = useTheme();
  const robotIp = useRobot((s) => s.ip);
  const remote = useRobot((s) => s.via === 'rendezvous' && s.conn === 'connected');
  const current = useWifi((s) => s.current);
  const refreshCurrent = useWifi((s) => s.refreshCurrent);
  const [picker, setPicker] = useState(false);

  useEffect(() => { refreshCurrent(); }, [refreshCurrent]);

  const ssid = current?.ssid ?? null;
  const ip = current?.ip ?? null;
  const onRobotNet = sameSubnet(ip, robotIp);
  const tint = !ssid ? c.dim : onRobotNet ? c.green : c.amber;

  const bars = <Bars strength={ssid ? (current && current.signal > 0 ? current.signal : null) : null} color={tint} />;
  const badge = ssid && !remote ? (
    <View style={[styles.badge, {
      flexShrink: 0,
      borderRadius: radius.sm,
      backgroundColor: onRobotNet ? 'rgba(63,185,80,0.12)' : 'rgba(210,153,34,0.12)',
      borderColor: onRobotNet ? 'rgba(63,185,80,0.5)' : 'rgba(210,153,34,0.5)',
    }]}>
      <Text style={{ color: onRobotNet ? c.greenTx : c.amberTx, fontSize: 9.5, fontWeight: '700' }}>
        {onRobotNet ? t('로봇 망') : t('로봇 망 아님')}
      </Text>
    </View>
  ) : null;
  const myIp = ssid ? `${t('내 IP')} ${ip ?? '—'}` : t('네트워크를 선택해 연결하세요');
  const buttons = (
    <>
      {onGoConn ? (
        <Tappable onPress={onGoConn} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          <Icon name="wifi" size={14} color={c.muted} />
          <Text style={{ color: c.muted, fontSize: 12, fontWeight: '600' }}>{t('연결 설정 ▸')}</Text>
        </Tappable>
      ) : (
        <Tappable onPress={() => { try { fetch('/wifi-settings'); } catch { } }} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          <Icon name="gear" size={14} color={c.muted} />
          <Text style={{ color: c.muted, fontSize: 12, fontWeight: '600' }}>{t('시스템 설정')}</Text>
        </Tappable>
      )}
      <Tappable onPress={() => setPicker(true)} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <Icon name="wifi" size={14} color={c.text} />
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('WiFi 선택')}</Text>
      </Tappable>
    </>
  );
  const warn = ssid && !onRobotNet && !remote ? (
    <View style={[styles.warn, { borderRadius: radius.md }]}>
      <Text style={{ color: c.amberTx, fontSize: 10.5, lineHeight: 15 }}>
        ⚠ {t('로봇')}({robotIp}){t('과 다른 망입니다 — 같은 망이 아니면 로봇에 연결할 수 없습니다. 로봇 AP를 선택해 연결하세요.')}
      </Text>
    </View>
  ) : null;
  const pickerEl = picker ? <WifiPicker onClose={() => { setPicker(false); refreshCurrent(); }} /> : null;


  return (
    <>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: embedded ? 0 : 20, marginBottom: 8 }}>
        {t('내 기기')} WiFi <Text style={{ fontWeight: '500' }}>— {t('이 기기가 붙어 있는 망')}</Text>
      </Text>
      <View style={[styles.card, { backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.md }]}>
        {bars}
        <View style={styles.info}>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <Text numberOfLines={1} style={{ color: c.text, fontSize: 13, fontWeight: '600', flexShrink: 1 }}>{ssid ?? t('WiFi 미연결')}</Text>
            {badge}
          </View>
          <Text numberOfLines={1} style={{ color: c.dim, fontSize: 10, marginTop: 2, fontFamily: fonts.mono }}>{myIp}</Text>
        </View>
        <View style={styles.btnRow}>{buttons}</View>
      </View>
      {warn}
      {pickerEl}
    </>
  );
}

function AndroidWifiBlock({ embedded, onGoConn }: BlockProps) {
  const { c, radius, fonts } = useTheme();
  const robotIp = useRobot((s) => s.ip);
  const remote = useRobot((s) => s.via === 'rendezvous' && s.conn === 'connected');
  const [net, setNet] = useState<NetInfoState | null>(null);
  const [locGranted, setLocGranted] = useState(true);

  const refresh = useCallback(() => { NetInfo.refresh().then(setNet).catch(() => {}); }, []);
  useEffect(() => {
    if (Platform.OS !== 'android') return;
    PermissionsAndroid.check(PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION).then(setLocGranted).catch(() => {});
    const unsub = NetInfo.addEventListener(setNet);
    return unsub;
  }, []);
  const askLocation = () =>
    PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION)
      .then((r) => { setLocGranted(r === 'granted'); refresh(); })
      .catch(() => {});

  if (Platform.OS !== 'android') return null;

  const wifi = net?.type === 'wifi';
  const d: any = wifi ? net?.details : null;
  const ssid: string | null = d?.ssid && d.ssid !== '<unknown ssid>' ? d.ssid : null;
  const ip: string | null = d?.ipAddress ?? null;
  const strength: number | null = typeof d?.strength === 'number' ? d.strength : null;
  const onRobotNet = sameSubnet(ip, robotIp);
  const tint = !wifi ? c.dim : onRobotNet ? c.green : c.amber;

  const name = !wifi ? t('WiFi 미연결') : ssid ?? t('WiFi 이름 확인 불가');
  const badge = wifi && !remote ? (
    <View style={[styles.badge, {
      flexShrink: 0,
      borderRadius: radius.sm,
      backgroundColor: onRobotNet ? 'rgba(63,185,80,0.12)' : 'rgba(210,153,34,0.12)',
      borderColor: onRobotNet ? 'rgba(63,185,80,0.5)' : 'rgba(210,153,34,0.5)',
    }]}>
      <Text style={{ color: onRobotNet ? c.greenTx : c.amberTx, fontSize: 9.5, fontWeight: '700' }}>
        {onRobotNet ? t('로봇 망') : t('로봇 망 아님')}
      </Text>
    </View>
  ) : null;
  const myIp = wifi
    ? `${t('내 IP')} ${ip ?? '—'}${strength != null ? ` · ${t('신호')} ${strength}%` : ''}`
    : remote ? t('원격 연결에는 WiFi가 필요 없습니다') : t('시스템 설정에서 WiFi를 켜세요');
  const buttons = (
    <>
      <Tappable onPress={openWifiPanel} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <Icon name="wifi" size={14} color={c.text} />
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('시스템 WiFi 설정')}</Text>
      </Tappable>
      {onGoConn && (
        <Tappable onPress={onGoConn} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          <Icon name="gear" size={14} color={c.muted} />
          <Text style={{ color: c.muted, fontSize: 12, fontWeight: '600' }}>{t('연결 설정 ▸')}</Text>
        </Tappable>
      )}
    </>
  );
  const warn = wifi && !onRobotNet && !remote ? (
    <View style={[styles.warn, { borderRadius: radius.md }]}>
      <Text style={{ color: c.amberTx, fontSize: 10.5, lineHeight: 15 }}>
        ⚠ {t('로봇')}({robotIp}){t('과 다른 망입니다 — 같은 망이 아니면 로봇에 연결할 수 없습니다. 시스템 설정에서 로봇 AP에 연결하세요.')}
      </Text>
    </View>
  ) : null;
  const locHint = wifi && !ssid ? (!locGranted ? (
    <Tappable onPress={askLocation}>
      <Text style={{ color: c.accent2, fontSize: 10, marginTop: 6 }}>
        {t('WiFi 이름 표시엔 위치 권한이 필요합니다 — 탭해서 허용')}
      </Text>
    </Tappable>
  ) : (
    <Tappable onPress={() => IntentLauncher.startActivityAsync('android.settings.LOCATION_SOURCE_SETTINGS').catch(() => {})}>
      <Text style={{ color: c.accent2, fontSize: 10, marginTop: 6 }}>
        {t('기기 위치가 꺼져 있어 WiFi 이름을 읽지 못합니다 — 탭해서 위치 켜기')}
      </Text>
    </Tappable>
  )) : null;


  return (
    <>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: embedded ? 0 : 20, marginBottom: 8 }}>
        {t('내 기기')} WiFi <Text style={{ fontWeight: '500' }}>— {t('이 기기가 붙어 있는 망')}</Text>
      </Text>
      <View style={[styles.card, { backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.md }]}>
        <Bars strength={wifi ? strength ?? 100 : null} color={tint} />
        <View style={styles.info}>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <Text numberOfLines={1} style={{ color: c.text, fontSize: 13, fontWeight: '600', flexShrink: 1 }}>{name}</Text>
            {badge}
          </View>
          <Text style={{ color: c.dim, fontSize: 10, marginTop: 2, fontFamily: fonts.mono }}>{myIp}</Text>
        </View>
        <View style={styles.btnRow}>{buttons}</View>
      </View>
      {warn}
      {locHint}
      <Text style={{ color: c.dim, fontSize: 9.5, marginTop: 6 }}>
        {t('안드로이드 정책상 앱 안에서 직접 스캔·연결은 불가 — 기기의 WiFi 설정이 열립니다.')}
      </Text>
    </>
  );
}

const styles = StyleSheet.create({
  card: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 11, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 11 },
  info: { flex: 1, minWidth: 180 },
  btnRow: { flexDirection: 'row', gap: 8, flexShrink: 0, marginLeft: 'auto' },
  badge: { borderWidth: 1, paddingHorizontal: 8, paddingVertical: 2 },
  btn: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 12, borderWidth: 1 },
  warn: { backgroundColor: 'rgba(210,153,34,0.10)', borderWidth: 1, borderColor: 'rgba(210,153,34,0.5)', padding: 10, marginTop: 8, maxWidth: 440 },
  actBtn: { flexDirection: 'row', alignItems: 'center', gap: 5, height: 28, paddingHorizontal: 10, borderWidth: 1 },
});
