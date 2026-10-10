import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, Modal as RNModal, Pressable, ScrollView, TextInput, ActivityIndicator } from 'react-native';
import { useWifi } from '@/store/wifi';
import { useRobot } from '@/store/robot';
import { isDesktop } from '@/lib/desktopBridge';
import { useDevMode, useSettings } from '@/store/settings';
import { useFeatures } from '@/store/capability';
import { Segmented } from '@/components/ui/controls';
import { RobotRegisterDialog, useCanOfferRegister } from '@/components/ui/RobotRegister';
import { useRouter } from 'expo-router';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { useRobots, useProfiles, SIM_SERIAL, type RobotProfile, type RobotVersion } from '@/store/robots';
import { robotVersions, autoRobotVersion } from '@/modules/registry';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

const VIA: Record<RobotProfile['lastVia'], string> = { lan: '로봇망', wan: 'WAN', rendezvous: '원격' };

function ago(ts: number): string {
  if (!ts) return '';
  const m = Math.floor((Date.now() - ts) / 60000);
  if (m < 1) return t('방금');
  if (m < 60) return `${m}${t('분 전')}`;
  const h = Math.floor(m / 60);
  if (h < 24) return `${h}${t('시간 전')}`;
  return `${Math.floor(h / 24)}${t('일 전')}`;
}

function RowEdit({ p, dev, onClose }: { p: RobotProfile; dev: boolean; onClose: () => void }) {
  const { c, fonts, radius } = useTheme();
  const [name, setName] = useState(p.name);
  const [vision, setVision] = useState(p.lan?.visionIp ?? '');
  const [ask, setAsk] = useState(false);
  const level3 = useSettings((s) => s.accessLevel >= 3);
  const features = useFeatures((s) => s.features);
  const curSerial = useRobots((s) => s.currentSerial);
  const autoVer = p.serial === curSerial && features ? (autoRobotVersion(features)?.label.toUpperCase() ?? null) : null;
  const [ver, setVer] = useState<RobotVersion>(p.robotVersion ?? 'none');
  const save = () => {
    useRobots.getState().rename(p.serial, name);
    if (dev && p.lan) useRobots.getState().setVisionIp(p.serial, vision);
    if (level3 && !autoVer) useRobots.getState().setRobotVersion(p.serial, ver);
    onClose();
  };
  const input = (v: string, set: (s: string) => void, ph: string) => (
    <TextInput value={v} onChangeText={set} placeholder={ph} placeholderTextColor={c.dim} autoCapitalize="none" autoCorrect={false}
      style={{ flex: 1, color: c.text, fontFamily: fonts.mono, fontSize: 12.5, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, paddingHorizontal: 8, paddingVertical: 6, backgroundColor: c.bg }} />
  );
  return (
    <View style={[styles.row, styles.edit, { borderColor: c.line, backgroundColor: c.elev, borderRadius: 10 }]}>
      <Text style={{ color: c.muted, fontSize: 11, fontFamily: fonts.mono }}>{p.serial}{p.lan?.ssid ? ` · ${p.lan.ssid}` : ''}{p.wan ? ` · WAN ${p.wan.ip}` : ''}</Text>
      <View style={styles.editRow}><Text style={[styles.lab, { color: c.muted }]}>{t('이름')}</Text>{input(name, setName, p.serial)}</View>
      {dev && p.lan && <View style={styles.editRow}><Text style={[styles.lab, { color: c.muted }]}>Vision IP</Text>{input(vision, setVision, t('비우면 로봇 IP 사용'))}</View>}
      {level3 && robotVersions.length > 0 && (
        <View style={styles.editRow}>
          <Text style={[styles.lab, { color: c.muted }]}>{t('로봇 버전')}</Text>
          {autoVer ? (
            <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '700' }}>{autoVer} · {t('자동 확정')}</Text>
          ) : (
            <View style={{ flex: 1 }}>
              <Segmented options={[{ key: 'none', label: 'None' }, ...robotVersions.map((v) => ({ key: v.key, label: v.label }))]}
                value={ver} onChange={(v) => setVer(v)} />
            </View>
          )}
        </View>
      )}
      <View style={[styles.editRow, { justifyContent: 'flex-end', gap: 6 }]}>
        {ask ? (
          <>
            <Text style={{ color: c.redTx, fontSize: 12, flex: 1 }}>{t('이 기기에서만 지웁니다 · 서버 배정은 대시보드에서')}</Text>
            <Tappable onPress={() => setAsk(false)} style={[styles.btn, { borderColor: c.line, borderRadius: radius.sm }]}><Text style={{ color: c.muted, fontSize: 12 }}>{t('취소')}</Text></Tappable>
            <Tappable onPress={() => { useRobots.getState().remove(p.serial); onClose(); }} style={[styles.btn, { borderColor: c.red, backgroundColor: c.red, borderRadius: radius.sm }]}><Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '700' }}>{t('삭제')}</Text></Tappable>
          </>
        ) : (
          <>
            <Tappable onPress={() => setAsk(true)} style={[styles.btn, { borderColor: c.line, borderRadius: radius.sm, marginRight: 'auto' }]}><Icon name="trash" size={14} color={c.redTx} /></Tappable>
            <Tappable onPress={onClose} style={[styles.btn, { borderColor: c.line, borderRadius: radius.sm }]}><Text style={{ color: c.muted, fontSize: 12 }}>{t('취소')}</Text></Tappable>
            <Tappable onPress={save} style={[styles.btn, { borderColor: c.accent, backgroundColor: c.accent, borderRadius: radius.sm }]}><Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '700' }}>{t('저장')}</Text></Tappable>
          </>
        )}
      </View>
    </View>
  );
}

export function RobotSheet({ visible, onClose }: { visible: boolean; onClose: () => void }) {
  const { c, fonts, radius } = useTheme();
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const serial = useRobots((s) => s.currentSerial);
  const profiles = useProfiles();
  const [edit, setEdit] = useState<string | null>(null);
  const dev = useDevMode();
  const canRegister = useCanOfferRegister();
  const [regOpen, setRegOpen] = useState<string | null>(null);
  const conn = useRobot((s) => s.conn);
  const { networks, scanning, scan } = useWifi();
  const [scanned, setScanned] = useState(false);
  const [offMsg, setOffMsg] = useState<string | null>(null);
  const canScan = isDesktop();
  const refresh = () => { if (!canScan || scanning) return; setOffMsg(null); setScanned(false); void scan().then(() => setScanned(true)); };
  useEffect(() => { if (visible) refresh(); else setOffMsg(null); }, [visible]); // eslint-disable-line react-hooks/exhaustive-deps
  const state = (p: RobotProfile): 'ok' | 'on' | 'off' => {
    if (p.serial === serial && conn === 'connected') return 'ok';
    if (canScan && scanned && p.lan?.ssid) return networks.some((n) => n.ssid === p.lan!.ssid) ? 'on' : 'off';
    return 'off';
  };
  const knownOff = (p: RobotProfile) => canScan && scanned && !scanning && !!p.lan?.ssid && !p.wan && !p.rendezvous && state(p) === 'off';
  const pick = (s: string) => { useRobots.getState().select(s); onClose(); if (s === SIM_SERIAL) goTop('/'); };
  const row = (key: string, main: React.ReactNode, onPress: () => void, extra?: { active?: boolean }) => (
    <Tappable key={key} onPress={onPress}
      style={[styles.row, { borderColor: c.line2, backgroundColor: extra?.active ? c.elev : 'transparent' }]}>
      {main}
    </Tappable>
  );
  return (
    <RNModal transparent visible={visible} animationType="fade" onRequestClose={onClose} supportedOrientations={MODAL_ORIENTATIONS}>
      <View style={[styles.wrap, { paddingLeft: 12 + insets.left, paddingRight: 12 + insets.right, paddingTop: 60 + insets.top, paddingBottom: 12 + insets.bottom }]}>
        <Pressable style={StyleSheet.absoluteFill} onPress={onClose} />
        <LinearGradient colors={[c.sheetA, c.sheetB]} style={[styles.sheet, { borderColor: c.line, borderRadius: radius.lg }]}>
          <View style={styles.head}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{t('로봇 선택')}</Text>
            <View style={{ flex: 1 }} />
            {canScan && (
              <Tappable onPress={refresh} accessibilityLabel={t('새로고침')} hitSlop={8} style={[styles.btn, { borderColor: c.line, borderRadius: radius.sm, flexDirection: 'row', gap: 6, marginRight: 10, opacity: scanning ? 0.6 : 1 }]}>
                {scanning ? <ActivityIndicator size="small" color={c.accent2} /> : <Icon name="recover" size={14} color={c.muted} />}
                <Text style={{ color: c.muted, fontSize: 11 }}>{scanning ? t('검색 중…') : t('새로고침')}</Text>
              </Tappable>
            )}
            <Tappable onPress={onClose} accessibilityLabel={t('닫기')} hitSlop={10}><Icon name="x" size={18} color={c.muted} /></Tappable>
          </View>
          <ScrollView style={{ maxHeight: 300 }} contentContainerStyle={{ gap: 4 }}>
            {profiles.length === 0 && (
              <Text style={{ color: c.muted, fontSize: 12, paddingHorizontal: 12, paddingVertical: 10 }}>
                {t('저장된 로봇이 없습니다 — [+ 로봇 추가]')}
              </Text>
            )}
            {profiles.map((p) => edit === p.serial
              ? <RowEdit key={p.serial} p={p} dev={dev} onClose={() => setEdit(null)} />
              : row(p.serial,
                <>
                  <View style={[styles.dot, { backgroundColor: { ok: c.green, on: c.accent, off: c.dim }[state(p)] }]} />
                  <View style={{ flex: 1 }}>
                    <Text numberOfLines={1} style={{ color: c.text, fontWeight: '700' }}>{p.name}</Text>
                    <Text numberOfLines={1} style={{ color: c.muted, fontSize: 11, fontFamily: fonts.mono }}>
                      {p.serial} · {[p.lan && (p.lan.ssid ? `${t(VIA.lan)} ${p.lan.ssid}` : t(VIA.lan)), p.wan && t(VIA.wan), p.rendezvous && t(VIA.rendezvous)].filter(Boolean).join(' · ')}{p.lastSeenAt ? ` · ${ago(p.lastSeenAt)}` : ''}
                    </Text>
                    {offMsg === p.serial && <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 2 }}>{t('로봇 WiFi 신호가 잡히지 않습니다')} · {p.lan?.ssid}</Text>}
                  </View>
                  {p.serial === serial && canRegister && !p.serial.includes(':') && (
                    <Tappable onPress={() => setRegOpen(p.serial)} accessibilityLabel={t('원격 등록')} hitSlop={6}
                      style={[styles.btn, { borderColor: c.accent, borderRadius: radius.sm, paddingVertical: 4, minHeight: 28 }]}>
                      <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '700' }}>{t('원격 등록')}</Text>
                    </Tappable>
                  )}
                  {(p.lan || p.wan) && (
                    <Tappable onPress={() => setEdit(p.serial)} accessibilityLabel={t('로봇 설정')} hitSlop={8} style={styles.more}>
                      <Icon name="sliders" size={16} color={c.muted} />
                    </Tappable>
                  )}
                </>,
                () => (knownOff(p) ? setOffMsg(p.serial) : pick(p.serial)), { active: p.serial === serial }))}
          </ScrollView>
          <View style={[styles.foot, { borderTopColor: c.line2 }]}>
            {row('sim', <><Icon name="monitor" size={18} color={c.accent2} /><Text style={{ color: c.text, fontWeight: '700' }}>{t('시뮬레이터')}</Text><Text style={{ color: c.muted, fontSize: 11 }}>{t('앱 내 물리 시뮬')}</Text></>,
              () => pick(SIM_SERIAL), { active: serial === SIM_SERIAL })}
            {row('add', <><Icon name="plus" size={18} color={c.accent2} /><Text style={{ color: c.text, fontWeight: '700' }}>{t('로봇 추가')}</Text><Text style={{ color: c.muted, fontSize: 11 }}>{t('찾기 · 주소 · 접근 코드')}</Text></>,
              () => { onClose(); router.push('/add-robot'); })}
          </View>
        </LinearGradient>
      </View>
      {regOpen && <RobotRegisterDialog serial={regOpen} onClose={() => setRegOpen(null)} />}
    </RNModal>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, backgroundColor: 'rgba(0,0,0,0.45)', justifyContent: 'flex-start', alignItems: 'flex-start', padding: 12, paddingTop: 60 },
  sheet: { width: 360, maxWidth: '100%', borderWidth: 1, padding: 8, gap: 6 },
  head: { flexDirection: 'row', alignItems: 'center', paddingHorizontal: 8, paddingVertical: 6 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, minHeight: 48, paddingHorizontal: 12, borderRadius: 10, borderWidth: 1 },
  dot: { width: 8, height: 8, borderRadius: 4 },
  foot: { borderTopWidth: 1, paddingTop: 6, gap: 4 },
  more: { width: 32, height: 32, alignItems: 'center', justifyContent: 'center' },
  edit: { flexDirection: 'column', alignItems: 'stretch', gap: 8, paddingVertical: 10 },
  editRow: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  lab: { width: 64, fontSize: 11 },
  btn: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 6, minHeight: 32, alignItems: 'center', justifyContent: 'center' },
});
