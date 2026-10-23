import { View, Text, StyleSheet } from 'react-native';
import { useState } from 'react';
import { useRouter } from 'expo-router';
import { useAccount } from '@/store/account';
import { useSettings } from '@/store/settings';
import { AccountPopover } from '@/components/hub/AccountPopover';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { EStop, HlcChip, Logo, SubBanners } from '@/components/TopBar';
import { QuitButton } from '@/components/QuitButton';
import { Battery } from '@/components/Battery';
import { useMyNetwork, openWifiPanel } from '@/components/WifiStatus';
import { WifiPicker } from '@/components/WifiPicker';
import { Modal } from '@/components/ui/overlays';
import { isDesktop, wifiConnect } from '@/lib/desktopBridge';
import { useRobot } from '@/store/robot';
import { useRobots, useProfiles, SIM_PROFILE } from '@/store/robots';
import { reconnectCurrent } from '@/lib/connectNow';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

function useShowEStop(): boolean {
  const conn = useRobot((s) => s.conn);
  const droppedAt = useRobot((s) => s.droppedAt);
  const isSim = useRobots((s) => s.currentSerial) === SIM_PROFILE.serial;
  return !isSim && (conn === 'connected' || (conn === 'connecting' && droppedAt != null));
}

export function SafetyCorner() {
  const show = useShowEStop();
  return (
    <View pointerEvents="box-none" style={styles.corner}>
      <HlcChip />
      {show && <EStop />}
    </View>
  );
}

export function HubHeader({ onOpenSheet, title, subtitle }: {
  onOpenSheet?: () => void; title?: string; subtitle?: string;
}) {
  const { c, fonts, radius } = useTheme();
  const router = useRouter();
  const acct = useAccount((s) => s.account);
  const nick = useSettings((s) => s.nickname);
  const [acctOpen, setAcctOpen] = useState(false);
  const [wifiPick, setWifiPick] = useState(false);
  const showEStop = useShowEStop();
  const codeNeeded = useRobots((s) => s.codeNeeded);
  const conn = useRobot((s) => s.conn);
  const err = useRobot((s) => s.connError);
  const batt = useRobot((s) => s.battPct);
  const serial = useRobots((s) => s.currentSerial);
  const profiles = useProfiles();
  const via = useRobots((s) => s.local.find((p) => p.serial === s.currentSerial)?.lastVia);
  const cur = serial === SIM_PROFILE.serial ? SIM_PROFILE : profiles.find((p) => p.serial === serial) ?? null;
  const viaEff = via ?? (cur?.rendezvous ? 'rendezvous' : cur?.lastVia);
  const viaLabel = viaEff === 'rendezvous' ? t('원격') : viaEff === 'wan' ? 'WAN' : t('로봇망');
  const isSim = serial === SIM_PROFILE.serial;
  const net = useMyNetwork();
  const offNet = net.supported && !!net.ip && !net.ip.startsWith('192.168.0.');
  const lanLike = !cur || (viaEff === 'lan' && (cur.lan?.ip ?? '192.168.0.10').startsWith('192.168.0.'));
  const wifiSwitching = useRobots((s) => s.wifiSwitching);
  const issue = useRobots((s) => s.selectIssue);
  const ask = useRobots((s) => s.wifiAsk);
  const switching = useRobots((s) => s.switching);
  const st = wifiSwitching ? 'wifi' : switching && !isSim ? 'busy' : conn === 'connected' && !isSim ? 'ok' : !cur ? (offNet ? 'nolan' : 'none') : isSim ? 'sim' : codeNeeded ? 'code'
    : lanLike && offNet ? 'nolan' : conn === 'connecting' ? 'busy' : err ? 'fail' : 'off';
  const dot = { none: c.dim, off: c.dim, busy: c.accent, wifi: c.accent, ok: c.green, fail: c.red, sim: c.accent2, code: c.amber, nolan: c.amber }[st];
  const knownSsid = cur?.lan?.ssid && cur.lan.ssid !== net.ssid ? cur.lan.ssid : undefined;
  const label = {
    none: t('로봇 WiFi에 연결하세요'), off: t('미연결'), busy: t('연결 중…'), sim: t('앱 내 물리 시뮬'), code: t('원격 — 접근 코드 필요'),
    ok: `${t('연결됨')} · ${viaLabel}`, fail: `${t('연결 실패')} · ${err ?? ''}`,
    wifi: `${wifiSwitching ?? ''} ${t('로 전환 중…')}`,
    nolan: knownSsid ? `${t('로봇 WiFi')}(${knownSsid})${t('로 바꿔 주세요')}` : `${t('로봇 WiFi가 아닙니다')}${net.ssid ? ` · ${net.ssid}` : ''}`,
  }[st];
  const sub = !!title;
  const controlOn = (conn === 'connected' && !switching) || isSim;
  return (
    <>
    <View pointerEvents="box-none" style={styles.row}>
      {sub ? (
        <>
          <Logo />
          <Tappable onPress={controlOn ? () => goTop('/') : undefined} disabled={!controlOn} accessibilityLabel={t('컨트롤')}
            style={[styles.ctrl, { backgroundColor: controlOn ? 'rgba(77,156,245,0.18)' : c.glass, borderColor: controlOn ? 'rgba(77,156,245,0.6)' : c.glassLine, opacity: controlOn ? 1 : 0.45 }]}>
            <Icon name="gamepad" size={18} color={controlOn ? c.accent2 : c.muted} />
          </Tappable>
          <Text numberOfLines={1} style={{ color: c.text, fontSize: 22, fontWeight: '700', flexShrink: 1 }}>{title}</Text>
          {subtitle ? <Text numberOfLines={1} style={{ color: c.muted, fontSize: 13, fontFamily: fonts.mono, flexShrink: 1 }}>{subtitle}</Text> : null}
        </>
      ) : (
        <>
      <View style={[styles.dot, { backgroundColor: dot }]} />
      <Tappable onPress={onOpenSheet} accessibilityLabel={t('로봇 선택')} style={styles.name}>
        <Text numberOfLines={1} style={{ color: c.text, fontSize: 22, fontWeight: '700', flexShrink: 1 }}>{cur?.name ?? t('로봇 없음')}</Text>
        <Icon name="caret" size={16} color={c.muted} />
      </Tappable>
      <Text numberOfLines={1} style={{ color: st === 'fail' ? c.redTx : c.muted, fontSize: 13, fontFamily: fonts.mono, flexShrink: 1 }}>{label}</Text>
      {st === 'ok' && batt > 0 && <Battery pct={batt} />}
      {st === 'fail' && (
        <Tappable onPress={() => reconnectCurrent()} style={[styles.retry, { borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{t('다시 연결')}</Text>
        </Tappable>
      )}
      {st === 'nolan' && (
        <Tappable onPress={() => { if (isDesktop()) { if (knownSsid) wifiConnect(knownSsid).catch(() => setWifiPick(true)); else setWifiPick(true); } else openWifiPanel(); }}
          style={[styles.retry, { borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{isDesktop() && knownSsid ? t('WiFi 전환') : t('WiFi 설정')}</Text>
        </Tappable>
      )}
      {st === 'code' && (
        <Tappable onPress={() => router.push('/pin')} style={[styles.retry, { borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{t('코드 입력')}</Text>
        </Tappable>
      )}
        </>
      )}
      <View style={{ flex: 1 }} />
      <Tappable onPress={() => setAcctOpen(true)} accessibilityLabel={t('프로필')}
        style={[styles.avatar, { backgroundColor: acct ? c.accent : c.elev, borderColor: acct ? 'transparent' : c.line }]}>
        {acct ? <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '800' }}>{(nick || acct.accountName).slice(0, 1).toUpperCase()}</Text>
              : <Icon name="user" size={18} color={c.text} />}
      </Tappable>
      <HlcChip />
      {showEStop && <EStop />}
      <QuitButton />
      {acctOpen && <AccountPopover onClose={() => setAcctOpen(false)} />}
      {wifiPick && <WifiPicker onClose={() => setWifiPick(false)} />}
      {ask && (
        <Modal onClose={() => useRobots.getState().answerWifiAsk(false)}>
          <View style={[styles.issue, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('로봇 WiFi로 바꿀까요?')}</Text>
            <Text numberOfLines={1} style={{ color: c.text, fontSize: 15, fontWeight: '700', fontFamily: fonts.mono }}>{ask.ssid}</Text>
            <Text style={{ color: c.muted, fontSize: 13, lineHeight: 19 }}>{t('로봇 AP 로 옮기면 지금 망이 끊깁니다.')}</Text>
            <View style={{ flexDirection: 'row', justifyContent: 'flex-end', gap: 8 }}>
              <Tappable onPress={() => useRobots.getState().answerWifiAsk(false)} style={[styles.retry, { borderColor: c.line, borderRadius: radius.sm }]}>
                <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700' }}>{t('아니오')}</Text>
              </Tappable>
              <Tappable onPress={() => useRobots.getState().answerWifiAsk(true)} style={[styles.retry, { borderColor: c.accent, backgroundColor: c.accent, borderRadius: radius.sm }]}>
                <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('WiFi 전환')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}
      {issue && (
        <Modal onClose={() => useRobots.getState().clearSelectIssue()}>
          <View style={[styles.issue, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>
              {issue.kind === 'off' ? t('로봇 WiFi를 찾을 수 없습니다') : issue.kind === 'wifi_pw' ? t('WiFi 비밀번호가 필요합니다') : t('로봇 WiFi로 전환하지 못했습니다')}
            </Text>
            <Text style={{ color: c.muted, fontSize: 13, lineHeight: 19 }}>
              {issue.name} · {issue.ssid}{'\n'}
              {issue.kind === 'off' ? t('이 자리에서는 신호가 잡히지 않습니다. 로봇 전원과 거리를 확인한 뒤 다시 시도하세요.')
                : issue.kind === 'wifi_pw' ? t('이 WiFi에 처음 연결합니다. WiFi 선택에서 비밀번호를 한 번 입력하세요.')
                : t('다시 시도하거나 WiFi 선택에서 직접 연결하세요.')}
            </Text>
            <View style={{ flexDirection: 'row', justifyContent: 'flex-end', gap: 8 }}>
              {issue.kind !== 'off' && (
                <Tappable onPress={() => { useRobots.getState().clearSelectIssue(); setWifiPick(true); }} style={[styles.retry, { borderColor: c.line, borderRadius: radius.sm }]}>
                  <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700' }}>{t('WiFi 선택')}</Text>
                </Tappable>
              )}
              <Tappable onPress={() => useRobots.getState().clearSelectIssue()} style={[styles.retry, { borderColor: c.accent, backgroundColor: c.accent, borderRadius: radius.sm }]}>
                <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('확인')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}
    </View>
    <SubBanners top={sub ? 104 : 60} />
    </>
  );
}
const styles = StyleSheet.create({
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, height: 56, paddingHorizontal: 16, zIndex: 10 },
  dot: { width: 12, height: 12, borderRadius: 6 },
  name: { flexDirection: 'row', alignItems: 'center', gap: 6, maxWidth: '45%' },
  retry: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 5 },
  issue: { width: 360, maxWidth: '92%', borderWidth: 1, padding: 18, gap: 12 },
  ctrl: { width: 32, height: 32, borderRadius: 9, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  avatar: { width: 36, height: 36, borderRadius: 18, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  corner: { position: 'absolute', top: 10, right: 16, flexDirection: 'row', alignItems: 'center', gap: 10, zIndex: 20 },
});
