import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Toggle } from '@/components/ui/controls';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { PDU_PORT } from '@/lib/robotState';
import { actions } from '@/lib/rest';
import { calibAllowed, type CalibFix } from '@/lib/commissioning';
import { t } from '@/lib/i18n';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { Modal } from '@/components/ui/overlays';
import { useFeatureCanFd } from '@/store/capability';
import { FdPowerRails } from '@/components/panels/FdPowerPanel';
import { CalibFixButton } from '@/components/CalibFixButton';
import { useLegible, H2, Desc, Group } from '@/components/panels/settings/common';
import { VisionResetRow } from '@/components/panels/VisionResetRow';
import { notify } from '@/components/DoneToast';

function hexA(hex: string, a: number) {
  const h = hex.replace('#', '');
  return `rgba(${parseInt(h.slice(0, 2), 16)},${parseInt(h.slice(2, 4), 16)},${parseInt(h.slice(4, 6), 16)},${a})`;
}

export function TRow({ nm, sub, subColor, right }: { nm: string; sub: string; subColor?: string; right: React.ReactNode }) {
  const { c } = useTheme();
  const lg = useLegible();
  return (
    <View style={[styles.trow, { borderTopColor: c.line2 }]}>
      <View style={{ flex: 1 }}>
        <Text style={{ color: c.text, fontSize: lg ? 15 : 13, fontWeight: lg ? '600' : undefined }}>{nm}</Text>
        <Text style={{ color: subColor ?? (lg ? c.muted : c.dim), fontSize: lg ? 12 : 10.5, marginTop: 2 }}>{sub}</Text>
      </View>
      {right}
    </View>
  );
}

export type PowerZone = 'top' | 'side' | 'camera' | 'internal';
export const ZONE_ORDER: PowerZone[] = ['top', 'side', 'camera', 'internal'];
const ZONE_TITLE: Record<PowerZone, string> = { top: '상단 포트', side: '전/후방', camera: '카메라', internal: '내부' };

type PduBoolKey = 'amp' | 'lidar' | 'cctv' | 'thermal' | 'irled' | 'cam5v' | 'audio5v' | 'visionPc' | 'comm' | 'fetLeg' | 'fetAdd' | 'fetExt';
export const PDU_RAILS: { port: number; bit: PduBoolKey; nm: string; sub: string; zone: PowerZone; danger?: boolean; askOff?: boolean }[] = [
  { port: PDU_PORT.LIDAR, bit: 'lidar', nm: '12V 포트 1', sub: '상단 12V 포트 전원', zone: 'top', askOff: true },
  { port: PDU_PORT.CCTV, bit: 'cctv', nm: '12V 포트 2', sub: '상단 12V 포트 전원', zone: 'top', askOff: true },
  { port: PDU_PORT.THERMAL, bit: 'thermal', nm: '12V 포트 3', sub: '상단 12V 포트 전원', zone: 'top', askOff: true },
  { port: PDU_PORT.COMM, bit: 'comm', nm: '12V 포트 4', sub: '상단 12V 포트 전원', zone: 'top', askOff: true },
  { port: PDU_PORT.IRLED, bit: 'irled', nm: '전/후방 12V', sub: '전방·후방 사이드 보드(IR LED) 전원 — 함께 켜고 끔', zone: 'side' },
  { port: PDU_PORT.CAMERAS_5V, bit: 'cam5v', nm: '카메라 (5V)', sub: '비전 카메라 전원 — 끄면 영상 끊김', zone: 'camera', danger: true },
  { port: PDU_PORT.AUDIO_USBHUB_5V, bit: 'audio5v', nm: '오디오·USB 허브 (5V)', sub: '오디오 코덱/사이드캠 허브 — 끄면 오디오 장치 소실', zone: 'camera', danger: true },
  { port: PDU_PORT.SPEAKER, bit: 'amp', nm: '스피커 앰프 (12V)', sub: '워키토키 소리 출력 — 꺼져 있으면 무음', zone: 'internal' },
  { port: PDU_PORT.VISION_PC, bit: 'visionPc', nm: 'User PC (12V)', sub: '⚠ 끄면 카메라·오디오·이 화면의 텔레메트리 일부가 끊깁니다', zone: 'internal', danger: true },
];

export const DRIVE_RAILS: { port: number; bit: PduBoolKey; railKey: 'leg' | 'add' | 'ext'; nm: string; sub: string; zone: PowerZone; legGuard?: boolean }[] = [
  { port: PDU_PORT.ADD_48V, bit: 'fetAdd', railKey: 'add', nm: 'ARM · ADDON (48V)', sub: '팔·부가장치 전원 — 팔이 정지 상태인지 확인 후 끄기', zone: 'top' },
  { port: PDU_PORT.EXT_48V, bit: 'fetExt', railKey: 'ext', nm: 'EXTERN (48V)', sub: '외부 확장 포트 전원', zone: 'top' },
  { port: PDU_PORT.LEG_48V, bit: 'fetLeg', railKey: 'leg', nm: 'LEGS (48V 구동)', sub: '다리 모터 전원 — 끄면 로봇이 주저앉음. 끄는 것은 비기립 상태에서만', zone: 'internal', legGuard: true },
];
export const LEG_OFF_OK = ['SITTING', 'CONTROL_OFF', 'FALL_MODE'];

export type ConfirmReq = { title: string; message?: string; confirmLabel: string; danger: boolean; skipKey?: string; run: () => void; extra?: React.ReactNode };

export const askOffToggle = (nm: string, set: (v: boolean) => void, request: (r: ConfirmReq) => void) => (v: boolean) =>
  v ? set(true) : request({ title: t('정말 끌까요?'), message: `${nm} — ${t('연결된 장비의 전원이 꺼집니다. 통신장비가 연결되어 있다면 앱과의 연결이 끊길 수 있습니다.')}`, confirmLabel: t('끄기'), danger: true, run: () => set(false) });

export function CalibRow({ nm, sub, blocked, blockReason, confirmMsg, fire, request, allow = calibAllowed, confirmExtra, onRequest, fix }: {
  nm: string; sub: string; blocked: boolean; blockReason: string; confirmMsg: string;
  fire: () => void; request: (req: ConfirmReq) => void;
  allow?: () => boolean;
  confirmExtra?: React.ReactNode;
  onRequest?: () => void;
  fix?: CalibFix;
}) {
  const { c, radius } = useTheme();
  const onPress = () => {
    if (blocked) return;
    const guardedFire = () => { if (allow()) fire(); };
    onRequest?.();
    request({ title: t('보정을 실행할까요?'), message: `${nm} — ${confirmMsg}`, confirmLabel: t('실행'), danger: true, run: guardedFire, extra: confirmExtra });
  };
  return (
    <TRow nm={nm} sub={blocked ? `⛔ ${blockReason}` : sub} subColor={blocked ? c.redbright : undefined}
      right={
        <View style={styles.runRow}>
          {blocked && fix && <CalibFixButton fix={fix} />}
          <Tappable disabled={blocked} onPress={onPress}
            style={[styles.run, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: blocked ? 0.4 : 1 }]}>
            <Text style={[styles.runTxt, { color: c.text }]}>{t('실행')}</Text>
          </Tappable>
        </View>
      } />
  );
}

export function DriveRailRow({ nm, sub, on, volts, blocked, blockReason, fire, request }: {
  nm: string; sub: string; on: boolean; volts: number | null; blocked: boolean; blockReason: string;
  fire: (next: boolean) => void; request: (req: ConfirmReq) => void;
}) {
  const { c, fonts, radius } = useTheme();
  const lg = useLegible();
  const btn = useActBtn();
  const next = !on;
  const danger = !next;
  const color = danger ? c.redbright : c.green;
  const onPress = () => {
    if (blocked) return;
    request({
      title: danger ? t('정말 끌까요?') : t('정말 켤까요?'),
      message: `${nm} — ${sub}`,
      confirmLabel: danger ? t('끄기') : t('켜기'),
      danger, run: () => fire(next),
    });
  };
  return (
    <TRow nm={nm} sub={blocked ? `⛔ ${blockReason}` : sub} subColor={blocked ? c.redbright : c.amberTx}
      right={
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          <Text style={{ color: on ? c.greenTx : c.dim, fontSize: lg ? 12 : 10.5, fontFamily: fonts.mono }}>
            {on ? 'ON' : 'OFF'}{volts != null ? ` ${volts.toFixed(1)}V` : ''}
          </Text>
          <Tappable disabled={blocked} onPress={onPress}
            style={[btn.box, { borderRadius: radius.sm, borderColor: hexA(color, 0.5), backgroundColor: hexA(color, 0.1), opacity: blocked ? 0.4 : 1 }]}>
            <Text style={{ color, fontSize: btn.font, fontWeight: btn.weight }}>{danger ? t('끄기') : t('켜기')}</Text>
          </Tappable>
        </View>
      } />
  );
}

export function SecGap() {
  return <View style={{ height: useLegible() ? 28 : 12 }} />;
}

export function useActBtn() {
  return useLegible()
    ? { box: styles.run, font: 19, weight: '800' as const, icon: 18 }
    : { box: styles.action, font: 11, weight: '700' as const, icon: 13 };
}

const ZONE_EN: Record<PowerZone, string> = { top: 'Top Ports', side: 'Front / Rear', camera: 'Cameras', internal: 'Internal' };
const ZONE_DESC: Record<PowerZone, string> = {
  top: '사용자가 바깥에서 물리는 포트입니다. 12V 포트는 무엇을 물리느냐가 로봇마다 달라 번호로 부릅니다.',
  side: '전방·후방 사이드 보드(IR LED) 전원입니다.',
  camera: '끄면 영상·오디오가 끊깁니다 — 켜고 끌 때 한 번 더 확인합니다.',
  internal: '로봇 안쪽 전원입니다. LEGS 는 로봇이 서 있지 않을 때만 끌 수 있습니다.',
};

export function ZoneSection({ zone, children }: { zone: PowerZone; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  const [help, setHelp] = useState(false);
  return (
    <>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
        <H2>{t(ZONE_TITLE[zone])} <Text style={{ color: c.muted, fontSize: 11 }}>{ZONE_EN[zone]}</Text></H2>
        {zone === 'top' && (
          <Tappable onPress={() => setHelp(true)} hitSlop={6} accessibilityLabel={t('상단 포트 도움말')}
            style={{ height: 22, paddingHorizontal: 9, borderRadius: 11, borderWidth: 1, alignItems: 'center', justifyContent: 'center', borderColor: c.line, backgroundColor: c.elev, marginBottom: 4 }}>
            <Text style={{ color: c.muted, fontSize: 10.5 }}>? {t('도움말')}</Text>
          </Tappable>
        )}
      </View>
      <Desc>{t(ZONE_DESC[zone])}</Desc>
      <Group>{children}</Group>
      {help && (
        <Modal onClose={() => setHelp(false)}>
          <View style={{ width: 520, maxWidth: '100%', padding: 18, gap: 12, borderRadius: radius.lg, backgroundColor: c.panel }}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{t('상단 포트')}</Text>
            <View style={{ height: 260, borderRadius: radius.md, borderWidth: 1, borderStyle: 'dashed', borderColor: c.line, backgroundColor: c.elev, alignItems: 'center', justifyContent: 'center' }}>
              <Text style={{ color: c.dim, fontSize: 12 }}>{t('포트 위치 그림은 추후 추가될 예정입니다.')}</Text>
            </View>
            <Tappable onPress={() => setHelp(false)} style={[styles.action, { alignSelf: 'flex-end', borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
              <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('닫기')}</Text>
            </Tappable>
          </View>
        </Modal>
      )}
    </>
  );
}

export function PowerPanel() {
  const { c, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const gait = useRobot((s) => s.gait);
  const pdu = useTelemetry((s) => s.pdu);
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const canFd = useFeatureCanFd();
  const btn = useActBtn();
  return (
    <>
      <Desc>{canFd ? t('상태·전압·전류는 로봇 PDU(CAN-FD)의 실측값 — 반영에 1초 정도 걸릴 수 있습니다.') : t('토글은 로봇 PDU의 실제 상태(Motion TCP)를 표시 — 반영에 1초 정도 걸릴 수 있습니다.')}{pdu || canFd ? '' : t(' — 레일 상태 수신 대기 중…')}</Desc>
      {canFd && <FdPowerRails />}
      {!canFd && ZONE_ORDER.map((z) => (
        <View key={z}>
          <ZoneSection zone={z}>
          {PDU_RAILS.filter((r) => r.zone === z).map((r) => r.danger ? (
            <DriveRailRow key={r.port} nm={t(r.nm)} sub={t(r.sub)} on={pdu ? pdu[r.bit] : false} volts={null}
              blocked={!pdu} blockReason={t('레일 상태 수신 전 — 조작 불가')}
              fire={(next) => actions.pduPower(ip, r.port, next).catch(() => {})} request={setConfirmReq} />
          ) : (
            <TRow key={r.port} nm={t(r.nm)} sub={t(r.sub)}
              right={<Toggle value={pdu ? pdu[r.bit] : false} disabled={!pdu} onChange={r.askOff
                ? askOffToggle(t(r.nm), (v) => actions.pduPower(ip, r.port, v).catch(() => {}), setConfirmReq)
                : (v) => actions.pduPower(ip, r.port, v).catch(() => {})} />} />
          ))}
          {DRIVE_RAILS.filter((r) => r.zone === z).map((r) => {
            const on = pdu ? pdu[r.bit] : false;
            const legBlocked = !!r.legGuard && on && !LEG_OFF_OK.includes(gait);
            return (
              <DriveRailRow key={r.port} nm={t(r.nm)} sub={t(r.sub)} on={on}
                volts={pdu?.rails?.[r.railKey]?.v ?? null}
                blocked={!pdu || legBlocked}
                blockReason={!pdu ? t('레일 상태 수신 전 — 조작 불가') : t('현재 {gait} — 비기립(SITTING/CONTROL_OFF/FALL)에서만 끌 수 있음').replace('{gait}', gait)}
                fire={(next) => actions.pduPower(ip, r.port, next).catch(() => {})} request={setConfirmReq} />
            );
          })}
          </ZoneSection>
          <SecGap />
        </View>
      ))}
      <H2>{t('로봇 PC')} <Text style={{ color: c.muted, fontSize: 11 }}>Restart / Reboot</Text></H2>
      <Desc>{t('재시작·재부팅하는 동안 로봇이 모터 전원을 잃고 앱 연결이 끊깁니다 — 로봇을 앉힌 뒤에 하세요.')}</Desc>
      <Group>
      <TRow nm={t('로봇 서비스 재시작')} sub={t('PC 는 켜 둔 채 로봇 소프트웨어(rbq.service)를 다시 시작합니다')}
        right={
          <Tappable onPress={() => {
            setConfirmReq({ title: t('로봇 서비스를 재시작할까요?'), message: t('재시작 동안 로봇 연결이 끊깁니다'), confirmLabel: t('로봇 서비스 재시작'), danger: true, run: () => actions.restart(ip).then(() => notify(t('로봇을 재시작하는 중입니다 — 잠시 뒤 다시 연결해 주세요')), (e) => notify(`${t('재시작 요청 실패')}: ${e?.message ?? e}`, 'warn')) });
          }} style={[btn.box, { flexDirection: 'row', gap: 6, backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
            <Icon name="power" size={btn.icon} color={c.text} />
            <Text style={{ color: c.text, fontSize: btn.font, fontWeight: btn.weight }}>{t('로봇 서비스 재시작')}</Text>
          </Tappable>
        } />
      <TRow nm={t('로봇 PC 재부팅')} sub={t('로봇 PC를 재시작합니다')}
        right={
          <Tappable onPress={() => {
            setConfirmReq({ title: t('로봇 PC를 재부팅할까요?'), message: t('재부팅 동안 로봇 연결이 끊깁니다'), confirmLabel: t('로봇 PC 재부팅'), danger: true, run: () => actions.reboot(ip).then(() => notify(t('재부팅을 요청했습니다 — 잠시 뒤 연결이 끊깁니다')), (e) => notify(`${t('재부팅 요청 실패')}: ${e?.message ?? e}`, 'warn')) });
          }} style={[btn.box, { flexDirection: 'row', gap: 6, backgroundColor: 'rgba(231,51,28,0.85)', borderColor: c.dangerLine, borderRadius: radius.sm }]}>
            <Icon name="power" size={btn.icon} color="#fff" />
            <Text style={{ color: '#fff', fontSize: btn.font, fontWeight: btn.weight }}>{t('로봇 PC 재부팅')}</Text>
          </Tappable>
        } />
      </Group>
      <SecGap />
      <H2>{t('비전')} <Text style={{ color: c.muted, fontSize: 11 }}>Vision</Text></H2>
      <Desc>{t('카메라 파이프라인을 다시 시작합니다 — 영상이 수 초 끊깁니다.')}</Desc>
      <Group><VisionResetRow /></Group>
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)}>
          {confirmReq.extra}
        </ConfirmModal>
      )}
    </>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  runRow: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  run: { alignItems: 'center', justifyContent: 'center', height: 42, minWidth: 112, paddingHorizontal: 18, borderWidth: 1 },
  runTxt: { fontSize: 19, fontWeight: '800' },
  trow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 12, borderTopWidth: 1 },
});
