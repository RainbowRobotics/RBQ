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
import { commissioning, calibAllowed, useCalibGuard, useSitImuCalibGuard, sitImuCalibAllowed, type CalibFix } from '@/lib/commissioning';
import { t } from '@/lib/i18n';
import { ConfirmModal, confirmSkipped } from '@/components/ui/ConfirmModal';
import { ImuLevelHint } from '@/components/BubbleLevel';
import { SitRobotHint } from '@/components/SitRobotHint';
import { AccCalibModal } from '@/components/AccCalibModal';
import { useFeatureCanFd } from '@/store/capability';
import { FdPowerRails } from '@/components/panels/FdPowerPanel';
import { ImuNullModal } from '@/components/ImuNullModal';
import { CalibFixButton } from '@/components/CalibFixButton';
import { useLegible } from '@/components/panels/settings/common';

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

type PduBoolKey = 'amp' | 'lidar' | 'cctv' | 'thermal' | 'irled' | 'cam5v' | 'audio5v' | 'visionPc' | 'comm' | 'fetLeg' | 'fetAdd' | 'fetExt';
export const PDU_RAILS: { port: number; bit: PduBoolKey; nm: string; sub: string; danger?: boolean; noL1?: boolean }[] = [
  { port: PDU_PORT.SPEAKER, bit: 'amp', nm: '스피커 앰프 (12V)', sub: '워키토키 소리 출력 — 꺼져 있으면 무음' },
  { port: PDU_PORT.LIDAR, bit: 'lidar', nm: 'LiDAR (12V)', sub: '라이다 센서 전원', noL1: true },
  { port: PDU_PORT.CCTV, bit: 'cctv', nm: 'CCTV (12V)', sub: 'CCTV 카메라 전원' },
  { port: PDU_PORT.THERMAL, bit: 'thermal', nm: '열화상 (12V)', sub: '열화상 카메라 전원' },
  { port: PDU_PORT.IRLED, bit: 'irled', nm: 'IR LED (12V)', sub: '야간 조명 전원' },
  { port: PDU_PORT.CAMERAS_5V, bit: 'cam5v', nm: '카메라 (5V)', sub: '비전 카메라 전원 — 끄면 영상 끊김', danger: true },
  { port: PDU_PORT.AUDIO_USBHUB_5V, bit: 'audio5v', nm: '오디오·USB 허브 (5V)', sub: '오디오 코덱/사이드캠 허브 — 끄면 오디오 장치 소실', danger: true },
  { port: PDU_PORT.VISION_PC, bit: 'visionPc', nm: 'Vision PC (12V)', sub: '⚠ 끄면 카메라·오디오·이 화면의 텔레메트리 일부가 끊깁니다', danger: true },
  { port: PDU_PORT.COMM, bit: 'comm', nm: '통신 (12V)', sub: '⚠ 끄면 이 앱과의 연결 자체가 끊깁니다', danger: true },
];

export const DRIVE_RAILS: { port: number; bit: PduBoolKey; railKey: 'leg' | 'add' | 'ext'; nm: string; sub: string; legGuard?: boolean; l1?: boolean }[] = [
  { port: PDU_PORT.LEG_48V, bit: 'fetLeg', railKey: 'leg', nm: 'LEGS (48V 구동)', sub: '다리 모터 전원 — 끄면 로봇이 주저앉음. 끄는 것은 비기립 상태에서만', legGuard: true },
  { port: PDU_PORT.ADD_48V, bit: 'fetAdd', railKey: 'add', nm: 'ARM · ADDON (48V)', sub: '팔·부가장치 전원 — 팔이 정지 상태인지 확인 후 끄기', l1: true },
  { port: PDU_PORT.EXT_48V, bit: 'fetExt', railKey: 'ext', nm: 'EXTERN (48V)', sub: '외부 확장 포트 전원', l1: true },
];
const LEG_OFF_OK = ['SITTING', 'CONTROL_OFF', 'FALL_MODE'];

export type ConfirmReq = { title: string; message?: string; confirmLabel: string; danger: boolean; skipKey?: string; run: () => void; extra?: React.ReactNode };

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

export function DriveRailRow({ nm, sub, on, volts, blocked, blockReason, fire, request, allowSkip = false }: {
  nm: string; sub: string; on: boolean; volts: number | null; blocked: boolean; blockReason: string;
  fire: (next: boolean) => void; request: (req: ConfirmReq) => void; allowSkip?: boolean;
}) {
  const { c, fonts, radius } = useTheme();
  const next = !on;
  const danger = !next;
  const color = danger ? c.redbright : c.green;
  const onPress = () => {
    if (blocked) return;
    const skipKey = allowSkip ? `pdu:${nm}` : undefined;
    const run = () => fire(next);
    if (skipKey && confirmSkipped(skipKey)) { run(); return; }
    request({
      title: danger ? t('정말 끌까요?') : t('정말 켤까요?'),
      message: `${nm} — ${sub}`,
      confirmLabel: danger ? t('끄기') : t('켜기'),
      danger, skipKey, run,
    });
  };
  return (
    <TRow nm={nm} sub={blocked ? `⛔ ${blockReason}` : sub} subColor={blocked ? c.redbright : c.amberTx}
      right={
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          <Text style={{ color: on ? c.greenTx : c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>
            {on ? 'ON' : 'OFF'}{volts != null ? ` ${volts.toFixed(1)}V` : ''}
          </Text>
          <Tappable disabled={blocked} onPress={onPress}
            style={[styles.action, { borderRadius: radius.sm, borderColor: hexA(color, 0.5), backgroundColor: hexA(color, 0.1), opacity: blocked ? 0.4 : 1 }]}>
            <Text style={{ color, fontSize: 11, fontWeight: '700' }}>{danger ? t('끄기') : t('켜기')}</Text>
          </Tappable>
        </View>
      } />
  );
}

export function PowerPanel() {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const gait = useRobot((s) => s.gait);
  const pdu = useTelemetry((s) => s.pdu);
  const calib = useCalibGuard();
  const sitGuard = useSitImuCalibGuard();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const [accOpen, setAccOpen] = useState(false);
  const canFd = useFeatureCanFd();
  const [imuOpen, setImuOpen] = useState(false);
  return (
    <>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>⚡ {t('전원 / 시스템')}</Text>
      <Text style={{ color: c.dim, fontSize: 11, marginBottom: 16 }}>
        {canFd ? t('CAN-FD PDU 포트 제어 · 시스템 재부팅 (pdu/fd · system/reboot)') : t('PDU 전원 레일 제어 · 시스템 재부팅 (pdu/power · system/reboot)')}{pdu || canFd ? '' : t(' — 레일 상태 수신 대기 중…')}
      </Text>
      {canFd && <FdPowerRails level="l2" withStatus />}
      {!canFd && PDU_RAILS.map((r) => r.danger ? (
        <DriveRailRow key={r.port} nm={t(r.nm)} sub={t(r.sub)} on={pdu ? pdu[r.bit] : false} volts={null}
          blocked={!pdu} blockReason={t('레일 상태 수신 전 — 조작 불가')} allowSkip
          fire={(next) => actions.pduPower(ip, r.port, next).catch(() => {})} request={setConfirmReq} />
      ) : (
        <TRow key={r.port} nm={t(r.nm)} sub={t(r.sub)}
          right={<Toggle value={pdu ? pdu[r.bit] : false} disabled={!pdu} onChange={(v) => actions.pduPower(ip, r.port, v).catch(() => {})} />} />
      ))}
      {!canFd && <Text style={{ color: c.dim, fontSize: 10, marginTop: 6, marginBottom: 10 }}>
        {t('토글은 로봇 PDU의 실제 상태(Motion TCP)를 표시 — 반영에 1초 정도 걸릴 수 있습니다.')}
      </Text>}
      {!canFd && <Text style={{ color: c.redbright, fontSize: 11, fontWeight: '700', marginTop: 4, marginBottom: 2 }}>
        {t('⚠ 48V 구동 레일 — 위험 조작 (#23)')}
      </Text>}
      {!canFd && DRIVE_RAILS.map((r) => {
        const on = pdu ? pdu[r.bit] : false;
        const legBlocked = !!r.legGuard && on && !LEG_OFF_OK.includes(gait);
        return (
          <DriveRailRow key={r.port} nm={t(r.nm)} sub={t(r.sub)} on={on} allowSkip
            volts={pdu?.rails?.[r.railKey]?.v ?? null}
            blocked={!pdu || legBlocked}
            blockReason={!pdu ? t('레일 상태 수신 전 — 조작 불가') : t('현재 {gait} — 비기립(SITTING/CONTROL_OFF/FALL)에서만 끌 수 있음').replace('{gait}', gait)}
            fire={(next) => actions.pduPower(ip, r.port, next).catch(() => {})} request={setConfirmReq} />
        );
      })}
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 16, marginBottom: 2 }}>
        {t('센서 캘리브 (IMU 드리프트 현장 보정)')}
      </Text>
      <CalibRow nm={t('가속도계 보정 (ACC)')} sub={t('앉은 상태에서 평지에 두고 실행 — 약 2초')}
        blocked={sitGuard.blocked} blockReason={sitGuard.reason} allow={sitImuCalibAllowed}
        confirmMsg={t('로봇이 앉은 상태에서만 실행할 수 있습니다. 2초 동안 중력 크기를 평균 내 보정값을 저장합니다 — 그동안 로봇을 건드리지 마세요.')}
        confirmExtra={<SitRobotHint />}
        fire={() => setAccOpen(true)} request={setConfirmReq} />
      <CalibRow nm={t('IMU 롤/피치 영점')} sub={t('현재 자세를 수평 기준으로 영점 — 약 0.5초, 자세 유지')}
        blocked={calib.blocked} blockReason={calib.reason}
        confirmMsg={t('지금 자세가 수평 기준이 됩니다. 기울어진 상태로 실행하면 이후 제어가 전부 틀어집니다.')}
        confirmExtra={<ImuLevelHint />}
        onRequest={commissioning.standIfStanding}
        fire={() => setImuOpen(true)} request={setConfirmReq} />
      <Text style={{ color: c.amberTx, fontSize: 10, marginTop: 4, marginBottom: 8 }}>
        ⚠ {t('보정 명령은 로봇 정지 상태에서만 — 주행 중 실행 금지')}
      </Text>
      <TRow nm={t('시스템 재부팅')} sub={t('로봇 PC를 재시작합니다')}
        right={
          <Tappable onPress={() => {
            const skipKey = 'system:reboot';
            const run = () => actions.reboot(ip).catch(() => {});
            if (confirmSkipped(skipKey)) { run(); return; }
            setConfirmReq({ title: t('로봇 PC를 재부팅할까요?'), message: t('재부팅 동안 로봇 연결이 끊깁니다'), confirmLabel: t('재부팅'), danger: true, skipKey, run });
          }} style={[styles.action, { backgroundColor: 'rgba(231,51,28,0.85)', borderColor: c.dangerLine, borderRadius: 10 }]}>
            <Icon name="power" size={14} color="#fff" />
            <Text style={{ color: '#fff', fontSize: 12, fontWeight: '600' }}>{t('재부팅')}</Text>
          </Tappable>
        } />
      {accOpen && <AccCalibModal onClose={() => setAccOpen(false)} />}
      {imuOpen && <ImuNullModal onClose={() => setImuOpen(false)} />}
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger} skipKey={confirmReq.skipKey}
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
