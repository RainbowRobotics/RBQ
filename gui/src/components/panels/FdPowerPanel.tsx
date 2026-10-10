import { useEffect, useRef, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Toggle } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { useFocusedInterval } from '@/lib/useFocusedInterval';
import { t } from '@/lib/i18n';
import { TRow, DriveRailRow, type ConfirmReq } from '@/components/panels/PowerPanel';
import { LedBottomGroup } from '@/components/panels/LedBottomGroup';
import { normalizePduFd, fdPortsForLevel, batteryFaults, type PduFdState, type FdPortDef, type FdPortGroup } from '@/lib/pduFd';

const LEG_OFF_OK = ['SITTING', 'CONTROL_OFF', 'FALL_MODE'];
const POLL_MS = 1000;

export function usePduFd(enabled = true): PduFdState | null {
  const ip = useRobot((s) => s.ip);
  const [state, setState] = useState<PduFdState | null>(null);
  const gen = useRef(0);
  const busy = useRef(false);
  useEffect(() => { gen.current++; busy.current = false; setState(null); }, [ip, enabled]);
  useFocusedInterval(() => {
    if (busy.current) return;
    busy.current = true;
    const g = gen.current;
    actions.pduFd(ip)
      .then((r) => { if (g === gen.current) setState(Array.isArray(r?.pdu_fd?.port_out) ? normalizePduFd(r) : null); })
      .catch(() => { if (g === gen.current) setState(null); })
      .finally(() => { if (g === gen.current) busy.current = false; });
  }, POLL_MS, enabled, () => { gen.current++; busy.current = false; setState(null); });
  return state;
}

const GROUP_TITLE: Record<FdPortGroup, string> = {
  '48v': '⚠ 48V 구동 레일 — 위험 조작',
  '12v': '12V 포트',
  camera: '카메라 전원 (USB)',
  audio: '오디오',
};
const GROUP_ORDER: FdPortGroup[] = ['12v', 'camera', 'audio', '48v'];

function stateLabel(s: 1 | 0 | -1) { return s === 1 ? 'ON' : s === -1 ? 'ERROR' : 'OFF'; }
const PORT_IN_LABEL: Record<number, string> = { 0: 'EMO 입력', 1: '충전 입력 (스테이션)', 2: '충전 입력 (외부 포트)' };

export function FdPowerRails({ level, withStatus }: { level: 'l1' | 'l2'; withStatus?: boolean }) {
  const { c, fonts } = useTheme();
  const ip = useRobot((s) => s.ip);
  const gait = useRobot((s) => s.gait);
  const fd = usePduFd();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const ports = fdPortsForLevel(level);
  const fire = (p: FdPortDef, next: boolean) => actions.pduFdPort(ip, p.index, next).catch(() => {});

  const row = (p: FdPortDef) => {
    const po = fd?.portOut[p.index];
    const on = po?.state === 1;
    const err = po?.state === -1;
    const readback = po ? `${stateLabel(po.state)} ${po.voltage.toFixed(1)}V ${po.current.toFixed(2)}A` : '';
    if (p.readOnly) {
      return (
        <TRow key={p.index} nm={t(p.nm)} sub={t(p.sub)}
          right={<Text style={{ color: on ? c.greenTx : c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>{readback}</Text>} />
      );
    }
    if (p.danger) {
      const legBlocked = !!p.legGuard && on && !LEG_OFF_OK.includes(gait);
      return (
        <DriveRailRow key={p.index} nm={t(p.nm)} sub={err ? `${t(p.sub)} · ${readback}` : t(p.sub)} on={on}
          volts={po ? po.voltage : null} allowSkip={level === 'l2'}
          blocked={!fd || legBlocked}
          blockReason={!fd ? t('포트 상태 수신 전 — 조작 불가') : t('현재 {gait} — 비기립(SITTING/CONTROL_OFF/FALL)에서만 끌 수 있음').replace('{gait}', gait)}
          fire={(next) => fire(p, next)} request={setConfirmReq} />
      );
    }
    return (
      <TRow key={p.index} nm={t(p.nm)} sub={po ? `${t(p.sub)} · ${readback}` : t(p.sub)} subColor={err ? c.redbright : undefined}
        right={<Toggle value={on} disabled={!fd} onChange={(v) => fire(p, v)} />} />
    );
  };

  const group = (g: FdPortGroup) => {
    const list = ports.filter((p) => p.group === g);
    if (!list.length) return null;
    const danger = g === '48v';
    return (
      <View key={g}>
        <Text style={{ color: danger ? c.redbright : c.dim, fontSize: 11, fontWeight: '700', marginTop: 10, marginBottom: 2 }}>
          {t(GROUP_TITLE[g])}
        </Text>
        {list.map(row)}
      </View>
    );
  };

  return (
    <>
      {GROUP_ORDER.filter((g) => g !== '48v').map(group)}
      <LedBottomGroup />
      {group('48v')}
      <Text style={{ color: c.dim, fontSize: 10, marginTop: 6, marginBottom: 6 }}>
        {t('상태·전압·전류는 로봇 PDU(CAN-FD)의 실측값 — 반영에 1초 정도 걸릴 수 있습니다.')}
      </Text>
      {withStatus && <FdPowerStatus fd={fd} />}
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger} skipKey={confirmReq.skipKey}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)} />
      )}
    </>
  );
}

export function FdPowerStatus({ fd }: { fd: PduFdState | null }) {
  const { c, fonts } = useTheme();
  const bat = fd?.battery ?? [];
  const temps = fd?.temperature ?? [];
  const ins = fd?.portIn ?? [];
  return (
    <View>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 10, marginBottom: 2 }}>{t('입력 포트 · 배터리 · 온도')}</Text>
      {!fd && <Text style={{ color: c.dim, fontSize: 10.5 }}>{t('포트 상태 수신 대기 중…')}</Text>}
      {ins.map((p) => (
        <TRow key={p.index} nm={t(PORT_IN_LABEL[p.index] ?? p.name)} sub={`${p.voltage.toFixed(1)}V ${p.current.toFixed(2)}A`}
          right={<Text style={{ color: p.state === -1 ? c.redbright : p.state === 1 ? c.greenTx : c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>
            {stateLabel(p.state)}
          </Text>} />
      ))}
      {bat.map((b) => {
        const faults = batteryFaults(b);
        const nm = b.index === 0 ? t('배터리 좌') : t('배터리 우');
        const sub = b.detect ? `${b.voltage.toFixed(1)}V ${b.current.toFixed(2)}A · SOC ${b.soc.toFixed(0)}%` : t('미장착');
        return (
          <TRow key={b.index} nm={nm} sub={sub}
            right={<Text style={{ color: faults.length ? c.redbright : c.greenTx, fontSize: 10.5, fontFamily: fonts.mono, fontWeight: faults.length ? '700' : '400' }}>
              {faults.length ? faults.join(' ') : b.detect ? 'OK' : '--'}
            </Text>} />
        );
      })}
      {temps.length > 0 && (
        <TRow nm={t('온도 (°C)')} sub={t('배터리 좌 / 우 · 상판 전원 · PDU 전원 · PDU 신호')}
          right={<Text style={{ color: temps.some((x) => x.value >= 70) ? c.redbright : c.text, fontSize: 10.5, fontFamily: fonts.mono }}>
            {temps.map((x) => x.value.toFixed(0)).join(' / ')}
          </Text>} />
      )}
    </View>
  );
}
