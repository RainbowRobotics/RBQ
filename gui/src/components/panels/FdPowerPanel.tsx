import { useEffect, useRef, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Toggle } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { useFocusedInterval } from '@/lib/useFocusedInterval';
import { t } from '@/lib/i18n';
import { TRow, DriveRailRow, ZoneSection, SecGap, ZONE_ORDER, LEG_OFF_OK, askOffToggle, useActBtn, type ConfirmReq } from '@/components/panels/PowerPanel';
import { H2, Desc, Group, useLegible } from '@/components/panels/settings/common';
import { LedBottomGroup } from '@/components/panels/LedBottomGroup';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { notify } from '@/components/DoneToast';
import { normalizePduFd, FD_PORT, FD_PORTS, SIDE_HUBS, SIDE_HUB_RESULT, sideHubRaw, batteryFaults, type PduFdState, type FdPortDef, type FdPortGroup, type FdSideHub } from '@/lib/pduFd';

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

function stateLabel(s: 1 | 0 | -1) { return s === 1 ? 'ON' : s === -1 ? 'ERROR' : 'OFF'; }
const PORT_IN_LABEL: Record<number, string> = { 0: 'EMO 입력', 1: '충전 입력 (스테이션)', 2: '충전 입력 (외부 포트)' };

export function FdPowerRails() {
  const { c, fonts } = useTheme();
  const lg = useLegible();
  const ip = useRobot((s) => s.ip);
  const gait = useRobot((s) => s.gait);
  const fd = usePduFd();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const fire = (p: FdPortDef, next: boolean) => actions.pduFdPort(ip, p.index, next).catch(() => {});

  const row = (p: FdPortDef) => {
    const po = fd?.portOut[p.index];
    const on = po?.state === 1;
    const err = po?.state === -1;
    const readback = po ? `${stateLabel(po.state)} ${po.voltage.toFixed(1)}V ${po.current.toFixed(2)}A` : '';
    if (p.readOnly) {
      return (
        <TRow key={p.index} nm={t(p.nm)} sub={t(p.sub)}
          right={<Text style={{ color: on ? c.greenTx : c.dim, fontSize: lg ? 12 : 10.5, fontFamily: fonts.mono }}>{readback}</Text>} />
      );
    }
    if (p.danger) {
      const legBlocked = !!p.legGuard && on && !LEG_OFF_OK.includes(gait);
      return (
        <DriveRailRow key={p.index} nm={t(p.nm)} sub={err ? `${t(p.sub)} · ${readback}` : t(p.sub)} on={on}
          volts={po ? po.voltage : null}
          blocked={!fd || legBlocked}
          blockReason={!fd ? t('포트 상태 수신 전 — 조작 불가') : t('현재 {gait} — 비기립(SITTING/CONTROL_OFF/FALL)에서만 끌 수 있음').replace('{gait}', gait)}
          fire={(next) => fire(p, next)} request={setConfirmReq} />
      );
    }
    return (
      <TRow key={p.index} nm={t(p.nm)} sub={po ? `${t(p.sub)} · ${readback}` : t(p.sub)} subColor={err ? c.redbright : undefined}
        right={<Toggle value={on} disabled={!fd} onChange={p.askOff ? askOffToggle(t(p.nm), (v) => fire(p, v), setConfirmReq) : (v) => fire(p, v)} />} />
    );
  };

  const hubRows = SIDE_HUBS.map((h) => (
    <HubResetRow key={`hub-${h.side}`} def={h} hub={fd?.sideHub.find((x) => x.side === h.side)} ready={!!fd} request={setConfirmReq} />
  ));
  const group = (g: FdPortGroup) => (
    <View key={g}>
      <ZoneSection zone={g}>{FD_PORTS.filter((p) => p.group === g).flatMap((p) => (p.index === FD_PORT.CAMERA_5 ? [row(p), ...hubRows] : [row(p)]))}</ZoneSection>
      <SecGap />
    </View>
  );

  return (
    <>
      {ZONE_ORDER.map(group)}
      <LedBottomGroup />
      <FdPowerStatus fd={fd} />
      <SecGap />
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)} />
      )}
    </>
  );
}

function HubResetRow({ def, hub, ready, request }: { def: (typeof SIDE_HUBS)[number]; hub?: FdSideHub; ready: boolean; request: (r: ConfirmReq) => void }) {
  const { c, radius } = useTheme();
  const btn = useActBtn();
  const ip = useRobot((s) => s.ip);
  const [wait, setWait] = useState<number | null>(null);
  const resultText = (h: FdSideHub) => t(SIDE_HUB_RESULT[h.result] ?? '알 수 없는 결과');
  useEffect(() => {
    if (wait === null || !hub || hub.running || hub.seq === wait) return;
    setWait(null);
    notify(`${t(def.nm)} — ${resultText(hub)}`, hub.result === 0 ? 'ok' : 'warn');
  }, [hub?.seq, hub?.running, wait]);
  useEffect(() => {
    if (wait === null) return;
    const id = setTimeout(() => { setWait(null); notify(`${t(def.nm)} — ${t('응답 없음(10초)')}`, 'warn'); }, 10000);
    return () => clearTimeout(id);
  }, [wait]);
  const busy = wait !== null || !!hub?.running;
  const fire = () => {
    setWait(hub ? hub.seq : -1);
    actions.pduFdHubReset(ip, def.side).catch((e: any) => { setWait(null); notify(String(e?.message ?? e), 'warn'); });
  };
  const ask = () => request({
    title: t('허브를 리셋할까요?'),
    message: `${t(def.nm)} — ${t('{cams} 영상이 잠시 끊깁니다. 보드가 허브를 0.5초 붙들었다가 다시 붙입니다.').replace('{cams}', t(def.cams))}`,
    confirmLabel: t('리셋'), danger: true, run: fire,
  });
  const done = !!hub && hub.seq > 0;
  const sub = done ? `${resultText(hub!)} · ${sideHubRaw(hub!)}` : t('{cams} 영상이 잠시 끊김 — 보드가 USB 허브를 SMBus 로 다시 시작').replace('{cams}', t(def.cams));
  const off = !ready || busy;
  return (
    <TRow nm={t(def.nm)} sub={sub} subColor={done && hub!.result !== 0 ? c.redbright : undefined}
      right={
        <Tappable disabled={off} onPress={ask}
          style={[btn.box, { flexDirection: 'row', gap: 6, borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.10)', borderRadius: radius.sm, opacity: off ? 0.5 : 1 }]}>
          <Icon name="recover" size={btn.icon} color={c.amber} />
          <Text style={{ color: c.amber, fontSize: btn.font, fontWeight: btn.weight }}>{busy ? t('리셋 중…') : t('리셋')}</Text>
        </Tappable>
      } />
  );
}

export function FdPowerStatus({ fd }: { fd: PduFdState | null }) {
  const { c, fonts } = useTheme();
  const lg = useLegible();
  const bat = fd?.battery ?? [];
  const temps = fd?.temperature ?? [];
  const ins = fd?.portIn ?? [];
  return (
    <View>
      <H2>{t('입력 포트 · 배터리 · 온도')} <Text style={{ color: c.muted, fontSize: 11 }}>Inputs · Battery</Text></H2>
      <Desc>{fd ? t('충전 입력 · 배터리 두 팩 · 보드 온도 — 표시만 합니다.') : t('포트 상태 수신 대기 중…')}</Desc>
      <Group>
      {ins.map((p) => (
        <TRow key={p.index} nm={t(PORT_IN_LABEL[p.index] ?? p.name)} sub={`${p.voltage.toFixed(1)}V ${p.current.toFixed(2)}A`}
          right={<Text style={{ color: p.state === -1 ? c.redbright : p.state === 1 ? c.greenTx : c.dim, fontSize: lg ? 12 : 10.5, fontFamily: fonts.mono }}>
            {stateLabel(p.state)}
          </Text>} />
      ))}
      {bat.map((b) => {
        const faults = batteryFaults(b);
        const nm = b.index === 0 ? t('배터리 좌') : t('배터리 우');
        const sub = b.detect ? `${b.voltage.toFixed(1)}V ${b.current.toFixed(2)}A · SOC ${b.soc.toFixed(0)}%` : t('미장착');
        return (
          <TRow key={b.index} nm={nm} sub={sub}
            right={<Text style={{ color: faults.length ? c.redbright : c.greenTx, fontSize: lg ? 12 : 10.5, fontFamily: fonts.mono, fontWeight: faults.length ? '700' : '400' }}>
              {faults.length ? faults.join(' ') : b.detect ? 'OK' : '--'}
            </Text>} />
        );
      })}
      {temps.length > 0 && (
        <TRow nm={t('온도 (°C)')} sub={t('배터리 좌 / 우 · 상판 전원 · PDU 전원 · PDU 신호')}
          right={<Text style={{ color: temps.some((x) => x.value >= 70) ? c.redbright : c.text, fontSize: lg ? 12 : 10.5, fontFamily: fonts.mono }}>
            {temps.map((x) => x.value.toFixed(0)).join(' / ')}
          </Text>} />
      )}
      </Group>
    </View>
  );
}
