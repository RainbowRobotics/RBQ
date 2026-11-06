import { useSettings } from '@/store/settings';
import { useState, useEffect } from 'react';
import { View, Text, Pressable, ActivityIndicator, useWindowDimensions } from 'react-native';
import { Modal } from '@/components/ui/overlays';
import { Tappable } from '@/components/anim';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { useRobots } from '@/store/robots';
import { useTelemetry } from '@/store/telemetry';
import { simEngine } from '@/lib/simEngine';
import { useSimulatedRobot } from '@/lib/simulatedRobot';
import { ARM_ALREADY_STARTED, ARM_START_RETRY_MS, armStartBlockReason, armStartFeedback, armStartRequests, requestArmAutoStart, type ArmStartContext } from '@/lib/armAutoStart';
import { t } from '@/lib/i18n';

function currentContext(acknowledgedRunning = false): ArmStartContext {
  const r = useRobot.getState(), telemetry = useTelemetry.getState();
  return { accessLevel: useSettings.getState().accessLevel, connected: r.conn === 'connected', switching: useRobots.getState().switching,
    owner: r.owner, isMine: r.isMine, robot: telemetry.robot, pdu:telemetry.pdu, lastRobotAt: telemetry.lastRobotAt,
    robotKey: useRobots.getState().currentSerial || r.ip,
    isSim: simEngine.active || useRobots.getState().currentSerial === 'sim',
    lastRobotStatusAt: r.lastRobotStatusAt, droppedAt: r.droppedAt,
    acknowledgedRunning,
    legStarting: !!r.robot?.autostart?.running };
}

export function ArmAutoStartModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { width } = useWindowDimensions();
  const telemetryRobot = useTelemetry(s => s.robot);
  const pdu = useTelemetry(s=>s.pdu);
  const conn = useRobot(s => s.conn);
  const [, setNow] = useState(Date.now());
  const [robotKey] = useState(() => currentContext().robotKey);
  const attempt = armStartRequests.get(robotKey);
  const [localPhase, setPhase] = useState<'idle' | 'sending' | 'sent' | 'unknown'>(attempt?.phase ?? 'idle');
  const phase = attempt?.phase ?? localPhase;
  const [error, setError] = useState('');
  const [details, setDetails] = useState(false);
  const [sentAt, setSentAt] = useState<number | null>(attempt?.at ?? null);
  const [settleFrom, setSettleFrom] = useState<number | null>(null);
  const [dropped, setDropped] = useState(false);
  const inFlight = sentAt !== null && Date.now() - sentAt < ARM_START_RETRY_MS;
  const retry = sentAt !== null && !inFlight;
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 500); return () => clearInterval(timer); }, []);
  const [ack, setAck] = useState(false);
  const context = currentContext(ack);
  const sameRobot = context.robotKey === robotKey && !context.switching;
  const robot = sameRobot ? telemetryRobot : undefined;
  const reason = armStartBlockReason(context, Date.now(), robotKey);
  const armRunning = armStartBlockReason(currentContext(false), Date.now(), robotKey) === ARM_ALREADY_STARTED;
  const disabled = !!reason || (inFlight && !ack);
  const start = async () => {
    if (inFlight) return;
    const current = currentContext(ack);
    const blocked = armStartBlockReason(current, Date.now(), robotKey);
    if (blocked) { setError(blocked); return; }
    setError(''); setPhase('sending'); setSentAt(Date.now());
    setSettleFrom(Date.now()); setDropped(false); setAck(false);
    try { await requestArmAutoStart(current, robotKey); setPhase('sent'); }
    catch (e) {
      setError(e instanceof Error ? e.message : String(e)); setPhase('unknown');
    }
  };
  const fresh = sameRobot && conn === 'connected' && !!context.lastRobotAt
    && Date.now() >= context.lastRobotAt && Date.now() - context.lastRobotAt <= 3000
    && context.lastRobotAt > (context.droppedAt ?? 0);
  const simulation=useSimulatedRobot();
  const powered=fresh&&(simulation||pdu?.fetAdd===true);
  const powerOff=fresh&&!simulation&&pdu?.fetAdd===false;
  const counts=armStartFeedback(robot,sameRobot?pdu:undefined,simulation);
  const motors = robot?.joints.slice(12, 18) ?? [];
  const settling = settleFrom !== null && !dropped && Date.now() - settleFrom < 4000;
  useEffect(() => { if (settleFrom !== null && counts[0] < 6) setDropped(true); }, [settleFrom, counts]);
  const stages = [
    { label: t('통신'), count: counts[0], reported: true },
    { label: t('모터 RUN'), count: counts[1], reported: !simulation },
    { label: t('제어 출력'), count: counts[2], reported: true },
  ];
  const status = !sameRobot ? t('대상 변경') : !fresh ? t('상태 수신 대기')
    : powerOff ? t('팔 전원 꺼짐') : !powered ? t('전원 상태 수신 대기')
    : settling ? t('재기동 중')
    : simulation ? t('시뮬레이션')
    : phase === 'sending' ? t('요청 중') : phase === 'unknown' ? t('응답 미확인')
    : phase === 'sent' ? t('요청 접수') : t('기동 준비');
  const notice = !sameRobot ? t('연결 대상이 바뀌었습니다. 창을 닫고 다시 확인하세요.')
    : !fresh ? t('최신 로봇 상태를 기다리고 있습니다.')
    : powerOff ? t('팔 전원이 꺼져 있습니다. 정비에서 전원과 제어 상태를 확인하세요.')
    : !powered ? t('전원 상태 수신 대기')
    : error || (phase === 'idle' || retry ? reason : null)
    || (retry ? t('이 앱에서 이미 팔 기동을 요청했습니다. 실제 상태와 로그를 확인하세요.') : null)
    || (simulation ? t('시뮬레이션에는 PDU와 모터 보드가 없습니다. 모터 RUN은 보고되지 않고, 제어 출력은 지금 지령 중인 게인이며 1 미만 게인은 0으로 보고됩니다.') : null);
  return <Modal onClose={onClose}>
    <View style={{ width: Math.min(460, width - 32), padding: 20, backgroundColor: c.panel,
      borderColor: c.line, borderWidth: 1, borderRadius: radius.lg, gap: 14 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 8 }}>
        <Text style={{ color: c.text, fontSize: 18, fontWeight: '700', flexGrow: 1 }}>{t('팔 자동 기동')}</Text>
        {phase === 'sending' && <ActivityIndicator size="small" color={c.accent} />}
        <Text accessibilityLiveRegion="polite" style={{ color: !fresh || phase === 'unknown' ? c.amberTx : c.accent2,
          fontSize: 12, backgroundColor: c.elev, paddingHorizontal: 10, paddingVertical: 5, borderRadius: radius.md }}>{status}</Text>
      </View>
      {phase === 'idle' && <Text style={{ color: c.muted, fontSize: 12 }}>{t('팔 주변을 비우고 로봇을 정지시킨 뒤 실행하세요.')}</Text>}
      <View style={{ flexDirection: 'row', gap: 6 }}>
        {stages.map(stage => {
          const ok = powered && stage.reported && !settling && stage.count === 6;
          const color = !powered || !stage.reported || settling ? c.dim : ok ? c.greenTx : stage.count ? c.amberTx : c.muted;
          const value = settling ? t('재기동 중') : !stage.reported ? t('시뮬 미보고') : powered ? `${stage.count}/6` : powerOff ? 'OFF' : '—';
          return <View key={stage.label} accessibilityLabel={`${stage.label}: ${value}`}
            style={{ flex: 1, minWidth: 0, paddingVertical: 12, paddingHorizontal: 4, alignItems: 'center', gap: 6,
              borderRadius: radius.md, backgroundColor: c.elev, borderWidth: 1, borderColor: ok ? c.greenTx : c.line }}>
            <Text style={{ color, fontSize: 18, fontWeight: '700' }}>{ok ? '✓' : !powered || !stage.reported || settling ? '—' : '○'}</Text>
            <Text style={{ color: c.text, fontSize: 12, textAlign: 'center' }}>{stage.label}</Text>
            <Text style={{ color, fontSize: stage.reported && !settling ? 20 : 11, fontWeight: '700' }}>{value}</Text>
          </View>;
        })}
      </View>
      <View style={{ gap: 8 }}>
        <Text style={{ color: c.muted, fontSize: 12 }}>{t('관절별 RUN 상태')}</Text>
        <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 6 }}>
          {Array.from({ length: 6 }, (_, i) => {
            const joint = motors[i];
            const live = powered && !settling && !!joint?.connected;
            const on = !simulation && live && joint.run;
            const label = settling ? t('재기동 중') : simulation ? t('시뮬 미보고') : powerOff ? 'OFF' : !live ? t('미확인') : on ? 'RUN' : t('대기');
            return <View key={i} accessibilityLabel={`J${i + 1}: ${label}`}
              style={{ flexBasis: '30%', flexGrow: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between',
                padding: 9, borderRadius: radius.md, backgroundColor: c.elev, gap: 4 }}>
              <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>J{i + 1}</Text>
              <Text style={{ color: !live ? c.dim : on ? c.greenTx : c.muted, fontSize: 11 }}>{on ? '● ' : '○ '}{label}</Text>
            </View>;
          })}
        </View>
      </View>
      {notice && <Text accessibilityRole="alert" style={{ color: c.amberTx, fontSize: 12 }}>{t(notice)}</Text>}
      {armRunning && <Pressable onPress={() => setAck(v => !v)} hitSlop={6} accessibilityRole="checkbox"
        accessibilityState={{ checked: ack }} style={{ flexDirection: 'row', alignItems: 'flex-start', gap: 8 }}>
        <View style={{ width: 16, height: 16, marginTop: 1, borderWidth: 1, borderRadius: 4, alignItems: 'center', justifyContent: 'center',
          borderColor: c.line, backgroundColor: ack ? c.accent : 'transparent' }}>
          {ack && <Text style={{ color: c.onAccent, fontSize: 11, fontWeight: '800', lineHeight: 13 }}>✓</Text>}
        </View>
        <Text style={{ color: c.muted, fontSize: 12, flex: 1, lineHeight: 17 }}>
          {t('다시 기동하면 팔 전원이 끊겼다 들어오며 그 동안 팔이 힘을 잃고 내려앉습니다. 팔을 안전한 자세로 두고 주변을 비운 뒤 확인하세요.')}</Text>
      </Pressable>}
      <Pressable onPress={() => setDetails(v => !v)} accessibilityRole="button"
        accessibilityState={{ expanded: details }} style={{ paddingVertical: 6 }}>
        <Text style={{ color: c.muted, fontSize: 12 }}>{details ? '▾ ' : '▸ '}{t('기동 안내')}</Text>
      </Pressable>
      {details && <View style={{ gap: 8 }}>
        <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{t('팔 하드웨어 기동을 요청합니다. RB1은 팔·애드온 전원 재인가와 브레이크 해제, RBA-1은 축별 초기화와 그리퍼 원점 찾기가 포함됩니다. 팔 주변을 비우고 로봇을 정지시킨 뒤 실행하세요.')}</Text>
        <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{t('제어 출력은 관절의 참조 게인 보고입니다. 모든 표시가 켜져도 그리퍼 원점 찾기나 전체 기동 완료를 뜻하지 않습니다.')}</Text>
        <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{t('수동제어 시작과 준비 자세 이동은 별도입니다. 이 버튼은 해당 동작을 자동 실행하지 않습니다.')}</Text>
        {phase !== 'idle' && <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{t('창을 닫아도 시작된 팔 기동은 중단되지 않습니다. 재실행 전 실제 상태를 확인하세요.')}</Text>}
        {attempt && <Text style={{ color: c.muted, fontSize: 12 }}>{t('이 앱에서 이미 팔 기동을 요청했습니다. 실제 상태와 로그를 확인하세요.')}</Text>}
      </View>}
      <View style={{ flexDirection: 'row', gap: 8 }}>
        <Tappable onPress={onClose} style={{ flex: 1, padding: 12, alignItems: 'center', borderRadius: radius.md, backgroundColor: c.elev }}>
          <Text style={{ color: c.text }}>{t('닫기')}</Text>
        </Tappable>
        <Tappable disabled={disabled} onPress={disabled ? undefined : start}
          style={{ flex: 1, padding: 12, alignItems: 'center', borderRadius: radius.md, backgroundColor: c.accent, opacity: disabled ? .4 : 1 }}>
          <Text style={{ color: c.onAccent, fontWeight: '700' }}>{t(sentAt !== null ? '팔 자동 기동 다시 실행' : '팔 자동 기동 실행')}</Text>
        </Tappable>
      </View>
    </View>
  </Modal>;
}
