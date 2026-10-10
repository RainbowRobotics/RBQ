import { useCallback, useEffect, useRef, useState } from 'react';
import { View, ScrollView } from 'react-native';
import { useFocusEffect } from 'expo-router';
import { Text as RbText } from '@/rb/native';
import { useRb } from '@/rb/theme';
import { RBTable, type TableColumn } from '@/rb/components/RBTable';
import { RBBadge } from '@/rb/components/RBBadge';
import { RBSpinner } from '@/rb/components/RBSpinner';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { H2, Desc, TRow, SBtn } from '@/components/panels/settings/common';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useDevMode } from '@/store/settings';
import { useFeatureFwUpdate, useFeatures } from '@/store/capability';
import { useBoardFw, refreshBoardFw, usePowerOffHint } from '@/store/boardFirmware';
import { actions } from '@/lib/rest';
import { t } from '@/lib/i18n';
import {
  summarize, neverChecked, fwChecking, fwStopped, fwCommonGuard, fwAllGuard, fwAsk, sameAsk, fwReadyText, boardUpdatable, bootRecover,
  variantFix, fwConfirmLines, fwRunRows, fwRunResult, fwPostError, runKindLabel, fwReasonText, modeLabel, modeTone, hwLabel,
  hwStateText, verdictLabel, verdictTone, stepLabel, fmtVer, reachable, motorGroup, motorsUpdatable, motorsCommon,
  motorVerdictLabel, motorVerdictTone, motorModeLabel, FW_BODY_SLOTS, FW_RETRY_MAX,
  type FwAsk, type FwBoard, type FwStatus, type FwGuardInput, type FwRunRow, type FwTarget, type FwMotorGroup,
} from '@/lib/boardFirmware';

const ATTN_VERDICT: ReadonlySet<string> = new Set(['downgrade', 'no_file', 'hw_unknown']);
const CHECK_WAIT_MS = 15_000;

const fmtTime = (ms: number) => new Date(ms).toTimeString().slice(0, 8);

function guardInputNow(): FwGuardInput {
  const r = useRobot.getState();
  const robot = useTelemetry.getState().robot;
  return {
    connected: r.conn === 'connected',
    featureOn: !!useFeatures.getState().features?.fw_update,
    status: useBoardFw.getState().status,
    gaitId: robot ? robot.gaitId : null,
    controlOn: !!robot?.joints?.some((j) => j.connected && j.run) || !!r.robot?.autostart?.running,
    otherOwner: !!r.owner && !r.isMine,
  };
}

function useGuardInput(): FwGuardInput {
  const connected = useRobot((s) => s.conn === 'connected');
  const featureOn = useFeatureFwUpdate();
  const status = useBoardFw((s) => s.status);
  const gaitId = useTelemetry((s) => (s.robot ? s.robot.gaitId : null));
  const jointRun = useTelemetry((s) => !!s.robot?.joints?.some((j) => j.connected && j.run));
  const autostart = useRobot((s) => !!s.robot?.autostart?.running);
  const otherOwner = useRobot((s) => !!s.owner && !s.isMine);
  return { connected, featureOn, status, gaitId, controlOn: jointRun || autostart, otherOwner };
}

export function BoardFirmwarePanel() {
  const dev = useDevMode();
  const ip = useRobot((s) => s.ip);
  const gin = useGuardInput();
  const powerHint = usePowerOffHint();
  const [ask, setAsk] = useState<FwAsk | null>(null);
  const [msg, setMsg] = useState<string | null>(null);
  const msgAt = useRef<'sum' | 'table'>('sum');
  const [sending, setSending] = useState(false);
  const [checkWait, setCheckWait] = useState<{ seq: number; until: number } | null>(null);

  useFocusEffect(useCallback(() => {
    useBoardFw.getState().setPanelOpen(true);
    void refreshBoardFw();
    const id = setInterval(() => { void refreshBoardFw(); }, 1000);
    return () => { clearInterval(id); useBoardFw.getState().setPanelOpen(false); void refreshBoardFw(); };
  }, []));

  const seqNow = gin.status?.checkSeq ?? 0;
  const robotChecking = !!gin.status && fwChecking(gin.status);
  useEffect(() => {
    if (!checkWait) return;
    if (seqNow > checkWait.seq) { setCheckWait(null); return; }
    const id = setTimeout(() => {
      if (robotChecking) { setCheckWait({ ...checkWait, until: Date.now() + 5000 }); return; }
      setCheckWait(null);
      setMsg(t('버전 확인이 끝나지 않았습니다 — 로봇이 다른 작업을 하는 중일 수 있습니다. 잠시 뒤 다시 누르세요.'));
    }, Math.max(0, checkWait.until - Date.now()));
    return () => clearTimeout(id);
  }, [checkWait, seqNow, robotChecking]);

  const check = () => {
    msgAt.current = 'sum';
    setMsg(null);
    setCheckWait({ seq: seqNow, until: Date.now() + CHECK_WAIT_MS });
    actions.boardFwCheck(ip)
      .then((r) => {
        const seq = r?.check_seq;
        if (typeof seq === 'number') setCheckWait((w) => (w ? { ...w, seq } : w));
        return refreshBoardFw();
      })
      .catch((e: unknown) => { setCheckWait(null); setMsg(fwPostError(e).text); });
  };

  const openAsk = (target: FwTarget) => {
    msgAt.current = target === 'all' ? 'sum' : 'table';
    const g = target === 'all' ? fwAllGuard(gin) : fwCommonGuard(gin);
    if (g.blocked || !gin.status) return;
    const a = fwAsk(target, gin.status, g.needPowerDown);
    if (!a) return;
    setMsg(null);
    setAsk(a);
  };
  const askNow = (prev: FwAsk): FwAsk | null => {
    const now = guardInputNow();
    const g = prev.target === 'all' ? fwAllGuard(now) : fwCommonGuard(now);
    if (g.blocked || !now.status) { setMsg(g.reason); return null; }
    const a = fwAsk(prev.target, now.status, prev.powerDown || g.needPowerDown);
    if (!a) setMsg(t('업데이트할 보드가 없습니다'));
    return a;
  };
  const send = (a: FwAsk) => {
    setSending(true);
    const prevRun = useBoardFw.getState().status?.run.id ?? 0;
    actions.boardFwUpdate(ip, a.target, a.boards.some((b) => b.verdict === 'downgrade'), a.powerDown, a.checkSeq)
      .then(() => { useBoardFw.getState().noteSent(prevRun, a.powerDown); return refreshBoardFw(); })
      .catch((e: unknown) => {
        const r = fwPostError(e);
        if (r.changed) {
          void refreshBoardFw().then(() => { const n = askNow(a); if (n) setAsk({ ...n, changed: true }); });
          return;
        }
        if (r.needPowerDown && !a.powerDown) { setAsk({ ...a, powerDown: true }); return; }
        setMsg(r.text);
      })
      .finally(() => setSending(false));
  };
  const confirm = () => {
    const a = ask;
    setAsk(null);
    if (!a) return;
    const now = askNow(a);
    if (!now) return;
    if (!sameAsk(now, a)) { setAsk({ ...now, changed: true }); return; }
    send(now);
  };

  const showBoards = dev && gin.connected && gin.featureOn && !!gin.status?.supported;
  const lines = ask ? fwConfirmLines(ask) : [];
  const checking = !!checkWait || robotChecking;
  return (
    <View style={{ flex: 1 }}>
      <ScrollView showsVerticalScrollIndicator={false}>
        <SummarySection gin={gin} onUpdateAll={() => openAsk('all')} onCheck={check} checking={checking} sending={sending}
          msg={msgAt.current === 'sum' ? msg : null} powerHint={powerHint} />
        {showBoards ? <BoardsSection gin={gin} onUpdate={openAsk} sending={sending} msg={msgAt.current === 'table' ? msg : null} /> : null}
      </ScrollView>
      {ask ? (
        <ConfirmModal title={t('보드 펌웨어 업데이트')}
          message={lines[0]}
          confirmLabel={ask.powerDown ? t('전원 내리고 업데이트') : t('업데이트')}
          onConfirm={confirm} onClose={() => setAsk(null)}>
          <ConsentLines lines={lines.slice(1)} />
        </ConfirmModal>
      ) : null}
    </View>
  );
}

function ConsentLines({ lines }: { lines: string[] }) {
  const { c, t: ty } = useRb();
  return (
    <View style={{ gap: 8, maxWidth: 460 }}>
      {lines.map((l) => (
        <View key={l} style={{ flexDirection: 'row', gap: 8, alignItems: 'flex-start' }}>
          <View style={{ paddingTop: 2 }}><Icon name="warn" size={14} color={c('fg-warning')} /></View>
          <RbText style={[ty('body-sm-normal'), { color: c('fg-default'), flex: 1 }]}>{l}</RbText>
        </View>
      ))}
    </View>
  );
}


function SummarySection({ gin, onUpdateAll, onCheck, checking, sending, msg, powerHint }: {
  gin: FwGuardInput; onUpdateAll: () => void; onCheck: () => void; checking: boolean; sending: boolean; msg: string | null; powerHint: boolean;
}) {
  const checkedAt = useBoardFw((s) => s.checkedAt);
  const status = gin.status;
  const head = (
    <>
      <H2>{t('보드 펌웨어')}</H2>
      <Desc>{t('로봇 안 보드들의 펌웨어를 로봇에 있는 최신 파일로 올립니다.')}</Desc>
    </>
  );
  if (!gin.connected || !gin.featureOn || !status || !status.supported) {
    const reason = fwCommonGuard(gin).reason;
    return (
      <>
        {head}
        <FetchLost />
        <TRow nm={t('상태')} sub={reason} right={<Blocked><SBtn kind="ghost" icon="search" label={t('버전 확인')} disabled /></Blocked>} />
        <TRow nm={t('전체 업데이트')} sub={reason} right={<Blocked><SBtn kind="danger" icon="download" label={t('전체 업데이트')} disabled /></Blocked>} />
        <PowerNote />
      </>
    );
  }
  const all = fwAllGuard(gin);
  const sm = summarize(status);
  const runBusy = status.run.state === 'running' || status.busy === 'update';
  const moving = gin.gaitId != null && !fwStopped(gin.gaitId, gin.controlOn);
  const counts = fwChecking(status) ? t('버전을 확인하는 중입니다')
    : neverChecked(status) ? t('아직 버전을 확인하지 않았습니다')
    : [
      `${t('업데이트 가능')} ${sm.updatable}`,
      `${t('최신')} ${sm.latest}`,
      `${t('확인 필요')} ${sm.needCheck.length}`,
      ...(sm.noResponse ? [`${t('응답 없음')} ${sm.noResponse}`] : []),
      ...(sm.unmanaged ? [`${t('대상 아님')} ${sm.unmanaged}`] : []),
      ...(sm.motorsLater ? [`${t('모터')} ${t('다리 꺼짐')}`] : []),
      ...(sm.motors.mixed ? [t('모터 펌웨어 섞임')] : []),
      checkedAt ? `${t('마지막 확인')} ${fmtTime(checkedAt)}` : t('연결 전에 확인됨'),
    ].join(' · ');
  const checkBtn = <SBtn kind="ghost" icon="search" label={checking ? t('확인 중…') : t('버전 확인')}
    disabled={!status.supported || runBusy || checking || moving} onPress={onCheck} />;
  return (
    <>
      {head}
      <FetchLost />
      <RunBlock status={status} powerHint={powerHint} />
      <TRow nm={t('상태')} sub={moving ? `${counts} · ${t('구동 중에는 버전을 확인할 수 없습니다')}` : counts}
        right={moving ? <Blocked>{checkBtn}</Blocked> : checkBtn} />
      {all.running ? null : (
        <TRow nm={t('전체 업데이트')} sub={all.blocked ? all.reason : fwReadyText(sm)}
          right={all.blocked
            ? <Blocked><SBtn kind="danger" icon="download" label={t('전체 업데이트')} disabled /></Blocked>
            : <SBtn kind="danger" icon="download" label={sending ? t('시작하는 중…') : t('전체 업데이트')} disabled={sending} onPress={onUpdateAll} />} />
      )}
      {msg ? <Msg text={msg} /> : null}
      <PowerNote />
    </>
  );
}

function Blocked({ children }: { children: React.ReactNode }) {
  const { c } = useRb();
  return (
    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, opacity: 0.45 }}>
      <Icon name="warn" size={12} color={c('fg-danger')} />
      {children}
    </View>
  );
}

function Msg({ text, warn }: { text: string; warn?: boolean }) {
  const { c, t: ty } = useRb();
  const tone = warn ? c('fg-warning') : c('fg-danger');
  return (
    <View style={{ flexDirection: 'row', gap: 6, alignItems: 'flex-start', marginTop: 8, marginBottom: warn ? 6 : 0 }}>
      <View style={{ paddingTop: 1 }}><Icon name="warn" size={13} color={tone} /></View>
      <RbText style={[ty('body-xs-normal'), { color: warn ? c('fg-default') : tone, flex: 1 }]}>{text}</RbText>
    </View>
  );
}

function FetchLost() {
  const err = useBoardFw((s) => s.error);
  const at = useBoardFw((s) => s.at);
  const has = useBoardFw((s) => !!s.status);
  if (!err) return null;
  return <Msg warn text={has && at
    ? `${t('{time} 이후 보드 펌웨어 상태를 받지 못하고 있습니다').replace('{time}', fmtTime(at))} — ${err}`
    : `${t('보드 펌웨어 상태를 받지 못했습니다')} — ${err}`} />;
}

function PowerNote() {
  const { c, t: ty } = useRb();
  const { radius } = useTheme();
  return (
    <View style={{ flexDirection: 'row', gap: 8, marginTop: 14, padding: 10, borderWidth: 1, borderColor: c('border-subtle'), borderRadius: radius.sm }}>
      <View style={{ paddingTop: 1 }}><Icon name="power" size={13} color={c('fg-warning')} /></View>
      <RbText style={[ty('body-xs-normal'), { color: c('fg-subtle'), flex: 1 }]}>
        {t('전체 업데이트는 다리·팔 전원을 끄고 진행하며, 끝나도 꺼진 채로 남습니다 — 끝나면 다시 기동하세요. PDU를 올릴 때 UPC 전원이, TOP을 올릴 때 12V 포트가 잠시 꺼집니다.')}
      </RbText>
    </View>
  );
}


function RunBlock({ status, powerHint }: { status: FwStatus; powerHint: boolean }) {
  const { c, t: ty } = useRb();
  const { radius } = useTheme();
  const running = status.run.state === 'running';
  const res = running ? null : fwRunResult(status, powerHint);
  if (!running && !res) return null;
  const rows = fwRunRows(status, powerHint);
  const tone = running ? c('fg-information') : res!.ok ? c('fg-success') : c('fg-danger');
  return (
    <View style={{ borderWidth: 1, borderColor: c('border-subtle'), borderRadius: radius.md, backgroundColor: c('bg-card'), paddingHorizontal: 14, paddingVertical: 12, marginBottom: 10, gap: 4 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 4 }}>
        {running ? <RBSpinner size={14} color={tone} />
          : res!.ok ? <RbText style={[ty('body-sm-semibold'), { color: tone }]}>✓</RbText>
          : <Icon name="warn" size={14} color={tone} />}
        <RbText style={[ty('body-sm-semibold'), { color: running ? c('fg-default') : tone, flex: 1 }]}>
          {running ? `${t('업데이트 진행 중')} — ${runKindLabel(status)}` : res!.title}
        </RbText>
      </View>
      {res?.text ? <RbText style={[ty('body-sm-normal'), { color: c('fg-default'), marginBottom: 2 }]}>{res.text}</RbText> : null}
      {res?.power ? (
        <View style={{ flexDirection: 'row', gap: 6, alignItems: 'center', marginBottom: 6 }}>
          <Icon name="power" size={13} color={c('fg-warning')} />
          <RbText style={[ty('body-sm-normal'), { color: c('fg-default'), flex: 1 }]}>{res.power}</RbText>
        </View>
      ) : null}
      {rows.map((r) => <StepRow key={r.key} row={r} />)}
      {running ? (
        <RbText style={[ty('body-xs-normal'), { color: c('fg-subtle'), marginTop: 6 }]}>{t('이 화면을 닫아도 로봇에서 계속 진행됩니다.')}</RbText>
      ) : null}
    </View>
  );
}

function StepRow({ row }: { row: FwRunRow }) {
  const { c, t: ty } = useRb();
  const dim = row.state === 'pending' || row.state === 'skipped';
  const mark = row.state === 'done' ? <RbText style={[ty('body-sm-semibold'), { color: c('fg-success') }]}>✓</RbText>
    : row.state === 'active' ? <RBSpinner size={12} color={c('fg-information')} />
    : row.state === 'failed' ? <Icon name="x" size={13} color={c('fg-danger')} />
    : <RbText style={[ty('body-sm-normal'), { color: c('fg-subtlest') }]}>{row.state === 'skipped' ? '–' : '·'}</RbText>;
  return (
    <View style={{ paddingVertical: 3 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
        <View style={{ width: 16, alignItems: 'center' }}>{mark}</View>
        <RbText style={[ty('body-sm-normal'), { color: dim ? c('fg-subtler') : row.state === 'failed' ? c('fg-danger') : c('fg-default'), minWidth: 84 }]}>{row.label}</RbText>
        {row.note ? <RbText numberOfLines={1} style={[ty('compact-xs-normal'), { color: c('fg-subtle'), flexShrink: 1 }]}>{row.note}</RbText> : null}
        {row.retry ? (
          <RBBadge type="capsule" color={row.state === 'failed' ? 'red' : 'orange'}>
            {t('재시도 {n}/{max}').replace('{n}', String(row.retry)).replace('{max}', String(FW_RETRY_MAX))}
          </RBBadge>
        ) : null}
      </View>
      {row.pct != null ? <Bar pct={row.pct} /> : null}
    </View>
  );
}

function Bar({ pct }: { pct: number }) {
  const { c } = useRb();
  return (
    <View style={{ height: 6, borderRadius: 3, backgroundColor: c('bg-alt-2'), overflow: 'hidden', marginLeft: 24, marginTop: 4 }}>
      <View style={{ height: 6, width: `${pct}%`, backgroundColor: c('fill-brand') }} />
    </View>
  );
}


type Row = { key: string; name: string; board?: FwBoard; group?: FwMotorGroup };

const COLS = { name: 76, mode: 104, hw: 132, app: 84, file: 84, verdict: 140, act: 112 };
const COLS_SUM = Object.values(COLS).reduce((a, b) => a + b, 0);

function BoardsSection({ gin, onUpdate, sending, msg }: { gin: FwGuardInput; onUpdate: (target: FwTarget) => void; sending: boolean; msg: string | null }) {
  const [w, setW] = useState(COLS_SUM);
  const status = gin.status;
  if (!status) return null;
  const common = fwCommonGuard(gin);
  const g = motorGroup(status);
  const rows: Row[] = [
    ...FW_BODY_SLOTS.map((slot) => status.boards.find((b) => b.slot === slot)).filter((b): b is FwBoard => !!b)
      .map((b) => ({ key: `b${b.slot}`, name: b.name, board: b })),
    { key: 'motors', name: t('모터'), group: g },
  ];
  const k = w / COLS_SUM;
  const col = (n: keyof typeof COLS) => Math.floor(COLS[n] * k);
  const btn = (target: FwTarget) => <SBtn kind="ghost" icon="download" label={t('업데이트')} disabled={common.blocked || sending} onPress={() => onUpdate(target)} />;
  const columns: TableColumn<Row>[] = [
    { fieldId: 'name', label: t('보드'), width: col('name'), expandable: true, render: (r) => r.name },
    { fieldId: 'mode', label: t('모드'), width: col('mode'), expandable: true,
      render: (r) => (r.board
        ? (r.board.present && r.board.mode === 'app' ? modeLabel(r.board)
          : <RBBadge type={reachable(r.board) ? 'capsule' : 'soft'} color={modeTone(r.board)}>{modeLabel(r.board)}</RBBadge>)
        : r.group!.live.some((m) => m.mode !== 'app') || !r.group!.live.length
          ? <RBBadge type="soft" color={r.group!.live.length ? 'orange' : 'gray'}>{motorModeLabel(r.group!)}</RBBadge>
          : motorModeLabel(r.group!)) },
    { fieldId: 'hw', label: 'hw', width: col('hw'), expandable: true,
      render: (r) => (r.board ? hwLabel(r.board) : motorsCommon(r.group!, hwLabel) ?? '—') },
    { fieldId: 'app', label: t('보드 버전'), width: col('app'), expandable: true,
      render: (r) => (r.board ? fmtVer(r.board.appVersion) : r.group!.live.length ? (motorsCommon(r.group!, (m) => fmtVer(m.appVersion)) ?? t('섞임')) : '—') },
    { fieldId: 'file', label: t('파일 버전'), width: col('file'), expandable: true,
      render: (r) => fmtVer(r.board ? r.board.file.version : motorsCommon(r.group!, (m) => m.file.version)) },
    { fieldId: 'verdict', label: t('판정'), width: col('verdict'), expandable: true,
      render: (r) => (r.board
        ? <RBBadge type={ATTN_VERDICT.has(r.board.verdict) ? 'capsule' : 'soft'} color={verdictTone(r.board)}>{verdictLabel(r.board.verdict)}</RBBadge>
        : (
          <View style={{ flexDirection: 'row', gap: 4, flexWrap: 'wrap' }}>
            <RBBadge type={r.group!.verdict === 'downgrade' || r.group!.verdict === 'check' ? 'capsule' : 'soft'} color={motorVerdictTone(r.group!)}>{motorVerdictLabel(r.group!)}</RBBadge>
            {r.group!.mixed ? <RBBadge type="capsule" color="orange">{t('섞임')}</RBBadge> : null}
          </View>
        )) },
    { fieldId: 'act', label: '', width: col('act'), align: 'right',
      render: (r) => (r.board ? (boardUpdatable(r.board) ? btn(r.board.slot) : '') : motorsUpdatable(r.group!) ? btn('motors') : '') },
  ];
  return (
    <>
      <View style={{ height: 28 }} />
      <H2>{t('보드별 상태')}</H2>
      <Desc>{t('보드마다 모드·hw·버전과 로봇이 내린 판정입니다. 모터는 모두 같은 펌웨어여야 해서 한 줄로 봅니다. 줄을 누르면 자세히 나옵니다.')}</Desc>
      {msg ? <Msg text={msg} /> : null}
      <View onLayout={(e) => { const nw = Math.floor(e.nativeEvent.layout.width); if (nw > 0 && nw !== w) setW(nw); }} style={{ width: '100%' }}>
        <RBTable list={rows} columns={columns} expand={(r) => (r.board ? <BoardDetail b={r.board} /> : <MotorDetail g={r.group!} />)} />
      </View>
    </>
  );
}

function MotorDetail({ g }: { g: FwMotorGroup }) {
  const { c, t: ty } = useRb();
  const cell = (flex: number) => ({ flex, minWidth: 0 });
  const head = [ty('compact-xs-normal'), { color: c('fg-subtle') }];
  const body = [ty('compact-xs-normal'), { color: c('fg-default') }];
  return (
    <View style={{ paddingVertical: 4, gap: 3, maxWidth: 520 }}>
      <View style={{ flexDirection: 'row', gap: 12 }}>
        <RbText style={[head, cell(1)]}>{t('관절')}</RbText>
        <RbText style={[head, cell(1.4)]}>{t('모드')}</RbText>
        <RbText style={[head, cell(1)]}>{t('앱 버전')}</RbText>
        <RbText style={[head, cell(1.6)]}>{t('판정')}</RbText>
      </View>
      {g.motors.map((m) => (
        <View key={m.slot} style={{ flexDirection: 'row', gap: 12, alignItems: 'center' }}>
          <RbText style={[body, cell(1)]}>{m.name}</RbText>
          <RbText style={[body, cell(1.4)]}>{modeLabel(m)}</RbText>
          <RbText style={[body, cell(1)]}>{fmtVer(m.appVersion)}</RbText>
          <View style={[cell(1.6), { flexDirection: 'row' }]}>
            <RBBadge type="soft" color={verdictTone(m)}>{verdictLabel(m.verdict)}</RBBadge>
          </View>
        </View>
      ))}
    </View>
  );
}

function BoardDetail({ b }: { b: FwBoard }) {
  const va = variantFix(b);
  const kv: [string, string][] = [
    [t('칸 번호'), String(b.slot)],
    [t('hw 상태'), `${hwStateText(b)}${b.hw.remembered ? ` · ${t('이번 실행에서 읽어 둔 hw 있음')}` : ''}`],
    [t('펌웨어 대상'), b.fwTargetA ? `HW${b.fwTargetA}` : '—'],
    [t('부트 버전'), fmtVer(b.bootVersion)],
    [t('파일'), b.file.name ?? t('맞는 파일 없음')],
    ...(b.verdict === 'unmanaged' ? [[t('판정'), t('이 종류는 아직 hw가 이름에 든 파일이 없어 업데이트 대상이 아닙니다')] as [string, string]] : []),
    ...(bootRecover(b) ? [[t('업데이트'), t('부트에 남아 있어 앱으로 보내 확인한 뒤 올립니다')] as [string, string]] : []),
    ...(va ? [[t('업데이트'), t('맞는 변형(HW{a})으로 바꿉니다').replace('{a}', String(va))] as [string, string]] : []),
    ...(b.reason ? [[t('사유'), fwReasonText(b.reason, b.name, b.result !== 'failed')] as [string, string]] : []),
    ...(b.step !== 'idle' ? [[t('단계'), `${stepLabel(b.step)}${b.retry ? ` · ${t('재시도 {n}/{max}').replace('{n}', String(b.retry)).replace('{max}', String(FW_RETRY_MAX))}` : ''}`] as [string, string]] : []),
  ];
  return <KV rows={kv} />;
}

function KV({ rows }: { rows: [string, string][] }) {
  const { c, t: ty } = useRb();
  return (
    <View style={{ gap: 3, paddingVertical: 4 }}>
      {rows.map(([k, v], i) => (
        <View key={`${i}:${k}`} style={{ flexDirection: 'row', gap: 12 }}>
          <RbText style={[ty('compact-xs-normal'), { color: c('fg-subtle'), width: 96 }]}>{k}</RbText>
          <RbText style={[ty('compact-xs-normal'), { color: c('fg-default'), flex: 1 }]}>{v}</RbText>
        </View>
      ))}
    </View>
  );
}
