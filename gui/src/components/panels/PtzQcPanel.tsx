import { useCallback, useEffect, useMemo, useState } from 'react';
import { View, Text } from 'react-native';
import { useFocusEffect } from 'expo-router';
import { useTheme } from '@/theme';
import { Segmented } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useSettings } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { useHasPtz } from '@/store/capability';
import { useNotMine } from '@/lib/spectating';
import { isDemo } from '@/lib/demoFlag';
import { t } from '@/lib/i18n';
import { ptzQcApi } from '@/lib/ptzQcApi';
import {
  usePtzQc, refreshPtzQc, refreshPtzQcHistory, usePtzQcUnseen, usePtzQcHistory, usePtzQcClosed, markPtzQcSeen, closePtzQcNotice,
} from '@/store/ptzQc';
import {
  daemonErrorText, isBusy, kindLabel, phaseOf, recentUse, refusalText, shortTime, verdictWord, type HistoryRow, type Kind, type Level, type SoakMin,
} from '@/lib/ptzQc';
import { IdleView, RunView, type StartKind } from '@/components/ptzqc/CheckView';
import { ResultsView } from '@/components/ptzqc/ResultsView';
import { ReportView, type Report } from '@/components/ptzqc/ReportView';
import { AskDialog, RerunDialog, SerialDialog, StartDialog } from '@/components/ptzqc/dialogs';
import { BigBtn, Note } from '@/components/ptzqc/parts';
import { staffTools } from '@/modules/registry';

type Tab = 'check' | 'result';

export function PtzQcPanel() {
  const { c } = useTheme();
  const level = Math.min(useSettings((s) => s.accessLevel), 3) as Level;
  const staff = staffTools.useOn();
  const hasPtz = useHasPtz();
  const connected = useRobot((s) => s.conn === 'connected') || isDemo();
  const notMine = useNotMine();
  const status = usePtzQc((s) => s.status);
  const error = usePtzQc((s) => s.error);
  const at = usePtzQc((s) => s.at);
  const history = usePtzQcHistory();
  const historyError = usePtzQc((s) => s.historyError);
  const sentAt = usePtzQc((s) => s.sentAt);
  const unseen = usePtzQcUnseen();
  const closed = usePtzQcClosed();
  const phase = phaseOf(status, sentAt);
  const busy = isBusy(phase);
  const canAct = connected && !notMine;

  const [tab, setTab] = useState<Tab>('check');
  const [soakMin, setSoakMin] = useState<SoakMin>(720);
  const [startAsk, setStartAsk] = useState<{ kind: Kind; steps?: number[]; base?: string } | null>(null);
  const [stopAsk, setStopAsk] = useState(false);
  const [stopping, setStopping] = useState(false);
  const [rerunFor, setRerunFor] = useState<HistoryRow | null>(null);
  const [delFor, setDelFor] = useState<HistoryRow | null>(null);
  const [serialAsk, setSerialAsk] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [report, setReport] = useState<{ stem: string; rep?: Report; err?: string } | null>(null);

  useFocusEffect(useCallback(() => {
    void refreshPtzQc();
    void refreshPtzQcHistory();
    const id = setInterval(() => { void refreshPtzQc(); }, 1000);
    return () => clearInterval(id);
  }, []));
  useEffect(() => { if (!busy) setStopping(false); }, [busy]);

  const daemon = status?.daemon;
  const blocked = !connected ? t('로봇에 연결되지 않았습니다')
    : notMine ? t('다른 사람이 조종하는 중이라 시작할 수 없습니다(관전 중)')
    : !status && error ? `${t('PTZ 점검 상태를 받지 못했습니다')} — ${error}`
    : !status ? t('상태를 받는 중입니다')
    : phase === 'noDaemon' ? t('PTZ 프로그램이 돌고 있지 않습니다 — PTZ 가 연결됐는지 확인해 주세요')
    : daemon && daemon.lan2can === false ? t('이 PTZ 는 점검을 지원하지 않습니다(LAN2CAN 게이트웨이가 아님)')
    : daemon && daemon.engineInstalled === false ? t('검사 프로그램이 이 로봇에 설치되지 않았습니다 — 로봇 소프트웨어를 업데이트해 주세요')
    : busy ? t('검사가 도는 중입니다')
    : null;
  const last = history.find((r) => !r.running) ?? null;
  const stale = !!error && !!status && Date.now() - at > 5000 ? `${t('상태를 받지 못하고 있습니다 — 아래 값은 마지막으로 받은 것입니다')} (${error})` : null;

  const send = (kind: Kind, presets: boolean, steps?: number[], base?: string) => {
    setMsg(null);
    const lv = level;
    ptzQcApi.start({ level: lv, kind, presets, ...(kind === 'aging' ? { soakMin } : {}), ...(kind === 'rerun' ? { steps, base } : {}) })
      .then(() => { usePtzQc.getState().markSent(); setTab('check'); void refreshPtzQc(); })
      .catch((e: unknown) => setMsg(refusalText(e)));
  };
  const openReport = (r: HistoryRow) => {
    setTab('result');
    setReport({ stem: r.stem });
    markPtzQcSeen(r.stem);
    ptzQcApi.report(r.stem)
      .then((rep: Report) => setReport((cur) => (cur?.stem === r.stem ? { stem: r.stem, rep } : cur)))
      .catch((e: unknown) => setReport((cur) => (cur?.stem === r.stem ? { stem: r.stem, err: refusalText(e) } : cur)));
  };

  const eng = status?.status;
  const done = !busy && eng?.state === 'done' ? eng.done : null;
  const rerun = eng?.kind === 'rerun';
  const reportStem = rerun ? eng?.base : done?.stem;
  const doneRow = reportStem ? history.find((r) => r.stem === reportStem) ?? null : null;
  const byStem = !!done?.stem && !rerun;
  const showDone = !!done && (byStem ? unseen?.stem === done.stem : closed !== (eng?.startedAt ?? ''));
  const closeDone = () => (byStem ? markPtzQcSeen(done!.stem!) : closePtzQcNotice(eng?.startedAt ?? ''));
  const startFail = !busy && sentAt && Date.now() - sentAt < 120000 ? daemonErrorText(daemon?.lastError) : null;

  const closeReport = useCallback(() => setReport(null), []);
  const reportView = useMemo(() => (report?.rep ? <ReportView rep={report.rep} level={level} onBack={closeReport} /> : null), [report?.rep, level, closeReport]);

  const body = tab === 'result'
    ? (report ? (reportView ?? (
        <View style={{ gap: 10 }}>
          <View style={{ flexDirection: 'row' }}><BigBtn small kind="ghost" icon="chevL" label={t('이력')} onPress={closeReport} /></View>
          {report.err ? <Note tone="danger" text={`${t('보고서를 받지 못했습니다')} — ${report.err}`} />
            : <Text style={{ color: c.muted, fontSize: 13 }}>{t('보고서를 받는 중…')} ({report.stem})</Text>}
        </View>
      ))
      : <ResultsView rows={history} staff={staff} busy={busy || !connected || notMine} unseen={unseen} error={historyError}
          onOpen={openReport} onRerun={(r) => setRerunFor(r)} onDelete={(r) => setDelFor(r)} />)
    : busy
      ? <RunView phase={phase} status={status} level={level} stopping={stopping} canStop={canAct} stale={stale} onStop={() => setStopAsk(true)} />
      : (
        <View style={{ flex: 1, gap: 10 }}>
          {startFail ? (
            <Note tone="danger" title={t('검사를 시작하지 못했습니다')} text={startFail}
              right={<BigBtn small kind="ghost" icon="x" label={t('닫기')} onPress={() => usePtzQc.setState({ sentAt: 0 })} />} />
          ) : null}
          {done && showDone ? (
            done.stem ? (
              <Note icon="check" title={`${rerun ? t('단계 다시 돌리기가 끝났습니다') : t('검사가 끝났습니다')} — ${verdictWord(done.verdict)}`}
                text={done.reason ? done.reason : doneRow ? `${shortTime(doneRow.startedAt)} · ${kindLabel(doneRow.kind, doneRow.soakMin)}` : ''}
                right={<View style={{ flexDirection: 'row', gap: 6 }}>
                  {doneRow ? <BigBtn small kind="ghost" icon="log" label={t('결과 보기')} onPress={() => openReport(doneRow)} /> : null}
                  <BigBtn small kind="ghost" icon="x" label={t('닫기')} onPress={closeDone} />
                </View>} />
            ) : (
              <Note tone="warn" title={`${t('검사가 보고서 없이 끝났습니다')} — ${verdictWord(done.verdict)}`} text={done.reason ?? ''}
                right={<BigBtn small kind="ghost" icon="x" label={t('닫기')} onPress={closeDone} />} />
            )
          ) : null}
          {error && status ? <Note tone="warn" text={`${t('상태를 받지 못하고 있습니다')} — ${error}`} /> : null}
          <IdleView level={level} staff={staff} status={status} last={last} hasPtz={hasPtz} blocked={blocked} canSerial={canAct} soakMin={soakMin} setSoakMin={setSoakMin}
            onStart={(k: StartKind) => setStartAsk({ kind: k })} onSerial={() => setSerialAsk(true)}
            onOpenLast={() => last && openReport(last)} msg={null} />
        </View>
      );

  return (
    <View style={{ flex: 1 }}>
      <View style={{ flexDirection: 'row', alignItems: 'flex-start', gap: 12, marginBottom: 12 }}>
        <View style={{ flex: 1 }}>
          <Text style={{ color: c.text, fontSize: 18, fontWeight: '700' }}>{t('PTZ 점검')}</Text>
          <Text style={{ color: c.muted, fontSize: 12.5, marginTop: 2 }}>
            {report && tab === 'result' ? t('이 로봇의 PTZ 검사 기록 — QC PC 웹 보고서와 같은 절 · 같은 순서입니다')
              : t('PTZ 가 제대로 도는지 축 · 카메라 · 소리 · 통신을 차례로 검사합니다. 결과는 [결과] 칸에 남습니다.')}
          </Text>
        </View>
        <View style={{ width: 170 }}>
          <Segmented<Tab> options={[{ key: 'check', label: t('검사') }, { key: 'result', label: t('결과') }]} value={tab}
            onChange={(v) => { setTab(v); if (v === 'result') void refreshPtzQcHistory(); }} />
        </View>
      </View>
      {msg ? <View style={{ marginBottom: 10 }}><Note tone="danger" text={msg} right={<BigBtn small kind="ghost" icon="x" label={t('닫기')} onPress={() => setMsg(null)} />} /></View> : null}
      <View style={{ flex: 1 }}>{body}</View>

      {startAsk ? (
        <StartDialog level={level} kind={startAsk.kind} steps={startAsk.steps} soakMin={soakMin} recentWarn={recentUse(status).warn} onClose={() => setStartAsk(null)}
          onStart={(presets) => { const a = startAsk; setStartAsk(null); send(a.kind, presets, a.steps, a.base); }} />
      ) : null}
      {phase === 'ask' && status?.status?.gate ? (
        <AskDialog st={status.status} canAnswer={canAct} stale={stale}
          onEnd={() => { ptzQcApi.answer(false).then(() => refreshPtzQc()).catch((e: unknown) => setMsg(refusalText(e))); }}
          onGo={() => { ptzQcApi.answer(true).then(() => refreshPtzQc()).catch((e: unknown) => setMsg(refusalText(e))); }} />
      ) : null}
      {stopAsk ? (
        <ConfirmModal title={t('검사를 멈출까요?')} confirmLabel={t('멈춤')}
          message={t("지금까지의 결과는 '무효'로 남습니다(제품 판정이 아닙니다). 축은 그 자리에 섭니다.")}
          onClose={() => setStopAsk(false)}
          onConfirm={() => {
            setStopAsk(false); setStopping(true);
            ptzQcApi.stop('앱에서 멈춤')
              .then(() => refreshPtzQc()).catch((e: unknown) => { setStopping(false); setMsg(refusalText(e)); });
          }} />
      ) : null}
      {rerunFor ? (
        <RerunDialog base={`${shortTime(rerunFor.startedAt)} ${kindLabel(rerunFor.kind, rerunFor.soakMin)}`} level={level}
          onClose={() => setRerunFor(null)}
          onRun={(steps) => { const r = rerunFor; setRerunFor(null); setStartAsk({ kind: 'rerun', steps, base: r.stem }); }} />
      ) : null}
      {delFor ? (
        <ConfirmModal title={t('보고서를 지울까요?')} confirmLabel={t('지우기')}
          message={`${shortTime(delFor.startedAt)} · ${kindLabel(delFor.kind, delFor.soakMin)} · ${verdictWord(delFor.verdict)} — ${t('녹화와 다시 돌린 기록까지 로봇에서 지웁니다. 되돌릴 수 없습니다.')}`}
          onClose={() => setDelFor(null)}
          onConfirm={() => {
            const r = delFor; setDelFor(null);
            ptzQcApi.remove(r.stem).then(() => refreshPtzQcHistory()).catch((e: unknown) => setMsg(refusalText(e)));
          }} />
      ) : null}
      {serialAsk ? (
        <SerialDialog current={status?.daemon?.lastSerial?.serial || last?.serial || ''} onClose={() => setSerialAsk(false)}
          onWrite={(s) => { setSerialAsk(false); ptzQcApi.serial(s).then(() => { usePtzQc.getState().markSent(); void refreshPtzQc(); }).catch((e: unknown) => setMsg(refusalText(e))); }} />
      ) : null}
    </View>
  );
}
