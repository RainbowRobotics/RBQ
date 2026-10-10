import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, FlatList, TextInput, ActivityIndicator, Pressable, Modal as RNModal, Platform } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import Animated, { FadeIn } from 'react-native-reanimated';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobot, getLiveLogs } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { t } from '@/lib/i18n';
import { useCompactH } from '@/lib/layout';
import { rest, actions, type LogFileEntry , restErrorText } from '@/lib/rest';
import { saveLogsAsJsonl, saveBlackboxZip, saveSystemlogRaw } from '@/lib/download';
import { uploadSystemlog, uploadBlackbox, pendingCount, flushPending, clearPending } from '@/lib/logUpload';
import { uploadConfigured, fmtBytes } from '@/lib/logUploadCommon';
import { useAccessLevel } from '@/store/settings';
import { Modal } from '@/components/ui/overlays';
import { pickDocument } from '@/lib/pickDocument';
import { BlackBoxPlayer } from '@/components/BlackBoxPlayer';
import { parseBlackboxZip, type BlackboxSession } from '@/lib/blackbox';
import { libSupported, libList, libPut, libRead, libRemove } from '@/lib/bbLibrary';
import { isDemo } from '@/lib/demoFlag';
import { libName, libSerials, libDates, libSessions, serialKey, NO_SERIAL, type LibEntry } from '@/lib/bbLibraryCommon';
import { SysLogPlayer } from '@/components/SysLogPlayer';
import type { LogLevel, LogLine } from '@/types/robot';
import { inputVFix } from '@/components/ui/controls';

type Tab = 'rt' | 'sys' | 'bb';
const TABS: { key: Tab; label: string }[] = [
  { key: 'rt', label: 'Real Time' }, { key: 'sys', label: 'System Log' }, { key: 'bb', label: 'Black Box' },
];
const LEVELS: LogLevel[] = ['TRACE', 'DEBUG', 'INFO', 'SUCCESS', 'WARNING', 'ERROR', 'FATAL'];

const LEVEL_HEX: Record<LogLevel, string> = {
  TRACE: '#888888', DEBUG: '#90A4AE', INFO: '#4FC3F7', SUCCESS: '#81C784',
  WARNING: '#FFB74D', ERROR: '#E57373', FATAL: '#B71C1C',
};
const PROC_PALETTE = [
  '#4FC3F7', '#81C784', '#FFB74D', '#E57373', '#BA68C8', '#4DB6AC',
  '#FFD54F', '#F06292', '#7986CB', '#A1887F', '#90A4AE', '#AED581',
];
const _procColor = new Map<string, string>();
function procColor(app: string) {
  if (!app) return '#888888';
  let c = _procColor.get(app);
  if (c === undefined) { c = PROC_PALETTE[_procColor.size % PROC_PALETTE.length]; _procColor.set(app, c); }
  return c;
}

const MAX_VIEW = 2500;
const fmtDate = (d: string) => (/^\d{8}$/.test(d) ? `${d.slice(0, 4)}-${d.slice(4, 6)}-${d.slice(6, 8)}` : d);
const fmtTime = (t: string) => t.replace(/_/g, ':');

function PickDrop({ label, value, options, fmt, onPick }: {
  label: string; value: string; options: string[]; fmt: (v: string) => string; onPick: (v: string) => void;
}) {
  const { c, fonts, radius } = useTheme();
  const btnRef = useRef<View>(null);
  const [pos, setPos] = useState<{ x: number; y: number } | null>(null);
  const openAt = () => btnRef.current?.measureInWindow((x, y, _w, h) => setPos({ x, y: y + h + 4 }));
  const open = pos != null;
  return (
    <View ref={btnRef} collapsable={false}>
      <Tappable onPress={() => options.length && (open ? setPos(null) : openAt())}
        style={[styles.pick, { flexDirection: 'row', alignItems: 'center', gap: 6, borderColor: open ? c.accent : c.line, backgroundColor: c.elev, opacity: options.length ? 1 : 0.45 }]}>
        <Text style={{ color: c.dim, fontSize: 9, letterSpacing: 0.5 }}>{label}</Text>
        <Text style={{ color: value ? c.accent2 : c.dim, fontSize: 10, fontFamily: fonts.mono }}>{value ? fmt(value) : '—'}</Text>
        <Icon name="caret" size={10} color={c.dim} />
      </Tappable>
      <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent visible={open} animationType="fade" onRequestClose={() => setPos(null)}>
        <Pressable style={StyleSheet.absoluteFill} onPress={() => setPos(null)}>
          {pos && (
            <Pressable onPress={() => {}}
              style={[styles.dropList, { left: pos.x, top: pos.y, backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
              <ScrollView showsVerticalScrollIndicator style={{ maxHeight: 240 }}>
                {options.map((o) => {
                  const on = o === value;
                  return (
                    <Tappable key={o} onPress={() => { setPos(null); onPick(o); }}
                      style={[styles.dropRow, on && { backgroundColor: hexA('#4d9cf5', 0.14) }]}>
                      <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 11, fontFamily: fonts.mono }}>{fmt(o)}</Text>
                    </Tappable>
                  );
                })}
              </ScrollView>
            </Pressable>
          )}
        </Pressable>
      </RNModal>
    </View>
  );
}

function Chip({ color, label, off, onPress }: { color: string; label: string; off: boolean; onPress: () => void }) {
  const { c } = useTheme();
  return (
    <Tappable onPress={onPress} style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, opacity: off ? 0.32 : 1 }]}>
      <View style={[styles.dot, { backgroundColor: color }]} />
      <Text style={{ color: c.text, fontSize: 9.5, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
}

export function LogPanel({ focus, onSubtitle }: {
  focus?: { ts: string; msg: string } | null;
  onSubtitle?: (s: string) => void;
}) {
  const { c, fonts } = useTheme();
  const [tab, setTab] = useState<Tab>('rt');

  const logSeq = useRobot((s) => s.logSeq);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const rtLogs = useMemo(() => getLiveLogs() as LogLine[], [logSeq]);
  const conn = useRobot((s) => s.conn);
  const ip = useRobot((s) => s.ip);
  const accessLevel = useAccessLevel();

  const [hideLv, setHideLv] = useState<Set<LogLevel>>(new Set());
  const [hideProc, setHideProc] = useState<Set<string>>(new Set());
  const [q, setQ] = useState('');
  const [autoscroll, setAutoscroll] = useState(true);
  const [saving, setSaving] = useState(false);
  const [saveErr, setSaveErr] = useState('');
  const scrollRef = useRef<FlatList>(null);

  const [sysFiles, setSysFiles] = useState<LogFileEntry[]>([]);
  const [sysDates, setSysDates] = useState<string[]>([]);
  const [sysDate, setSysDate] = useState('');
  const [sysLogs, setSysLogs] = useState<LogLine[]>([]);
  const [sysLoading, setSysLoading] = useState(false);
  const [sysErr, setSysErr] = useState('');
  const [sysPlayer, setSysPlayer] = useState(false);

  const [bbFiles, setBbFiles] = useState<LogFileEntry[]>([]);
  const [bbDate, setBbDate] = useState('');
  const [bbSession, setBbSession] = useState('');
  const [bbLogs, setBbLogs] = useState<LogLine[]>([]);
  const [bbLoading, setBbLoading] = useState(false);
  const [bbErr, setBbErr] = useState('');
  const [bbPlayer, setBbPlayer] = useState(false);
  const [bbListMode, setBbListMode] = useState(false);
  const [bbLocal, setBbLocal] = useState<{ name: string; sess: BlackboxSession } | null>(null);
  const [bbSrc, setBbSrc] = useState<'robot' | 'lib'>(() => (conn === 'connected' || !libSupported ? 'robot' : 'lib'));
  const [lib, setLib] = useState<LibEntry[]>([]);
  const [libSel, setLibSel] = useState<{ serial: string; date: string; session: string }>({ serial: '', date: '', session: '' });
  const [libBusy, setLibBusy] = useState(false);
  const [libDelAsk, setLibDelAsk] = useState(false);
  const [bbSavingNow, setBbSavingNow] = useState(false);

  const toggle = <T,>(setter: React.Dispatch<React.SetStateAction<Set<T>>>, v: T) =>
    setter((prev) => { const n = new Set(prev); n.has(v) ? n.delete(v) : n.add(v); return n; });

  useEffect(() => {
    onSubtitle?.(bbPlayer ? t('Black Box · 재생') : sysPlayer ? t('System Log · 재생') : TABS.find((t) => t.key === tab)!.label);
  }, [tab, bbPlayer, sysPlayer]); // eslint-disable-line react-hooks/exhaustive-deps

  const loadSysList = async () => {
    if (conn !== 'connected') return;
    setSysErr(''); setSysLoading(true);
    try {
      const r = await rest.systemlogList(ip);
      setSysFiles(r.files);
      const dates = [...new Set(r.files.map((f) => f.path.split('/')[0]).filter(Boolean))].sort().reverse();
      setSysDates(dates);
      const d = dates[0] || '';
      setSysDate(d);
      if (d) await loadSysDate(d); else { setSysLogs([]); setSysLoading(false); }
    } catch (e: any) { setSysErr(t('목록을 불러오지 못했습니다') + ' (' + restErrorText(e) + ')'); setSysLoading(false); }
  };
  const loadSysDate = async (date: string) => {
    setSysErr(''); setSysLoading(true); setSysDate(date);
    try { setSysLogs(await rest.systemlogByDate(ip, date)); }
    catch (e: any) { setSysErr(t('로그를 불러오지 못했습니다') + ' (' + restErrorText(e) + ')'); setSysLogs([]); }
    finally { setSysLoading(false); }
  };

  const loadBbList = async () => {
    if (conn !== 'connected') return;
    setBbErr(''); setBbLoading(true);
    try {
      const r = await rest.blackboxList(ip);
      setBbFiles(r.files);
      const dates = [...new Set(r.files.map((f) => f.path.split('/')[0]).filter(Boolean))].sort().reverse();
      const d = dates[0] || '';
      const sess = [...new Set(r.files.filter((f) => f.path.startsWith(d + '/')).map((f) => f.path.split('/')[1]))].sort().reverse();
      const s0 = sess[0] || '';
      setBbDate(d); setBbSession(s0);
      if (d && s0) await loadBbFile(d, s0); else { setBbLogs([]); setBbLoading(false); }
    } catch (e: any) { setBbErr(t('목록을 불러오지 못했습니다') + ' (' + restErrorText(e) + ')'); setBbLoading(false); }
  };
  const loadBbFile = async (date: string, session: string) => {
    setBbErr(''); setBbLoading(true); setBbDate(date); setBbSession(session);
    try { setBbLogs(await rest.blackboxFile(ip, `${date}/${session}/systemlog.log`)); }
    catch (e: any) { setBbErr(t('세션 로그를 불러오지 못했습니다') + ' (' + restErrorText(e) + ')'); setBbLogs([]); }
    finally { setBbLoading(false); }
  };

  useEffect(() => {
    if (tab === 'sys' && sysDates.length === 0 && !sysLoading) loadSysList();
    if (tab === 'bb' && bbFiles.length === 0 && !bbLoading) loadBbList();
  }, [tab, conn]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => {
    if (tab === 'bb') setBbListMode(false);
  }, [tab]);

  useEffect(() => {
    if (tab === 'bb' && bbDate && bbSession && !bbListMode) setBbPlayer(true);
  }, [tab, bbDate, bbSession, bbListMode]);

  const bbDates = useMemo(
    () => [...new Set(bbFiles.map((f) => f.path.split('/')[0]).filter(Boolean))].sort().reverse(), [bbFiles]);
  const bbSessions = useMemo(
    () => [...new Set(bbFiles.filter((f) => f.path.startsWith(bbDate + '/')).map((f) => f.path.split('/')[1]).filter(Boolean))].sort().reverse(),
    [bbFiles, bbDate]);

  const source = tab === 'rt' ? rtLogs : tab === 'sys' ? sysLogs : bbLogs;
  const srcSeq = tab === 'rt' ? logSeq : 0;
  const procsInSrc = useMemo(() => {
    const seen: string[] = [];
    for (const l of source) if (l.process && !seen.includes(l.process)) seen.push(l.process);
    return seen;
  }, [source, srcSeq]); // eslint-disable-line react-hooks/exhaustive-deps
  const filtered = useMemo(() => {
    const needle = q.trim().toLowerCase();
    return source.filter((l) =>
      !hideLv.has(l.level) && !hideProc.has(l.process) &&
      (needle === '' || l.msg.toLowerCase().includes(needle) || l.process.toLowerCase().includes(needle)));
  }, [source, srcSeq, hideLv, hideProc, q]); // eslint-disable-line react-hooks/exhaustive-deps
  const view = filtered.length > MAX_VIEW ? filtered.slice(-MAX_VIEW) : filtered;
  const loading = tab === 'sys' ? sysLoading : tab === 'bb' ? bbLoading : false;
  const err = tab === 'sys' ? sysErr : tab === 'bb' ? bbErr : '';

  const upSrcBytes = useMemo(() => {
    const prefix = tab === 'sys' ? `${sysDate}/` : `${bbDate}/${bbSession}/`;
    const files = tab === 'sys' ? sysFiles : bbFiles;
    return files.filter((f) => f.path.startsWith(prefix)).reduce((n, f) => n + (f.size || 0), 0);
  }, [tab, sysFiles, bbFiles, sysDate, bbDate, bbSession]);

  const saveDisabled = saving || (tab === 'bb' ? !(bbDate && bbSession) : tab === 'sys' ? !sysDate : filtered.length === 0);
  const onSave = async () => {
    if (saveDisabled) return;
    setSaving(true); setSaveErr('');
    try {
      if (tab === 'bb') {
        const serial = await rest.serialNumber(ip).then((r) => r.serial_number).catch(() => '');
        await saveBlackboxZip(ip, `${bbDate}/${bbSession}`, serial, upSrcBytes);
      }
      else if (tab === 'sys') await saveSystemlogRaw(ip, sysDate, upSrcBytes);
      else await saveLogsAsJsonl('realtime-log.jsonl', filtered);
    } catch (e: any) {
      setSaveErr(e?.message || t('저장하지 못했습니다'));
    }
    finally { setSaving(false); }
  };

  useEffect(() => {
    if (tab === 'rt' && autoscroll && view.length) scrollRef.current?.scrollToEnd({ animated: false });
  }, [view.length, tab, autoscroll]);

  const [upAsk, setUpAsk] = useState(false);
  const [upStage, setUpStage] = useState('');
  const [upDone, setUpDone] = useState('');
  const [upErr, setUpErr] = useState('');
  const canUpload = uploadConfigured && accessLevel >= 3 && conn === 'connected' &&
    (tab === 'sys' ? !!sysDate : tab === 'bb' ? !!(bbDate && bbSession) : false);

  const upLabel = tab === 'sys' ? `${t('시스템 로그')} ${fmtDate(sysDate)}` : `${t('블랙박스')} ${fmtDate(bbDate)} ${fmtTime(bbSession)}`;

  const [pending, setPending] = useState(0);
  const refreshPending = () => pendingCount().then(setPending).catch(() => {});
  useEffect(() => { if (uploadConfigured && accessLevel >= 3) refreshPending(); }, [accessLevel]);

  const onUpload = async () => {
    setUpAsk(false); setUpErr(''); setUpDone(''); setUpStage(t('준비 중…'));
    try {
      const serial = await rest.serialNumber(ip).then((r) => r.serial_number).catch(() => '');
      const r = tab === 'sys'
        ? await uploadSystemlog(ip, serial, sysDate, setUpStage)
        : await uploadBlackbox(ip, serial, `${bbDate}/${bbSession}`, setUpStage);
      setUpDone(r.queued
        ? `${t('인터넷에 연결되지 않아 보류함에 담았습니다')} (${fmtBytes(r.bytes)}) — ${t('인터넷이 되는 곳에서 [보류] 버튼으로 보내세요.')}`
        : `${t('전송 완료')} · ${fmtBytes(r.bytes)}`);
    } catch (e: any) {
      setUpErr(e?.message || t('전송 실패'));
    } finally { setUpStage(''); refreshPending(); }
  };

  const [dropAsk, setDropAsk] = useState(false);
  const onDropPending = async () => {
    setDropAsk(false);
    try { await clearPending(); setUpDone(t('보류함을 비웠습니다')); }
    catch (e: any) { setUpErr(e?.message || t('전송 실패')); }
    finally { refreshPending(); }
  };

  const onFlush = async () => {
    setUpErr(''); setUpDone(''); setUpStage(t('보류함 전송…'));
    try {
      const { sent, failed, dropped } = await flushPending(setUpStage);
      const drop = dropped ? ` · ${t('서버 거부로 {k}건 버림').replace('{k}', String(dropped))}` : '';
      if (failed) setUpErr(t('{n}건 보냄, {m}건 실패 — 인터넷 연결을 확인하세요.').replace('{n}', String(sent)).replace('{m}', String(failed)) + drop);
      else setUpDone(`${t('보류함 전송 완료')} · ${sent}${t('건')}${drop}`);
    } catch (e: any) {
      setUpErr(e?.message || t('전송 실패'));
    } finally { setUpStage(''); refreshPending(); }
  };

  const [focusLine, setFocusLine] = useState<{ ts: string; msg: string } | null>(null);
  const [focusBlink, setFocusBlink] = useState(false);
  const focusScrolled = useRef(true);
  useEffect(() => {
    if (!focus) return;
    setTab('rt');
    setAutoscroll(false);
    setFocusLine(focus);
    focusScrolled.current = false;
    let n = 0;
    const t = setInterval(() => {
      setFocusBlink((v) => !v);
      if (++n >= 6) { clearInterval(t); setFocusBlink(false); }
    }, 280);
    return () => clearInterval(t);
  }, [focus]);

  useEffect(() => {
    if (tab !== 'rt' || !focusLine || focusScrolled.current || !view.length) return;
    let idx = -1;
    for (let i = view.length - 1; i >= 0; i--) {
      if (view[i].ts === focusLine.ts && view[i].msg === focusLine.msg) { idx = i; break; }
    }
    if (idx < 0) { focusScrolled.current = true; return; }
    focusScrolled.current = true;
    setTimeout(() => scrollRef.current?.scrollToIndex({ index: idx, viewPosition: 0.5, animated: true }), 200);
  }, [tab, focusLine, view]);

  const notMine = useNotMine();
  const onBbSaveNow = async () => {
    if (bbSavingNow) return;
    setBbSavingNow(true);
    try {
      await actions.blackboxSaveNow(ip);
      await new Promise((r) => setTimeout(r, 1200));
      await loadBbList();
    } catch { setBbErr(t('저장 요청이 실패했습니다')); }
    finally { setBbSavingNow(false); }
  };

  const openLib = async (all: LibEntry[], serial: string, date: string, session: string) => {
    setLibSel({ serial, date, session });
    const e = all.find((x) => x.serial === serial && x.date === date && x.session === session);
    if (!e) { setBbLocal(null); return; }
    setLibBusy(true);
    try { setBbLocal({ name: e.name, sess: parseBlackboxZip(await libRead(e), e.name) }); setBbErr(''); }
    catch (err: any) { setBbLocal(null); setBbErr(t('zip을 열지 못했습니다 (') + restErrorText(err) + ')'); }
    finally { setLibBusy(false); }
  };
  const refreshLib = async (want?: { serial?: string; date?: string; session?: string }) => {
    setLibBusy(true);
    let all: LibEntry[] = [];
    try { all = await libList(); } catch (e: any) { setBbErr(t('보관함을 읽지 못했습니다 (') + restErrorText(e) + ')'); }
    setLib(all);
    setLibBusy(false);
    const serials = libSerials(all);
    const serial = [want?.serial, libSel.serial].find((x) => x && serials.includes(x)) ?? serials[0] ?? '';
    const dates = libDates(all, serial);
    const date = want?.date && dates.includes(want.date) ? want.date : dates[0] ?? '';
    const sessions = libSessions(all, serial, date);
    const session = want?.session && sessions.includes(want.session) ? want.session : sessions[0] ?? '';
    await openLib(all, serial, date, session);
  };
  const setSrc = (src: 'robot' | 'lib') => {
    if (src === bbSrc) return;
    setBbSrc(src); setBbLocal(null); setBbErr('');
    if (src === 'lib') refreshLib();
  };
  useEffect(() => {
    if (tab === 'bb' && bbSrc === 'lib') refreshLib();
  }, [tab]); // eslint-disable-line react-hooks/exhaustive-deps
  const onLibDelete = async () => {
    setLibDelAsk(false);
    const e = lib.find((x) => x.serial === libSel.serial && x.date === libSel.date && x.session === libSel.session);
    if (!e) return;
    try { await libRemove(e); } catch (err: any) { setSaveErr(restErrorText(err)); }
    await refreshLib({ serial: e.serial });
  };
  const serialLabel = (sn: string) => (sn === NO_SERIAL ? t('S/N 없음') : sn);

  const onBbOpenZip = async () => {
    try {
      const r = await pickDocument({ copyToCacheDirectory: true });
      const a = r.assets?.[0];
      if (r.canceled || !a) return;
      const bytes = new Uint8Array(await (await fetch(a.uri)).arrayBuffer());
      const sess = parseBlackboxZip(bytes, a.name);
      setBbErr('');
      if (libSupported && sess.id && !isDemo()) {
        const serial = sess.robot?.serial ?? '';
        try {
          await libPut(serial, libName(serial, sess.id.date, sess.id.session),
            Platform.OS === 'web' ? { blob: new Blob([bytes], { type: 'application/zip' }) } : { uri: a.uri });
          setBbSrc('lib');
          await refreshLib({ serial: serialKey(serial), ...sess.id });
          return;
        } catch (e: any) { setSaveErr(t('보관함에 넣지 못했습니다 — 이번만 재생합니다.') + ' (' + restErrorText(e) + ')'); }
      }
      setBbLocal({ name: a.name, sess });
    } catch (e: any) { setBbErr(t('zip을 열지 못했습니다 (') + restErrorText(e) + ')'); }
  };

  const bbPlayerActive = tab === 'bb' && (!!bbLocal || (bbSrc === 'robot' && bbPlayer && !!bbDate && !!bbSession));
  const compact = useCompactH();
  if (sysPlayer && sysDate) {
    return <SysLogPlayer date={fmtDate(sysDate)} logs={filtered} onClose={() => setSysPlayer(false)} />;
  }

  const connView = conn === 'connected'
    ? { col: c.green, label: t('연결됨') }
    : conn === 'connecting' ? { col: c.amber, label: t('연결 중') } : { col: c.redTx, label: t('끊김') };

  return (
    <View style={styles.body}>
      {!(compact && bbPlayerActive) && (
      <View style={styles.tabbar}>
        {TABS.map((t) => {
          const on = t.key === tab;
          return (
            <Tappable key={t.key} onPress={() => setTab(t.key)}
              style={[styles.tab, { backgroundColor: on ? c.panel : 'transparent', borderColor: on ? c.line : 'transparent' }]}>
              <Text style={{ color: on ? c.text : c.muted, fontSize: 12, fontWeight: '600' }}>{t.label}</Text>
            </Tappable>
          );
        })}
        <View style={styles.conn}>
          <View style={[styles.connDot, { backgroundColor: connView.col }]} />
          <Text style={{ color: connView.col, fontSize: 11 }}>{connView.label}</Text>
        </View>
      </View>
      )}

      <View style={[styles.panel, { backgroundColor: c.panel, borderColor: c.line }]}>
        {tab !== 'rt' && !(compact && bbPlayerActive) && (
          <View style={[styles.srcbar, { borderBottomColor: c.line2, backgroundColor: c.panel2 }]}>
            {tab === 'bb' && libSupported && (
              <View style={[styles.seg, { borderColor: c.line }]}>
                {([['robot', t('로봇')], ['lib', t('보관함')]] as const).map(([k, label]) => (
                  <Tappable key={k} onPress={() => setSrc(k)}
                    style={[styles.segBtn, bbSrc === k && { backgroundColor: hexA(c.accent2, 0.16) }]}>
                    <Text style={{ color: bbSrc === k ? c.accent2 : c.muted, fontSize: 10, fontWeight: '700' }}>{label}</Text>
                  </Tappable>
                ))}
              </View>
            )}
            <Tappable onPress={() => (tab === 'sys' ? loadSysList() : bbSrc === 'lib' ? refreshLib() : loadBbList())}
              style={[styles.fetch, { backgroundColor: c.accent }]}>
              <Icon name="recover" size={12} color={c.onAccent} />
              <Text style={{ color: c.onAccent, fontSize: 10, fontWeight: '700' }}>{t('불러오기')}</Text>
            </Tappable>
            {tab === 'bb' && bbSrc === 'lib' ? (
              <>
                <PickDrop label="S/N" value={libSel.serial} options={libSerials(lib)} fmt={serialLabel}
                  onPick={(sn) => refreshLib({ serial: sn })} />
                <PickDrop label="DATE" value={libSel.date} options={libDates(lib, libSel.serial)} fmt={fmtDate}
                  onPick={(d) => openLib(lib, libSel.serial, d, libSessions(lib, libSel.serial, d)[0] ?? '')} />
                <PickDrop label="SESSION" value={libSel.session} options={libSessions(lib, libSel.serial, libSel.date)} fmt={fmtTime}
                  onPick={(ss) => openLib(lib, libSel.serial, libSel.date, ss)} />
                <Tappable onPress={() => libSel.session && setLibDelAsk(true)}
                  style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: c.line, backgroundColor: c.elev, opacity: libSel.session ? 1 : 0.4 }]}>
                  <Icon name="x" size={11} color={c.redTx} />
                  <Text style={{ color: c.redTx, fontSize: 9 }}>{t('삭제')}</Text>
                </Tappable>
              </>
            ) : (
              <>
                <PickDrop label="DATE" value={tab === 'sys' ? sysDate : bbDate} options={tab === 'sys' ? sysDates : bbDates} fmt={fmtDate}
                  onPick={(d) => {
                    if (tab === 'sys') loadSysDate(d);
                    else { setBbDate(d); const sess = [...new Set(bbFiles.filter((f) => f.path.startsWith(d + '/')).map((f) => f.path.split('/')[1]))].sort().reverse(); const s0 = sess[0] || ''; if (s0) loadBbFile(d, s0); }
                  }} />
                {tab === 'bb' && (
                  <PickDrop label="SESSION" value={bbSession} options={bbSessions} fmt={fmtTime}
                    onPick={(s) => loadBbFile(bbDate, s)} />
                )}
              </>
            )}
          </View>
        )}

        {!bbPlayerActive && (
        <View style={[styles.filt, { borderBottomColor: c.line2 }]}>
          {compact ? (
            <ScrollView horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={styles.procRow} style={{ flex: 1 }}>
              <Text style={{ color: c.dim, fontSize: 9, letterSpacing: 0.6 }}>LEVEL</Text>
              {LEVELS.map((lv) => (
                <Chip key={lv} color={LEVEL_HEX[lv]} label={lv} off={hideLv.has(lv)} onPress={() => toggle(setHideLv, lv)} />
              ))}
              <View style={{ width: 1, height: 14, backgroundColor: c.line, marginHorizontal: 4 }} />
              <Text style={{ color: c.dim, fontSize: 9, letterSpacing: 0.6 }}>PROC</Text>
              {procsInSrc.map((p) => (
                <Chip key={p} color={procColor(p)} label={p} off={hideProc.has(p)} onPress={() => toggle(setHideProc, p)} />
              ))}
            </ScrollView>
          ) : (
            <>
              <Text style={{ color: c.dim, fontSize: 9, letterSpacing: 0.6 }}>LEVEL</Text>
              {LEVELS.map((lv) => (
                <Chip key={lv} color={LEVEL_HEX[lv]} label={lv} off={hideLv.has(lv)} onPress={() => toggle(setHideLv, lv)} />
              ))}
            </>
          )}
          <View style={[styles.searchbox, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Icon name="search" size={12} color={c.dim} />
            <TextInput value={q} onChangeText={setQ} placeholder={t('검색')} placeholderTextColor={c.dim}
              style={[{ color: c.text, fontSize: 11, padding: 0, minWidth: 80 }, inputVFix]} />
          </View>
        </View>
        )}

        <View style={[styles.actbar, { borderBottomColor: c.line2, backgroundColor: c.panel2 }]}>
          {bbPlayerActive || compact ? (
            <View style={{ flex: 1 }} />
          ) : (
            <>
              <Text style={{ color: c.dim, fontSize: 9, letterSpacing: 0.6 }}>PROC</Text>
              <ScrollView horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={styles.procRow}>
                {procsInSrc.map((p) => (
                  <Chip key={p} color={procColor(p)} label={p} off={hideProc.has(p)} onPress={() => toggle(setHideProc, p)} />
                ))}
              </ScrollView>
            </>
          )}
          {tab === 'rt' && (
            <Tappable onPress={() => setAutoscroll((v) => !v)}
              style={[styles.act, { borderColor: c.line, backgroundColor: autoscroll ? hexA(c.accent2, 0.16) : c.elev }]}>
              <Text style={{ color: autoscroll ? c.accent2 : c.dim, fontSize: 9 }}>{t('자동스크롤')}</Text>
            </Tappable>
          )}
          {tab === 'sys' && (
            <Tappable onPress={() => { if (sysDate && filtered.length) setSysPlayer(true); }}
              style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: 'rgba(63,185,80,0.5)', backgroundColor: 'rgba(63,185,80,0.10)', opacity: sysDate && filtered.length ? 1 : 0.4 }]}>
              <Icon name="play2" size={11} color={c.green} />
              <Text style={{ color: c.greenTx, fontSize: 9, fontWeight: '600' }}>{t('타임라인 재생')}</Text>
            </Tappable>
          )}
          {tab === 'bb' && bbSrc === 'robot' && (
            <>
              <Tappable onPress={() => {
                if (bbPlayerActive) { setBbPlayer(false); setBbListMode(true); }
                else if (bbDate && bbSession) { setBbPlayer(true); setBbListMode(false); }
              }}
                style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: 'rgba(63,185,80,0.5)', backgroundColor: bbPlayerActive ? 'rgba(63,185,80,0.28)' : 'rgba(63,185,80,0.10)', opacity: bbDate && bbSession ? 1 : 0.4 }]}>
                <Icon name={bbPlayerActive ? 'log' : 'play2'} size={11} color={c.green} />
                <Text style={{ color: c.greenTx, fontSize: 9, fontWeight: '600' }}>{bbPlayerActive ? t('로그 목록') : t('타임라인 재생')}</Text>
              </Tappable>
              <Tappable onPress={onBbSaveNow} disabled={notMine}
                style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: c.line, backgroundColor: c.elev, opacity: bbSavingNow || notMine ? 0.4 : 1 }]}>
                <Icon name="save" size={11} color={c.accent2} />
                <Text style={{ color: c.accent2, fontSize: 9 }}>{bbSavingNow ? t('저장 중…') : t('지금 저장')}</Text>
              </Tappable>
            </>
          )}
          {tab === 'bb' && (
            <>
              <Tappable onPress={onBbOpenZip}
                style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: c.line, backgroundColor: bbLocal ? 'rgba(77,156,245,0.14)' : c.elev }]}>
                <Icon name="log" size={11} color={c.accent2} />
                <Text style={{ color: c.accent2, fontSize: 9 }}>{bbLocal && bbSrc === 'robot' ? t('zip 재생 중') : t('zip 열기')}</Text>
              </Tappable>
            </>
          )}
          {!(tab === 'bb' && bbSrc === 'lib') && (
          <Tappable onPress={onSave}
            style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: c.line, backgroundColor: c.elev, opacity: saveDisabled ? 0.4 : 1 }]}>
            <Icon name="download" size={11} color={c.accent2} />
            <Text style={{ color: c.accent2, fontSize: 9 }}>{saving ? t('저장 중…') : tab === 'bb' ? t('zip 저장') : t('저장')}</Text>
          </Tappable>
          )}
          {uploadConfigured && accessLevel >= 3 && tab !== 'rt' && !(tab === 'bb' && bbSrc === 'lib') && (
            <Tappable onPress={() => canUpload && !upStage && setUpAsk(true)}
              style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: c.line, backgroundColor: c.elev, opacity: canUpload && !upStage ? 1 : 0.4 }]}>
              <Icon name="upload" size={11} color={c.accent2} />
              <Text style={{ color: c.accent2, fontSize: 9 }}>{upStage || t('서버로 보내기')}</Text>
            </Tappable>
          )}
          {uploadConfigured && accessLevel >= 3 && pending > 0 && (
            <Tappable onPress={() => !upStage && onFlush()} onLongPress={() => !upStage && setDropAsk(true)}
              style={[styles.act, { flexDirection: 'row', alignItems: 'center', gap: 5, borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.12)', opacity: upStage ? 0.4 : 1 }]}>
              <Icon name="upload" size={11} color={c.amberTx} />
              <Text style={{ color: c.amberTx, fontSize: 9, fontWeight: '700' }}>{t('보류')} {pending}</Text>
            </Tappable>
          )}
          <Text style={{ color: c.dim, fontSize: 10, marginLeft: 4 }}>
            {filtered.length.toLocaleString()}{filtered.length !== source.length ? `/${source.length.toLocaleString()}` : ''}{t('줄')}
          </Text>
        </View>

        <Animated.View key={tab + (bbPlayerActive ? ':play' : '')} entering={FadeIn.duration(160)} style={{ flex: 1 }}>
          {bbPlayerActive ? (
            bbLocal ? (
              <View style={{ flex: 1 }}>
                <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 16, paddingTop: 6 }}>
                  <Text style={{ color: c.accent2, fontSize: 10, fontWeight: '700' }}>📂 {bbLocal.name}</Text>
                  {bbSrc === 'robot' && (
                    <Tappable onPress={() => setBbLocal(null)}
                      style={[styles.act, { borderColor: c.line, backgroundColor: c.elev }]}>
                      <Text style={{ color: c.dim, fontSize: 9 }}>{t('닫기 — 로봇 세션으로')}</Text>
                    </Tappable>
                  )}
                </View>
                <BlackBoxPlayer ip={ip} date={bbDate} session={bbSession} preloaded={bbLocal.sess} />
              </View>
            ) : (
              <BlackBoxPlayer ip={ip} date={bbDate} session={bbSession} />
            )
          ) : tab === 'bb' && bbSrc === 'lib' ? (
            <View style={styles.center}>
              {libBusy ? <ActivityIndicator color={c.accent2} /> : (
                <Text style={{ color: bbErr ? c.redTx : c.dim, fontSize: 12, textAlign: 'center', paddingHorizontal: 24 }}>
                  {bbErr || (lib.length === 0
                    ? t('보관함이 비어 있습니다. 로봇 세션에서 zip 저장을 누르거나 zip 열기로 가져오면 로봇 S/N 별로 여기에 모입니다.')
                    : t('세션을 고르세요.'))}
                </Text>
              )}
            </View>
          ) : loading ? (
            <View style={styles.center}><ActivityIndicator color={c.accent2} /><Text style={{ color: c.dim, fontSize: 11, marginTop: 8 }}>{t('불러오는 중…')}</Text></View>
          ) : err ? (
            <View style={styles.center}><Icon name="x" size={20} color={c.redTx} /><Text style={{ color: c.muted, fontSize: 11, marginTop: 8, textAlign: 'center' }}>{err}</Text></View>
          ) : tab !== 'rt' && conn !== 'connected' && source.length === 0 ? (
            <View style={styles.center}>
              <Text style={{ color: c.dim, fontSize: 12, textAlign: 'center' }}>
                {tab === 'bb' ? t('로봇에 연결되면 불러옵니다. 저장해 둔 세션은 보관함에서 볼 수 있습니다.') : t('로봇에 연결되면 불러옵니다.')}
              </Text>
            </View>
          ) : view.length === 0 ? (
            <View style={styles.center}>
              <Text style={{ color: c.dim, fontSize: 12 }}>
                {source.length === 0 ? (tab === 'rt' ? t('로그 수신 대기 중…') : t('기록이 없습니다.')) : t('필터에 맞는 로그가 없습니다.')}
              </Text>
            </View>
          ) : (
            <FlatList
              ref={scrollRef}
              data={view}
              keyExtractor={(_, i) => String(i)}
              contentContainerStyle={styles.logBody}
              initialNumToRender={30}
              maxToRenderPerBatch={40}
              windowSize={9}
              removeClippedSubviews
              extraData={[focusLine, focusBlink]}
              onScrollToIndexFailed={(info) => {
                scrollRef.current?.scrollToOffset({ offset: info.averageItemLength * info.index, animated: false });
                setTimeout(() => scrollRef.current?.scrollToIndex({ index: info.index, viewPosition: 0.5, animated: true }), 120);
              }}
              ListHeaderComponent={filtered.length > MAX_VIEW ? (
                <Text style={{ color: c.amber, fontSize: 10, paddingBottom: 4 }}>{t('※ {n}줄 중 최신 {m}줄만 표시').replace('{n}', filtered.length.toLocaleString()).replace('{m}', MAX_VIEW.toLocaleString())}</Text>
              ) : null}
              renderItem={({ item: ln }: { item: LogLine }) => {
                const focused = tab === 'rt' && !!focusLine && ln.ts === focusLine.ts && ln.msg === focusLine.msg;
                return (
                  <View style={[
                    styles.ln,
                    tab === 'bb' && { borderLeftWidth: 2, borderLeftColor: procColor(ln.process), paddingLeft: 6 },
                    focused && {
                      backgroundColor: hexA(c.accent2, focusBlink ? 0.38 : 0.16),
                      borderRadius: 5, marginHorizontal: -6, paddingHorizontal: 6, paddingVertical: 2,
                    },
                  ]}>
                    <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 11 }}>{ln.ts}</Text>
                    <Text style={{ color: procColor(ln.process), fontFamily: fonts.mono, fontSize: 11, width: 92 }} numberOfLines={1}>[{ln.process}]</Text>
                    <Text style={{ color: LEVEL_HEX[ln.level], fontFamily: fonts.mono, fontSize: 11, fontWeight: ln.level === 'FATAL' ? '700' : '400' }}>{ln.level}</Text>
                    <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 11, flex: 1 }}>{ln.msg}</Text>
                  </View>
                );
              }}
            />
          )}
        </Animated.View>
      </View>

      {libDelAsk && (
        <Modal onClose={() => setLibDelAsk(false)}>
          <View style={[styles.dlg, { backgroundColor: c.panel, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('보관함에서 이 세션을 지울까요?')}</Text>
            <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18, fontFamily: fonts.mono }}>
              {`${serialLabel(libSel.serial)} · ${fmtDate(libSel.date)} ${fmtTime(libSel.session)}`}
            </Text>
            <View style={{ flexDirection: 'row', gap: 8, justifyContent: 'flex-end' }}>
              <Tappable onPress={() => setLibDelAsk(false)} style={[styles.act, { borderColor: c.line, backgroundColor: c.elev, paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.muted, fontSize: 12 }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={onLibDelete} style={[styles.act, { borderColor: c.redTx, backgroundColor: 'rgba(229,115,115,0.14)', paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.redTx, fontSize: 12, fontWeight: '700' }}>{t('삭제')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}

      {dropAsk && (
        <Modal onClose={() => setDropAsk(false)}>
          <View style={[styles.dlg, { backgroundColor: c.panel, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('보류함을 비울까요?')}</Text>
            <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>
              {t('보내지 않은 로그 {n}건이 지워집니다. 되돌릴 수 없습니다.').replace('{n}', String(pending))}
            </Text>
            <View style={{ flexDirection: 'row', gap: 8, justifyContent: 'flex-end' }}>
              <Tappable onPress={() => setDropAsk(false)} style={[styles.act, { borderColor: c.line, backgroundColor: c.elev, paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.muted, fontSize: 12 }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={onDropPending} style={[styles.act, { borderColor: c.redTx, backgroundColor: 'rgba(229,115,115,0.14)', paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.redTx, fontSize: 12, fontWeight: '700' }}>{t('비우기')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}

      {upAsk && (
        <Modal onClose={() => setUpAsk(false)}>
          <View style={[styles.dlg, { backgroundColor: c.panel, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('로그를 서버로 보냅니다')}</Text>
            <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>
              {t('선택한 로그를 압축해 제조사 로그 서버로 업로드합니다. 로그에는 로봇의 동작 기록과 오류 내용이 담깁니다.')}
            </Text>
            <View style={{ gap: 4 }}>
              <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono }}>{t('대상')}: {upLabel}</Text>
              {upSrcBytes > 0 && (
                <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono }}>{t('원본 크기')}: {fmtBytes(upSrcBytes)}</Text>
              )}
            </View>
            <View style={{ flexDirection: 'row', gap: 8, justifyContent: 'flex-end' }}>
              <Tappable onPress={() => setUpAsk(false)} style={[styles.act, { borderColor: c.line, backgroundColor: c.elev, paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.muted, fontSize: 12 }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={onUpload} style={[styles.act, { borderColor: c.accent, backgroundColor: c.accent, paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '700' }}>{t('보내기')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}

      {(upDone || upErr || saveErr) && (
        <Modal onClose={() => { setUpDone(''); setUpErr(''); setSaveErr(''); }}>
          <View style={[styles.dlg, { backgroundColor: c.panel, borderColor: c.line }]}>
            <Text style={{ color: upErr || saveErr ? c.redTx : c.greenTx, fontSize: 13, fontWeight: '700' }}>
              {saveErr ? t('저장 실패') : upErr ? t('전송 실패') : t('전송 완료')}
            </Text>
            <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{saveErr || upErr || upDone}</Text>
            <View style={{ flexDirection: 'row', justifyContent: 'flex-end' }}>
              <Tappable onPress={() => { setUpDone(''); setUpErr(''); setSaveErr(''); }} style={[styles.act, { borderColor: c.line, backgroundColor: c.elev, paddingHorizontal: 14, paddingVertical: 8 }]}>
                <Text style={{ color: c.muted, fontSize: 12 }}>{t('닫기')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}
    </View>
  );
}

function hexA(hex: string, a: number) {
  const h = hex.replace('#', '');
  const r = parseInt(h.substring(0, 2), 16), g = parseInt(h.substring(2, 4), 16), b = parseInt(h.substring(4, 6), 16);
  return `rgba(${r},${g},${b},${a})`;
}

const styles = StyleSheet.create({
  seg: { flexDirection: 'row', borderWidth: 1, borderRadius: 7, overflow: 'hidden' },
  segBtn: { paddingHorizontal: 10, paddingVertical: 5 },
  body: { flex: 1 },
  tabbar: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 16, paddingTop: 11 },
  tab: { paddingHorizontal: 15, paddingVertical: 8, borderTopLeftRadius: 9, borderTopRightRadius: 9, borderWidth: 1, borderBottomWidth: 0 },
  conn: { marginLeft: 'auto', flexDirection: 'row', alignItems: 'center', gap: 6 },
  connDot: { width: 7, height: 7, borderRadius: 4 },
  panel: { flex: 1, marginHorizontal: 16, marginBottom: 15, borderWidth: 1, borderTopLeftRadius: 0, borderRadius: 12, overflow: 'hidden' },
  srcbar: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 14, paddingVertical: 8, borderBottomWidth: 1, zIndex: 30 },
  dropList: { position: 'absolute', minWidth: 150, borderWidth: 1, paddingVertical: 4 },
  dropRow: { paddingVertical: 7, paddingHorizontal: 12 },
  fetch: { flexDirection: 'row', alignItems: 'center', gap: 5, paddingHorizontal: 10, paddingVertical: 5, borderRadius: 7 },
  pick: { paddingHorizontal: 9, paddingVertical: 4, borderRadius: 7, borderWidth: 1 },
  filt: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 14, paddingVertical: 9, borderBottomWidth: 1, flexWrap: 'wrap' },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 5, paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  dot: { width: 7, height: 7, borderRadius: 4 },
  searchbox: { marginLeft: 'auto', flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 10, paddingVertical: 5, borderRadius: 7, borderWidth: 1 },
  actbar: { flexDirection: 'row', alignItems: 'center', gap: 7, paddingHorizontal: 14, paddingVertical: 7, borderBottomWidth: 1 },
  procRow: { flexDirection: 'row', gap: 6, alignItems: 'center', paddingRight: 4 },
  act: { paddingHorizontal: 9, paddingVertical: 4, borderRadius: 7, borderWidth: 1 },
  logBody: { padding: 14, gap: 3 },
  ln: { flexDirection: 'row', gap: 9 },
  center: { flex: 1, alignItems: 'center', justifyContent: 'center', padding: 20 },
  dlg: { width: 380, maxWidth: '92%', gap: 12, padding: 18, borderWidth: 1, borderRadius: 14 },
});
