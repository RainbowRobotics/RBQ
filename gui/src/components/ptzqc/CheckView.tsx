import { View, Text, ScrollView } from 'react-native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { RBBadge } from '@/rb/components/RBBadge';
import { Segmented } from '@/components/ui/controls';
import { Icon } from '@/components/Icon';
import { useCompactW } from '@/lib/layout';
import { t } from '@/lib/i18n';
import { getLang } from '@/store/lang';
import {
  SOAK_CHOICES, clockLabel, currentStep, durLabel, elapsedOf, kindLabel, pick, planRows, progressOf, recentUse,
  shortTime, stepCount, type EngineStatus, type HistoryRow, type Level, type Phase, type QcStatus, type SoakLive, type SoakMin,
} from '@/lib/ptzQc';
import { BigBtn, Card, KvRow, Note, Tag, VerdictChip, bareName, Bar, styles as ps } from '@/components/ptzqc/parts';
import { QcChart } from '@/components/ptzqc/QcChart';

export type StartKind = 'basic' | 'aging';

function Columns({ left, right }: { left: React.ReactNode; right: React.ReactNode }) {
  const narrow = useCompactW();
  const col = { gap: 12, paddingBottom: 12 };
  if (narrow) {
    return (
      <ScrollView style={{ flex: 1 }} contentContainerStyle={col} showsVerticalScrollIndicator={false}>
        {left}{right}
      </ScrollView>
    );
  }
  return (
    <View style={{ flex: 1, flexDirection: 'row', gap: 14 }}>
      <ScrollView style={{ flex: 1.9 }} contentContainerStyle={col} showsVerticalScrollIndicator={false}>{left}</ScrollView>
      <ScrollView style={{ flex: 1 }} contentContainerStyle={col} showsVerticalScrollIndicator={false}>{right}</ScrollView>
    </View>
  );
}


export function IdleView({ level, staff, status, last, hasPtz, blocked, canSerial, soakMin, setSoakMin, onStart, onSerial, onOpenLast, msg }: {
  level: Level; staff: boolean; status: QcStatus | null; last: HistoryRow | null; hasPtz: boolean;
  blocked: string | null;
  canSerial: boolean;
  soakMin: SoakMin; setSoakMin: (m: SoakMin) => void;
  onStart: (k: StartKind) => void; onSerial: () => void; onOpenLast: () => void; msg: string | null;
}) {
  return (
    <Columns
      left={<>
        <DeviceCard staff={staff} status={status} last={last} hasPtz={hasPtz} canSerial={canSerial && !blocked} onSerial={onSerial} />
        <StartCard level={level} blocked={blocked} soakMin={soakMin} setSoakMin={setSoakMin} onStart={onStart} />
        {msg ? <Note tone="danger" text={msg} /> : null}
        <LastCard last={last} onOpen={onOpenLast} />
      </>}
      right={<PlanCard level={level} />} />
  );
}

function DeviceCard({ staff, status, last, hasPtz, canSerial, onSerial }: {
  staff: boolean; status: QcStatus | null; last: HistoryRow | null; hasPtz: boolean; canSerial: boolean; onSerial: () => void;
}) {
  const { c } = useTheme();
  const ls = status?.daemon?.lastSerial;
  const serial = (ls?.ok && ls.serial) || last?.serial || '';
  const v = last?.versions;
  const use = recentUse(status);
  return (
    <Card n="1" title={t('장비')} right={hasPtz ? <RBBadge type="capsule" color="green">{t('PTZ 인식')}</RBBadge> : <RBBadge type="soft" color="gray">{t('PTZ 없음')}</RBBadge>}>
      <KvRow k={t('게이트웨이 UID')} v={last?.uid || '—'} mono />
      <KvRow k={t('시리얼')} v={serial || t('모름')} mono={!!serial}
        extra={staff ? <BigBtn small kind="ghost" icon="tag" label={t('시리얼 고치기')} disabled={!canSerial} onPress={onSerial} /> : null} />
      <KvRow k={t('펌웨어')} v={v ? `${t('게이트웨이')} ${v.gw ?? '—'} · YAW ${v.yaw ?? '—'} · PITCH ${v.pitch ?? '—'}` : '—'} mono />
      <KvRow k={t('최근 PTZ 조작')} v={use.text}
        extra={<Text style={{ color: use.warn ? c.amberTx : c.dim, fontSize: 12 }}>— {t('10초 안이면 시작할 때 경고합니다')}</Text>} />
      {ls && !ls.ok ? <Note tone="warn" text={`${t('시리얼을 쓰지 못했습니다')} — ${ls.why ?? ''}`} /> : null}
      {last ? <Text style={{ color: c.dim, fontSize: 11.5, marginTop: 4 }}>{t('UID · 펌웨어는 마지막 검사 때 읽은 값입니다.')}</Text> : null}
    </Card>
  );
}

function StartCard({ level, blocked, soakMin, setSoakMin, onStart }: {
  level: Level; blocked: string | null; soakMin: SoakMin; setSoakMin: (m: SoakMin) => void; onStart: (k: StartKind) => void;
}) {
  const { c, radius } = useTheme();
  const box = { flex: 1, minWidth: 220, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, padding: 12, gap: 10, backgroundColor: c.elev };
  return (
    <Card n="2" title={t('검사')}>
      {level === 1 ? (
        <View style={[ps.row, { gap: 14, alignItems: 'flex-start' }]}>
          <BigBtn kind="primary" icon="play2" label={t('검사 시작')} disabled={!!blocked} onPress={() => onStart('basic')} />
          <Text style={{ color: c.muted, fontSize: 12.5, flex: 1, minWidth: 220, lineHeight: 19 }}>
            {t('16단계 · 약 2분. 축이 끝에서 끝까지 움직이고, 카메라 스피커에서 삐 소리가 납니다. 프리셋 · 영점 보정 · 펌웨어 굽기는 하지 않습니다.')}
          </Text>
        </View>
      ) : (
        <>
          <View style={[ps.row, { gap: 12, alignItems: 'stretch' }]}>
            <View style={box}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('기본 검사')} <Text style={{ color: c.dim, fontSize: 12, fontWeight: '400' }}>{t('1~18단계 · 약 2분')}</Text></Text>
              <View style={{ alignItems: 'flex-start' }}><BigBtn kind="primary" icon="play2" label={t('기본 검사 시작')} disabled={!!blocked} onPress={() => onStart('basic')} /></View>
            </View>
            <View style={box}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('에이징 검사')} <Text style={{ color: c.dim, fontSize: 12, fontWeight: '400' }}>{t('1~20단계 · 공회전 뒤 재검사')}</Text></Text>
              <View style={{ maxWidth: 260 }}>
                <Segmented<string> options={SOAK_CHOICES.map((m) => ({ key: String(m), label: durLabel(m * 60) }))} value={String(soakMin)}
                  onChange={(v) => setSoakMin(Number(v) as SoakMin)} />
              </View>
              <View style={{ alignItems: 'flex-start' }}><BigBtn kind="primary" icon="play2" label={t('에이징 검사 시작')} disabled={!!blocked} onPress={() => onStart('aging')} /></View>
            </View>
          </View>
          <Text style={{ color: c.dim, fontSize: 12, marginTop: 8 }}>
            {t('영점 보정 포함 · 프리셋은 시작할 때 동의 · 어느 기기에서든 시작하고 멈출 수 있습니다')}
          </Text>
        </>
      )}
      {blocked ? <View style={{ marginTop: 10 }}><Note tone="warn" text={blocked} /></View> : null}
    </Card>
  );
}

function LastCard({ last, onOpen }: { last: HistoryRow | null; onOpen: () => void }) {
  const { c } = useTheme();
  const en = getLang() === 'en';
  return (
    <Card n="3" title={t('마지막 결과')}>
      {last ? (
        <View style={[ps.row, { gap: 12, flexWrap: 'nowrap' }]}>
          <VerdictChip v={last.verdict} big />
          <View style={{ flex: 1, minWidth: 0 }}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{shortTime(last.startedAt)} · {kindLabel(last.kind, last.soakMin)}</Text>
            <Text style={{ color: c.muted, fontSize: 12.5 }} numberOfLines={2}>{pick(last.whyBi, en) || last.why}</Text>
          </View>
          <BigBtn small kind="ghost" icon="log" label={t('결과 보기')} onPress={onOpen} />
        </View>
      ) : (
        <Text style={{ color: c.dim, fontSize: 13 }}>{t('이 로봇에서 한 PTZ 검사가 아직 없습니다.')}</Text>
      )}
    </Card>
  );
}

function PlanCard({ level }: { level: Level }) {
  const { c, fonts } = useTheme();
  return (
    <Card title={t('검사 순서')} right={<Tag label={level === 1 ? t('레벨 1 · 16단계') : t('레벨 2 · 3')} />}>
      {planRows(level).map((r) => (
        <View key={r.no} style={{ flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 4, borderBottomWidth: 1, borderColor: c.line, opacity: r.off ? 0.5 : 1 }}>
          <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 12, width: 22, textAlign: 'right' }}>{r.no}</Text>
          <Text style={{ color: c.text, fontSize: 12.5, flex: 1 }} numberOfLines={1}>{r.name}</Text>
          {r.tag ? <Tag label={r.tag} /> : null}
        </View>
      ))}
    </Card>
  );
}


export function RunView({ phase, status, onStop, stopping, canStop, stale }: {
  phase: Phase; status: QcStatus | null; level: Level; onStop: () => void; stopping: boolean;
  canStop: boolean;
  stale: string | null;
}) {
  const raw = status?.status;
  const st = raw && (raw.state === 'running' || phase === 'finishing') ? raw : undefined;
  const soaking = phase === 'soak';
  return (
    <Columns
      left={<>
        {stale ? <Note tone="warn" text={stale} /> : null}
        <ProgressCard phase={phase} st={st} onStop={onStop} stopping={stopping} canStop={canStop} />
        {soaking && st?.soak ? <CurrentCard soak={st.soak} /> : <StepTable st={st} phase={phase} />}
      </>}
      right={<>
        {soaking && st?.soak ? <SoakStatusCard soak={st.soak} /> : <PtzNowCard st={st} />}
        <RobotCard st={st} />
        <Note text={t('검사 중에는 PTZ 를 조작할 수 없습니다. 앱을 닫아도 로봇에서 끝까지 돌고, 결과는 [결과] 칸에 남습니다.')} />
      </>} />
  );
}

function headOf(phase: Phase, st: EngineStatus | undefined): { chip: string; tone: 'blue' | 'red' | 'orange'; title: string; sub?: string } {
  const kind = st?.kind ?? 'basic';
  const name = kindLabel(kind, kind === 'aging' && st?.soak?.plannedS ? st.soak.plannedS / 60 : undefined);
  const cur = currentStep(st);
  const { done: decided, total: n } = stepCount(st);
  const at = n ? `${Math.min(decided + 1, n)}/${n}` : '';
  switch (phase) {
    case 'starting': return { chip: t('시작 중'), tone: 'blue', title: st ? `${name} · ${t('게이트웨이를 넘겨받고 PTZ 에 붙는 중')}` : t('게이트웨이를 넘겨받고 PTZ 에 붙는 중') };
    case 'serial': return { chip: t('시리얼'), tone: 'blue', title: t('제품 시리얼을 쓰는 중') };
    case 'ask': return { chip: 'QC FAIL', tone: 'red', title: `${name} · ${t('QC 1~18 끝')} — FAIL ${(st?.gate?.fails ?? []).length}${t('곳')}` };
    case 'interim': return { chip: t('중간 보고'), tone: 'blue', title: `${name} · ${t('QC 끝 — 1분 뒤 공회전을 시작합니다')}`, sub: `${t('QC 판정')} ${st?.qc ?? '—'}` };
    case 'soak': {
      const s = st?.soak;
      return { chip: t('에이징 중'), tone: 'blue', title: `19 ${t('공회전')} · ${durLabel(s?.elapsedS)} / ${durLabel(s?.plannedS)}`, sub: s?.brief };
    }
    case 'recheck': return { chip: t('재검사 중'), tone: 'blue', title: `20 ${t('재검사')} · ${at}${cur ? ` ${bareName(cur.name)}` : ''}` };
    case 'finishing': return { chip: t('마무리'), tone: 'blue', title: `${name} · ${t('보고서를 만드는 중')}` };
    default: return { chip: t('검사 중'), tone: 'blue', title: `${name} · ${at}${cur ? ` ${bareName(cur.name)}` : ''}` };
  }
}

function ProgressCard({ phase, st, onStop, stopping, canStop }: {
  phase: Phase; st: EngineStatus | undefined; onStop: () => void; stopping: boolean; canStop: boolean;
}) {
  const { c, fonts } = useTheme();
  const h = headOf(phase, st);
  const elapsed = elapsedOf(st);
  const soak = st?.soak;
  const endsAt = phase === 'soak' && soak?.plannedS != null && soak.elapsedS != null
    ? new Date(Date.now() + (soak.plannedS - soak.elapsedS) * 1000) : null;
  const pct = progressOf(st);
  return (
    <Card>
      <View style={[ps.row, { gap: 10, flexWrap: 'nowrap' }]}>
        <RBBadge type="capsule" color={h.tone}>{h.chip}</RBBadge>
        <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', flexShrink: 1 }} numberOfLines={1}>{h.title}</Text>
        {h.sub ? <Text style={{ color: c.greenTx, fontSize: 12, fontFamily: fonts.mono, flexShrink: 2 }} numberOfLines={1}>{h.sub}</Text> : null}
        <View style={{ flex: 1 }} />
        {endsAt ? <Text style={{ color: c.muted, fontSize: 12 }}>{t('끝나는 때')} {`${String(endsAt.getMonth() + 1).padStart(2, '0')}-${String(endsAt.getDate()).padStart(2, '0')} ${String(endsAt.getHours()).padStart(2, '0')}:${String(endsAt.getMinutes()).padStart(2, '0')}`}</Text>
          : elapsed != null ? <Text style={{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }}>{clockLabel(elapsed)}</Text> : null}
        {phase !== 'serial' && phase !== 'finishing' ? (
          <BigBtn small kind="danger" icon="x" label={stopping || st?.stopping ? t('멈추는 중…') : t('중지')} disabled={!canStop || stopping || !!st?.stopping} onPress={onStop} />
        ) : null}
      </View>
      <View style={{ marginTop: 10 }}><Bar pct={phase === 'starting' ? 0.02 : pct} tone={phase === 'ask' ? 'danger' : 'brand'} /></View>
      {st?.kind === 'aging' ? (
        <Text style={{ color: c.dim, fontSize: 12, marginTop: 6 }}>
          {t('1~18 QC → 19 공회전 → 20 재검사 뒤 최종 보고 · 앱을 닫아도 로봇에서 계속 돕니다')}
        </Text>
      ) : null}
    </Card>
  );
}

function StepTable({ st, phase }: { st: EngineStatus | undefined; phase: Phase }) {
  const { c, fonts } = useTheme();
  const { c: rc } = useRb();
  const steps = st?.steps ?? [];
  if (!steps.length) {
    return <Card><Text style={{ color: c.dim, fontSize: 13 }}>{phase === 'starting' ? t('PTZ 에 붙는 중입니다 — 몇 초 걸립니다.') : t('단계 표를 기다리는 중입니다.')}</Text></Card>;
  }
  return (
    <Card>
      <View style={{ flexDirection: 'row', paddingBottom: 6, borderBottomWidth: 1, borderColor: c.line }}>
        <Text style={{ color: c.dim, fontSize: 11.5, width: 28 }} />
        <Text style={{ color: c.dim, fontSize: 11.5, width: 150 }}>{t('검사')}</Text>
        <Text style={{ color: c.dim, fontSize: 11.5, width: 74 }}>{t('판정')}</Text>
        <Text style={{ color: c.dim, fontSize: 11.5, flex: 1 }}>{t('결과')}</Text>
      </View>
      {steps.map((s) => {
        const run = s.verdict === 'RUN';
        const wait = s.verdict === 'WAIT' || !s.verdict;
        return (
          <View key={s.n} style={{ flexDirection: 'row', alignItems: 'center', paddingVertical: 6, borderBottomWidth: 1, borderColor: c.line,
            backgroundColor: run ? rc('fill-information-subtlest') : 'transparent', opacity: wait ? 0.55 : 1 }}>
            <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 12, width: 28, textAlign: 'right', paddingRight: 8 }}>{s.n}</Text>
            <Text style={{ color: c.text, fontSize: 12.5, fontWeight: wait ? '400' : '700', width: 150 }} numberOfLines={1}>{bareName(s.name)}</Text>
            <View style={{ width: 74 }}><VerdictChip v={s.verdict} /></View>
            <Text style={{ color: c.muted, fontSize: 12, flex: 1 }} numberOfLines={1}>{s.measured}</Text>
          </View>
        );
      })}
    </Card>
  );
}

function PtzNowCard({ st }: { st: EngineStatus | undefined }) {
  const { c, fonts, radius } = useTheme();
  const { c: rc } = useRb();
  const p = st?.ptz;
  const tile = (label: string, value: string, ok?: boolean) => (
    <View style={{ flex: 1, minWidth: 120, borderWidth: 1, borderRadius: radius.sm, padding: 10, gap: 2,
      backgroundColor: ok ? rc('fill-success-subtler') : c.elev, borderColor: ok ? rc('border-success') : c.line }}>
      <Text style={{ color: c.muted, fontSize: 12, fontWeight: '600' }}>{label}</Text>
      <Text style={{ color: ok ? c.greenTx : c.text, fontSize: 19, fontWeight: '700', fontFamily: fonts.mono }}>{value}</Text>
    </View>
  );
  const drive = p?.drive === 'on' ? `ON${p.homed ? ` · ${t('홈 완료')}` : ''}` : p?.drive === 'off' ? 'OFF' : p?.drive === 'fault' ? t('폴트') : '—';
  return (
    <Card title={t('PTZ 지금')}>
      <View style={[ps.row, { gap: 8 }]}>
        {tile('YAW', p?.yaw != null ? `${p.yaw.toFixed(1)}°` : '—', p?.yaw != null)}
        {tile('PITCH', p?.pitch != null ? `${p.pitch.toFixed(1)}°` : '—', p?.pitch != null)}
      </View>
      <View style={[ps.row, { gap: 8, marginTop: 8 }]}>
        {tile(t('구동'), drive)}
        {tile(t('왕복 p99'), p?.rttP99 != null ? `${p.rttP99.toFixed(1)} ms` : '—')}
      </View>
    </Card>
  );
}

function RobotCard({ st }: { st: EngineStatus | undefined }) {
  const { c } = useTheme();
  const r = st?.robot;
  if (!r?.seen) return null;
  return (
    <Card title={t('로봇')}>
      <KvRow k={t('자세')} v={r.gait || '—'} />
      <KvRow k={t('움직임')} v={r.moving ? t('움직이는 중 — 그동안 잰 값에는 흔들림이 섞일 수 있습니다') : t('서 있음')} />
      <KvRow k={t('배터리')} v={r.battery != null ? `${Math.round(r.battery)} %` : t('외부 전원')}
        extra={r.battery != null ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('10 % 이하가 되면 검사를 멈춥니다(무효)')}</Text> : null} />
    </Card>
  );
}


function CurrentCard({ soak }: { soak: SoakLive }) {
  const { c } = useTheme();
  const mm = soak.cur?.minuteMax ?? [];
  const cap = soak.cur?.capState === 'on' ? soak.cur?.pitchCapA ?? null : null;
  return (
    <Card>
      <View style={[ps.row, { gap: 8, marginBottom: 6 }]}>
        <Text style={{ color: c.text, fontSize: 13.5, fontWeight: '700' }}>{t('전류 — 100 ms 기록 · 1분 최대')} [A]</Text>
        <Text style={{ color: c.dim, fontSize: 12 }}>{t('파랑 PITCH · 회색 YAW')}</Text>
        <View style={{ flex: 1 }} />
        {cap != null ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('상한({a} A)에 닿은 시간').replace('{a}', String(cap))} {durLabel(soak.cur?.atCapS ?? 0)}</Text> : null}
      </View>
      <QcChart bucketS={60} totalS={soak.plannedS ?? mm[0]?.length * 60} height={200} limit={cap} limitLabel={cap != null ? t('에이징 상한 {a} A').replace('{a}', String(cap)) : undefined}
        series={[{ values: mm[0] ?? [], color: c.dim, width: 1.2 }, { values: mm[1] ?? [], color: c.accent, width: 1.6 }]} />
      {soak.cur?.capState && soak.cur.capState !== 'on' ? (
        <Text style={{ color: c.dim, fontSize: 12, marginTop: 4 }}>
          {soak.cur.capState === 'refused' ? t('모터가 임시 상한을 거절해 상한 없이 돕니다.') : soak.cur.capState === 'noReply' ? t('모터 펌웨어에 임시 상한 기능이 없어 상한 없이 돕니다.') : ''}
        </Text>
      ) : null}
    </Card>
  );
}

function SoakStatusCard({ soak }: { soak: SoakLive }) {
  const { c, fonts } = useTheme();
  const ok = (soak.verdict ?? 'PASS') === 'PASS';
  const row = (name: string, sub: string, yaw: string | null, pitch: string | null, good: boolean, both?: string) => (
    <View style={{ flexDirection: 'row', alignItems: 'center', paddingVertical: 8, borderBottomWidth: 1, borderColor: c.line }}>
      <View style={{ flex: 1.3 }}>
        <Text style={{ color: c.text, fontSize: 13 }}>{name}</Text>
        <Text style={{ color: c.dim, fontSize: 11 }}>{sub}</Text>
      </View>
      {both != null ? <Text style={{ color: c.text, flex: 2, textAlign: 'right', fontFamily: fonts.mono, fontSize: 13, fontWeight: '700' }}>{both}</Text> : (
        <>
          <Text style={{ color: c.text, flex: 1, textAlign: 'right', fontFamily: fonts.mono, fontSize: 13, fontWeight: '700' }}>{yaw ?? '—'}</Text>
          <Text style={{ color: c.text, flex: 1, textAlign: 'right', fontFamily: fonts.mono, fontSize: 13, fontWeight: '700' }}>{pitch ?? '—'}</Text>
        </>
      )}
      <View style={{ width: 28, alignItems: 'flex-end' }}><Icon name={good ? 'check' : 'warn'} size={16} color={good ? c.green : c.amber} /></View>
    </View>
  );
  const fb = soak.fb, tr = soak.track, rtt = soak.rtt, fps = soak.fps;
  const n = (v: number | null | undefined, d = 0, u = '') => (v == null || v < 0 ? null : `${v.toFixed(d)}${u}`);
  const cam = (i: number) => (fps?.measured?.[i] === false ? null : n(fps?.min?.[i], 0));
  const fbOk = !(fb?.lowS ?? []).some((x) => x > 0);
  const trOk = !(tr?.hardS ?? []).some((x) => x > 0);
  const rttOk = !rtt?.latencyS && !rtt?.lost && !rtt?.linkDownS;
  const fpsOk = !(fps?.badS ?? []).some((x) => x > 0);
  const lim = (tr?.limit ?? []).map((x) => (x == null ? '—' : `${x.toFixed(1)}°`)).join(' · ');
  return (
    <Card title={t('지금까지 상태')} right={<View style={[ps.row, { gap: 6 }]}>
      <Text style={{ color: c.dim, fontSize: 12 }}>{t('{d} 동안').replace('{d}', durLabel(soak.elapsedS))}</Text>
      <RBBadge type="capsule" color={ok ? 'green' : 'orange'}>{ok ? t('이상 없음') : (soak.verdict ?? '')}</RBBadge>
    </View>}>
      <View style={{ flexDirection: 'row', paddingBottom: 4, borderBottomWidth: 1, borderColor: c.line }}>
        <Text style={{ color: c.dim, fontSize: 11.5, flex: 1.3 }}>{t('항목')}</Text>
        <Text style={{ color: c.dim, fontSize: 11.5, flex: 1, textAlign: 'right' }}>YAW</Text>
        <Text style={{ color: c.dim, fontSize: 11.5, flex: 1, textAlign: 'right' }}>PITCH</Text>
        <View style={{ width: 28 }} />
      </View>
      {row(t('위치 신호'), t('초당 {n}번 이상').replace('{n}', String(fb?.limit ?? 90)), n(fb?.min?.[0], 0, '/s'), n(fb?.min?.[1], 0, '/s'), fbOk)}
      {row(t('명령과 위치 차이'), soak.baselineReady ? `${lim} ${t('이하')}` : t('기준 구간이 끝나면 한계를 정합니다'), n(tr?.max?.[0], 2, '°'), n(tr?.max?.[1], 2, '°'), trOk)}
      {row(t('통신 응답 시간'), `${rtt?.limitMs ?? 100} ms ${t('이하')}`, null, null, rttOk, n(rtt?.p99Ms, 1, ' ms') ?? '—')}
      {row(t('카메라 영상'), t('끊기면 이상'), null, null, fpsOk,
        `${t('줌')} ${cam(0) ?? '—'} · ${t('열화상')} ${cam(1) ?? '—'} fps`)}
      {(soak.reasons ?? []).length ? (
        <Text style={{ color: c.amberTx, fontSize: 12, marginTop: 8 }}>{(soak.reasons ?? []).join(' · ')}</Text>
      ) : null}
    </Card>
  );
}
