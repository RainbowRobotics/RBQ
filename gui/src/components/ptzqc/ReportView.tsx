import { useRef, useState } from 'react';
import { View, Text, ScrollView, Pressable, Image } from 'react-native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { getLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { ptzQcApi } from '@/lib/ptzQcApi';
import { durLabel, kindLabel, pick, refusalText, verdictWord, type Bi, type Level } from '@/lib/ptzQc';
import { BigBtn, Card, KvRow, Note, VerdictChip, styles as ps } from '@/components/ptzqc/parts';
import { QcChart } from '@/components/ptzqc/QcChart';
import { QcVideo } from '@/components/ptzqc/QcVideo';

type Cell = { t: string; en: string | null; c: string };
type Table = { cols: Bi[]; rows: Cell[][]; num: number[]; caption: Bi; note: Bi; fold: Bi; levelMin?: number };
type Step = {
  no: number; name: Bi; verdict: string; line: Bi; skip: Bi; what: Bi; how: Bi; why: Bi;
  crit: { tag: string; text: Bi }[]; tables: Table[]; notes: Bi[]; moving?: boolean;
};
type Panel = { key: string; group: string; who: Bi; unit: string; lo: number | null; hi: number | null; limFrom: number | null;
  min: (number | null)[]; avg: (number | null)[]; max: (number | null)[]; lat: [number, number][]; real: [number, number][] };
type Series = { bucketS: number; n: number; extentS: number; lastS: number; baseS: number; groups: { key: string; title: Bi; unit: string; desc: Bi }[]; panels: Panel[] };
type Clip = { cam: string; label: Bi; file: string; bytes: number; part: string; steps: Record<string, number> };
type Attempt = { n: number; stem: string; startedAt: string; overall?: string; picked?: number[]; ran?: number[]; missing?: boolean;
  steps?: { no: number; name: Bi; verdict: string; line: Bi; measured: string; prep: boolean }[];
  cond?: { versions?: string; gait?: string; battery?: number | null; sinceFirst?: string } };
export type Report = {
  stem: string; stage: string; kind: string; level: number; demo?: boolean;
  head: { serial: string; uid: string; startedAt: string; endedAt: string; seconds: string; versions: { gw?: string; yaw?: string; pitch?: string };
    presets?: boolean; robotMoving?: boolean; batteryMin?: number | null; notice?: Bi; verdict: string; why: Bi; customer?: Bi; category?: string };
  clips: Clip[];
  summary: { no: number; name: Bi; verdict: string; line: Bi; moving?: boolean; attempts?: number; lastAttempt?: string; effective?: string }[];
  metrics: { label: Bi; value: string; unit: string; limit: Bi; near: string; nearEn: string | null }[];
  metricsNote: Bi;
  steps: Step[];
  soak: (Step & { series?: Series | null; curCap?: { pitchA: number; on: boolean; state: string; atCapS: number } | null }) | null;
  recheck: (Step & { stem2?: string; clips?: Clip[]; steps2?: Step[] }) | null;
  reruns: Attempt[];
  photos: { file: string; caption: Bi }[];
  appendix: [string, string][];
};

export function ReportView({ rep, level, onBack }: { rep: Report; level: Level; onBack: () => void }) {
  const { c } = useTheme();
  const en = getLang() === 'en';
  const scroll = useRef<ScrollView>(null);
  const at = useRef<Record<number, number>>({});
  const go = (no: number) => { const y = at.current[no]; if (y != null) scroll.current?.scrollTo({ y: Math.max(0, y - 8), animated: true }); };
  const mark = (no: number) => (e: { nativeEvent: { layout: { y: number } } }) => { at.current[no] = e.nativeEvent.layout.y; };
  const qcRows = rep.summary.filter((r) => r.no <= 18);
  const counts = (['PASS', 'WARN', 'FAIL', 'SKIP'] as const).map((v) => [v, qcRows.filter((r) => r.verdict === v).length] as const);
  const reran = qcRows.filter((r) => r.attempts).length;
  return (
    <ScrollView ref={scroll} contentContainerStyle={{ gap: 14, paddingBottom: 24 }} showsVerticalScrollIndicator>
      <View style={{ flexDirection: 'row' }}><BigBtn small kind="ghost" icon="chevL" label={t('이력')} onPress={onBack} /></View>
      <Head rep={rep} en={en} />
      {rep.head.customer && rep.head.verdict !== 'PASS' ? <Note tone={rep.head.verdict === 'FAIL' ? 'danger' : 'warn'} title={pick(rep.head.customer, en)} /> : null}
      {rep.head.notice ? <Note tone="warn" text={pick(rep.head.notice, en)} /> : null}
      <View style={[ps.row, { gap: 10 }]}>
        <Text style={{ color: c.muted, fontSize: 13 }}>{t('QC 1~18단계')}</Text>
        {counts.map(([v, n]) => <View key={v} style={[ps.row, { gap: 4 }]}><VerdictChip v={v} /><Text style={{ color: c.text, fontSize: 13 }}>{n}</Text></View>)}
        {reran ? <Text style={{ color: c.text, fontSize: 13 }}>· {t('다시 돌린 단계')} {reran}</Text> : null}
        {rep.demo ? <Text style={{ color: c.dim, fontSize: 12 }}>· {t('데모 보고서(실측 자료를 줄인 것)')}</Text> : null}
      </View>
      {rep.clips.length ? <Clips clips={rep.clips} en={en} /> : null}
      <Summary rep={rep} en={en} onGo={go} />
      {rep.metrics.length ? <Metrics rep={rep} en={en} /> : null}
      {rep.steps.map((s) => <View key={s.no} onLayout={mark(s.no)}><StepSection s={s} en={en} level={level} attempts={rep.reruns} /></View>)}
      {rep.soak ? <View onLayout={mark(19)}><StepSection s={rep.soak} en={en} level={level}><SoakCharts soak={rep.soak} en={en} /></StepSection></View> : null}
      {rep.recheck ? (
        <View onLayout={mark(20)}>
          <StepSection s={rep.recheck} en={en} level={level}>
            {rep.recheck.clips?.length ? <Clips clips={rep.recheck.clips} en={en} /> : null}
          </StepSection>
          {rep.recheck.steps2?.length ? <Recheck2 steps={rep.recheck.steps2} en={en} level={level} /> : null}
        </View>
      ) : null}
      {rep.reruns.length ? <Reruns rep={rep} en={en} /> : null}
      {rep.photos.length ? <Photos photos={rep.photos} en={en} /> : null}
      {level >= 2 && rep.appendix.length ? <Appendix rows={rep.appendix} /> : null}
    </ScrollView>
  );
}

function Head({ rep, en }: { rep: Report; en: boolean }) {
  const { c, fonts, radius } = useTheme();
  const h = rep.head;
  const tone = { PASS: c.greenTx, WARN: c.amberTx, FAIL: c.redTx }[h.verdict as 'PASS'] ?? c.muted;
  const lineTone = { PASS: c.green, WARN: c.amber, FAIL: c.red }[h.verdict as 'PASS'] ?? c.line;
  const v = h.versions ?? {};
  return (
    <View style={[ps.row, { gap: 16, alignItems: 'flex-start' }]}>
      <View style={{ flex: 1, minWidth: 280, gap: 2 }}>
        <Text style={{ color: c.accent, fontSize: 12, fontWeight: '700' }}>
          {t('PTZ 점검 기록')} · {kindLabel(rep.kind, rep.soak?.series?.extentS ? Math.round(rep.soak.series.extentS / 60) : undefined)} · {t('레벨')} {rep.level === 1 ? 1 : '2 · 3'}
        </Text>
        <Text style={{ color: c.text, fontSize: 30, fontWeight: '800', fontFamily: fonts.mono, marginBottom: 6 }}>{h.serial || t('시리얼 모름')}</Text>
        <KvRow k={t('게이트웨이 UID')} v={h.uid || '—'} mono />
        <KvRow k={t('펌웨어')} v={`${t('게이트웨이')} ${v.gw ?? '—'} · YAW ${v.yaw ?? '—'} · PITCH ${v.pitch ?? '—'}`} mono />
        <KvRow k={t('검사 일시')} v={h.startedAt || '—'} mono />
        <KvRow k={t('걸린 시간')} v={h.seconds ? durLabel(Number(h.seconds)) : '—'} />
        <KvRow k={t('결과 파일')} v={`${rep.stem}.csv`} mono />
        {h.batteryMin != null ? <KvRow k={t('가장 낮았던 배터리')} v={`${Math.round(h.batteryMin)} %`} /> : null}
        {h.robotMoving ? <KvRow k={t('로봇 움직임')} v={t('검사 중에 로봇이 움직인 때가 있습니다 — 해당 단계에 표시')} /> : null}
      </View>
      <View style={{ minWidth: 260, flex: 0.8, borderWidth: 2, borderColor: lineTone, borderRadius: radius.sm, padding: 12, alignItems: 'center', gap: 4 }}>
        <Text style={{ color: tone, fontSize: 32, fontWeight: '800', fontFamily: fonts.mono }}>{verdictWord(h.verdict)}</Text>
        <Text style={{ color: tone, fontSize: 12, textAlign: 'center' }}>{t('출하 판정')} — {pick(h.why, en)}</Text>
      </View>
    </View>
  );
}

function Clips({ clips, en }: { clips: Clip[]; en: boolean }) {
  const { c, fonts, radius } = useTheme();
  const marks = Object.entries(clips[0]?.steps ?? {}).sort((a, b) => a[1] - b[1]);
  const remote = !ptzQcApi.mediaUrl(clips[0].file);
  return (
    <Card title={t('녹화')}>
      {marks.length ? (
        <View style={[ps.row, { gap: 6, marginBottom: 8 }]}>
          {marks.map(([no, s]) => (
            <View key={no} style={{ borderWidth: 1, borderColor: c.line, borderRadius: 12, paddingHorizontal: 10, paddingVertical: 3 }}>
              <Text style={{ color: c.text, fontSize: 12 }}>{no} <Text style={{ color: c.accent, fontFamily: fonts.mono }}>{s.toFixed(1)} s</Text></Text>
            </View>
          ))}
        </View>
      ) : null}
      {remote ? <Note tone="warn" text={t('녹화는 로봇과 같은 망에서만 볼 수 있습니다 — 원격(중계) 연결이나 데모에서는 볼 수 없습니다.')} /> : null}
      <View style={[ps.row, { gap: 10, marginTop: 8 }]}>
        {clips.map((k) => (
          <View key={k.file} style={{ flex: 1, minWidth: 260, gap: 4 }}>
            <View style={{ aspectRatio: 16 / 9, backgroundColor: '#000', borderRadius: radius.sm, overflow: 'hidden' }}>
              <QcVideo path={k.file} bytes={k.bytes} />
            </View>
            <Text style={{ color: c.muted, fontSize: 12 }}>{pick(k.label, en)}</Text>
          </View>
        ))}
      </View>
    </Card>
  );
}

function Summary({ rep, en, onGo }: { rep: Report; en: boolean; onGo: (no: number) => void }) {
  const { c, fonts } = useTheme();
  return (
    <Card title={t('검사 결과')}>
      <Text style={{ color: c.dim, fontSize: 12, marginBottom: 6 }}>{t('검사 프로그램이 수치로 판정한 결과입니다. 검사 이름을 누르면 그 단계의 자세한 설명으로 갑니다.')}</Text>
      {rep.summary.map((r) => (
        <View key={r.no} style={{ flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 7, borderBottomWidth: 1, borderColor: c.line }}>
          <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 12, width: 22, textAlign: 'right' }}>{r.no}</Text>
          <Pressable onPress={() => onGo(r.no)} accessibilityRole="link" style={{ width: 170 }}>
            <Text style={{ color: c.accent, fontSize: 13 }} numberOfLines={1}>{pick(r.name, en)}</Text>
            {r.attempts ? <Text style={{ color: c.dim, fontSize: 11 }}>{t('시도 {n}').replace('{n}', String(r.attempts + 1))} · {t('지금')} {r.effective}</Text> : null}
          </Pressable>
          <View style={{ width: 64 }}><VerdictChip v={r.verdict} /></View>
          <Text style={{ color: c.text, fontSize: 12.5, flex: 1 }}>{pick(r.line, en)}{r.moving ? ` · ${t('로봇이 움직이는 동안 잼')}` : ''}</Text>
        </View>
      ))}
    </Card>
  );
}

function Metrics({ rep, en }: { rep: Report; en: boolean }) {
  const { c, fonts } = useTheme();
  return (
    <Card title={t('주요 수치')}>
      {rep.metrics.map((m) => {
        const hot = /가까움|한계 밖/.test(m.near);
        return (
          <View key={pick(m.label, false)} style={{ flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 6, borderBottomWidth: 1, borderColor: c.line }}>
            <Text style={{ color: c.text, fontSize: 12.5, flex: 2 }}>{pick(m.label, en)}</Text>
            <Text style={{ color: c.text, fontSize: 13, fontFamily: fonts.mono, width: 90, textAlign: 'right' }}>{m.value} {m.unit}</Text>
            <Text style={{ color: c.muted, fontSize: 12, flex: 1.2 }}>{pick(m.limit, en)}</Text>
            <Text style={{ color: hot ? c.amberTx : c.dim, fontSize: 12, width: 110, textAlign: 'right' }}>{en ? m.nearEn ?? m.near : m.near}</Text>
          </View>
        );
      })}
      {rep.metricsNote ? <Text style={{ color: c.dim, fontSize: 12, marginTop: 8, lineHeight: 18 }}>{pick(rep.metricsNote, en)}</Text> : null}
    </Card>
  );
}

function StepSection({ s, en, level, attempts, children }: { s: Step; en: boolean; level: Level; attempts?: Attempt[]; children?: React.ReactNode }) {
  const { c, fonts } = useTheme();
  const [open, setOpen] = useState(false);
  const tries = (attempts ?? []).filter((a) => a.picked?.includes(s.no));
  return (
    <Card>
      <View style={[ps.row, { gap: 10 }]}>
        <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 14 }}>{s.no}</Text>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{pick(s.name, en)}</Text>
        <VerdictChip v={s.verdict} />
        {tries.length ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('다시 돌림')} ×{tries.length}</Text> : null}
        {s.moving ? <Text style={{ color: c.amberTx, fontSize: 12 }}>{t('로봇이 움직이는 동안 잼')}</Text> : null}
      </View>
      <Text style={{ color: c.text, fontSize: 13.5, marginTop: 6 }}>{pick(s.line, en)}</Text>
      {s.skip ? <Text style={{ color: c.muted, fontSize: 12.5, marginTop: 4 }}>{t('건너뛴 까닭')} — {pick(s.skip, en)}</Text> : null}
      {children}
      {s.tables.filter((tb) => (tb.levelMin ?? 1) <= level).map((tb, i) => <ReportTable key={i} tb={tb} en={en} />)}
      {s.notes.map((n, i) => <Text key={i} style={{ color: c.muted, fontSize: 12, marginTop: 6, lineHeight: 18 }}>{pick(n, en)}</Text>)}
      <Pressable onPress={() => setOpen((o) => !o)} style={{ marginTop: 8 }}>
        <Text style={{ color: c.accent, fontSize: 12.5 }}>{open ? t('설명 접기') : t('무엇을 · 어떻게 · 왜 · 기준 보기')}</Text>
      </Pressable>
      {open ? (
        <View style={{ gap: 6, marginTop: 6 }}>
          {([['무엇을', s.what], ['어떻게', s.how], ['왜', s.why]] as const).filter(([, b]) => b).map(([k, b]) => (
            <View key={k} style={{ flexDirection: 'row', gap: 10 }}>
              <Text style={{ color: c.accent, fontSize: 12.5, width: 52 }}>{t(k)}</Text>
              <Text style={{ color: c.text, fontSize: 12.5, flex: 1, lineHeight: 19 }}>{pick(b, en)}</Text>
            </View>
          ))}
          {s.crit.length ? <Text style={{ color: c.accent, fontSize: 12.5, marginTop: 4 }}>{t('기준')}</Text> : null}
          {s.crit.map((k, i) => (
            <View key={i} style={{ flexDirection: 'row', gap: 10, alignItems: 'flex-start' }}>
              <View style={{ width: 64 }}>{k.tag ? <VerdictChip v={k.tag} /> : null}</View>
              <Text style={{ color: c.text, fontSize: 12.5, flex: 1, lineHeight: 19 }}>{pick(k.text, en)}</Text>
            </View>
          ))}
        </View>
      ) : null}
    </Card>
  );
}

function ReportTable({ tb, en }: { tb: Table; en: boolean }) {
  const { c, fonts } = useTheme();
  const { c: rc } = useRb();
  const [open, setOpen] = useState(!tb.fold);
  const tone = (k: string) => ({ ok: c.greenTx, bad: c.redTx, warn: c.amberTx, muted: c.dim }[k] ?? c.text);
  const bg = (k: string) => ({ bad: rc('fill-danger-subtlest'), warn: rc('fill-caution-subtlest') }[k] ?? 'transparent');
  const minW = Math.max(480, tb.cols.length * 130);
  return (
    <View style={{ marginTop: 10 }}>
      {tb.caption ? <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', marginBottom: 4 }}>{pick(tb.caption, en)}</Text> : null}
      {tb.fold ? (
        <Pressable onPress={() => setOpen((o) => !o)}><Text style={{ color: c.accent, fontSize: 12.5, marginBottom: 4 }}>{open ? t('접기') : pick(tb.fold, en)}</Text></Pressable>
      ) : null}
      {open ? (
        <ScrollView horizontal showsHorizontalScrollIndicator contentContainerStyle={{ minWidth: '100%' }}>
          <View style={{ minWidth: minW, flex: 1 }}>
            <View style={{ flexDirection: 'row', borderBottomWidth: 1, borderColor: c.line, paddingBottom: 4 }}>
              {tb.cols.map((h, i) => (
                <Text key={i} style={{ color: c.dim, fontSize: 11.5, fontWeight: '600', flex: i === 0 ? 1.2 : 1, paddingHorizontal: 4, textAlign: tb.num.includes(i) ? 'right' : 'left' }}>{pick(h, en)}</Text>
              ))}
            </View>
            {tb.rows.map((r, ri) => (
              <View key={ri} style={{ flexDirection: 'row', borderBottomWidth: 1, borderColor: c.line, paddingVertical: 5 }}>
                {r.map((cl, i) => (
                  <Text key={i} style={{ color: tone(cl.c), backgroundColor: bg(cl.c), fontSize: 12, flex: i === 0 ? 1.2 : 1, paddingHorizontal: 4,
                    textAlign: tb.num.includes(i) ? 'right' : 'left', fontFamily: tb.num.includes(i) ? fonts.mono : undefined }}>{en && cl.en ? cl.en : cl.t}</Text>
                ))}
              </View>
            ))}
          </View>
        </ScrollView>
      ) : null}
      {tb.note && open ? <Text style={{ color: c.muted, fontSize: 12, marginTop: 4, lineHeight: 18 }}>{pick(tb.note, en)}</Text> : null}
    </View>
  );
}

function SoakCharts({ soak, en }: { soak: NonNullable<Report['soak']>; en: boolean }) {
  const { c } = useTheme();
  const s = soak.series;
  if (!s) return null;
  return (
    <View style={{ gap: 12, marginTop: 10 }}>
      {s.groups.map((g) => {
        const ps2 = s.panels.filter((p) => p.group === g.key);
        if (!ps2.length) return null;
        return (
          <View key={g.key} style={{ gap: 6 }}>
            <Text style={{ color: c.text, fontSize: 13.5, fontWeight: '700' }}>{pick(g.title, en)} <Text style={{ color: c.dim, fontWeight: '400' }}>[{g.unit}]</Text></Text>
            <Text style={{ color: c.muted, fontSize: 12, lineHeight: 18 }}>{pick(g.desc, en)}</Text>
            <View style={[ps.row, { gap: 10 }]}>
              {ps2.map((p) => {
                const lim = p.lo ?? p.hi;
                return (
                  <View key={p.key} style={{ flex: 1, minWidth: 260 }}>
                    <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{pick(p.who, en)}</Text>
                    <QcChart bucketS={s.bucketS} totalS={s.extentS} height={150} baseS={g.key === 'tr' || g.key === 'fps' ? s.baseS : undefined}
                      limit={lim} limitLabel={lim != null ? `${g.key === 'cur' ? t('임시 상한') : t('한계')} ${lim}` : undefined}
                      marks={{ lat: p.lat.map(([k]) => k), real: p.real.map(([k]) => k) }}
                      series={[{ values: p.avg, color: g.key === 'cur' && p.key.endsWith('0') ? c.muted : c.accent, band: { min: p.min, max: p.max } }]} />
                  </View>
                );
              })}
            </View>
          </View>
        );
      })}
    </View>
  );
}

function Recheck2({ steps, en, level }: { steps: Step[]; en: boolean; level: Level }) {
  const { c } = useTheme();
  const [open, setOpen] = useState(false);
  return (
    <View style={{ marginTop: 10 }}>
      <Pressable onPress={() => setOpen((o) => !o)}><Text style={{ color: c.accent, fontSize: 13 }}>{open ? t('재검사 단계별 자세히 접기') : t('재검사 단계별 자세히 보기')}</Text></Pressable>
      {open ? <View style={{ gap: 10, marginTop: 8 }}>{steps.map((s) => <StepSection key={s.no} s={s} en={en} level={level} />)}</View> : null}
    </View>
  );
}

function Reruns({ rep, en }: { rep: Report; en: boolean }) {
  const { c, fonts } = useTheme();
  const first = rep.head.startedAt;
  const rows: { key: string; step: string; n: string; at: string; v: string; m: string; cond: string; dim?: boolean }[] = [];
  const pickedAll = new Set(rep.reruns.flatMap((a) => a.picked ?? []));
  for (const no of [...pickedAll].sort((a, b) => a - b)) {
    const s0 = rep.summary.find((r) => r.no === no);
    rows.push({ key: `${no}-0`, step: `${no} ${pick(s0?.name, en)}`, n: '1', at: first.slice(5, 16), v: s0?.verdict ?? '—', m: pick(s0?.line, en), cond: t('처음 검사') });
    rep.reruns.forEach((a) => {
      const s = a.steps?.find((x) => x.no === no && !x.prep);
      if (!s) return;
      const cond = [a.cond?.sinceFirst ? `+${a.cond.sinceFirst}` : '', a.cond?.gait ?? '', a.cond?.battery != null ? `${t('배터리')} ${Math.round(a.cond.battery)} %` : '', a.cond?.versions ?? '']
        .filter(Boolean).join(' · ');
      rows.push({ key: `${no}-${a.n}`, step: `${no} ${pick(s.name, en)}`, n: String(a.n + 1), at: a.startedAt.slice(5, 16), v: s.verdict, m: s.measured || pick(s.line, en), cond });
    });
  }
  rep.reruns.forEach((a) => {
    const prep = (a.steps ?? []).filter((x) => x.prep);
    if (prep.length) rows.push({ key: `p${a.n}`, step: t('준비로 돎'), n: '', at: a.startedAt.slice(5, 16), v: prep.some((x) => x.verdict === 'FAIL') ? 'FAIL' : 'PASS',
      m: prep.map((x) => `${x.no} ${pick(x.name, en)}`).join(' · '), cond: '', dim: true });
  });
  return (
    <Card title={`↻ ${t('단계 다시 돌린 기록')}`}>
      <Text style={{ color: c.dim, fontSize: 12, marginBottom: 6 }}>{t('처음 FAIL 은 지워지지 않습니다. 다시 돌려 PASS 가 나와도 종합은 WARN 까지만 올라갑니다.')}</Text>
      <ScrollView horizontal contentContainerStyle={{ minWidth: '100%' }}>
        <View style={{ minWidth: 820, flex: 1 }}>
          <View style={{ flexDirection: 'row', gap: 8, borderBottomWidth: 1, borderColor: c.line, paddingBottom: 4 }}>
            {[t('단계'), t('시도'), t('언제'), t('판정'), t('측정값'), t('조건')].map((h, i) => (
              <Text key={h} style={{ color: c.dim, fontSize: 11.5, flex: [1.5, 0.4, 0.9, 0.7, 2.4, 2][i] }}>{h}</Text>
            ))}
          </View>
          {rows.map((r) => (
            <View key={r.key} style={{ flexDirection: 'row', gap: 8, alignItems: 'center', paddingVertical: 6, borderBottomWidth: 1, borderColor: c.line, opacity: r.dim ? 0.7 : 1 }}>
              <Text style={{ color: c.text, fontSize: 12.5, flex: 1.5 }}>{r.step}</Text>
              <Text style={{ color: c.text, fontSize: 12.5, flex: 0.4 }}>{r.n}</Text>
              <Text style={{ color: c.text, fontSize: 12.5, flex: 0.9, fontFamily: fonts.mono }}>{r.at}</Text>
              <View style={{ flex: 0.7 }}><VerdictChip v={r.v} /></View>
              <Text style={{ color: c.text, fontSize: 12, flex: 2.4 }}>{r.m}</Text>
              <Text style={{ color: c.muted, fontSize: 12, flex: 2 }}>{r.cond}</Text>
            </View>
          ))}
        </View>
      </ScrollView>
    </Card>
  );
}

function Photos({ photos, en }: { photos: Report['photos']; en: boolean }) {
  const { c, radius } = useTheme();
  const [img, setImg] = useState<Record<string, string>>({});
  const [err, setErr] = useState<string | null>(null);
  const load = (f: string) => { ptzQcApi.photo(f).then((u) => setImg((m) => ({ ...m, [f]: u }))).catch((e: unknown) => setErr(refusalText(e))); };
  return (
    <Card title={t('사진')}>
      <View style={[ps.row, { gap: 10 }]}>
        {photos.map((p) => (
          <View key={p.file} style={{ flex: 1, minWidth: 240, gap: 4 }}>
            {img[p.file] ? <Image source={{ uri: img[p.file] }} style={{ width: '100%', aspectRatio: 16 / 9, borderRadius: radius.sm }} resizeMode="contain" />
              : <BigBtn small kind="ghost" icon="camera" label={t('사진 받기')} onPress={() => load(p.file)} />}
            <Text style={{ color: c.muted, fontSize: 12 }}>{pick(p.caption, en)}</Text>
          </View>
        ))}
      </View>
      {err ? <Note tone="warn" text={err} /> : null}
    </Card>
  );
}

function Appendix({ rows }: { rows: [string, string][] }) {
  const { c, fonts } = useTheme();
  const [open, setOpen] = useState(false);
  return (
    <Card>
      <Pressable onPress={() => setOpen((o) => !o)}>
        <Text style={{ color: c.accent, fontSize: 13 }}>{open ? t('부록 접기') : t('부록 — 검사 프로그램이 남긴 세부 값 보기')} ({rows.length})</Text>
      </Pressable>
      {open ? rows.map(([k, v], i) => (
        <View key={`${i}:${k}`} style={{ flexDirection: 'row', gap: 10, paddingVertical: 3, borderBottomWidth: 1, borderColor: c.line }}>
          <Text style={{ color: c.muted, fontSize: 11.5, flex: 1 }}>{k}</Text>
          <Text style={{ color: c.text, fontSize: 11.5, flex: 2, fontFamily: fonts.mono }}>{v}</Text>
        </View>
      )) : null}
    </Card>
  );
}
