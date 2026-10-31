import { createContext, useContext, useEffect, useRef, useState } from 'react';
import { useRouter } from 'expo-router';
import { View, Text, StyleSheet, ScrollView, type TextStyle } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import Animated, { FadeIn } from 'react-native-reanimated';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useSettings, useDevMode } from '@/store/settings';
import { rest, actions, HttpError, type PayloadResp } from '@/lib/rest';
import type { SystemInfo } from '@/types/robot';
import { useTripPoll } from '@/lib/useTripPoll';
import { StateTables } from '@/components/StateTables';
import { sensorActive } from '@/lib/visionSources';
import { dashboardRows, dashboardCards, type DashboardRow, type DashboardCard } from '@/modules/registry';
import { VISION_PROGRAMS } from '@/components/panels/VisionPanel';
import { programHealth } from '@/lib/visionProgramStates';
import { webrtcClient } from '@/lib/webrtcClient';
import { AttitudeDial } from '@/components/AttitudeDial';
import { t } from '@/lib/i18n';
import { useCompactH, EDGE } from '@/lib/layout';
import { useDashType } from '@/lib/dashType';
import { RpcEvents, RpcPerf, useSystemStats } from '@/components/dashboard/RpcPerf';

const deg = (r: number) => (r * 180) / Math.PI;

const NUM = (fontSize: number, color: string, fontWeight: '500' | '600' = '500'): TextStyle =>
  ({ fontSize, color, fontWeight, fontVariant: ['tabular-nums'] });

function Grid({ compact, children }: { compact: boolean; children: React.ReactNode }) {
  return compact ? (
    <ScrollView style={styles.gridC} contentContainerStyle={styles.colC} showsVerticalScrollIndicator={false}>
      <View style={styles.col}>{children}</View>
    </ScrollView>
  ) : (
    <View style={styles.grid}><View style={styles.row}>{children}</View></View>
  );
}

const CardFit = createContext(false);

function Card({ i, icon, title, badge, children }: {
  i: number; icon: IconName; title: string; badge?: { kind: 'ok' | 'warn' | 'chg'; text: string }; children: React.ReactNode;
}) {
  const { c, radius } = useTheme();
  const compact = useCompactH();
  const fit = useContext(CardFit);
  const ty = useDashType();
  const bc = badge?.kind === 'warn' ? { bg: 'rgba(240,136,62,0.16)', fg: c.amberTx }
    : badge?.kind === 'chg' ? { bg: 'rgba(77,156,245,0.14)', fg: c.accent2 }
    : { bg: 'rgba(63,185,80,0.14)', fg: c.greenTx };
  return (
    <Animated.View entering={FadeIn.delay(i * 45).duration(220)} style={compact ? undefined : fit ? styles.cardWrapFit : styles.cardWrap}>
      <LinearGradient colors={[c.cardA, c.cardB]} style={[styles.card, compact && styles.cardC, fit && styles.cardFit, { borderColor: c.line, borderRadius: radius.lg }]}>
        <View style={styles.cardH}>
          <Icon name={icon} size={16} color={c.accent2} />
          <Text style={{ color: c.muted, fontSize: ty.title, fontWeight: '700' }}>{title}</Text>
          {badge && (
            <View style={[styles.badge, { backgroundColor: bc.bg }]}>
              <Text style={{ color: bc.fg, fontSize: ty.badge, fontWeight: '600' }}>{badge.text}</Text>
            </View>
          )}
        </View>
        {compact || fit ? <View>{children}</View> : (
          <ScrollView style={{ flex: 1 }} contentContainerStyle={{ flexGrow: 1 }} showsVerticalScrollIndicator={false}>
            {children}
          </ScrollView>
        )}
      </LinearGradient>
    </Animated.View>
  );
}

function KV({ k, v, tone }: { k: string; v: string; tone?: 'green' | 'amber' | 'red' }) {
  const { c } = useTheme();
  const ty = useDashType();
  const col = tone === 'green' ? c.greenTx2 : tone === 'red' ? c.redTx : tone === 'amber' ? c.amber : c.text;
  return (
    <View style={styles.kv}>
      <Text style={{ color: c.muted, fontSize: ty.body, fontWeight: '500' }}>{k}</Text>
      <Text style={NUM(ty.body, col, '600')}>{v}</Text>
    </View>
  );
}

function SRow({ nm, ok, offText }: { nm: string; ok: boolean; offText?: string }) {
  const { c } = useTheme();
  const dot = ok ? c.green : offText ? c.dim : c.red;
  const tx = ok ? c.greenTx2 : offText ? c.dim : c.redTx;
  const ty = useDashType();
  return (
    <View style={styles.sRow}>
      <View style={[styles.sDot, { backgroundColor: dot }]} />
      <Text style={{ color: c.text, fontSize: ty.body, fontWeight: '500' }}>{nm}</Text>
      <Text style={[NUM(ty.body, tx, '600'), { marginLeft: 'auto' }]}>{ok ? t('정상') : offText ?? t('점검')}</Text>
    </View>
  );
}

function Bar({ label, pct, val, warn, open }: { label: string; pct: number; val: string; warn?: boolean; open?: boolean }) {
  const { c } = useTheme();
  const ty = useDashType();
  return (
    <View>
      <View style={styles.barLab}>
        <Text style={NUM(ty.body, c.muted, '600')}>{label}</Text>
        <Text style={NUM(ty.body, c.muted, '600')}>{val}{open != null ? `  ${open ? '▾' : '▸'}` : ''}</Text>
      </View>
      <View style={[styles.bar, { backgroundColor: c.elev }]}>
        <LinearGradient colors={warn ? [c.amber, c.amber] : [c.accent, c.accent2]}
          start={{ x: 0, y: 0 }} end={{ x: 1, y: 0 }} style={[styles.barFill, { width: `${pct}%` }]} />
      </View>
    </View>
  );
}

function ExtRow({ row }: { row: DashboardRow }) {
  const visible = row.useVisible();
  const ok = row.useOk();
  return visible ? <SRow nm={row.label} ok={ok} offText={t('미감지')} /> : null;
}

function ExtCard({ card, i }: { card: DashboardCard; i: number }) {
  const { c } = useTheme();
  const ty = useDashType();
  const d = card.useData();
  if (!d) return null;
  return (
    <Card i={i} icon={card.icon} title={d.title}
      badge={d.live ? { kind: 'ok', text: 'LIVE' } : { kind: 'chg', text: t('신호 없음') }}>
      {d.rows.length ? d.rows.map(([k, v]) => <KV key={k} k={k} v={v} />) : (
        <Text style={{ color: c.muted, fontSize: ty.caption, fontWeight: '500', lineHeight: ty.caption * 1.45 }}>
          {d.empty}
        </Text>
      )}
    </Card>
  );
}

function SubHead({ text, first }: { text: string; first?: boolean }) {
  const { c } = useTheme();
  const ty = useDashType();
  return (
    <Text style={{ color: c.muted, fontSize: ty.label, fontWeight: '700', letterSpacing: 0.6, marginTop: first ? 0 : 12, marginBottom: 4 }}>
      {text.toUpperCase()}
    </Text>
  );
}

function TV({ k, v, tone }: { k: string; v?: string; tone?: 'green' | 'amber' | 'red' }) {
  const { c } = useTheme();
  const ty = useDashType();
  const col = tone === 'green' ? c.greenTx2 : tone === 'red' ? c.redTx : tone === 'amber' ? c.amber : c.text;
  return (
    <View style={styles.kv}>
      <Text style={{ color: c.muted, fontSize: ty.body, fontWeight: '500', marginRight: 10, flexShrink: 0 }}>{k}</Text>
      <Text style={{ color: col, fontSize: ty.body, fontWeight: '600', flexShrink: 1, textAlign: 'right' }}>{v || '—'}</Text>
    </View>
  );
}

function Divider() {
  const { c } = useTheme();
  return <View style={{ height: 1, backgroundColor: c.line, opacity: 0.6, marginVertical: 8 }} />;
}

type View3 = 'sum' | 'detail' | 'vision' | 'rpc';

const DEV = { PTZ: 15, CCTV: 34 } as const;

function ViewTabs({ view, setView, devMode }: { view: View3; setView: (v: View3) => void; devMode: boolean }) {
  const { c, radius } = useTheme();
  const tabs: [string, View3][] = [['요약', 'sum'], ...(devMode ? [['로봇', 'detail'] as [string, View3]] : []), ['비전', 'vision'], ['R-PC', 'rpc']];
  return (
    <View style={{ flexDirection: 'row', gap: 8, paddingHorizontal: EDGE }}>
      {tabs.map(([label, v]) => {
        const on = view === v;
        return (
          <Tappable key={v} onPress={() => setView(v)}
            style={{
              height: 36, paddingHorizontal: 14, justifyContent: 'center', borderRadius: radius.md, borderWidth: 1,
              backgroundColor: on ? 'rgba(77,156,245,0.18)' : c.glass,
              borderColor: on ? 'rgba(77,156,245,0.6)' : c.glassLine,
            }}>
            <Text style={{ color: on ? c.text : c.muted, fontSize: 12.5, fontWeight: '600' }}>{t(label)}</Text>
          </Tappable>
        );
      })}
    </View>
  );
}

export default function Dashboard() {
  const compact = useCompactH();
  const ty = useDashType();
  const { c, radius } = useTheme();
  const router = useRouter();
  const devMode = useDevMode();
  const gyroEnabled = useSettings((s) => s.gyroWidgetEnabled);
  const [view, setView] = useState<View3>('sum');
  const robot = useRobot((s) => s.robot);
  const pc = useRobot((s) => s.pc);
  const trip = useRobot((s) => s.trip);
  useTripPoll();
  const battPct = useRobot((s) => s.battPct);
  const battV = useRobot((s) => s.battV);
  const ip = useRobot((s) => s.ip);
  const motionConn = useTelemetry((s) => s.motionConn);
  const tel = useTelemetry((s) => s.robot);
  const sensors = useTelemetry((s) => s.sensors);
  const programs = useTelemetry((s) => s.visionPrograms);
  const programWire = useTelemetry((s) => s.visionProgramWire);
  const devices = useTelemetry((s) => s.devices);
  const camOk = (name: string) => !!sensors?.some((x) => x.name === name && sensorActive(x));
  const conn = useRobot((s) => s.conn);
  const rpcStats = useSystemStats(ip, conn, devMode && view === 'rpc');
  const [serial, setSerial] = useState('');
  useEffect(() => {
    if (conn !== 'connected') { setSerial(''); return; }
    let dead = false;
    rest.serialNumber(ip)
      .then((r) => { if (!dead) setSerial(r.serial_number || ''); })
      .catch(() => { if (!dead) setSerial(''); });
    return () => { dead = true; };
  }, [conn, ip]);
  const [pay, setPay] = useState<PayloadResp['payload'] | null>(null);
  const [slots, setSlots] = useState<{ on: number; all: number } | null>(null);
  useEffect(() => {
    rest.payload(ip).then((r) => {
      setPay(r.total ?? r.payload);
      setSlots(r.slots ? { on: r.slots.filter((s) => Math.abs(s.mass_kg) > 1e-6).length, all: r.slots.length } : null);
    }).catch(() => {});
  }, [ip]);
  useEffect(() => { if (view === 'vision' && ip) webrtcClient.ensureConnected(ip); }, [view, ip]);
  const [sys, setSys] = useState<SystemInfo | 'old' | null>(null);
  const nicPrev = useRef<{ at: number; bytes: Record<string, [number, number]> } | null>(null);
  const [nicRate, setNicRate] = useState<Record<string, [number, number]>>({});
  const [nicOpen, setNicOpen] = useState<Record<string, boolean>>({});
  const [sysVer, setSysVer] = useState<{ version: string; system?: string } | null>(null);
  useEffect(() => {
    if (!devMode || view !== 'rpc' || conn !== 'connected') return;
    let dead = false;
    const pull = () => rest.systemInfo(ip)
      .then((r) => {
        if (dead) return;
        setSys(r);
        const at = Date.now();
        const bytes: Record<string, [number, number]> = {};
        for (const nic of r.interfaces ?? []) {
          if (nic.rx_bytes != null && nic.tx_bytes != null) bytes[nic.name] = [nic.rx_bytes, nic.tx_bytes];
        }
        const prev = nicPrev.current;
        const rate: Record<string, [number, number]> = {};
        if (prev && at > prev.at) {
          const dt = (at - prev.at) / 1000;
          for (const name of Object.keys(bytes)) {
            const cur = bytes[name];
            const p = prev.bytes[name];
            if (p && cur[0] >= p[0] && cur[1] >= p[1]) rate[name] = [((cur[0] - p[0]) * 8) / dt, ((cur[1] - p[1]) * 8) / dt];
          }
        }
        nicPrev.current = { at, bytes };
        setNicRate(rate);
      })
      .catch((e) => { if (!dead && e instanceof HttpError && e.status === 404) setSys('old'); });
    pull();
    rest.version(ip).then((r) => { if (!dead) setSysVer(r); }).catch(() => {});
    const id = setInterval(pull, 5000);
    return () => { dead = true; clearInterval(id); nicPrev.current = null; };
  }, [devMode, view, conn, ip]);
  const toProgram = (id: number, running: boolean) => { actions.visionProgram(ip, id, running).catch(() => {}); };

  const motionBadge = (motionConn === 'connected' ? { kind: 'ok', text: 'LIVE' }
    : motionConn === 'connecting' ? { kind: 'chg', text: t('연결중') } : { kind: 'warn', text: t('끊김') }) as { kind: 'ok' | 'warn' | 'chg'; text: string };

  const memPct = pc ? Math.round(pc.mem_used_pct) : 0;
  const cpu = pc ? Math.round((pc.cpu_core_usage?.[0] ?? 0)) : 0;
  const km = (mm?: number) => mm == null ? '—' : `${(mm / 1000).toFixed(1)} m`;
  const uptime = (s?: number) => {
    if (s == null) return '—';
    const d = Math.floor(s / 86400), h = Math.floor((s % 86400) / 3600), m = Math.floor((s % 3600) / 60);
    return d > 0 ? `${d}d ${h}h ${m}m` : `${h}h ${m}m`;
  };
  const [procOpen, setProcOpen] = useState({ cpu: false, mem: false, info: false, host: false });
  const [bootsOpen, setBootsOpen] = useState(false);
  const [boots, setBoots] = useState<number[] | null>(null);
  const toggleBoots = () => {
    const open = !bootsOpen;
    setBootsOpen(open);
    if (!open) return;
    setBoots(null);
    rest.systemBoots(ip)
      .then((r) => setBoots(r.boots.map((b) => b.boot_unix).reverse()))
      .catch(() => setBoots([]));
  };
  const bootAt = (sec?: number) => {
    if (!sec) return '—';
    const d = new Date(sec * 1000), p = (x: number) => String(x).padStart(2, '0');
    return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
  };

  const battNoData = battV <= 0 && battPct <= 0;
  const battCrit = !battNoData && battPct < 10 && battV < 47;
  const battBadge = (battNoData ? { kind: 'chg', text: t('신호 없음') }
    : battCrit ? { kind: 'warn', text: t('저전압 위험 — 즉시 충전') }
    : battPct < 15 ? { kind: 'warn', text: t('부족') } : { kind: 'ok', text: t('정상') }) as { kind: 'ok' | 'warn' | 'chg'; text: string };
  const healthy = !!(robot?.imu && robot?.can_bus && robot?.find_pose && robot?.control_started);
  const robotBadge = (healthy ? { kind: 'ok', text: 'OK' } : { kind: 'warn', text: t('점검') }) as { kind: 'ok' | 'warn' | 'chg'; text: string };

  if (devMode && view === 'detail') {
    return (
      <Screen>
        <HubHeader title={t('대시보드')} subtitle={t('상세 상태 · 개발자')} />
        <ViewTabs view={view} setView={setView} devMode={devMode} />
        <ScrollView contentContainerStyle={{ paddingHorizontal: EDGE, paddingTop: 8, paddingBottom: EDGE }}>
          <View style={[styles.cols, compact && { flexDirection: 'column' }]}>
            <View style={[styles.col2, compact ? { width: '100%' } : { flex: 1 }]}>
              <StateTables sections={['pdu']} />
              <StateTables sections={['devpwr']} />
            </View>
            <View style={[styles.col2, compact ? { width: '100%' } : { flex: 1 }]}>
              <StateTables sections={['imu']}
                imuFooter={
                  <Tappable onPress={() => router.push('/maintenance?sec=cal')}
                    style={[styles.mini, { alignSelf: 'flex-start', marginTop: 0, backgroundColor: c.elev, borderColor: c.line }]}>
                    <Icon name="anchor" size={13} color={c.muted} /><Text style={{ color: c.text, fontSize: ty.button, fontWeight: '600' }}>{t('IMU 캘리브레이션 ▸')}</Text>
                  </Tappable>
                } />
              <StateTables sections={['joint']} />
              <StateTables sections={['arm']} />
            </View>
          </View>
        </ScrollView>
      </Screen>
    );
  }

  if (devMode && view === 'vision') {
    return (
      <Screen>
        <HubHeader title={t('대시보드')} subtitle={t('비전')} />
        <ViewTabs view={view} setView={setView} devMode={devMode} />
        <ScrollView contentContainerStyle={{ paddingHorizontal: EDGE, paddingTop: 8, paddingBottom: EDGE }}>
         <View style={[styles.cols, compact && { flexDirection: 'column' }]}>
          <View style={[styles.vpanel, compact ? { width: '100%' } : { flex: 1 }, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <SubHead text={t('비전 프로그램')} first />
            {VISION_PROGRAMS.map((p) => {
              const h = programHealth(programs?.[p.id]);
              const on = h === 'run';
              const label = h === null ? '—' : on ? t('동작 중') : h === 'error' ? t('오류') : t('정지됨');
              const dot = on ? c.green : h === null ? c.dim : c.red;
              const tx = on ? c.greenTx : h === null ? c.dim : c.redTx;
              return (
                <View key={p.id} style={[styles.vprogRow, { borderTopColor: c.line2 }]}>
                  <View style={[styles.vdot, { backgroundColor: dot }]} />
                  <Text style={{ color: c.text, fontSize: ty.body, fontWeight: '600' }}>{p.name}</Text>
                  <Text style={[NUM(ty.body, tx, '600'), styles.vprogState]}>{label}</Text>
                  {!on && (
                    <Tappable onPress={() => toProgram(p.id, true)}
                      style={[styles.vprogAct, { borderColor: c.line, backgroundColor: c.elev }]}>
                      <Text style={{ color: c.greenTx, fontSize: ty.button, fontWeight: '700' }}>ON</Text>
                    </Tappable>
                  )}
                  {(on || h === null || h === 'error') && (
                    <Tappable onPress={() => toProgram(p.id, false)}
                      style={[styles.vprogAct, { borderColor: c.line, backgroundColor: c.elev }]}>
                      <Text style={{ color: c.redTx, fontSize: ty.button, fontWeight: '700' }}>OFF</Text>
                    </Tappable>
                  )}
                </View>
              );
            })}
            {VISION_PROGRAMS.filter((p) => programs?.[p.id]?.errorCode).map((p) => (
              <Text key={p.id} style={{ color: c.redTx, fontSize: ty.caption, fontWeight: '600', marginTop: 4 }}>
                {`${p.name} — ${programs?.[p.id]?.errorMsg || `error ${programs?.[p.id]?.errorCode}`}`}
              </Text>
            ))}
            {!programs && (
              <Text style={{ color: c.muted, fontSize: ty.caption, fontWeight: '500', marginTop: 8, lineHeight: ty.caption * 1.45 }}>
                {programWire
                  ? t('로봇이 보내는 데몬 상태 형식이 다릅니다 ({n}B ≠ 2760B) — 로봇 소프트웨어를 갱신하세요.').replace('{n}', String(programWire))
                  : t('데몬 상태 미수신 — 상태 채널(WebRTC)이 아직 안 붙었거나 로봇 소프트웨어가 구버전입니다. 기동/정지는 보낼 수 있습니다.')}
              </Text>
            )}
            <View style={[styles.spread, { marginTop: 10 }]}>
              <Tappable onPress={() => router.push('/maintenance?sec=vision')} style={[styles.mini, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Icon name="wrench" size={13} color={c.muted} /><Text style={{ color: c.text, fontSize: ty.button, fontWeight: '600' }}>{t('비전 정비 ▸')}</Text>
              </Tappable>
            </View>
          </View>
          <View style={compact ? { width: '100%' } : { flex: 1 }}>
            <StateTables sections={['vision']} />
          </View>
         </View>
        </ScrollView>
      </Screen>
    );
  }

  if (devMode && view === 'rpc') {
    const si = sys && sys !== 'old' ? sys : null;
    const gb = (kb: number) => `${(kb / 1048576).toFixed(1)} GB`;
    const bps = (b: number) => (b >= 1e6 ? `${(b / 1e6).toFixed(1)} Mb/s` : `${(b / 1e3).toFixed(0)} kb/s`);
    const bytesTxt = (b: number) => (b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : `${(b / 1e6).toFixed(1)} MB`);
    const num = (n?: number) => (n ?? 0).toLocaleString('en-US');
    const mbit = (b: number) => (b >= 1e6 ? `${b / 1e6} Mbit/s` : `${b / 1e3} kbit/s`);
    const canTone = (st: string) => (st === 'ERROR-ACTIVE' ? 'green' : st === 'BUS-OFF' ? 'red' : st.startsWith('ERROR-') ? 'amber' : undefined);
    const roles = si?.cpu_roles;
    const statsNow = rpcStats.all[rpcStats.all.length - 1];
    const roleCores: { label: string; v?: number }[] = roles
      ? ([['daemon', roles.daemon, 0], ['task', roles.task, 1], ['can', roles.can, 2], ['comm', roles.comm, 3],
          ['irq', roles.irq, -1], ['heightmap', roles.heightmap, -1]] as [string, number, number][])
        .filter(([, n]) => n >= 0)
        .map(([nm, n, i]) => ({ label: `CPU${n} ${nm}`, v: statsNow?.core[n] ?? (i >= 0 ? pc?.cpu_core_usage?.[i] : undefined) }))
      : [0, 1, 2, 3].map((i) => ({ label: `Core ${i + 1}`, v: pc?.cpu_core_usage?.[i] }));
    const memUsedKb = pc ? pc.mem_total_kb - pc.mem_available_kb : 0;
    const cpuNow = statsNow?.cpu;
    const procsByCpu = [...(pc?.processes ?? [])].sort((a, b) => b.cpu_pct - a.cpu_pct);
    const procsByMem = [...(pc?.processes ?? [])].sort((a, b) => b.mem_rss_kb - a.mem_rss_kb);
    const procRows = (list: typeof procsByCpu, val: (p: (typeof procsByCpu)[number]) => string) => (list.length ? list.map((p) => (
      <View key={p.pid} style={styles.kv}>
        <Text style={{ color: c.text, fontSize: ty.body, fontWeight: '500', flexShrink: 1 }}>{p.name}</Text>
        <Text style={[NUM(ty.body, c.text, '600'), { marginLeft: 'auto' }]}>{val(p)}</Text>
      </View>
    )) : <Text style={{ color: c.muted, fontSize: ty.body }}>—</Text>);
    const cards = (
      <>
          <Card i={0} icon="pulse" title={t('호스트')}
            badge={si?.realtime ? { kind: 'ok', text: 'PREEMPT_RT' } : si ? { kind: 'chg', text: t('비 RT 커널') } : undefined}>
            {sys === 'old' && (
              <Text style={{ color: c.muted, fontSize: ty.caption, fontWeight: '500', marginBottom: 6, lineHeight: ty.caption * 1.45 }}>
                {t('로봇 소프트웨어가 구버전이라 PC 정보(호스트·CPU·디스크·네트워크)를 보내지 않습니다. 실시간 값만 보입니다.')}
              </Text>
            )}
            <Tappable onPress={() => setProcOpen((o) => ({ ...o, host: !o.host }))}>
              <TV k={t('로봇 소프트웨어')} v={`${sysVer?.version ?? '—'}  ${procOpen.host ? '▾' : '▸'}`} />
            </Tappable>
            {procOpen.host && (
              <View style={{ paddingLeft: 12 }}>
                <TV k={t('호스트 이름')} v={si?.hostname} />
                <TV k="OS" v={si?.os} />
                <TV k={t('커널')} v={si?.kernel} />
                <TV k={t('아키텍처')} v={si?.arch} />
                <TV k={t('패키지 계열')} v={sysVer?.system} />
              </View>
            )}
            <SubHead text={t('가동')} />
            <KV k={t('마지막 부팅')} v={bootAt(pc?.boot_time_unix)} />
            <KV k={t('가동 시간')} v={uptime(pc?.uptime_s)} />
            <Tappable onPress={toggleBoots} style={styles.kv}>
              <Text style={{ color: c.muted, fontSize: ty.body, fontWeight: '500' }}>{t('부팅 기록')}</Text>
              <Text style={{ color: c.muted, fontSize: ty.body }}>{bootsOpen ? '▾' : '▸'}</Text>
            </Tappable>
            {bootsOpen && (
              <View style={{ paddingLeft: 12, maxHeight: 180 }}>
                <ScrollView nestedScrollEnabled>
                  {boots == null
                    ? <Text style={{ color: c.muted, fontSize: ty.body }}>…</Text>
                    : boots.length === 0
                      ? <Text style={{ color: c.muted, fontSize: ty.body }}>—</Text>
                      : boots.map((b, i) => <Text key={i} style={NUM(ty.body, c.text, '500')}>{bootAt(b)}</Text>)}
                </ScrollView>
              </View>
            )}
            <SubHead text={t('저장장치')} />
            {si?.disks?.length ? si.disks.map((d) => {
              const used = d.total_kb - d.avail_kb;
              const pct = d.total_kb > 0 ? Math.round((100 * used) / d.total_kb) : 0;
              return <Bar key={d.mount} label={`${d.mount} (${d.fs})`} pct={pct} warn={pct >= 90}
                val={`${gb(used)} / ${gb(d.total_kb)} · ${pct}%`} />;
            }) : <Text style={{ color: c.muted, fontSize: ty.body }}>—</Text>}
            <SubHead text={t('메모리')} />
            <Tappable onPress={() => setProcOpen((o) => ({ ...o, mem: !o.mem }))}>
              <Bar label="MEM" pct={pc ? Math.round(pc.mem_used_pct) : 0}
                val={pc ? `${gb(memUsedKb)} / ${gb(pc.mem_total_kb)} · ${pc.mem_used_pct.toFixed(0)}%` : '—'} open={procOpen.mem} />
            </Tappable>
            {procOpen.mem && (
              <View style={{ paddingLeft: 12 }}>
                <Bar label="SWAP" pct={pc && pc.swap_total_kb > 0 ? Math.round(pc.swap_used_pct) : 0}
                  val={pc && pc.swap_total_kb > 0 ? `${gb(pc.swap_used_kb)} / ${gb(pc.swap_total_kb)} · ${pc.swap_used_pct.toFixed(0)}%` : '—'} />
                <SubHead text={t('프로세스 (메모리)')} />
                {procRows(procsByMem, (p) => `${(p.mem_rss_kb / 1024).toFixed(0)} MB`)}
              </View>
            )}
          </Card>

          <Card i={1} icon="pulse" title="CPU"
            badge={pc ? (pc.cpu_throttled ? { kind: 'warn', text: t('스로틀 발생') } : { kind: 'ok', text: 'LIVE' }) : { kind: 'chg', text: t('신호 없음') }}>
            <KV k={t('온도')} v={pc ? `${pc.cpu_temp_c.toFixed(1)} °C` : '—'} tone={pc && pc.cpu_temp_c >= 85 ? 'amber' : undefined} />
            <KV k={t('스로틀')} v={pc?.cpu_throttled ? t('발생') : t('정상')} tone={pc?.cpu_throttled ? 'amber' : 'green'} />
            <Tappable onPress={() => setProcOpen((o) => ({ ...o, info: !o.info }))}>
              <KV k={t('코어')} v={`${si?.cpu_count != null ? String(si.cpu_count) : '—'}  ${procOpen.info ? '▾' : '▸'}`} />
            </Tappable>
            {procOpen.info && (
              <View style={{ paddingLeft: 12 }}>
                <TV k={t('모델')} v={si?.cpu_model} />
                <TV k={t('격리 코어')} v={si ? (si.cpu_isolated || t('없음')) : undefined} />
                <KV k={t('클럭')} v={si?.cpu_mhz != null ? `${si.cpu_mhz}${si.cpu_max_mhz ? ` / ${si.cpu_max_mhz}` : ''} MHz` : '—'} />
                <KV k={t('부하 (1/5/15분)')} v={si?.load_avg ? si.load_avg.map((x) => x.toFixed(2)).join(' / ') : '—'} />
                {si?.throttle_count != null && <KV k={t('스로틀 누적')} v={String(si.throttle_count)} tone={si.throttle_count > 0 ? 'amber' : undefined} />}
              </View>
            )}
            <Tappable onPress={() => setProcOpen((o) => ({ ...o, cpu: !o.cpu }))} style={{ marginTop: 10 }}>
              <Bar label={t('사용률')} pct={cpuNow != null ? Math.round(cpuNow) : 0} val={cpuNow != null ? `${cpuNow.toFixed(0)}%` : '—'}
                warn={cpuNow != null && cpuNow >= 80} open={procOpen.cpu} />
            </Tappable>
            {procOpen.cpu && (
              <View style={{ paddingLeft: 12 }}>
                <SubHead text={t('역할 코어 사용률')} />
                {roleCores.map(({ label, v }) => (
                  <Bar key={label} label={label} pct={v != null ? Math.max(0, Math.min(100, v)) : 0} val={v != null ? `${v.toFixed(1)}%` : '—'} warn={v != null && v >= 90} />
                ))}
                <SubHead text={t('프로세스 (CPU)')} />
                {procRows(procsByCpu, (p) => `${p.cpu_pct.toFixed(1)}%`)}
              </View>
            )}
          </Card>

          <Card i={2} icon="box" title={t('네트워크')}>
            {si?.interfaces?.length ? si.interfaces.map((nic, n) => {
              const open = !!nicOpen[nic.name];
              const rxBad = (nic.rx_errors ?? 0) + (nic.rx_fifo_errors ?? 0) + (nic.rx_frame_errors ?? 0) > 0;
              const txBad = (nic.tx_errors ?? 0) + (nic.tx_fifo_errors ?? 0) + (nic.tx_carrier_errors ?? 0) + (nic.collisions ?? 0) > 0;
              const berrBad = (nic.can?.berr_tx ?? 0) + (nic.can?.berr_rx ?? 0) > 0;
              const busTone = (nic.can?.bus_off ?? 0) > 0 ? 'red' as const
                : (nic.can?.error_passive ?? 0) + (nic.can?.bus_error ?? 0) > 0 ? 'amber' as const : undefined;
              const hiddenBad = rxBad || txBad || berrBad || !!busTone;
              return (
              <View key={nic.name}>
                {n > 0 && <Divider />}
                <SRow nm={`${nic.name}${nic.wireless ? ' (WiFi)' : ''}`} ok={nic.state === 'up'} offText={nic.state || t('미감지')} />
                {nic.can && (
                  <KV k={t('CAN 상태')} v={`${nic.can.state}${nic.can.ctrlmode?.includes('FD') ? ' · FD' : ''}`}
                    tone={canTone(nic.can.state)} />
                )}
                {!nic.can && <TV k="IPv4" v={nic.ipv4.join(', ')} />}
                {nic.speed_mbps != null && nic.state === 'up' && (
                  <KV k={t('링크 속도')} v={`${nic.speed_mbps} Mb/s${nic.duplex === 'half' ? ` · ${t('반이중')}` : ''}`}
                    tone={nic.duplex === 'half' ? 'amber' : undefined} />
                )}
                {nic.wifi_signal_dbm != null && (
                  <KV k={t('신호')} v={`${nic.wifi_signal_dbm} dBm`}
                    tone={nic.wifi_signal_dbm >= -60 ? 'green' : nic.wifi_signal_dbm >= -75 ? 'amber' : 'red'} />
                )}
                {nicRate[nic.name] && nic.state === 'up' && (
                  <KV k={t('트래픽')} v={`↓ ${bps(nicRate[nic.name][0])} · ↑ ${bps(nicRate[nic.name][1])}`} />
                )}
                <Tappable onPress={() => setNicOpen((o) => ({ ...o, [nic.name]: !open }))} style={styles.kv}>
                  <Text style={{ color: hiddenBad && !open ? c.amber : c.muted, fontSize: ty.body, fontWeight: '500' }}>
                    {hiddenBad && !open ? `${t('상세')} — ${t('오류 있음')}` : t('상세')}
                  </Text>
                  <Text style={{ color: c.muted, fontSize: ty.body }}>{open ? '▾' : '▸'}</Text>
                </Tappable>
                {open && (
                  <View style={{ paddingLeft: 12 }}>
                    {nic.can && (
                      <>
                        {nic.can.bitrate != null && (
                          <TV k={t('비트레이트')} v={`${mbit(nic.can.bitrate)} (${nic.can.sample_point})${nic.can.dbitrate ? ` · ${t('데이터')} ${mbit(nic.can.dbitrate)} (${nic.can.dsample_point})` : ''}`} />
                        )}
                        {nic.can.berr_tx != null && (
                          <KV k={t('에러 카운터')} v={`TX ${nic.can.berr_tx} · RX ${nic.can.berr_rx ?? 0}`} tone={berrBad ? 'amber' : undefined} />
                        )}
                        {nic.can.bus_error != null && (
                          <TV k={t('버스 이벤트')} v={`bus-errors ${num(nic.can.bus_error)} · arbit-lost ${num(nic.can.arbitration_lost)} · error-warn ${num(nic.can.error_warning)} · error-pass ${num(nic.can.error_passive)} · bus-off ${num(nic.can.bus_off)} · re-started ${num(nic.can.restarts)}`}
                            tone={busTone} />
                        )}
                        {nic.can.restart_ms != null && (
                          <KV k={t('자동 재시작')} v={nic.can.restart_ms > 0 ? `${nic.can.restart_ms} ms` : t('끔')} />
                        )}
                      </>
                    )}
                    {nic.can && <TV k="IPv4" v={nic.ipv4.join(', ')} />}
                    <TV k="MAC" v={nic.mac} />
                    {nic.rx_bytes != null && (
                      <TV k="RX" v={`${num(nic.rx_packets)} packets · ${bytesTxt(nic.rx_bytes)}`} />
                    )}
                    {nic.rx_errors != null && (
                      <TV k={t('RX 오류')} v={`errors ${num(nic.rx_errors)} · dropped ${num(nic.rx_dropped)} · overruns ${num(nic.rx_fifo_errors)} · frame ${num(nic.rx_frame_errors)}`}
                        tone={rxBad ? 'amber' : undefined} />
                    )}
                    {nic.tx_bytes != null && (
                      <TV k="TX" v={`${num(nic.tx_packets)} packets · ${bytesTxt(nic.tx_bytes)}`} />
                    )}
                    {nic.tx_errors != null && (
                      <TV k={t('TX 오류')} v={`errors ${num(nic.tx_errors)} · dropped ${num(nic.tx_dropped)} · overruns ${num(nic.tx_fifo_errors)} · carrier ${num(nic.tx_carrier_errors)} · collisions ${num(nic.collisions)}`}
                        tone={txBad ? 'amber' : undefined} />
                    )}
                    {nic.flags?.length ? <TV k={t('플래그')} v={nic.flags.join(' · ')} /> : null}
                    {nic.mtu != null && <KV k="MTU" v={String(nic.mtu)} />}
                    {nic.tx_queue_len != null && <KV k="txqueuelen" v={String(nic.tx_queue_len)} />}
                    {(nic.driver || nic.bus_id) && <TV k={t('장치')} v={[nic.driver, nic.bus_id].filter(Boolean).join(' · ')} />}
                    {(nic.irq != null || nic.memory) && (
                      <TV k={t('인터럽트 · 메모리')} v={[nic.irq != null ? `IRQ ${nic.irq}` : '', nic.memory ?? ''].filter(Boolean).join(' · ')} />
                    )}
                  </View>
                )}
              </View>
              );
            }) : <Text style={{ color: c.muted, fontSize: ty.body }}>—</Text>}
          </Card>

          <Card i={3} icon="warn" title={t('이벤트')}>
            <RpcEvents stats={rpcStats} />
          </Card>
      </>
    );
    return (
      <Screen>
        <HubHeader title={t('대시보드')} subtitle="R-PC" />
        <ViewTabs view={view} setView={setView} devMode={devMode} />
        <ScrollView style={{ flex: 1 }} showsVerticalScrollIndicator={false}>
          <CardFit.Provider value>
            {compact ? <View style={[styles.colC, styles.col, { paddingBottom: 0 }]}>{cards}</View> : <View style={styles.rowFit}>{cards}</View>}
          </CardFit.Provider>
          <RpcPerf stats={rpcStats} />
        </ScrollView>
      </Screen>
    );
  }

  return (
    <Screen>
      <HubHeader title={t('대시보드')} subtitle={t('모니터링')} />
      {devMode && <ViewTabs view={view} setView={setView} devMode={devMode} />}
      <Grid compact={compact}>
          <Card i={0} icon="pulse" title={t('시스템')} badge={robotBadge}>
            <View style={styles.kv}>
              <Text style={{ color: c.muted, fontSize: ty.body, fontWeight: '500' }}>{t('로봇 S/N')}</Text>
              <Text style={{ color: c.text, fontSize: ty.body, fontWeight: '600' }}>{serial || '—'}</Text>
            </View>
            <SubHead text={t('주행거리 (TRIP)')} />
            <KV k="Total" v={km(trip?.total.distance_mm)} />
            <KV k="Trip A" v={km(trip?.tripA.distance_mm)} />
            <KV k="Trip B" v={km(trip?.tripB.distance_mm)} />
            <View style={[styles.spread, { marginTop: 6 }]}>
              <Tappable onPress={() => actions.tripReset(ip, 'A').catch(() => {})} style={[styles.mini, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Icon name="recover" size={13} color={c.muted} /><Text style={{ color: c.text, fontSize: ty.button, fontWeight: '600' }}>{t('A 리셋')}</Text>
              </Tappable>
              <Tappable onPress={() => actions.tripReset(ip, 'B').catch(() => {})} style={[styles.mini, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Icon name="recover" size={13} color={c.muted} /><Text style={{ color: c.text, fontSize: ty.button, fontWeight: '600' }}>{t('B 리셋')}</Text>
              </Tappable>
            </View>
            <SubHead text={t('배터리')} />
            <View style={styles.bigRow}>
              <Text style={NUM(ty.hero, battCrit ? c.redbright : c.greenTx2, '600')}>{battPct}</Text>
              <Text style={{ color: c.muted, fontSize: ty.heroSub, fontWeight: '600' }}>%</Text>
              <Text style={[NUM(ty.heroSub, battCrit ? c.redbright : c.muted), { marginLeft: 4 }]}>
                {battNoData ? '' : `${battV.toFixed(1)} V`}
              </Text>
              <Text style={{ color: battCrit ? c.redbright : battBadge.kind === 'warn' ? c.amber : c.muted, fontSize: ty.body, fontWeight: '600', marginLeft: 6 }}>{battBadge.text}</Text>
            </View>
            <SubHead text={t('로봇 헬스')} />
            <SRow nm="IMU" ok={!!robot?.imu} />
            <SRow nm="CAN" ok={!!robot?.can_bus} />
            <SRow nm="FindPose" ok={!!robot?.find_pose} />
            <SRow nm="Control" ok={!!robot?.control_started} />
            {devMode && (
              <View style={[styles.spread, { marginTop: 4, marginBottom: 2 }]}>
                <Tappable onPress={() => router.push('/maintenance?sec=pwr')} style={[styles.mini, { backgroundColor: c.elev, borderColor: c.line }]}>
                  <Icon name="wrench" size={13} color={c.muted} /><Text style={{ color: c.text, fontSize: ty.button, fontWeight: '600' }}>{t('전원 정비 ▸')}</Text>
                </Tappable>
              </View>
            )}
            <SubHead text={t('로봇 PC 상태')} />
            <Bar label="CPU" pct={cpu} val={`${cpu}%`} />
            <KV k={t('온도')} v={pc ? `${pc.cpu_temp_c.toFixed(1)} °C` : '—'} />
            <Bar label="MEM" pct={memPct} val={pc ? `${(pc.mem_total_kb - pc.mem_available_kb >> 10) / 1024 | 0} GB` : '—'} />
            <KV k={t('스로틀')} v={pc?.cpu_throttled ? t('발생') : t('정상')} tone={pc?.cpu_throttled ? 'amber' : 'green'} />
            <KV k={t('마지막 부팅')} v={bootAt(pc?.boot_time_unix)} />
            <KV k={t('가동 시간')} v={uptime(pc?.uptime_s)} />
            <Tappable onPress={toggleBoots} style={styles.kv}>
              <Text style={{ color: c.muted, fontSize: ty.body, fontWeight: '500' }}>{t('부팅 기록')}</Text>
              <Text style={{ color: c.muted, fontSize: ty.body }}>{bootsOpen ? '▾' : '▸'}</Text>
            </Tappable>
            {bootsOpen && (
              <View style={{ paddingLeft: 12, maxHeight: 180 }}>
                <ScrollView nestedScrollEnabled>
                  {boots == null
                    ? <Text style={{ color: c.muted, fontSize: ty.body }}>…</Text>
                    : boots.length === 0
                      ? <Text style={{ color: c.muted, fontSize: ty.body }}>—</Text>
                      : boots.map((b, i) => <Text key={i} style={NUM(ty.body, c.text, '500')}>{bootAt(b)}</Text>)}
                </ScrollView>
              </View>
            )}
          </Card>

          <Card i={1} icon="pulse" title={t('관절·자세')} badge={motionBadge}>
            <SubHead text={t('자세')} first />
            {gyroEnabled && (
              <View style={{ flexDirection: 'row', alignItems: 'center', gap: 12, marginBottom: 6 }}>
                <AttitudeDial size={84}
                  roll={tel ? deg(tel.imu.rpy[0]) : 0} pitch={tel ? deg(tel.imu.rpy[1]) : 0}
                  colors={{ sky: c.accent, ground: c.amber, line: c.text, cross: c.brand, border: c.line }} />
                <View style={{ flex: 1 }}>
                  <KV k="Roll" v={tel ? `${deg(tel.imu.rpy[0]).toFixed(1)}°` : '—'} />
                  <KV k="Pitch" v={tel ? `${deg(tel.imu.rpy[1]).toFixed(1)}°` : '—'} />
                  <KV k="Yaw" v={tel ? `${deg(tel.imu.rpy[2]).toFixed(1)}°` : '—'} />
                </View>
              </View>
            )}
            <KV k="GAIT_ID" v={tel ? String(tel.gaitId) : '—'} />
            <KV k={t('낙상')} v={tel ? (tel.isFall ? 'FALL' : t('정상')) : '—'} tone={tel?.isFall ? 'red' : 'green'} />
            <KV k="RPY (°)" v={tel ? tel.imu.rpy.map((r) => deg(r).toFixed(0)).join(' / ') : '—'} />
            <KV k={t('높이')} v={tel ? `${tel.worldPos[2].toFixed(2)} m` : '—'} />
            <KV k={t('배터리A')} v={tel ? `${tel.battery.current.toFixed(2)} A` : '—'} />
          </Card>

          <Card i={2} icon="box" title={t('구성')}>
            <SubHead text="Payload" first />
            {slots ? <KV k={t('장착 슬롯')} v={`${slots.on} / ${slots.all}`} /> : null}
            <KV k={t('총 질량')} v={pay ? `${pay.mass_kg.toFixed(1)} kg` : '—'} />
            <KV k={t('무게중심 X')} v={pay ? `${pay.center_of_mass.x_m.toFixed(2)} m` : '—'} />
            <KV k={t('무게중심 Y')} v={pay ? `${pay.center_of_mass.y_m.toFixed(2)} m` : '—'} />
            <KV k={t('무게중심 Z')} v={pay ? `${pay.center_of_mass.z_m.toFixed(2)} m` : '—'} />
            <SRow nm={t('로봇팔')} ok={!!tel?.attached.arm} offText={t('미감지')} />
            <SRow nm="PTZ" ok={!!devices?.[DEV.PTZ]?.connected} offText={t('미감지')} />
            {dashboardRows.map((r) => <ExtRow key={r.key} row={r} />)}
            <Divider />
            <SubHead text="Sensor" />
            <SRow nm="IMU" ok={!!tel?.imuSuccess} offText={t('미감지')} />
            <SRow nm={t('전방 카메라')} ok={camOk('FT0')} offText={t('미감지')} />
            <SRow nm={t('후방 카메라')} ok={camOk('RR0')} offText={t('미감지')} />
            {[0, 1, 2, 3].map((n) => (
              <SRow key={n} nm={`${t('배면 카메라')} ${n}`} ok={camOk(`BT${n}`)} offText={t('미감지')} />
            ))}
            <SRow nm="IR" ok={!!sensors?.some((x) => x.ir)} offText={t('미감지')} />
            <SRow nm="Depth" ok={!!sensors?.some((x) => x.depth)} offText={t('미감지')} />
            <SRow nm={t('IR프로젝터')} ok={!!sensors?.some((x) => x.projector)} offText={t('미감지')} />
          </Card>

          {dashboardCards.map((cd, n) => <ExtCard key={cd.key} card={cd} i={3 + n} />)}
      </Grid>
    </Screen>
  );
}

const styles = StyleSheet.create({
  grid: { flex: 1, padding: 15, gap: 13 },
  gridC: { flex: 1 },
  colC: { padding: 15 },
  row: { flex: 1, flexDirection: 'row', gap: 13 },
  col: { flexDirection: 'column', gap: 13 },
  cardWrap: { flex: 1 },
  card: { flex: 1, borderWidth: 1, padding: 13, overflow: 'hidden' },
  cardFit: { flexGrow: 1, flexShrink: 0, flexBasis: 'auto' },
  rowFit: { flexDirection: 'row', flexWrap: 'wrap', alignItems: 'stretch', gap: 13, padding: 15, paddingBottom: 0 },
  cardWrapFit: { flexGrow: 1, flexShrink: 1, flexBasis: 340, minWidth: 0 },
  cardC: { flex: 0 },
  cardH: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 8 },
  badge: { marginLeft: 'auto', paddingHorizontal: 7, paddingVertical: 2, borderRadius: 6 },
  bigRow: { flexDirection: 'row', alignItems: 'baseline', gap: 6, marginBottom: 6 },
  kv: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'baseline', paddingVertical: 2 },
  sRow: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 2 },
  sDot: { width: 8, height: 8, borderRadius: 4 },
  divline: { height: 1, marginVertical: 5 },
  barLab: { flexDirection: 'row', justifyContent: 'space-between', marginTop: 3 },
  bar: { height: 6, borderRadius: 3, marginVertical: 3, overflow: 'hidden' },
  barFill: { height: 6, borderRadius: 3 },
  spread: { flexDirection: 'row', gap: 8, marginTop: 'auto' },
  mini: { flexDirection: 'row', alignItems: 'center', gap: 5, height: 28, paddingHorizontal: 11, borderRadius: 7, borderWidth: 1 },
  cols: { flexDirection: 'row', gap: 8, alignItems: 'flex-start' },
  col2: { gap: 8 },
  vpanel: { borderWidth: 1, padding: 13 },
  vprogRow: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 8, borderTopWidth: 1 },
  vprogState: { marginLeft: 'auto', minWidth: 54, textAlign: 'right', marginRight: 8 },
  vprogAct: { paddingHorizontal: 14, paddingVertical: 6, borderRadius: 999, borderWidth: 1, minWidth: 48, alignItems: 'center' },
  vdot: { width: 7, height: 7, borderRadius: 4 },
});
