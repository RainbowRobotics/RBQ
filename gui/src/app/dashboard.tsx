import { useEffect, useState } from 'react';
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
import { rest, actions, type PayloadResp } from '@/lib/rest';
import { useTripPoll } from '@/lib/useTripPoll';
import { StateTables } from '@/components/StateTables';
import { sensorActive } from '@/lib/visionSources';
import { dashboardRows, dashboardCards, type DashboardRow, type DashboardCard } from '@/modules/registry';
import { VISION_PROGRAMS } from '@/components/panels/VisionPanel';
import { programHealth } from '@/lib/visionProgramStates';
import { webrtcClient } from '@/lib/webrtcClient';
import { AttitudeDial } from '@/components/AttitudeDial';
import { t } from '@/lib/i18n';
import { useCompactH } from '@/lib/layout';
import { useDashType } from '@/lib/dashType';

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

function Card({ i, icon, title, badge, children }: {
  i: number; icon: IconName; title: string; badge?: { kind: 'ok' | 'warn' | 'chg'; text: string }; children: React.ReactNode;
}) {
  const { c, radius } = useTheme();
  const compact = useCompactH();
  const ty = useDashType();
  const bc = badge?.kind === 'warn' ? { bg: 'rgba(240,136,62,0.16)', fg: c.amberTx }
    : badge?.kind === 'chg' ? { bg: 'rgba(77,156,245,0.14)', fg: c.accent2 }
    : { bg: 'rgba(63,185,80,0.14)', fg: c.greenTx };
  return (
    <Animated.View entering={FadeIn.delay(i * 45).duration(220)} style={compact ? undefined : styles.cardWrap}>
      <LinearGradient colors={[c.cardA, c.cardB]} style={[styles.card, compact && styles.cardC, { borderColor: c.line, borderRadius: radius.lg }]}>
        <View style={styles.cardH}>
          <Icon name={icon} size={16} color={c.accent2} />
          <Text style={{ color: c.muted, fontSize: ty.title, fontWeight: '700' }}>{title}</Text>
          {badge && (
            <View style={[styles.badge, { backgroundColor: bc.bg }]}>
              <Text style={{ color: bc.fg, fontSize: ty.badge, fontWeight: '600' }}>{badge.text}</Text>
            </View>
          )}
        </View>
        {compact ? <View>{children}</View> : (
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

function Bar({ label, pct, val, warn }: { label: string; pct: number; val: string; warn?: boolean }) {
  const { c } = useTheme();
  const ty = useDashType();
  return (
    <View>
      <View style={styles.barLab}>
        <Text style={NUM(ty.body, c.muted, '600')}>{label}</Text>
        <Text style={NUM(ty.body, c.muted, '600')}>{val}</Text>
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
  const d = card.use();
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

function Divider() {
  const { c } = useTheme();
  return <View style={{ height: 1, backgroundColor: c.line, opacity: 0.6, marginVertical: 8 }} />;
}

type View3 = 'sum' | 'detail' | 'vision';

const DEV = { PTZ: 15, CCTV: 16 } as const;

function ViewTabs({ view, setView, devMode }: { view: View3; setView: (v: View3) => void; devMode: boolean }) {
  const { c, radius } = useTheme();
  const tabs: [string, View3][] = [['요약', 'sum'], ...(devMode ? [['로봇', 'detail'] as [string, View3]] : []), ['비전', 'vision']];
  return (
    <View style={{ flexDirection: 'row', gap: 8, paddingHorizontal: 16, paddingTop: 4 }}>
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
  const toProgram = (id: number, running: boolean) => { actions.visionProgram(ip, id, running).catch(() => {}); };

  const motionBadge = (motionConn === 'connected' ? { kind: 'ok', text: 'LIVE' }
    : motionConn === 'connecting' ? { kind: 'chg', text: t('연결중') } : { kind: 'warn', text: t('끊김') }) as { kind: 'ok' | 'warn' | 'chg'; text: string };

  const memPct = pc ? Math.round(pc.mem_used_pct) : 0;
  const cpu = pc ? Math.round((pc.cpu_core_usage?.[0] ?? 0)) : 0;
  const km = (mm?: number) => mm == null ? '—' : `${(mm / 1000).toFixed(1)} m`;

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
        <ScrollView contentContainerStyle={{ paddingHorizontal: 15, paddingTop: 12, paddingBottom: 15 }}>
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
        <ScrollView contentContainerStyle={{ paddingHorizontal: 15, paddingTop: 12, paddingBottom: 15 }}>
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
