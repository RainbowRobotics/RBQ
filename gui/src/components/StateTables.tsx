import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useDashType } from '@/lib/dashType';
import { useTelemetry } from '@/store/telemetry';
import { PDU_RAILS } from '@/components/panels/PowerPanel';
import { t } from '@/lib/i18n';

const JOINT_NAMES = ['HRR0', 'HRP1', 'HRK2', 'HLR3', 'HLP4', 'HLK5', 'FRR6', 'FRP7', 'FRK8', 'FLR9', 'FLP10', 'FLK11'];
const ARM_NAMES = ['M0Y', 'M1P', 'M2P', 'M3Y', 'M4P', 'M5Y', 'M6E'];
const R2D = 180 / Math.PI;
const JOINT_NOTE = 'C=연결(N=미연결) · H=원점(CALIB) · R=구동(RUN)';
const CUR_LIMIT_LEG = 15;
const CUR_LIMIT_ARM = 5;
type Align = 'left' | 'right';

function Table({ title, note, widths, aligns, head, rows }: {
  title: string; note?: string; widths: number[]; aligns: Align[]; head: string[];
  rows: { cells: string[]; colors: (string | undefined)[] }[];
}) {
  const { c, radius } = useTheme();
  const ty = useDashType();
  const pad = Math.round(ty.body / 4);
  const cell = (v: string, i: number, color?: string, hdr = false) => (
    <Text key={i} numberOfLines={1}
      style={{
        flex: widths[i], textAlign: aligns[i], paddingVertical: pad, paddingHorizontal: 4,
        fontVariant: ['tabular-nums'], fontSize: hdr ? ty.label : ty.body, letterSpacing: hdr ? 0.4 : 0,
        fontWeight: hdr ? '700' : '500', color: hdr ? c.muted : color ?? c.text,
      }}>{v}</Text>
  );
  return (
    <View style={[styles.card, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
      <Text style={{ fontSize: ty.title, fontWeight: '700', letterSpacing: 0.5, color: c.muted, marginBottom: note ? 1 : 4 }}>{title}</Text>
      {note ? <Text style={{ fontSize: ty.caption, fontWeight: '500', color: c.muted, marginBottom: 4 }}>{note}</Text> : null}
      <View style={styles.tr}>{head.map((h, i) => cell(h, i, undefined, true))}</View>
      {rows.map((r, ri) => (
        <View key={ri} style={[styles.tr, { borderTopWidth: 1, borderTopColor: c.line2 }]}>
          {r.cells.map((v, i) => cell(v, i, r.colors[i]))}
        </View>
      ))}
    </View>
  );
}

export type StateTableSection = 'joint' | 'imu' | 'pdu' | 'devpwr' | 'vision' | 'arm';

export function StateTables({ sections, imuFooter }: {
  sections?: StateTableSection[];
  imuFooter?: React.ReactNode;
}) {
  const { c } = useTheme();
  const show = (s: StateTableSection) => !sections || sections.includes(s);
  const robot = useTelemetry((s) => s.robot);
  const pdu = useTelemetry((s) => s.pdu);
  const sensors = useTelemetry((s) => s.sensors);
  const GRN = c.greenTx, RED = c.redTx, ORG = c.amberTx, GRY = c.dim;

  const waitRow = [{ cells: [t('수신 대기 중…')], colors: [GRY] as (string | undefined)[] }];

  const curCell = (a: number, limit: number): [string, string | undefined] =>
    !Number.isFinite(a) ? ['-', GRY] : [a.toFixed(2), Math.abs(a) > limit ? RED : undefined];
  const torqueCell = (v: number) => (Number.isFinite(v) ? v.toFixed(1) : '-');

  const jointRows = JOINT_NAMES.map((nm, i) => {
    const j = robot?.joints?.[i];
    if (!j) return { cells: [nm, '-', '-', '-', '-', '-', '-'], colors: [undefined, GRY, GRY, GRY, GRY, GRY, GRY] };
    const status = `${j.connected ? 'C' : 'N'}·${j.calib ? 'H' : '-'}·${j.run ? 'R' : '-'}`;
    const stCol = j.run && j.calib ? GRN : j.run ? ORG : RED;
    const maxT = Math.max(j.temperature, j.statorTemp);
    const tCol = maxT >= 70 ? RED : maxT >= 60 ? ORG : undefined;
    const [cur, curCol] = curCell(j.current, CUR_LIMIT_LEG);
    return {
      cells: [nm, status, j.errors[0] ?? '-', `${j.temperature}/${j.statorTemp}`, (j.position * R2D).toFixed(1), torqueCell(j.torque), cur],
      colors: [undefined, stCol, j.errors.length ? RED : GRY, tCol, undefined, undefined, curCol],
    };
  });

  const armJoints = (robot?.joints ?? []).slice(12);
  const armRows = armJoints.map((j, i) => {
    const nm = ARM_NAMES[i] ?? `A${i}`;
    const status = `${j.connected ? 'C' : 'N'}·${j.calib ? 'H' : '-'}·${j.run ? 'R' : '-'}`;
    const stCol = j.run && j.calib ? GRN : j.run ? ORG : RED;
    const maxT = Math.max(j.temperature, j.statorTemp);
    const tCol = maxT >= 70 ? RED : maxT >= 60 ? ORG : undefined;
    const [cur, curCol] = curCell(j.current, CUR_LIMIT_ARM);
    return {
      cells: [nm, status, j.errors[0] ?? '-', `${j.temperature}/${j.statorTemp}`, (j.position * R2D).toFixed(1), torqueCell(j.torque), cur],
      colors: [undefined, stCol, j.errors.length ? RED : GRY, tCol, undefined, undefined, curCol],
    };
  });

  const rpy = robot?.imu?.rpy ?? [0, 0, 0];
  const gyro = robot?.imu?.gyro ?? [0, 0, 0];
  const acc = robot?.imu?.acc ?? [0, 0, 0];
  const imuRows = (['R(x)', 'P(y)', 'Y(z)'] as const).map((nm, i) => ({
    cells: [nm, (rpy[i] * R2D).toFixed(1), gyro[i].toFixed(2), acc[i].toFixed(2)],
    colors: [undefined, undefined, undefined, undefined] as (string | undefined)[],
  }));

  const b = (on?: boolean) => (on ? 'ON' : 'OFF');
  const sw = (on?: boolean) => (on ? GRN : RED);
  const swNames = (bits: [boolean | undefined, string][]) =>
    bits.filter(([on]) => on).map(([, nm]) => nm).join('·') || 'OFF';
  const pwRows = pdu ? [
    { cells: ['TOTAL', pdu.rails.total.v.toFixed(1), pdu.rails.total.a.toFixed(1), `${pdu.tempPdu}/${pdu.tempPs}`, '-'],
      colors: [undefined, GRN, undefined, pdu.tempPdu >= 70 || pdu.tempPs >= 70 ? RED : undefined, GRY] },
    { cells: ['BAT L', pdu.rails.batL.v.toFixed(1), pdu.rails.batL.a.toFixed(1), '-', b(pdu.batSwL)],
      colors: [undefined, pdu.batSwL ? GRN : GRY, undefined, GRY, sw(pdu.batSwL)] },
    { cells: ['BAT R', pdu.rails.batR.v.toFixed(1), pdu.rails.batR.a.toFixed(1), '-', b(pdu.batSwR)],
      colors: [undefined, pdu.batSwR ? GRN : GRY, undefined, GRY, sw(pdu.batSwR)] },
    { cells: ['LEGS', pdu.rails.leg.v.toFixed(1), pdu.rails.leg.a.toFixed(1), '-', b(pdu.fetLeg)],
      colors: [undefined, pdu.fetLeg ? GRN : GRY, undefined, GRY, sw(pdu.fetLeg)] },
    { cells: ['ADDON', pdu.rails.add.v.toFixed(1), pdu.rails.add.a.toFixed(1), '-', b(pdu.fetAdd)],
      colors: [undefined, pdu.fetAdd ? GRN : GRY, undefined, GRY, sw(pdu.fetAdd)] },
    { cells: ['EXTERN', pdu.rails.ext.v.toFixed(1), pdu.rails.ext.a.toFixed(1), '-', b(pdu.fetExt)],
      colors: [undefined, pdu.fetExt ? GRN : GRY, undefined, GRY, sw(pdu.fetExt)] },
    { cells: ['CHARGE', pdu.rails.chg.v.toFixed(1), pdu.rails.chg.a.toFixed(1), '-', swNames([[pdu.chgS, t('도킹')], [pdu.chgE, t('포트')]])],
      colors: [undefined, pdu.chgE ? GRN : GRY, undefined, GRY, sw(pdu.chgE || pdu.chgS)] },
  ] : waitRow;

  const devRows = pdu
    ? PDU_RAILS.map((r) => ({ cells: [t(r.nm), b(pdu[r.bit])], colors: [undefined, sw(pdu[r.bit])] }))
    : waitRow;

  const mod = (has: boolean, on: boolean, fps: number): [string, string] =>
    !has ? ['N', GRY] : !on ? ['X', GRY] : [String(fps), fps === 0 ? RED : GRN];
  const vsRows = (sensors ?? []).filter((s) => s.name).map((s) => {
    const status = s.running ? 'RUN' : s.sleep ? 'SLEEP' : s.idle ? 'IDLE'
      : !s.detected ? t('미감지') : !s.powered ? t('전원 꺼짐') : !s.connected ? t('미연결') : t('대기');
    const stCol = s.running ? GRN : s.sleep ? c.accent2 : s.idle ? ORG : !s.powered ? RED : ORG;
    const [rgb, rgbC] = mod(s.rgb, s.rgbOn, s.fps[0]);
    const [ir, irC] = mod(s.ir, s.irOn, s.fps[1]);
    const [dep, depC] = mod(s.depth, s.depthOn, s.fps[2]);
    const emit = !s.projector ? 'N' : s.projectorOn ? 'ON' : 'OFF';
    return {
      cells: [s.name, status, rgb, ir, dep, emit],
      colors: [undefined, stCol, rgbC, irC, depC, !s.projector ? GRY : s.projectorOn ? ORG : GRN],
    };
  });

  return (
    <View style={styles.wrap}>
      {show('joint') && (
        <View style={{ flex: 1.12 }}>
          <Table title={t('관절 · JOINT')} note={t(JOINT_NOTE)} widths={[14, 17, 11, 17, 15, 13, 13]} aligns={['left', 'left', 'left', 'right', 'right', 'right', 'right']}
            head={['ID', 'STATUS', 'ERR', '°C B/S', 'ANGLE°', 'Nm', 'A']} rows={jointRows} />
        </View>
      )}
      {(show('imu') || show('pdu') || show('devpwr')) && (
        <View style={{ flex: 1, gap: 8 }}>
          {show('imu') && (
            <>
              <Table title="IMU" widths={[15, 20, 35, 30]} aligns={['left', 'right', 'right', 'right']}
                head={['', 'ANGLE°', 'GYRO (rad/s)', 'ACC (m/s²)']} rows={imuRows} />
              {imuFooter}
            </>
          )}
          {show('devpwr') && (
            <Table title={t('장치 전원 · 12V/5V')} widths={[68, 32]} aligns={['left', 'right']}
              head={[t('장치'), t('상태')]} rows={devRows} />
          )}
          {show('pdu') && (
            <Table title={pdu ? `${t('전원 · PDU')} · ${t('배터리')} ${pdu.batPct}%` : t('전원 · PDU')}
              widths={[24, 20, 18, 20, 18]} aligns={['left', 'right', 'right', 'right', 'right']}
              head={['RAIL', 'V', 'A', '°C', t('상태')]} rows={pwRows} />
          )}
        </View>
      )}
      {(show('vision') || (show('arm') && armRows.length > 0)) && (
        <View style={{ flex: 1.05, gap: 8 }}>
          {show('vision') && (
            <Table title={t('비전 센서')} widths={[14, 21, 17, 16, 16, 16]} aligns={['left', 'left', 'right', 'right', 'right', 'right']}
              head={['ID', 'STATUS', 'RGB (fps)', 'IR (fps)', '3D (fps)', 'EMIT']}
              rows={vsRows.length ? vsRows : waitRow} />
          )}
          {show('arm') && armRows.length > 0 && (
            <Table title={t('팔 · ARM')} note={t(JOINT_NOTE)} widths={[14, 17, 11, 17, 15, 13, 13]} aligns={['left', 'left', 'left', 'right', 'right', 'right', 'right']}
              head={['ID', 'STATUS', 'ERR', '°C B/S', 'ANGLE°', 'Nm', 'A']} rows={armRows} />
          )}
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { flexDirection: 'row', gap: 8, alignItems: 'flex-start' },
  card: { borderWidth: 1, padding: 9 },
  tr: { flexDirection: 'row', alignItems: 'center' },
});
