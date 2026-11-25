import { useState } from 'react';
import { View, Text, StyleSheet, ScrollView, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Toggle } from '@/components/ui/controls';
import { notify } from '@/components/DoneToast';
import { t } from '@/lib/i18n';
import { useRobot } from '@/store/robot';
import { actions, type CameraLiveSensor } from '@/lib/rest';
import { CAMERA_NAMES, statusCell, usbLabel, type CellTone } from '@/lib/cameraCheckView';

const PENDING_MS = 3000;
type Pending = Record<string, { v: boolean; until: number }>;

const typeLabel = (s: CameraLiveSensor | undefined) =>
  s?.type === 'realsense' ? 'RealSense D430' : s?.type === 'oak' ? 'OAK-D' : '—';
const typeShort = (s: CameraLiveSensor | undefined) => (s?.type === 'realsense' ? 'RS' : s?.type === 'oak' ? 'OAK' : '—');
const res = (wh: [number, number] | undefined, fps: number | undefined) =>
  wh && wh[0] > 0 ? `${wh[0]}×${wh[1]} · ${fps ?? '—'}` : '—';

export function CameraSettings({ sensors, height, sel, onSel }: {
  sensors: CameraLiveSensor[] | undefined; height: number; sel: number; onSel: (i: number) => void;
}) {
  const { c, fonts, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const { width: winW } = useWindowDimensions();
  const narrow = winW < 700;
  const [pending, setPending] = useState<Pending>({});
  const s = sensors?.[sel];
  const cfg = s?.config;
  const now = Date.now();
  const shown = (key: string, robot: boolean) => (pending[key] && now < pending[key].until ? pending[key].v : robot);

  const send = (key: string, v: boolean, req: Promise<unknown>) => {
    setPending((p) => ({ ...p, [key]: { v, until: Date.now() + PENDING_MS } }));
    req.catch((e) => {
      setPending((p) => { const n = { ...p }; delete n[key]; return n; });
      notify(String(e?.message ?? e), 'warn');
    });
  };
  const projOn = shown(`p${sel}`, !!s?.status.projector_on);
  const floodOn = shown(`f${sel}`, !!s?.status.flood_on);
  const setProj = (v: boolean) => send(`p${sel}`, v, actions.irProjector(ip, v, sel));
  const setFlood = (v: boolean) => send(`f${sel}`, v, actions.irFlood(ip, sel, v));

  const toneColor = (tone: CellTone) => (tone === 'ok' ? c.greenTx : tone === 'fail' ? c.redTx : tone === 'warn' ? c.amberTx : c.muted);
  const toneBg = (tone: CellTone) => (tone === 'off' ? c.elev : `${toneColor(tone)}1f`);
  const st = s ? statusCell(s.status) : null;
  const guide = s?.status.dead ? t('자동 복구로 살아나지 않았습니다 — 케이블을 확인한 뒤 비전을 다시 시작하세요')
    : s?.status.recovering ? t('자동 복구 중입니다 — 잠시 기다리세요') : st?.text === 'RUN' ? t('정상') : '';

  const group = (title: string, children: React.ReactNode, right?: React.ReactNode) => (
    <View style={[styles.group, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
      <View style={styles.groupHead}>
        <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{title}</Text>
        {right}
      </View>
      {children}
    </View>
  );
  const row = (label: string, value: React.ReactNode) => (
    <View key={label} style={styles.row}>
      <Text style={{ color: c.text, fontSize: 13 }}>{label}</Text>
      {value}
    </View>
  );
  const cell = (label: string, value: string, off?: boolean) => (
    <View key={label} style={styles.cell}>
      <Text style={{ color: c.muted, fontSize: 12 }}>{label}</Text>
      <Text style={{ color: off ? c.dim : c.text, fontSize: 14, fontWeight: '700', fontFamily: fonts.mono }}>{value}</Text>
    </View>
  );
  const info = (label: string, value: string, off?: boolean) => (
    <View key={label} style={[styles.info, { backgroundColor: c.elev, borderRadius: radius.sm }]}>
      <Text style={{ color: c.muted, fontSize: 11 }}>{label}</Text>
      <Text style={{ color: off ? c.dim : c.text, fontSize: 13, fontWeight: '600', fontFamily: fonts.mono }}>{value}</Text>
    </View>
  );
  const b = (v: boolean | undefined) => (v === undefined ? '—' : String(v));

  const preset = s?.type === 'oak' ? 'ROBOTICS' : cfg ? (cfg.rs_high_density ? 'High Density' : 'High Accuracy') : '—';
  const video = s?.type === 'oak'
    ? [cell(t('스테레오 프리셋'), preset), cell('Color', res(cfg?.color, cfg?.fps), !cfg?.color_enabled),
      cell('Depth', res(cfg?.depth, cfg?.fps), !cfg?.depth_enabled), cell('IR', res(cfg?.depth, cfg?.fps), !cfg?.ir_enabled)]
    : [cell(t('프리셋'), preset), cell('Depth', res(cfg?.depth, cfg?.fps), !cfg?.depth_enabled),
      cell('IR', res(cfg?.depth, cfg?.fps), !cfg?.ir_enabled), cell('Color', res(cfg?.color, cfg?.fps), !cfg?.color_enabled),
      cell(t('Depth 필터'), b(cfg?.depth_filtering), cfg?.depth_filtering === false)];

  return (
    <View style={[styles.wrap, { height, borderColor: c.line, borderRadius: radius.md }]}>
      <ScrollView style={[styles.list, { borderRightColor: c.line }, narrow && { width: 110 }]}>
        {CAMERA_NAMES.map((name, i) => {
          const si = sensors?.[i];
          const cst = si ? statusCell(si.status) : null;
          const on = i === sel;
          return (
            <Tappable key={name} onPress={() => onSel(i)} scaleTo={1}
              style={[styles.item, { borderBottomColor: c.line, borderLeftColor: on ? c.accent : 'transparent' }, on && { backgroundColor: c.elev }]}>
              <Text style={{ color: c.text, fontSize: 13, fontWeight: on ? '700' : '600', width: narrow ? undefined : 64 }} numberOfLines={1}>{name}</Text>
              {!narrow && <Text style={{ color: c.muted, fontSize: 12, width: 34 }}>{typeShort(si)}</Text>}
              <Text style={{ color: cst ? toneColor(cst.tone) : c.dim, fontSize: 12, fontWeight: '700', fontFamily: fonts.mono, flex: 1, textAlign: 'right' }} numberOfLines={1}>
                {cst?.text ?? '—'}
              </Text>
            </Tappable>
          );
        })}
      </ScrollView>
      <ScrollView style={{ flex: 1, backgroundColor: c.elev }} contentContainerStyle={{ padding: 12, gap: 10 }}>
        <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>
          {CAMERA_NAMES[sel]}
          <Text style={{ color: c.muted, fontSize: 13, fontWeight: '500' }}>{`  ${typeLabel(s)}${s?.serial ? ` · ${s.serial}` : ''}`}</Text>
        </Text>
        {group(t('자동 복구'),
          <View style={styles.recovery}>
            <View style={[styles.pill, { backgroundColor: st ? toneBg(st.tone) : c.elev, borderRadius: radius.md }]}>
              <Text style={{ color: st ? toneColor(st.tone) : c.dim, fontSize: 13, fontWeight: '800' }}>{st?.text ?? '—'}</Text>
            </View>
            {!!guide && <Text style={{ color: c.text, fontSize: 13, flexShrink: 1 }}>{guide}</Text>}
          </View>)}
        <View style={[styles.cols, narrow && { flexDirection: 'column' }]}>
          <View style={{ flex: 1 }}>
            {group('IR lights',
              !s?.status.projector
                ? <Text style={{ color: c.dim, fontSize: 13 }}>{t('IR 프로젝터가 없는 카메라입니다')}</Text>
                : <>
                  {row(t('IR 프로젝터'), <Toggle value={projOn} onChange={setProj} />)}
                  {s.type === 'oak' && row(t('IR 조명'), <Toggle value={floodOn} onChange={setFlood} />)}
                </>,
              s?.type === 'oak' && s.status.projector
                ? <Tappable onPress={() => setFlood(!projOn)} style={[styles.mini, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                  <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('기본값')}</Text>
                </Tappable>
                : undefined)}
          </View>
          <View style={{ flex: 1 }}>{group(t('영상'), <View style={styles.cells}>{video}</View>)}</View>
        </View>
        {group(t('장치 정보'),
          <View style={styles.infos}>
            {info('USB', usbLabel(s?.usb) || t('연결 안 됨'), !usbLabel(s?.usb))}
            {info('Enabled', b(cfg?.enabled), cfg?.enabled === false)}
            {info('AlwaysON', b(cfg?.always_on), cfg?.always_on === false)}
            {info('ColorEnabled', b(cfg?.color_enabled), cfg?.color_enabled === false)}
            {info('IrEnabled', b(cfg?.ir_enabled), cfg?.ir_enabled === false)}
            {info('DepthEnabled', b(cfg?.depth_enabled), cfg?.depth_enabled === false)}
          </View>)}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { width: '100%', flexDirection: 'row', borderWidth: 1, overflow: 'hidden' },
  list: { width: 230, flexGrow: 0, borderRightWidth: StyleSheet.hairlineWidth },
  item: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 12, paddingVertical: 11, borderBottomWidth: StyleSheet.hairlineWidth, borderLeftWidth: 3 },
  group: { flexGrow: 1, borderWidth: 1, padding: 10, gap: 6 },
  groupHead: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', minHeight: 22 },
  row: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', minHeight: 30, gap: 8 },
  recovery: { flexDirection: 'row', alignItems: 'center', gap: 12, flexWrap: 'wrap' },
  cols: { flexDirection: 'row', gap: 10, alignItems: 'stretch' },
  cells: { flexDirection: 'row', flexWrap: 'wrap', rowGap: 10 },
  cell: { width: '50%', paddingRight: 8, gap: 2 },
  infos: { flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  info: { minWidth: 92, paddingHorizontal: 10, paddingVertical: 6, gap: 2 },
  pill: { paddingHorizontal: 10, paddingVertical: 2 },
  mini: { height: 26, paddingHorizontal: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
