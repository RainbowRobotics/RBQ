import { useState } from 'react';
import { View, Text, TextInput, StyleSheet, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { useView3d, type LidarMode } from '@/store/view3d';
import { useHeightmapCloud, HmChannel, isHeightmapGait } from '@/lib/heightmapCloud';
import { useTelemetry } from '@/store/telemetry';
import { MODERN_VIEW3D_THEMES } from '@/lib/view3dThemes';
import { t } from '@/lib/i18n';
import { inputVFix } from '@/components/ui/controls';
import { railAnchor } from '@/components/control/ToolRail';

export const viewAnchor = (winH: number, inset = 0) =>
  railAnchor(winH, 'right', inset) as { right: number; top: number; maxHeight: number };

const VIEW_MODES = ['Top', 'Robot', '3D'];
const LIDAR_OPTS: { label: string; value: LidarMode }[] = [
  { label: 'Off', value: 'off' }, { label: 'On', value: 'on' },
];

function Head({ text }: { text: string }) {
  const { c } = useTheme();
  return <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.6, marginTop: 10, marginBottom: 5 }}>{text}</Text>;
}

function Seg({ label, on, onPress, flex, disabled }: { label: string; on?: boolean; onPress?: () => void; flex?: number; disabled?: boolean }) {
  const { c, radius } = useTheme();
  return (
    <Tappable onPress={disabled ? undefined : onPress} style={[styles.seg, { flex: flex ?? 1, borderRadius: radius.sm, opacity: disabled ? 0.45 : 1,
      backgroundColor: on ? 'rgba(77,156,245,0.12)' : c.elev, borderColor: on ? 'rgba(77,156,245,0.5)' : c.line }]}>
      <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 11, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
}

function HmSeg({ label, color, on, live, onPress, disabled }: {
  label: string; color: string; on: boolean; live: boolean; onPress: () => void; disabled?: boolean;
}) {
  const { c, radius } = useTheme();
  return (
    <Tappable onPress={disabled ? undefined : onPress}
      style={[styles.hmSeg, { borderRadius: radius.sm, opacity: disabled ? 0.4 : 1,
        backgroundColor: on ? 'rgba(77,156,245,0.12)' : c.elev,
        borderColor: on ? 'rgba(77,156,245,0.5)' : c.line }]}>
      <View style={[styles.hmDot, { backgroundColor: color, opacity: on ? 1 : 0.35 }]} />
      <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 11, fontWeight: '600' }}>{label}</Text>
      {on && <View style={[styles.hmLive, { backgroundColor: live ? c.greenTx : 'transparent',
        borderColor: live ? 'transparent' : c.line }]} />}
    </Tappable>
  );
}

export function ViewSettingsPopover({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const s = useView3d();
  const hmClouds = useHeightmapCloud((st) => st.clouds);
  const hmGait = useTelemetry((st) => isHeightmapGait(st.robot?.gaitId));
  const has = (ch: HmChannel) => (hmClouds[ch]?.count ?? 0) > 0;
  const hmLive = {
    grid:  has(HmChannel.Map) || has(HmChannel.Edge),
    stair: has(HmChannel.Stair),
    edge:  has(HmChannel.StairEdge),
    foot:  has(HmChannel.FootQuery) || has(HmChannel.FootAnswer),
  };
  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const maxH = Math.max(220, winH - 96);
  const a = viewAnchor(winH, insets.right);
  const pos = { right: a.right, top: a.top, width: 262, maxHeight: Math.min(maxH, a.maxHeight), zIndex: 60, elevation: 60 };
  return (
    <Popover onClose={onClose} style={pos}>
      <View style={styles.popH}>
        <Icon name="sliders" size={13} color={c.accent2} />
        <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>{t('3D 씬 설정')}</Text>
      </View>

      <>
      <Head text={t('그리드')} />
      <View style={styles.row}>
        <Seg label={`Grid ${s.gridVisible ? 'ON' : 'OFF'}`} on={s.gridVisible} onPress={() => s.setGridVisible(!s.gridVisible)} flex={2} />
        <Tappable onPress={() => s.stepGridSize(-1)} style={[styles.step, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{'−'}</Text>
        </Tappable>
        <View style={[styles.size, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.sm }]}>
          <TextInput value={String(s.gridSize)} keyboardType="number-pad"
            onChangeText={(t) => { const n = parseInt(t.replace(/[^0-9]/g, ''), 10); if (!isNaN(n)) s.setGridSize(n); }}
            style={[{ color: c.text, fontSize: 13, textAlign: 'center', padding: 0 }, inputVFix]} />
        </View>
        <Text style={{ color: c.dim, fontSize: 11 }}>m</Text>
        <Tappable onPress={() => s.stepGridSize(1)} style={[styles.step, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>+</Text>
        </Tappable>
      </View>

      <Head text={t('씬 테마')} />
      <View style={styles.row}>
        {MODERN_VIEW3D_THEMES.map((t, i) => {
          const on = s.themeIndex === i;
          return (
            <Tappable key={t.label} onPress={() => s.setThemeIndex(i)}
              style={[styles.swatch, { borderRadius: radius.sm, backgroundColor: t.bg ?? c.panel2, borderColor: on ? c.accent : c.line, borderWidth: on ? 2 : 1 }]}>
              <Text style={{ color: c.text, fontSize: 10, fontWeight: on ? '700' : '500' }}>{t.label}</Text>
            </Tappable>
          );
        })}
      </View>

      <Head text={t('카메라 거동')} />
      <View style={styles.row}>
        {VIEW_MODES.map((label, i) => <Seg key={label} label={label} on={s.viewMode === i} onPress={() => s.setViewMode(i)} />)}
      </View>

      <Head text={t('LiDAR (표시)')} />
      <View style={styles.row}>
        {LIDAR_OPTS.map((o) => <Seg key={o.value} label={o.label} on={s.lidarMode === o.value} onPress={() => s.setLidarMode(o.value)} />)}
      </View>

      <Head text={t('Heightmap (표시)')} />
      <View style={styles.row}>
        <HmSeg label="Grid"  color="#3aa6a0" on={s.hmGrid}  live={hmLive.grid}  disabled={!hmGait} onPress={() => s.setHmLayer('hmGrid',  !s.hmGrid)} />
        <HmSeg label="Stair" color="#e69f00" on={s.hmStair} live={hmLive.stair} disabled={!hmGait} onPress={() => s.setHmLayer('hmStair', !s.hmStair)} />
        <HmSeg label="Edge"  color="#d55e00" on={s.hmEdge}  live={hmLive.edge}  disabled={!hmGait} onPress={() => s.setHmLayer('hmEdge',  !s.hmEdge)} />
        <HmSeg label="Foot Q/A" color="#ff4d6a" on={s.hmFoot} live={hmLive.foot} disabled={!hmGait} onPress={() => s.setHmLayer('hmFoot', !s.hmFoot)} />
      </View>
      {!hmGait && (
        <Text style={{ color: c.dim, fontSize: 10, marginTop: 4 }}>
          {t('계단·비전 보행에서만 표시됩니다')}
        </Text>
      )}
      </>
    </Popover>
  );
}

const styles = StyleSheet.create({
  fold: { flexDirection: 'row', alignItems: 'center', height: 30, paddingHorizontal: 9, borderWidth: 1, marginTop: 10 },
  popH: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 2 },
  row: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 6 },
  seg: { height: 28, paddingHorizontal: 10, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  hmSeg: { height: 28, paddingHorizontal: 9, flexDirection: 'row', alignItems: 'center', gap: 6, borderWidth: 1 },
  hmDot: { width: 8, height: 8, borderRadius: 4 },
  hmLive: { width: 5, height: 5, borderRadius: 3, borderWidth: 1 },
  swatch: { flex: 1, height: 30, alignItems: 'center', justifyContent: 'center' },
  step: { width: 30, height: 28, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  size: { width: 46, height: 28, borderWidth: 1, justifyContent: 'center', paddingHorizontal: 3 },
});
