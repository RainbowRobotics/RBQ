import { useEffect, useRef, useState } from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { gamepadInput, type GamepadDeviceInfo } from '@/lib/gamepad/input';
import { useGamepad } from '@/store/gamepad';
import { useDevMode, useSettings, useSettingsHydrated } from '@/store/settings';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

const FLUSH_MS = 66;
const LOG_MAX = 40;

const hex4 = (n: number) => n.toString(16).padStart(4, '0').toUpperCase();

function AxisRow({ label, axis, value }: { label: string; axis: number; value: number }) {
  const { c, fonts } = useTheme();
  const pct = Math.max(-1, Math.min(1, value));
  return (
    <View style={styles.axisRow}>
      <Text style={[styles.axisLabel, { color: c.muted, fontFamily: fonts.mono }]}>
        {label} <Text style={{ color: c.dim }}>({axis})</Text>
      </Text>
      <View style={[styles.axisBar, { backgroundColor: c.bg, borderColor: c.line }]}>
        <View style={[styles.axisCenter, { backgroundColor: c.line }]} />
        <View
          style={{
            position: 'absolute', top: 2, bottom: 2,
            left: pct >= 0 ? '50%' : `${50 + pct * 50}%`,
            width: `${Math.abs(pct) * 50}%`,
            backgroundColor: Math.abs(pct) > 0.02 ? c.accent2 : c.dim,
            borderRadius: 3,
          }}
        />
      </View>
      <Text style={[styles.axisVal, { color: Math.abs(value) > 0.02 ? c.accent2 : c.dim, fontFamily: fonts.mono }]}>
        {value.toFixed(3)}
      </Text>
    </View>
  );
}

function DeviceCard({ dev }: { dev: GamepadDeviceInfo }) {
  const { c, fonts } = useTheme();
  return (
    <View style={[styles.devCard, { backgroundColor: c.elev, borderColor: c.line }]}>
      <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>🎮 {dev.name}</Text>
      <Text style={{ color: c.muted, fontSize: 10.5, fontFamily: fonts.mono, marginTop: 4 }}>
        VID:PID {hex4(dev.vendorId)}:{hex4(dev.productId)} · id {dev.id} · {t('축 {n}개').replace('{n}', String(dev.axes.length))}
      </Text>
      <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono, marginTop: 2 }} numberOfLines={1}>
        {dev.descriptor}
      </Text>
      <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono, marginTop: 2 }}>
        {dev.axes.map((a) => a.label.replace('AXIS_', '')).join(' · ') || t('축 정보 없음')}
      </Text>
    </View>
  );
}

export default function GamepadDiag() {
  const { c, fonts } = useTheme();
  const devMode = useDevMode();
  const hydrated = useSettingsHydrated();
  useEffect(() => { if (hydrated && useSettings.getState().accessLevel < 2) goTop('/'); }, [hydrated, devMode]);
  const devices = useGamepad((s) => s.devices);

  const axesRef = useRef<Record<number, Record<string, number>>>({});
  const dirtyRef = useRef(false);
  const [axes, setAxes] = useState<Record<number, Record<string, number>>>({});
  const [pressed, setPressed] = useState<Record<string, boolean>>({});
  const [log, setLog] = useState<string[]>([]);

  useEffect(() => {
    const offAxes = gamepadInput.onAxes((e) => {
      axesRef.current = { ...axesRef.current, [e.deviceId]: e.axes };
      dirtyRef.current = true;
    });
    const offBtn = gamepadInput.onButton((e) => {
      setPressed((p) => ({ ...p, [`${e.deviceId}:${e.keyCode}`]: e.down }));
      if (e.repeat === 0) {
        const ts = new Date().toTimeString().slice(0, 8);
        setLog((l) => [`${ts}  #${e.deviceId}  ${e.label} (${e.keyCode})  ${e.down ? '⬇ DOWN' : '⬆ UP'}`, ...l].slice(0, LOG_MAX));
      }
    });
    const t = setInterval(() => {
      if (dirtyRef.current) { dirtyRef.current = false; setAxes({ ...axesRef.current }); }
    }, FLUSH_MS);
    return () => { offAxes(); offBtn(); clearInterval(t); };
  }, []);

  const pressedNow = Object.entries(pressed).filter(([, v]) => v).map(([k]) => k);

  if (!devMode) return null;

  return (
    <Screen>
      <HubHeader title={t('게임패드 진단')} subtitle={gamepadInput.available ? t('기기 {n}대').replace('{n}', String(devices.length)) : t('이 빌드는 미지원')} />
      <View style={styles.wrap}>
        <View style={[styles.col, { flex: 1.1, backgroundColor: c.panel, borderColor: c.line }]}>
          <Text style={[styles.h, { color: c.text }]}>{t('연결된 기기')}</Text>
          <ScrollView showsVerticalScrollIndicator={false}>
            {devices.length === 0 ? (
              <Text style={{ color: c.dim, fontSize: 12, lineHeight: 18 }}>
                {gamepadInput.available
                  ? t('감지된 게임패드가 없습니다.\nUSB/블루투스로 연결하면 자동으로 나타납니다.')
                  : t('이 빌드에는 게임패드 모듈이 없습니다. APK를 다시 빌드하세요.')}
              </Text>
            ) : (
              devices.map((d) => <DeviceCard key={d.descriptor + d.id} dev={d} />)
            )}
          </ScrollView>
        </View>

        <View style={[styles.col, { flex: 1.6, backgroundColor: c.panel, borderColor: c.line }]}>
          <Text style={[styles.h, { color: c.text }]}>{t('축 (원시값)')}</Text>
          <ScrollView showsVerticalScrollIndicator={false}>
            {devices.map((d) => (
              <View key={d.id} style={{ marginBottom: 12 }}>
                {devices.length > 1 && (
                  <Text style={{ color: c.muted, fontSize: 11, marginBottom: 4 }}>#{d.id} {d.name}</Text>
                )}
                {d.axes.map((a) => (
                  <AxisRow key={a.axis} label={a.label} axis={a.axis} value={axes[d.id]?.[String(a.axis)] ?? 0} />
                ))}
              </View>
            ))}
            {devices.length === 0 && <Text style={{ color: c.dim, fontSize: 12 }}>—</Text>}
          </ScrollView>
        </View>

        <View style={[styles.col, { flex: 1.6, backgroundColor: c.panel, borderColor: c.line }]}>
          <Text style={[styles.h, { color: c.text }]}>{t('버튼 이벤트')}</Text>
          <View style={styles.pressedWrap}>
            {pressedNow.length === 0 ? (
              <Text style={{ color: c.dim, fontSize: 11 }}>{t('눌린 버튼 없음')}</Text>
            ) : (
              pressedNow.map((k) => (
                <View key={k} style={[styles.chip, { backgroundColor: 'rgba(77,156,245,0.12)', borderColor: 'rgba(77,156,245,0.5)' }]}>
                  <Text style={{ color: c.accent2, fontSize: 10.5, fontFamily: fonts.mono }}>{k}</Text>
                </View>
              ))
            )}
          </View>
          <ScrollView showsVerticalScrollIndicator={false}>
            {log.map((line, i) => (
              <Text key={i} style={{ color: i === 0 ? c.text : c.muted, fontSize: 10.5, fontFamily: fonts.mono, lineHeight: 17 }}>
                {line}
              </Text>
            ))}
            {log.length === 0 && <Text style={{ color: c.dim, fontSize: 12 }}>{t('버튼을 누르면 키코드가 기록됩니다.')}</Text>}
          </ScrollView>
        </View>
      </View>
    </Screen>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, flexDirection: 'row', padding: 12, gap: 10 },
  col: { borderWidth: 1, borderRadius: 14, padding: 14 },
  h: { fontSize: 13, fontWeight: '700', marginBottom: 10 },
  devCard: { borderWidth: 1, borderRadius: 10, padding: 11, marginBottom: 8 },
  axisRow: { flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 6 },
  axisLabel: { width: 110, fontSize: 10.5 },
  axisBar: { flex: 1, height: 16, borderRadius: 5, borderWidth: 1, overflow: 'hidden' },
  axisCenter: { position: 'absolute', left: '50%', top: 0, bottom: 0, width: 1 },
  axisVal: { width: 52, fontSize: 10.5, textAlign: 'right' },
  pressedWrap: { flexDirection: 'row', flexWrap: 'wrap', gap: 6, marginBottom: 10, minHeight: 24 },
  chip: { borderWidth: 1, borderRadius: 6, paddingHorizontal: 7, paddingVertical: 3 },
});
