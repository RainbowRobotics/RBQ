import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useViewport } from '@/store/viewport';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Popover } from '@/components/ui/overlays';
import { Toggle, Segmented } from '@/components/ui/controls';
import { dockAnchor } from '@/components/control/ToolRail';
import { useSettings, useDevMode } from '@/store/settings';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { GamepadInputMap } from '@/components/GamepadInputMap';
import { useInputMode } from '@/store/inputMode';
import { t } from '@/lib/i18n';

function Row({ nm, sub, children }: { nm: string; sub?: string; children: React.ReactNode }) {
  const { c } = useTheme();
  return (
    <View style={[styles.row, { borderBottomColor: c.line2 }]}>
      <View style={{ flex: 1 }}>
        <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{nm}</Text>
        {!!sub && <Text style={{ color: c.dim, fontSize: 9.5, marginTop: 2 }}>{sub}</Text>}
      </View>
      {children}
    </View>
  );
}

export function ControlPopover({ onClose }: { onClose: () => void }) {
  const isSim = useViewport((v) => v.key) === 'sim';
  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const devMode = useDevMode();
  const uiMode = useSettings((s) => s.gpUiMode);
  const setUiMode = useSettings((s) => s.setGpUiMode);
  const extJoy = useTelemetry((s) => !!s.robot?.extJoy);
  const ip = useRobot((s) => s.ip);
  const inputMode = useInputMode();
  const pos = { ...dockAnchor(winH), width: 250 };
  return (
    <Popover onClose={onClose} style={pos}>
      <Text style={styles.head}>{t('조종')}</Text>
      <View style={{ paddingTop: 9, paddingBottom: 6 }}>
        <Text style={{ color: '#8b96a3', fontSize: 9.5, marginBottom: 5 }}>{t('입력 모드 — 자동 = 패드가 붙으면 패드')}</Text>
      </View>
      <Segmented value={uiMode} onChange={setUiMode}
        options={[{ key: 'auto' as const, label: t('자동') },
          { key: 'virtual' as const, label: t('가상') },
          { key: 'gamepad' as const, label: t('패드') }]} />
      {devMode && !isSim && (
        <Row nm={t('외부 조종 (SLAM·SDK)')} sub={t('외부에 조종권을 넘김')}>
          <Toggle value={extJoy} onChange={(v) => actions.gamepadExternal(ip, v).catch(() => {})} />
        </Row>
      )}
      {uiMode !== 'virtual' && inputMode === 'gamepad' && <GamepadInputMap inline scale={0.58} />}
    </Popover>
  );
}

const styles = StyleSheet.create({
  head: { color: '#8b96a3', fontSize: 11, fontWeight: '700', marginBottom: 6 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 9, borderBottomWidth: 1 },
});
