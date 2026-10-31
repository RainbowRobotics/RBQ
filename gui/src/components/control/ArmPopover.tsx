import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useState } from 'react';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { arm } from '@/lib/arm';
import { armMotionReady } from '@/lib/armMotion';
import { useArmAvailable } from '@/lib/useArmAvailable';
import { useTelemetry } from '@/store/telemetry';
import { dockAnchor } from '@/components/control/ToolRail';
import { t } from '@/lib/i18n';

export function ArmPopover({ onClose, onManual, onMotion }: {
  onClose: () => void;
  onManual?: () => void;
  onMotion?: () => void;
}) {
  const { c, radius } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const available = useArmAvailable();
  const stat = useTelemetry(s => s.robot?.armStat);
  const canRun = available && armMotionReady(stat);
  const [error, setError] = useState('');
  const run = (go: () => void) => {
    setError('');
    try { go(); } catch (e) { setError(e instanceof Error ? e.message : String(e)); }
  };
  const anchor = dockAnchor(winH), width = Math.min(250, winW > 0 ? winW - 24 : 250);
  const pos = { ...anchor, left: Math.min(anchor.left, Math.max(12, winW - width - 12)), width };
  const enter = (label: string, action: () => void) =>
    <Tappable onPress={action}
      style={[styles.enter, { borderRadius: radius.md, backgroundColor: 'rgba(77,156,245,0.12)', borderColor: c.accent2 }]}>
      <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{t(label)}</Text>
    </Tappable>;

  return (
    <Popover onClose={onClose} style={pos}>
      <View style={styles.head}>
        <Icon name="hand" size={13} color={c.accent2} />
        <Text style={{ fontSize: 12, fontWeight: '700', color: c.text }}>{t('팔 조작')}</Text>
      </View>

      {onManual && enter('수동제어', onManual)}
      {onMotion && enter('팔 모션', onMotion)}

      {!!error && <Text accessibilityRole="alert" style={{ color: c.redbright, fontSize: 10.5 }}>{error}</Text>}
      <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 6, marginTop: 6 }}>
        {(['Folding', 'Ready', 'Straight', 'Lift'] as const).map(pose =>
          <Tappable key={pose} disabled={!canRun} onPress={() => run(() => arm.goMotion(pose, pose === 'Folding' ? 1 : 0))}
            style={[styles.quick, { borderRadius: radius.md, backgroundColor: c.elev, borderColor: c.line, opacity: canRun ? 1 : .4 }]}>
            <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{pose}</Text>
          </Tappable>)}
      </View>
      {!canRun && <Text style={{ color: c.dim, fontSize: 9.5, marginTop: 6 }}>
        {t(!available ? '팔 전원 및 통신 확인 필요' : '팔 제어 시작을 확인하세요.')}</Text>}
    </Popover>
  );
}

const styles = StyleSheet.create({
  head: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 10 },
  enter: { height: 34, borderWidth: 1, alignItems: 'center', justifyContent: 'center', marginBottom: 6 },
  quick: { width: '48%', flexGrow: 1, height: 32, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
