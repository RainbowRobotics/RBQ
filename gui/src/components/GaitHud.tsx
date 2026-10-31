import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { useViewport } from '@/store/viewport';
import { gaitTone } from '@/lib/gaitTone';
import { useFeatureWheel } from '@/store/capability';
import { wheelGaitName } from '@/lib/wheelGait';
import { getLang } from '@/store/lang';

export function GaitHud({ bottom, left }: { bottom: number; left: number }) {
  const { c, fonts } = useTheme();
  const isSim = useViewport((s) => s.key) === 'sim';
  const conn = useRobot((s) => s.conn);
  const gait = useRobot((s) => s.gait);
  const gaitId = useRobot((s) => s.robot?.gait_id);
  const featureWheel = useFeatureWheel();
  if (isSim || conn !== 'connected' || !gait) return null;
  const tone = gaitTone(gaitId, gait);
  const tx = { fault: c.redTx, idle: c.accent2, active: c.greenTx, unknown: c.muted }[tone];
  const wheel = featureWheel ? wheelGaitName(gaitId) : null;
  const label = wheel ? (getLang() === 'en' ? wheel.en : wheel.ko) : gait;
  return (
    <Text pointerEvents="none" numberOfLines={1}
      style={[styles.txt, { bottom, left, color: tx, fontFamily: fonts.mono }]}>{label}</Text>
  );
}

const styles = StyleSheet.create({
  txt: {
    position: 'absolute', maxWidth: 220, zIndex: 12, elevation: 12,
    fontSize: 11.5, fontWeight: '700', letterSpacing: 0.4,
    textShadowColor: 'rgba(0,0,0,0.55)', textShadowOffset: { width: 0, height: 1 }, textShadowRadius: 3,
  },
});
