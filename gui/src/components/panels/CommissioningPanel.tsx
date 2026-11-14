import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { commissioning } from '@/lib/commissioning';
import { Tappable } from '@/components/anim';
import { INIT_STATE } from '@/types/robot';

type Step = { label: string; on: boolean; onPress?: () => void };

function StepChip({ s, isNext }: { s: Step; isNext: boolean }) {
  const { c, radius } = useTheme();
  const body = (
    <Text numberOfLines={2}
          style={{ color: s.on ? c.greenTx : c.redTx, fontSize: 14, fontWeight: '700', textAlign: 'center' }}>
      {s.label}
    </Text>
  );
  const style = [styles.flag, {
    borderRadius: radius.sm,
    backgroundColor: s.on ? 'rgba(63,185,80,0.14)' : 'rgba(231,51,28,0.12)',
    borderColor: s.on ? 'rgba(63,185,80,0.55)' : isNext ? c.accent : 'rgba(255,107,94,0.55)',
    borderWidth: isNext && !s.on ? 2 : 1,
  }];
  if (s.onPress && !s.on) return <Tappable onPress={s.onPress} style={style}>{body}</Tappable>;
  return <View style={style}>{body}</View>;
}

export function AutoStartSteps() {
  const robot = useRobot((s) => s.robot);
  const ip = useRobot((s) => s.ip);
  const seq = robot?.autostart;
  const powerOn = seq
    ? (seq.steps[2] === INIT_STATE.pass || seq.steps[2] === INIT_STATE.warn)
    : !!robot?.can_bus;

  const steps: Step[] = [
    { label: 'POWER', on: powerOn },
    { label: 'CAN-BUS', on: !!robot?.can_bus, onPress: () => { commissioning.canCheck(ip).catch(() => {}); } },
    { label: 'HOMING', on: !!robot?.find_pose, onPress: () => { commissioning.findHome(ip).catch(() => {}); } },
    { label: 'IMU-CHECK', on: !!robot?.imu },
    { label: 'START', on: !!robot?.control_started },
  ];
  const nextIdx = steps.findIndex((s) => !s.on);
  return (
    <View style={styles.grid}>
      {steps.map((s, i) => <StepChip key={s.label} s={s} isNext={i === nextIdx} />)}
    </View>
  );
}

const styles = StyleSheet.create({
  grid: { flexDirection: 'row', gap: 5, maxWidth: 520, alignSelf: 'center' },
  flag: { width: 100, flexShrink: 1, minWidth: 0, paddingVertical: 11, paddingHorizontal: 3,
          alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
});
