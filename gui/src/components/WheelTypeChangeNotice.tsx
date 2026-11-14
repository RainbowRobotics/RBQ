import { useEffect, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Svg, { Line, Polyline, Rect, Circle, Path } from 'react-native-svg';
import { Modal } from '@/components/ui/overlays';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { useTheme } from '@/theme';
import { t } from '@/lib/i18n';

export function WheelTypeChangeNotice() {
  const { c } = useTheme();
  const autostart = useRobot((s) => s.robot?.autostart);
  const [shownFor, setShownFor] = useState<number>();
  const [change, setChange] = useState(0);
  useEffect(() => {
    if (!autostart || autostart.running || !autostart.wheel_change || !autostart.start_unix_ms) return;
    if (autostart.start_unix_ms === shownFor) return;
    setShownFor(autostart.start_unix_ms);
    setChange(autostart.wheel_change);
  }, [autostart, shownFor]);
  if (!change) return null;
  const toWheel = change === 2;
  const close = () => setChange(0);
  const leg = t('레그 타입'), wheel = t('휠 타입');
  return (
    <Modal onClose={close}>
      <View style={[styles.card, { backgroundColor: c.panel, borderColor: c.line }]}>
        <Text style={[styles.title, { color: c.text }]}>{t('로봇 종류가 바뀌었습니다')}</Text>
        <View style={styles.art}>
          <View style={[styles.fig, { opacity: 0.45 }]}>
            <RobotSideArt wheel={!toWheel} />
            <Text style={[styles.cap, { color: c.muted }]}>{toWheel ? leg : wheel}</Text>
          </View>
          <Svg width={28} height={20} viewBox="0 0 28 20">
            <Path d="M3 10h20M16 4l7 6-7 6" fill="none" stroke={c.muted} strokeWidth={2.4} strokeLinecap="round" strokeLinejoin="round" />
          </Svg>
          <View style={styles.fig}>
            <RobotSideArt wheel={toWheel} />
            <Text style={[styles.cap, { color: c.text }]}>{toWheel ? wheel : leg}</Text>
          </View>
        </View>
        <Text style={[styles.body, { color: c.text }]}>
          {toWheel ? t('레그 타입 로봇에서 휠 타입 로봇으로 변경됩니다.') : t('휠 타입 로봇에서 레그 타입 로봇으로 변경됩니다.')}
        </Text>
        <Text style={[styles.hint, { color: c.muted }]}>
          {toWheel ? t('자동 기동에서 바퀴 보드가 감지되었습니다.') : t('자동 기동에서 바퀴 보드가 하나도 감지되지 않았습니다.')}
        </Text>
        <Tappable onPress={close} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
      </View>
    </Modal>
  );
}

function RobotSideArt({ wheel }: { wheel: boolean }) {
  const { c } = useTheme();
  const leg = (hx: number, far: boolean) => {
    const foot = wheel ? 106 : 116;
    const o = far ? 0.55 : 1;
    return (
      <>
        <Polyline points={`${hx},66 ${hx + 13},90 ${hx - 1},${foot}`} fill="none" stroke={c.muted} strokeWidth={7}
          strokeLinecap="round" strokeLinejoin="round" opacity={o} />
        {wheel && <Circle cx={hx - 1} cy={106} r={11} fill={c.elev2} stroke={c.muted} strokeWidth={3} opacity={o} />}
        {wheel && <Circle cx={hx - 1} cy={106} r={3} fill={c.muted} opacity={o} />}
      </>
    );
  };
  return (
    <Svg width={140} height={88} viewBox="0 0 200 126">
      <Line x1={10} y1={118} x2={190} y2={118} stroke={c.muted} strokeWidth={2} strokeLinecap="round" opacity={0.55} />
      {leg(68, true)}{leg(152, true)}
      {leg(62, false)}{leg(146, false)}
      <Rect x={46} y={42} width={112} height={28} rx={7} fill={c.elev2} stroke={c.muted} strokeWidth={1.6} />
      <Rect x={78} y={35} width={62} height={8} rx={3} fill={c.elev2} stroke={c.muted} strokeWidth={1.3} />
    </Svg>
  );
}

const styles = StyleSheet.create({
  card: { width: 404, maxWidth: '100%', borderRadius: 18, borderWidth: 1, padding: 26, alignItems: 'center', gap: 10 },
  title: { fontSize: 18, fontWeight: '700', textAlign: 'center' },
  art: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6 },
  fig: { alignItems: 'center', gap: 2 },
  cap: { fontSize: 12, fontWeight: '600' },
  body: { fontSize: 15, lineHeight: 22, textAlign: 'center' },
  hint: { fontSize: 12.5, lineHeight: 18, textAlign: 'center' },
  btn: { alignSelf: 'stretch', height: 46, marginTop: 8, borderRadius: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
