import { useRef, useState } from 'react';
import {
  View, Text, StyleSheet, Modal as RNModal, Pressable, ScrollView, useWindowDimensions,
  type StyleProp, type ViewStyle,
} from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { RailButton } from '@/components/control/RailButton';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { railAnchor } from '@/components/control/ToolRail';
import { RecoveryPanel } from '@/components/panels/RecoveryPanel';
import { connection } from '@/lib/connection';
import { gait as gaitApi } from '@/lib/gait';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import { useFeatureWheel } from '@/store/capability';
import { useDockButton } from '@/components/control/overlays';
import { t } from '@/lib/i18n';
import type { MotionName } from '@/types/robot';

type Fixed = 'sit' | 'stand' | 'walk';
const FIXED: { key: Fixed; icon: IconName; label: string }[] = [
  { key: 'sit', icon: 'sit', label: 'SIT' },
  { key: 'stand', icon: 'stand', label: 'STAND' },
  { key: 'walk', icon: 'walk', label: 'WALK' },
];

function MBtn({ icon, label, active, dashed, onPress, onLongPress, style }: {
  icon: IconName; label: string; active?: boolean; dashed?: boolean; onPress?: () => void;
  onLongPress?: (y: number) => void;
  style?: StyleProp<ViewStyle>;
}) {
  const ref = useRef<View>(null);
  const open = () => ref.current?.measureInWindow((_x, y) => onLongPress?.(y));
  return (
    <View ref={ref} collapsable={false} style={style}>
      <RailButton icon={icon} label={label} active={active} dashed={dashed} hint={!!onLongPress}
        onPress={onPress} onLongPress={onLongPress ? open : undefined} delayLongPress={500} style={StyleSheet.absoluteFill} />
    </View>
  );
}

export function MotionButtons({ active, onSelect, onMore, extra = [] }: {
  active?: MotionName | null; onSelect?: (m: MotionName) => void; onMore?: () => void;
  extra?: { key: MotionName; icon: IconName; label: string }[];
}) {
  const { c, radius } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const ip = useRobot((s) => s.ip);
  const accessLevel = useSettings((s) => s.accessLevel);
  const featureWheel = useFeatureWheel();
  const [sub, setSub] = useState<null | { kind: 'sit' | 'walk'; y: number }>(null);
  const [recOpen, setRecOpen] = useState(false);
  const closeSub = () => setSub(null);
  const fire = (m: MotionName) => { closeSub(); onSelect?.(m); };
  const subLabel = { sit: 'Static · Calib', walk: 'Dynamic Motions' } as const;
  const subBtn = (icon: IconName, label: string, onPress: () => void, warn = false) => (
    <Tappable key={label} onPress={onPress}
      style={[styles.subBtn, {
        borderColor: warn ? 'rgba(240,136,62,0.45)' : c.line,
        backgroundColor: warn ? 'rgba(240,136,62,0.08)' : c.elev,
        borderRadius: radius.sm,
      }]}>
      <Icon name={icon} size={14} color={warn ? c.amberTx : c.muted} />
      <Text numberOfLines={1} style={{ color: warn ? c.amberTx : c.text, fontSize: 11.5, fontWeight: '600', flex: 1 }}>{label}</Text>
    </Tappable>
  );
  return (
    <>
      {[...FIXED, ...extra].map((b) => (
        <MBtn key={b.key} icon={b.icon} label={b.label} active={active === b.key}
          onPress={() => onSelect?.(b.key)} style={styles.colBtn}
          onLongPress={b.key === 'sit' || b.key === 'walk'
            ? (y) => setSub({ kind: b.key as 'sit' | 'walk', y }) : undefined} />
      ))}
      <MBtn icon="plus" label={t('모션')} dashed onPress={onMore} style={styles.colBtn} />

      <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent visible={sub != null}
        animationType="fade" onRequestClose={closeSub}>
        <Pressable style={StyleSheet.absoluteFill} onPress={closeSub}>
          {sub && (() => {
            const items = sub.kind === 'sit'
              ? 1 + (accessLevel >= 3 ? 1 : 0) + (accessLevel >= 2 ? 3 : 0)
              : (featureWheel ? 2 : 3);
            const estH = 39 + items * 37;
            const top = Math.max(12, Math.min(sub.y, winH - Math.min(estH, winH - 24) - 12));
            return (
              <Pressable onPress={() => {}}
                style={[styles.subPop, { left: railAnchor(winH, 'left', insets.left).left, top, maxHeight: winH - 24, backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
                <View style={[styles.subHead, { borderBottomColor: c.line2 }]}>
                  <Icon name={sub.kind === 'sit' ? 'sit' : 'walk'} size={13} color={c.dim} />
                  <Text style={{ color: c.dim, fontSize: 10.5, fontWeight: '700', letterSpacing: 0.3 }}>{subLabel[sub.kind]}</Text>
                </View>
                <ScrollView showsVerticalScrollIndicator contentContainerStyle={{ gap: 6 }}>
                  {sub.kind === 'sit' ? (
                    <>
                      {accessLevel >= 3 && subBtn('wrench', 'Manual Recovery', () => { closeSub(); setRecOpen(true); }, true)}
                      {subBtn('wrench', 'ZMP Calibrate', () => { closeSub(); gaitApi.zmpCalibrate(ip).catch(() => {}); })}
                      {accessLevel >= 2 && subBtn('sliders', 'Lock Joints', () => fire('lock'))}
                      {accessLevel >= 2 && subBtn('sit', 'Position Sit', () => fire('pos_sit'))}
                      {accessLevel >= 2 && subBtn('stand', 'Position Stand', () => fire('pos_stand'))}
                    </>
                  ) : (
                    <>
                      {subBtn('walk', 'Model Walk', () => { closeSub(); connection.sendMotion('walk', true); })}
                      {subBtn('wave', 'Wave', () => fire('wave'))}
                      {!featureWheel && subBtn('run', 'Run', () => fire('run'))}
                    </>
                  )}
                </ScrollView>
              </Pressable>
            );
          })()}
        </Pressable>
      </RNModal>

      <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent visible={recOpen}
        animationType="fade" onRequestClose={() => setRecOpen(false)}>
        <Pressable style={[StyleSheet.absoluteFill, { backgroundColor: 'rgba(0,0,0,0.35)' }]} onPress={() => setRecOpen(false)}>
          <Pressable onPress={() => {}} style={[styles.recModal, {
            width: Math.min(560, winW - 24), marginLeft: -Math.min(280, (winW - 24) / 2),
            backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <ScrollView showsVerticalScrollIndicator contentContainerStyle={{ padding: 16, paddingTop: 14 }}>
              <RecoveryPanel />
            </ScrollView>
            <Tappable onPress={() => setRecOpen(false)} hitSlop={6}
              style={[styles.recClose, { backgroundColor: c.elev, borderColor: c.line }]}>
              <Icon name="x" size={13} color={c.muted} />
            </Tappable>
          </Pressable>
        </Pressable>
      </RNModal>
    </>
  );
}

export function usePadExtra(): { key: MotionName; icon: IconName; label: string }[] {
  const dock = useDockButton();
  return [
    { key: 'stairs' as MotionName, icon: 'stairs' as IconName, label: 'STAIRS' },
    ...(dock ? [{ key: 'dock' as MotionName, icon: 'dock' as IconName, label: 'DOCKING' }] : []),
  ];
}

const styles = StyleSheet.create({
  colBtn: { height: 50, alignSelf: 'stretch' },
  subPop: {
    position: 'absolute', width: 194, padding: 10, borderWidth: 1,
    shadowColor: '#000', shadowOpacity: 0.35, shadowRadius: 18, shadowOffset: { width: 0, height: 8 }, elevation: 12,
  },
  subHead: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingBottom: 7, marginBottom: 8, borderBottomWidth: 1 },
  subBtn: { flexDirection: 'row', alignItems: 'center', gap: 8, height: 34, paddingHorizontal: 10, borderWidth: 1 },
  recModal: { position: 'absolute', left: '50%', top: '50%', maxHeight: '86%', transform: [{ translateY: -180 }], borderWidth: 1, overflow: 'hidden' },
  recClose: { position: 'absolute', right: 10, top: 10, width: 26, height: 26, borderRadius: 13, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
});
