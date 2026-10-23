import { useEffect, useRef } from 'react';
import { View, Text, StyleSheet, Pressable, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { arm } from '@/lib/arm';
import { dockAnchor } from '@/components/control/ToolRail';
import { t } from '@/lib/i18n';

function Jog({ label, onStart, onStop, w = 66 }: {
  label: string; onStart: () => void; onStop: () => void; w?: number;
}) {
  const { c, radius } = useTheme();
  const held = useRef(false);
  const stopRef = useRef(onStop);
  stopRef.current = onStop;
  useEffect(() => () => { if (held.current) stopRef.current(); }, []);
  const start = () => { held.current = true; onStart(); };
  const stop = () => { held.current = false; onStop(); };
  return (
    <Pressable onPressIn={start} onPressOut={stop}
      style={({ pressed }) => [styles.jog, {
        width: w, borderRadius: radius.sm,
        backgroundColor: pressed ? 'rgba(77,156,245,0.18)' : c.elev,
        borderColor: pressed ? 'rgba(77,156,245,0.6)' : c.line,
      }]}>
      <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '700' }}>{label}</Text>
    </Pressable>
  );
}

export function ArmPopover({ onClose, onOpenDoor, onDetail }: {
  onClose: () => void;
  onOpenDoor?: () => void;
  onDetail?: () => void;
}) {
  const { c, radius } = useTheme();
  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const gripHeld = useRef(false);
  useEffect(() => () => { if (gripHeld.current) arm.gripper('stop'); }, []);
  const pos = { ...dockAnchor(winH), width: 250 };
  return (
    <Popover onClose={onClose} style={pos}>
      <View style={styles.head}>
        <Icon name="hand" size={13} color={c.accent2} />
        <Text style={{ fontSize: 12, fontWeight: '700', color: c.text }}>{t('팔 조작')}</Text>
      </View>

      <Text style={[styles.cap, { color: c.dim }]}>{t('그리퍼')}</Text>
      <View style={{ flexDirection: 'row', gap: 6 }}>
        <Pressable onPressIn={() => { gripHeld.current = true; arm.gripper('open'); }}
          onPressOut={() => { gripHeld.current = false; arm.gripper('stop'); }}
          style={({ pressed }) => [styles.grip, {
            borderRadius: radius.md, borderColor: 'rgba(63,185,80,0.5)',
            backgroundColor: pressed ? 'rgba(63,185,80,0.22)' : 'rgba(63,185,80,0.10)',
          }]}>
          <Text style={{ color: c.greenTx, fontSize: 11, fontWeight: '700' }}>{t('열기')}</Text>
        </Pressable>
        <Pressable onPressIn={() => { gripHeld.current = true; arm.gripper('close'); }}
          onPressOut={() => { gripHeld.current = false; arm.gripper('stop'); }}
          style={({ pressed }) => [styles.grip, {
            borderRadius: radius.md, borderColor: 'rgba(210,153,34,0.5)',
            backgroundColor: pressed ? 'rgba(210,153,34,0.22)' : 'rgba(210,153,34,0.10)',
          }]}>
          <Text style={{ color: c.amberTx, fontSize: 11, fontWeight: '700' }}>{t('닫기')}</Text>
        </Pressable>
      </View>

      <Text style={[styles.cap, { color: c.dim, marginTop: 12 }]}>{t('위치 이동')}</Text>
      <View style={{ alignItems: 'center', gap: 5 }}>
        <Jog label="N ↑" onStart={() => arm.jogXyz(1, 1)} onStop={() => arm.jogXyz(1, 0)} />
        <View style={{ flexDirection: 'row', gap: 5, alignItems: 'center' }}>
          <Jog label="W ←" onStart={() => arm.jogXyz(2, -1)} onStop={() => arm.jogXyz(2, 0)} />
          <Text style={{ color: c.dim, fontSize: 8.5, width: 18, textAlign: 'center' }}>XY</Text>
          <Jog label="E →" onStart={() => arm.jogXyz(2, 1)} onStop={() => arm.jogXyz(2, 0)} />
        </View>
        <Jog label="S ↓" onStart={() => arm.jogXyz(1, -1)} onStop={() => arm.jogXyz(1, 0)} />
        <View style={{ flexDirection: 'row', gap: 5, marginTop: 4 }}>
          <Jog label={t('IN 전진')} w={84} onStart={() => arm.jogXyz(3, 1)} onStop={() => arm.jogXyz(3, 0)} />
          <Jog label={t('OUT 후진')} w={84} onStart={() => arm.jogXyz(3, -1)} onStop={() => arm.jogXyz(3, 0)} />
        </View>
        <Text style={{ color: c.dim, fontSize: 8.5 }}>{t('떼면 정지')}</Text>
      </View>

      <View style={{ flexDirection: 'row', gap: 6, marginTop: 12 }}>
        {onOpenDoor && (
          <Tappable onPress={onOpenDoor} style={[styles.link, { borderRadius: radius.md, backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '600' }}>{t('🚪 문 열기')}</Text>
          </Tappable>
        )}
        {onDetail && (
          <Tappable onPress={onDetail} style={[styles.link, { borderRadius: radius.md, backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.muted, fontSize: 10.5, fontWeight: '600' }}>{t('자세히')}</Text>
          </Tappable>
        )}
      </View>
    </Popover>
  );
}

const styles = StyleSheet.create({
  head: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 10 },
  cap: { fontSize: 9, fontWeight: '700', letterSpacing: 0.6, marginBottom: 5 },
  jog: { height: 30, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  grip: { flex: 1, height: 38, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  link: { flex: 1, height: 30, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
